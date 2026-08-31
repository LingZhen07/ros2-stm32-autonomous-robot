#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_stm32_bridge/msg/bridge_status.hpp"
#include "robot_stm32_bridge/msg/wheel_state.hpp"
#include "robot_stm32_bridge/protocol_v1.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_stm32_bridge {
namespace {

using namespace std::chrono_literals;
namespace protocol = protocol_v1;

constexpr uint16_t kMotionHeartbeatFresh = 1U << 2U;
constexpr uint16_t kMotionDisarmedHandshakeSeen = 1U << 3U;
constexpr uint16_t kMotionAuthorityArmed = 1U << 4U;
constexpr uint16_t kMotionCommandFresh = 1U << 6U;
constexpr uint16_t kMotionMotorAuthorized = 1U << 7U;
constexpr uint16_t kMotionStbyEnabled = 1U << 8U;
constexpr uint16_t kMotionCriticalTasksHealthy = 1U << 9U;
constexpr uint16_t kMotionWatchdogRefreshAllowed = 1U << 10U;
constexpr uint16_t kMotionBodyCommandReady = 1U << 15U;

constexpr uint16_t kCommunicationErrorWarning = 1U << 0U;
constexpr uint16_t kCommunicationErrorPassive = 1U << 1U;
constexpr uint16_t kCommunicationBusOff = 1U << 2U;
constexpr uint16_t kCommunicationRxSoftwareOverflow = 1U << 3U;
constexpr uint16_t kCommunicationRxHardwareLoss = 1U << 4U;
constexpr uint16_t kCommunicationVersionMismatch = 1U << 7U;
constexpr uint16_t kCommunicationSessionActive = 1U << 8U;
constexpr uint16_t kCommunicationFdBrsConfigured = 1U << 10U;
constexpr uint16_t kCommunicationCommandSequenceSeen = 1U << 11U;
constexpr uint16_t kCommunicationHeartbeatSequenceSeen = 1U << 12U;

constexpr uint32_t kCriticalFaultMask =
    (1UL << 1U) | (1UL << 2U) | (1UL << 3U) | (1UL << 7U) | (1UL << 8U) |
    (1UL << 9U) | (1UL << 10U) | (1UL << 11U);

diagnostic_msgs::msg::KeyValue key_value(const std::string &key,
                                         const std::string &value) {
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template <typename T> std::string decimal_string(const T value) {
  return std::to_string(value);
}

std::string hex_string(const uint64_t value, const unsigned width) {
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex
         << std::setw(static_cast<int>(width)) << std::setfill('0') << value;
  return stream.str();
}

const char *system_state_name(const uint8_t state) {
  switch (state) {
  case 0U:
    return "BOOT";
  case 1U:
    return "INIT";
  case 2U:
    return "SAFE";
  case 3U:
    return "READY";
  case 4U:
    return "ACTIVE";
  case 5U:
    return "FAULT";
  default:
    return "INVALID";
  }
}

std::string fault_names(const uint32_t flags) {
  static constexpr std::array<const char *, 12> names{
      "IMU_INITIALIZATION",  "COMMAND_TIMEOUT",     "INVALID_MOTOR_COMMAND",
      "SUPERVISOR",          "ENCODER_VALIDITY",    "BATTERY_MEASUREMENT",
      "CONTROL_SATURATION",  "CONTROL_ABNORMAL",    "INTERNAL_CONFIGURATION",
      "RTOS_STACK_OVERFLOW", "RTOS_MALLOC_FAILURE", "FDCAN_COMMUNICATION"};
  if (flags == 0U) {
    return "none";
  }
  std::ostringstream stream;
  bool first = true;
  for (std::size_t bit = 0; bit < names.size(); ++bit) {
    if ((flags & (1UL << bit)) != 0U) {
      if (!first) {
        stream << ',';
      }
      stream << names[bit];
      first = false;
    }
  }
  return stream.str();
}

uint32_t generate_nonzero_session_id() {
  std::random_device random_device;
  const auto steady_value = static_cast<uint64_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
  std::seed_seq seed{random_device(), random_device(),
                     static_cast<unsigned>(steady_value),
                     static_cast<unsigned>(steady_value >> 32U),
                     static_cast<unsigned>(::getpid())};
  std::mt19937 generator(seed);
  uint32_t result = 0U;
  while (result == 0U) {
    result = generator();
  }
  return result;
}

} // namespace

class BridgeNode final : public rclcpp::Node {
public:
  BridgeNode()
      : Node("robot_stm32_bridge"),
        process_started_(std::chrono::steady_clock::now()),
        session_id_(generate_nonzero_session_id()) {
    can_interface_ = declare_parameter<std::string>("can_interface", "can3");
    cmd_vel_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "imu_link");
    command_timeout_ms_ = declare_parameter<int>("command_timeout_ms", 100);
    status_timeout_ms_ = declare_parameter<int>("status_timeout_ms", 350);
    reconnect_period_ms_ = declare_parameter<int>("reconnect_period_ms", 1000);
    unsupported_twist_epsilon_ =
        declare_parameter<double>("unsupported_twist_epsilon", 1.0e-9);

    if (can_interface_.empty() || command_timeout_ms_ < 20 ||
        command_timeout_ms_ >= 250 || status_timeout_ms_ < 200 ||
        reconnect_period_ms_ < 100 || unsupported_twist_epsilon_ < 0.0) {
      throw std::invalid_argument(
          "invalid bridge parameter: command_timeout_ms must be [20,249], "
          "status_timeout_ms >= 200, reconnect_period_ms >= 100");
    }

    status_publisher_ =
        create_publisher<msg::BridgeStatus>("/stm32/status", 10);
    wheel_publisher_ =
        create_publisher<msg::WheelState>("/stm32/wheel_state", 20);
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        "/stm32/imu_raw", rclcpp::SensorDataQoS());
    battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
        "/stm32/battery", rclcpp::SensorDataQoS());
    diagnostics_publisher_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
                                                                10);

    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, rclcpp::QoS(10),
        std::bind(&BridgeNode::on_cmd_vel, this, std::placeholders::_1));

    arm_service_ = create_service<std_srvs::srv::Trigger>(
        "~/arm", std::bind(&BridgeNode::on_arm, this, std::placeholders::_1,
                           std::placeholders::_2));
    disarm_service_ = create_service<std_srvs::srv::Trigger>(
        "~/disarm", std::bind(&BridgeNode::on_disarm, this,
                              std::placeholders::_1, std::placeholders::_2));

    rx_timer_ =
        create_wall_timer(2ms, std::bind(&BridgeNode::receive_frames, this));
    heartbeat_timer_ =
        create_wall_timer(100ms, std::bind(&BridgeNode::send_heartbeat, this));
    authority_timer_ =
        create_wall_timer(100ms, std::bind(&BridgeNode::send_authority, this));
    command_timer_ =
        create_wall_timer(20ms, std::bind(&BridgeNode::send_command, this));
    health_timer_ =
        create_wall_timer(50ms, std::bind(&BridgeNode::health_tick, this));
    status_timer_ =
        create_wall_timer(50ms, std::bind(&BridgeNode::publish_status, this));
    diagnostics_timer_ = create_wall_timer(
        1s, std::bind(&BridgeNode::publish_diagnostics, this));

    if (!open_socket()) {
      RCLCPP_WARN(get_logger(),
                  "SocketCAN interface %s is not available yet: %s",
                  can_interface_.c_str(), last_transport_error_.c_str());
    }
    RCLCPP_INFO(get_logger(),
                "Protocol 1.0 bridge started DISARMED on %s with session %s",
                can_interface_.c_str(), hex_string(session_id_, 8U).c_str());
  }

  ~BridgeNode() override {
    if (socket_fd_ >= 0) {
      const bool motion_was_requested = arm_requested_;
      arm_requested_ = false;
      if (motion_was_requested) {
        transmit_disabled_command();
      }
      transmit_authority(false);
      transmit_heartbeat(false);
      close_socket();
    }
  }

private:
  struct SequenceState {
    bool seen{false};
    uint16_t last{0U};
  };

  uint32_t host_uptime_ms() const {
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - process_started_)
            .count();
    return static_cast<uint32_t>(static_cast<uint64_t>(elapsed) &
                                 0xFFFFFFFFULL);
  }

  uint32_t
  age_ms(const std::chrono::steady_clock::time_point &time_point) const {
    if (time_point == std::chrono::steady_clock::time_point{}) {
      return std::numeric_limits<uint32_t>::max();
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - time_point)
                         .count();
    if (age <= 0) {
      return 0U;
    }
    return static_cast<uint32_t>(
        std::min<int64_t>(age, std::numeric_limits<uint32_t>::max()));
  }

  bool status_fresh() const {
    return have_system_status_ && age_ms(last_system_status_received_) <=
                                      static_cast<uint32_t>(status_timeout_ms_);
  }

  bool local_command_fresh() const {
    return have_cmd_vel_ && age_ms(last_cmd_vel_received_) <=
                                static_cast<uint32_t>(command_timeout_ms_);
  }

  bool stm32_critical_fault() const {
    return have_system_status_ &&
           (system_status_.fault_flags & kCriticalFaultMask) != 0U;
  }

  bool stm32_can_healthy() const {
    if (!have_system_status_) {
      return false;
    }
    const uint16_t active_errors = kCommunicationErrorWarning |
                                   kCommunicationErrorPassive |
                                   kCommunicationBusOff;
    return (system_status_.communication_flags & active_errors) == 0U;
  }

  bool can_healthy() const {
    return socket_fd_ >= 0 && !transport_fault_active_ &&
           !can_error_observed_ && stm32_can_healthy();
  }

  bool connected() const {
    return socket_fd_ >= 0 && status_fresh() && protocol_valid_ &&
           can_healthy() && session_acknowledged_ &&
           system_status_.active_session_id == session_id_ &&
           heartbeat_ack_consistent_ &&
           (system_status_.motion_flags & kMotionHeartbeatFresh) != 0U &&
           (system_status_.communication_flags &
            kCommunicationFdBrsConfigured) != 0U;
  }

  bool safe_to_arm(std::string &reason) const {
    if (socket_fd_ < 0) {
      reason = "SocketCAN socket is not open";
      return false;
    }
    if (!protocol_valid_) {
      reason = "Protocol 1.0 has not been validated or a violation is latched";
      return false;
    }
    if (!status_fresh()) {
      reason = "STM32 SYSTEM_STATUS is stale";
      return false;
    }
    if (!can_healthy()) {
      reason = "CAN health is not Error Active/healthy";
      return false;
    }
    if (stm32_critical_fault()) {
      reason = "STM32 critical fault is present";
      return false;
    }
    if ((system_status_.communication_flags & kCommunicationVersionMismatch) !=
        0U) {
      reason = "STM32 protocol-version mismatch latch is set";
      return false;
    }
    if (!session_acknowledged_ ||
        system_status_.active_session_id != session_id_ ||
        !heartbeat_ack_consistent_ ||
        (system_status_.motion_flags & kMotionHeartbeatFresh) == 0U) {
      reason = "current host session/heartbeat is not acknowledged";
      return false;
    }
    const bool already_armed_active =
        system_status_.system_state == msg::BridgeStatus::SYSTEM_ACTIVE &&
        system_status_.active_session_id == session_id_ &&
        (system_status_.motion_flags & kMotionAuthorityArmed) != 0U;
    if (system_status_.system_state != msg::BridgeStatus::SYSTEM_SAFE &&
        system_status_.system_state != msg::BridgeStatus::SYSTEM_READY &&
        !already_armed_active) {
      reason = std::string("STM32 state is ") +
               system_state_name(system_status_.system_state);
      return false;
    }
    if ((system_status_.communication_flags & kCommunicationFdBrsConfigured) ==
        0U) {
      reason = "STM32 does not report FD+BRS configured";
      return false;
    }
    const uint16_t required = kMotionCriticalTasksHealthy |
                              kMotionWatchdogRefreshAllowed |
                              kMotionBodyCommandReady;
    if ((system_status_.motion_flags & required) != required) {
      reason = "STM32 runtime/drivetrain readiness gates are incomplete";
      return false;
    }
    reason = "ready";
    return true;
  }

  bool open_socket() {
    close_socket();
    socket_fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_fd_ < 0) {
      last_transport_error_ = std::string("socket: ") + std::strerror(errno);
      schedule_reconnect();
      return false;
    }

    int enable_fd = 1;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                     sizeof(enable_fd)) != 0) {
      last_transport_error_ =
          std::string("CAN_RAW_FD_FRAMES: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }

    const std::array<can_filter, 4> filters{
        {{protocol::kSystemStatusCanId,
          CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
         {protocol::kWheelStateCanId,
          CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
         {protocol::kImuDataCanId, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG},
         {protocol::kBatteryStateCanId,
          CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG}}};
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(sizeof(filters))) != 0) {
      last_transport_error_ =
          std::string("CAN_RAW_FILTER: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }

    const can_err_mask_t error_mask = CAN_ERR_MASK;
    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask,
                     sizeof(error_mask)) != 0) {
      last_transport_error_ =
          std::string("CAN_RAW_ERR_FILTER: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }

    ifreq request{};
    if (can_interface_.size() >= IFNAMSIZ) {
      last_transport_error_ = "CAN interface name is too long";
      close_socket();
      schedule_reconnect();
      return false;
    }
    std::strncpy(request.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1U);
    if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) != 0) {
      last_transport_error_ =
          std::string("SIOCGIFINDEX: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }

    const int interface_index = request.ifr_ifindex;
    if (::ioctl(socket_fd_, SIOCGIFFLAGS, &request) != 0) {
      last_transport_error_ =
          std::string("SIOCGIFFLAGS: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }
    if ((request.ifr_flags & IFF_UP) == 0) {
      last_transport_error_ = "CAN interface is not UP";
      close_socket();
      schedule_reconnect();
      return false;
    }
    if (::ioctl(socket_fd_, SIOCGIFMTU, &request) != 0) {
      last_transport_error_ =
          std::string("SIOCGIFMTU: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }
    if (request.ifr_mtu != CANFD_MTU) {
      last_transport_error_ = "CAN interface MTU is not CAN FD (72 bytes)";
      close_socket();
      schedule_reconnect();
      return false;
    }

    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = interface_index;
    if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&address),
               sizeof(address)) != 0) {
      last_transport_error_ = std::string("bind: ") + std::strerror(errno);
      close_socket();
      schedule_reconnect();
      return false;
    }

    last_transport_error_.clear();
    transport_fault_active_ = false;
    next_reconnect_ = std::chrono::steady_clock::time_point{};
    start_new_session("SocketCAN connected");
    RCLCPP_INFO(get_logger(), "SocketCAN bound to %s in CAN FD mode",
                can_interface_.c_str());
    return true;
  }

  void close_socket() {
    if (socket_fd_ >= 0) {
      ::close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void schedule_reconnect() {
    next_reconnect_ = std::chrono::steady_clock::now() +
                      std::chrono::milliseconds(reconnect_period_ms_);
  }

  void start_new_session(const std::string &reason) {
    session_id_ = generate_nonzero_session_id();
    heartbeat_sequence_ = 0U;
    authority_sequence_ = 0U;
    command_sequence_ = 0U;
    last_transmitted_heartbeat_sequence_ = 0U;
    last_transmitted_authority_sequence_ = 0U;
    last_transmitted_command_sequence_ = 0U;
    heartbeat_sent_for_session_ = false;
    command_sent_for_session_ = false;
    disarmed_sent_for_session_ = false;
    session_acknowledged_ = false;
    arm_requested_ = false;
    arm_request_started_ = {};
    have_cmd_vel_ = false;
    last_linear_velocity_ = 0.0;
    last_angular_velocity_ = 0.0;
    sequence_states_.clear();
    command_ack_consistent_ = false;
    heartbeat_ack_consistent_ = false;
    sequence_ack_fault_latched_ = false;
    protocol_valid_ = false;
    RCLCPP_INFO(get_logger(), "New DISARMED Protocol 1.0 session %s: %s",
                hex_string(session_id_, 8U).c_str(), reason.c_str());
  }

  void mark_transport_failure(const std::string &operation) {
    if (!transport_fault_active_) {
      RCLCPP_ERROR(get_logger(), "SocketCAN transport failure: %s",
                   operation.c_str());
    }
    last_transport_error_ = operation;
    transport_fault_active_ = true;
    reconnect_inhibited_ = true;
    arm_requested_ = false;
  }

  bool
  transmit_frame(const uint32_t can_id, const protocol::CommandPayload &payload,
                 builtin_interfaces::msg::Time *transmission_stamp = nullptr) {
    if (socket_fd_ < 0) {
      return false;
    }
    canfd_frame frame{};
    frame.can_id = can_id;
    frame.len = static_cast<__u8>(payload.size());
    frame.flags = CANFD_BRS;
    std::copy(payload.begin(), payload.end(), frame.data);
    const ssize_t written = ::write(socket_fd_, &frame, CANFD_MTU);
    if (written != CANFD_MTU) {
      ++tx_failure_count_;
      const int error_number = errno;
      std::ostringstream stream;
      stream << "CAN ID " << hex_string(can_id, 3U) << " write: ";
      if (written < 0) {
        stream << std::strerror(error_number);
      } else {
        stream << "short write " << written;
      }
      mark_transport_failure(stream.str());
      return false;
    }
    ++tx_frame_count_;
    if (transmission_stamp != nullptr) {
      *transmission_stamp = now();
    }
    return true;
  }

  bool transmit_heartbeat(const bool bridge_ready) {
    const auto payload = protocol::encode_host_heartbeat(
        heartbeat_sequence_, session_id_, host_uptime_ms(), bridge_ready);
    if (!transmit_frame(protocol::kHostHeartbeatCanId, payload)) {
      return false;
    }
    last_transmitted_heartbeat_sequence_ = heartbeat_sequence_;
    ++heartbeat_sequence_;
    heartbeat_sent_for_session_ = bridge_ready;
    return true;
  }

  bool transmit_authority(const bool armed) {
    const auto payload = protocol::encode_motion_authority(
        authority_sequence_, session_id_, armed, host_uptime_ms());
    builtin_interfaces::msg::Time *stamp =
        armed ? nullptr : &last_disarm_tx_stamp_;
    if (!transmit_frame(protocol::kMotionAuthorityCanId, payload, stamp)) {
      return false;
    }
    last_transmitted_authority_sequence_ = authority_sequence_;
    ++authority_sequence_;
    if (!armed) {
      disarmed_sent_for_session_ = true;
    }
    return true;
  }

  bool transmit_disabled_command() {
    protocol::CommandPayload payload{};
    const auto result = protocol::encode_motion_command(
        command_sequence_, session_id_, false, 0.0, 0.0, payload);
    if (!result) {
      return false;
    }
    if (!transmit_frame(protocol::kMotionCommandCanId, payload,
                        &last_zero_command_tx_stamp_)) {
      return false;
    }
    last_transmitted_command_sequence_ = command_sequence_;
    command_sent_for_session_ = true;
    ++command_sequence_;
    return true;
  }

  bool transmit_body_command(const double linear, const double angular) {
    protocol::CommandPayload payload{};
    const auto result = protocol::encode_motion_command(
        command_sequence_, session_id_, true, linear, angular, payload);
    if (!result) {
      latch_protocol_fault(
          "local command serialization failed: " + result.reason, false);
      return false;
    }
    if (!transmit_frame(protocol::kMotionCommandCanId, payload)) {
      return false;
    }
    last_transmitted_command_sequence_ = command_sequence_;
    command_sent_for_session_ = true;
    ++command_sequence_;
    return true;
  }

  void request_safe_state(const std::string &reason, const bool log_error) {
    const bool was_requested = arm_requested_;
    arm_requested_ = false;
    have_cmd_vel_ = false;
    last_linear_velocity_ = 0.0;
    last_angular_velocity_ = 0.0;
    const bool safe_refresh_due = age_ms(last_safe_transmission_) >= 100U;
    if (socket_fd_ >= 0 && (was_requested || safe_refresh_due)) {
      if (was_requested) {
        transmit_disabled_command();
      }
      transmit_authority(false);
      last_safe_transmission_ = std::chrono::steady_clock::now();
    }
    if (was_requested || log_error) {
      if (log_error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Motion made safe: %s", reason.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "Motion disarmed: %s", reason.c_str());
      }
    }
  }

  void latch_protocol_fault(const std::string &reason,
                            const bool version_mismatch) {
    ++protocol_reject_count_;
    if (version_mismatch) {
      ++version_mismatch_count_;
    }
    const bool first_fault = !protocol_fault_latched_;
    protocol_valid_ = false;
    protocol_fault_latched_ = true;
    last_protocol_error_ = reason;
    if (first_fault) {
      request_safe_state(reason, true);
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr message) {
    const std::array<double, 6> components{
        message->linear.x,  message->linear.y,  message->linear.z,
        message->angular.x, message->angular.y, message->angular.z};
    if (!std::all_of(components.begin(), components.end(),
                     [](const double value) { return std::isfinite(value); })) {
      request_safe_state("non-finite Twist rejected", true);
      return;
    }
    if (std::abs(message->linear.y) > unsupported_twist_epsilon_ ||
        std::abs(message->linear.z) > unsupported_twist_epsilon_ ||
        std::abs(message->angular.x) > unsupported_twist_epsilon_ ||
        std::abs(message->angular.y) > unsupported_twist_epsilon_) {
      request_safe_state("unsupported Twist component rejected", true);
      return;
    }

    protocol::CommandPayload validation_payload{};
    const auto result = protocol::encode_motion_command(
        command_sequence_, session_id_, true, message->linear.x,
        message->angular.z, validation_payload);
    if (!result) {
      request_safe_state(result.reason, true);
      return;
    }
    last_linear_velocity_ = message->linear.x;
    last_angular_velocity_ = message->angular.z;
    last_cmd_vel_received_ = std::chrono::steady_clock::now();
    have_cmd_vel_ = true;
  }

  void on_arm(const std_srvs::srv::Trigger::Request::SharedPtr,
              std_srvs::srv::Trigger::Response::SharedPtr response) {
    std::string reason;
    if (!safe_to_arm(reason)) {
      response->success = false;
      response->message = "arm rejected: " + reason;
      return;
    }
    if (!heartbeat_sent_for_session_ || !disarmed_sent_for_session_) {
      response->success = false;
      response->message =
          "arm rejected: startup heartbeat/DISARMED transaction not sent yet";
      return;
    }
    arm_requested_ = true;
    arm_request_started_ = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "arm request accepted; waiting for STM32 DISARMED "
                        "handshake acknowledgement";
    RCLCPP_WARN(get_logger(), "Explicit motion arm requested for session %s",
                hex_string(session_id_, 8U).c_str());
  }

  void on_disarm(const std_srvs::srv::Trigger::Request::SharedPtr,
                 std_srvs::srv::Trigger::Response::SharedPtr response) {
    request_safe_state("explicit disarm service", false);
    response->success = socket_fd_ >= 0 && !transport_fault_active_;
    response->message = response->success
                            ? "DISARMED authority transmitted"
                            : "safe state requested; transport unavailable, "
                              "STM32 timeouts remain authoritative";
  }

  void send_heartbeat() {
    if (socket_fd_ < 0) {
      return;
    }
    transmit_heartbeat(!protocol_fault_latched_);
  }

  bool handshake_acknowledged() const {
    return status_fresh() && system_status_.active_session_id == session_id_ &&
           (system_status_.communication_flags & kCommunicationSessionActive) !=
               0U &&
           (system_status_.motion_flags & kMotionDisarmedHandshakeSeen) != 0U;
  }

  void send_authority() {
    if (socket_fd_ < 0 || !heartbeat_sent_for_session_) {
      return;
    }
    bool send_armed = false;
    if (arm_requested_) {
      std::string reason;
      if (!safe_to_arm(reason)) {
        request_safe_state("arm precondition lost: " + reason, true);
        return;
      }
      send_armed = handshake_acknowledged();
    }
    transmit_authority(send_armed);
  }

  void send_command() {
    if (socket_fd_ < 0 || !arm_requested_) {
      return;
    }
    if (!status_fresh() ||
        (system_status_.motion_flags & kMotionAuthorityArmed) == 0U ||
        system_status_.active_session_id != session_id_) {
      return;
    }
    if (!local_command_fresh()) {
      request_safe_state("local /cmd_vel timeout", true);
      return;
    }
    transmit_body_command(last_linear_velocity_, last_angular_velocity_);
  }

  void receive_frames() {
    if (socket_fd_ < 0) {
      return;
    }
    for (std::size_t frame_index = 0; frame_index < 256U; ++frame_index) {
      canfd_frame frame{};
      const ssize_t received =
          ::recv(socket_fd_, &frame, sizeof(frame), MSG_DONTWAIT);
      if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          return;
        }
        ++rx_socket_failure_count_;
        mark_transport_failure(std::string("receive: ") + std::strerror(errno));
        return;
      }
      if (received == 0) {
        mark_transport_failure("receive returned EOF");
        return;
      }
      ++rx_frame_count_;
      last_any_frame_received_ = std::chrono::steady_clock::now();

      if ((frame.can_id & CAN_ERR_FLAG) != 0U) {
        ++rx_error_frame_count_;
        handle_can_error(frame.can_id, frame.data,
                         static_cast<std::size_t>(received));
        continue;
      }

      const uint32_t can_id = frame.can_id & CAN_SFF_MASK;
      const bool protocol_id = can_id >= protocol::kSystemStatusCanId &&
                               can_id <= protocol::kBatteryStateCanId;
      if (!protocol_id) {
        continue;
      }
      if (received != CANFD_MTU ||
          (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG)) != 0U ||
          (frame.flags & CANFD_BRS) == 0U) {
        latch_protocol_fault(
            "Protocol ID received without Standard-ID CAN FD+BRS envelope",
            false);
        continue;
      }
      dispatch_protocol_frame(can_id, frame.data, frame.len);
    }
  }

  void handle_can_error(const canid_t can_id, const uint8_t *data,
                        const std::size_t received_size) {
    std::ostringstream reason;
    reason << "SocketCAN error frame " << hex_string(can_id, 8U) << " size "
           << received_size;
    const bool controller_active =
        (can_id & CAN_ERR_CRTL) != 0U && (data[1] & CAN_ERR_CRTL_ACTIVE) != 0U;
    const bool restarted = (can_id & CAN_ERR_RESTARTED) != 0U;
    if (controller_active || restarted) {
      can_error_observed_ = false;
    } else {
      can_error_observed_ = true;
    }
    if ((can_id & CAN_ERR_BUSOFF) != 0U) {
      bus_off_observed_ = true;
    } else if (restarted) {
      bus_off_observed_ = false;
    }
    request_safe_state(reason.str(), true);
    if (restarted) {
      start_new_session("SocketCAN controller restarted after bus-off");
    }
  }

  bool sequence_is_new(const uint32_t can_id, const uint16_t sequence) {
    auto &state = sequence_states_[can_id];
    if (!state.seen) {
      state.seen = true;
      state.last = sequence;
      return true;
    }
    const uint16_t delta = static_cast<uint16_t>(sequence - state.last);
    if (delta == 0U || delta >= 0x8000U) {
      ++sequence_discontinuity_count_;
      return false;
    }
    if (delta != 1U) {
      ++sequence_discontinuity_count_;
    }
    state.last = sequence;
    return true;
  }

  static bool sequence_ack_is_plausible(const uint16_t last_transmitted,
                                        const uint16_t acknowledged) {
    const uint16_t age = static_cast<uint16_t>(last_transmitted - acknowledged);
    return age < 0x8000U;
  }

  void dispatch_protocol_frame(const uint32_t can_id, const uint8_t *data,
                               const std::size_t length) {
    if (length >= 2U) {
      received_protocol_major_ = data[0];
      received_protocol_minor_ = data[1];
    }
    switch (can_id) {
    case protocol::kSystemStatusCanId:
      process_system_status(data, length);
      break;
    case protocol::kWheelStateCanId:
      process_wheel_state(data, length);
      break;
    case protocol::kImuDataCanId:
      process_imu_data(data, length);
      break;
    case protocol::kBatteryStateCanId:
      process_battery_state(data, length);
      break;
    default:
      break;
    }
  }

  bool process_decode_result(const protocol::DecodeResult &result) {
    if (result) {
      return true;
    }
    latch_protocol_fault(
        result.reason, result.error == protocol::DecodeError::kVersionMismatch);
    return false;
  }

  void process_system_status(const uint8_t *data, const std::size_t length) {
    protocol::SystemStatus decoded;
    const auto result = protocol::decode_system_status(data, length, decoded);
    if (!process_decode_result(result)) {
      return;
    }

    if (have_stm32_timestamp_ &&
        decoded.timestamp_ms < last_stm32_timestamp_ms_ &&
        (last_stm32_timestamp_ms_ - decoded.timestamp_ms) < 0x80000000UL) {
      request_safe_state("STM32 monotonic timestamp reset detected", true);
      start_new_session("STM32 restart detected");
      sequence_states_.clear();
    }
    last_stm32_timestamp_ms_ = decoded.timestamp_ms;
    have_stm32_timestamp_ = true;

    if (!sequence_is_new(protocol::kSystemStatusCanId, decoded.sequence)) {
      return;
    }
    if (session_acknowledged_ && decoded.active_session_id != session_id_) {
      request_safe_state("STM32 session replacement detected", true);
      start_new_session("STM32 session replacement");
    }
    const uint16_t safety_latches =
        kCommunicationRxSoftwareOverflow | kCommunicationRxHardwareLoss;
    const uint16_t new_safety_latches =
        static_cast<uint16_t>(decoded.communication_flags & safety_latches &
                              ~observed_stm32_communication_latches_);
    observed_stm32_communication_latches_ =
        static_cast<uint16_t>(observed_stm32_communication_latches_ |
                              (decoded.communication_flags & safety_latches));
    if (new_safety_latches != 0U) {
      request_safe_state("new STM32 CAN RX loss/overflow latch observed", true);
      start_new_session("STM32 CAN RX integrity event");
    }
    if ((decoded.communication_flags & kCommunicationVersionMismatch) != 0U &&
        !stm32_version_mismatch_latch_handled_) {
      stm32_version_mismatch_latch_handled_ = true;
      latch_protocol_fault(
          "STM32 reports a latched Protocol 1.0 version mismatch", true);
    }

    heartbeat_ack_consistent_ = false;
    command_ack_consistent_ = false;
    const bool current_session = decoded.active_session_id == session_id_;
    const bool heartbeat_ack_seen = (decoded.communication_flags &
                                     kCommunicationHeartbeatSequenceSeen) != 0U;
    const bool command_ack_seen =
        (decoded.communication_flags & kCommunicationCommandSequenceSeen) != 0U;
    if (current_session && heartbeat_ack_seen && heartbeat_sent_for_session_) {
      heartbeat_ack_consistent_ =
          sequence_ack_is_plausible(last_transmitted_heartbeat_sequence_,
                                    decoded.last_heartbeat_sequence);
    }
    if (current_session && command_ack_seen && command_sent_for_session_) {
      command_ack_consistent_ = sequence_ack_is_plausible(
          last_transmitted_command_sequence_, decoded.last_command_sequence);
    }
    const bool impossible_heartbeat_ack =
        current_session && heartbeat_ack_seen && heartbeat_sent_for_session_ &&
        !heartbeat_ack_consistent_;
    const bool impossible_command_ack = current_session && command_ack_seen &&
                                        command_sent_for_session_ &&
                                        !command_ack_consistent_;
    if ((impossible_heartbeat_ack || impossible_command_ack) &&
        !sequence_ack_fault_latched_) {
      sequence_ack_fault_latched_ = true;
      latch_protocol_fault("STM32 acknowledgement sequence is ahead of host TX",
                           false);
    }

    system_status_ = decoded;
    have_system_status_ = true;
    last_system_status_received_ = std::chrono::steady_clock::now();
    protocol_valid_ = !protocol_fault_latched_;
    if (decoded.active_session_id == session_id_ &&
        (decoded.communication_flags & kCommunicationSessionActive) != 0U) {
      session_acknowledged_ = true;
    }
  }

  void process_wheel_state(const uint8_t *data, const std::size_t length) {
    protocol::WheelState decoded;
    const auto result = protocol::decode_wheel_state(data, length, decoded);
    if (!process_decode_result(result) ||
        !sequence_is_new(protocol::kWheelStateCanId, decoded.sequence)) {
      return;
    }

    const auto invalid_i64 = std::numeric_limits<int64_t>::min();
    const auto invalid_i32 = std::numeric_limits<int32_t>::min();
    const auto invalid_i16 = std::numeric_limits<int16_t>::min();
    msg::WheelState message;
    message.header.stamp = now();
    message.header.frame_id = "base_link";
    message.sequence = decoded.sequence;
    message.stm32_timestamp_ms = decoded.timestamp_ms;
    message.logical_mapping_valid = (decoded.flags & 1U) != 0U;
    message.left_position_valid = message.logical_mapping_valid &&
                                  decoded.left_position_counts != invalid_i64;
    message.right_position_valid = message.logical_mapping_valid &&
                                   decoded.right_position_counts != invalid_i64;
    message.left_measured_valid = message.logical_mapping_valid &&
                                  decoded.left_measured_cps != invalid_i32;
    message.right_measured_valid = message.logical_mapping_valid &&
                                   decoded.right_measured_cps != invalid_i32;
    message.left_target_valid = (decoded.flags & (1U << 10U)) != 0U &&
                                decoded.left_target_cps != invalid_i32;
    message.right_target_valid = (decoded.flags & (1U << 11U)) != 0U &&
                                 decoded.right_target_cps != invalid_i32;
    message.left_controller_output_valid =
        (decoded.flags & (1U << 3U)) != 0U &&
        decoded.left_controller_output != invalid_i16;
    message.right_controller_output_valid =
        (decoded.flags & (1U << 4U)) != 0U &&
        decoded.right_controller_output != invalid_i16;
    message.left_position_counts =
        message.left_position_valid ? decoded.left_position_counts : 0;
    message.right_position_counts =
        message.right_position_valid ? decoded.right_position_counts : 0;
    message.left_measured_cps =
        message.left_measured_valid ? decoded.left_measured_cps : 0;
    message.right_measured_cps =
        message.right_measured_valid ? decoded.right_measured_cps : 0;
    message.left_target_cps =
        message.left_target_valid ? decoded.left_target_cps : 0;
    message.right_target_cps =
        message.right_target_valid ? decoded.right_target_cps : 0;
    message.left_controller_output =
        message.left_controller_output_valid
            ? static_cast<float>(decoded.left_controller_output) * 0.0001F
            : 0.0F;
    message.right_controller_output =
        message.right_controller_output_valid
            ? static_cast<float>(decoded.right_controller_output) * 0.0001F
            : 0.0F;
    message.encoder_1_raw_counter = decoded.encoder_1_raw_counter;
    message.encoder_2_raw_counter = decoded.encoder_2_raw_counter;
    message.encoder_1_raw_cps_valid = (decoded.flags & (1U << 1U)) != 0U &&
                                      decoded.encoder_1_raw_cps != invalid_i32;
    message.encoder_2_raw_cps_valid = (decoded.flags & (1U << 2U)) != 0U &&
                                      decoded.encoder_2_raw_cps != invalid_i32;
    message.encoder_1_raw_cps =
        message.encoder_1_raw_cps_valid ? decoded.encoder_1_raw_cps : 0;
    message.encoder_2_raw_cps =
        message.encoder_2_raw_cps_valid ? decoded.encoder_2_raw_cps : 0;
    message.flags = decoded.flags;
    message.encoder_1_age_ms = decoded.encoder_1_age_ms;
    message.encoder_2_age_ms = decoded.encoder_2_age_ms;
    message.sample_period_ms = decoded.sample_period_ms;
    wheel_publisher_->publish(message);
    last_wheel_received_ = std::chrono::steady_clock::now();
  }

  rclcpp::Time measurement_stamp(const uint16_t sample_age_ms) {
    const auto receipt = now();
    if (sample_age_ms == std::numeric_limits<uint16_t>::max()) {
      return receipt;
    }
    return receipt - rclcpp::Duration::from_seconds(
                         static_cast<double>(sample_age_ms) / 1000.0);
  }

  void process_imu_data(const uint8_t *data, const std::size_t length) {
    protocol::ImuData decoded;
    const auto result = protocol::decode_imu_data(data, length, decoded);
    if (!process_decode_result(result) ||
        !sequence_is_new(protocol::kImuDataCanId, decoded.sequence)) {
      return;
    }

    sensor_msgs::msg::Imu message;
    message.header.stamp = measurement_stamp(decoded.sample_age_ms);
    message.header.frame_id = imu_frame_id_;
    message.orientation_covariance[0] = -1.0;
    const bool acceleration_valid = (decoded.flags & (1U << 5U)) != 0U;
    const bool gyro_valid = (decoded.flags & (1U << 6U)) != 0U;
    if (acceleration_valid) {
      message.linear_acceleration.x =
          static_cast<double>(decoded.accel_x) * 0.0001;
      message.linear_acceleration.y =
          static_cast<double>(decoded.accel_y) * 0.0001;
      message.linear_acceleration.z =
          static_cast<double>(decoded.accel_z) * 0.0001;
    } else {
      message.linear_acceleration_covariance[0] = -1.0;
    }
    if (gyro_valid) {
      message.angular_velocity.x =
          static_cast<double>(decoded.gyro_x) * 0.00001;
      message.angular_velocity.y =
          static_cast<double>(decoded.gyro_y) * 0.00001;
      message.angular_velocity.z =
          static_cast<double>(decoded.gyro_z) * 0.00001;
    } else {
      message.angular_velocity_covariance[0] = -1.0;
    }
    imu_publisher_->publish(message);
    last_imu_received_ = std::chrono::steady_clock::now();
  }

  void process_battery_state(const uint8_t *data, const std::size_t length) {
    protocol::BatteryState decoded;
    const auto result = protocol::decode_battery_state(data, length, decoded);
    if (!process_decode_result(result) ||
        !sequence_is_new(protocol::kBatteryStateCanId, decoded.sequence)) {
      return;
    }

    sensor_msgs::msg::BatteryState message;
    message.header.stamp = measurement_stamp(decoded.sample_age_ms);
    message.header.frame_id = "base_link";
    const float nan = std::numeric_limits<float>::quiet_NaN();
    message.voltage =
        (decoded.flags & 1U) != 0U
            ? static_cast<float>(decoded.battery_voltage_mv) * 0.001F
            : nan;
    message.temperature = nan;
    message.current = nan;
    message.charge = nan;
    message.capacity = nan;
    message.design_capacity = nan;
    message.percentage = nan;
    message.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    message.power_supply_health =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    message.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    message.present = (decoded.flags & 1U) != 0U;
    battery_publisher_->publish(message);
    last_battery_received_ = std::chrono::steady_clock::now();
    last_battery_raw_adc_ = decoded.raw_adc;
    last_battery_flags_ = decoded.flags;
  }

  void health_tick() {
    if (transport_fault_active_) {
      close_socket();
      start_new_session("transport failure");
      if (reconnect_inhibited_) {
        next_reconnect_ = std::chrono::steady_clock::time_point::max();
        RCLCPP_ERROR(get_logger(),
                     "SocketCAN automatic reconnect inhibited after runtime "
                     "transport failure; repair can3 and restart the bridge");
      } else {
        schedule_reconnect();
      }
      transport_fault_active_ = false;
      return;
    }
    if (socket_fd_ < 0 &&
        (next_reconnect_ == std::chrono::steady_clock::time_point{} ||
         std::chrono::steady_clock::now() >= next_reconnect_)) {
      open_socket();
      return;
    }
    if (have_system_status_ && !status_fresh() && !status_timeout_latched_) {
      status_timeout_latched_ = true;
      request_safe_state("STM32 SYSTEM_STATUS timeout", true);
      start_new_session("status timeout");
    } else if (status_fresh()) {
      status_timeout_latched_ = false;
    }
    if (arm_requested_ &&
        arm_request_started_ != std::chrono::steady_clock::time_point{} &&
        age_ms(arm_request_started_) > 1500U &&
        (system_status_.motion_flags & kMotionAuthorityArmed) == 0U) {
      request_safe_state("STM32 arm acknowledgement timeout", true);
    }
  }

  void publish_status() {
    msg::BridgeStatus message;
    message.header.stamp = now();
    message.can_interface = can_interface_;
    message.socket_open = socket_fd_ >= 0;
    message.connected = connected();
    message.protocol_valid = protocol_valid_;
    message.can_healthy = can_healthy();
    message.can_error_seen = can_error_observed_;
    message.can_bus_off = bus_off_observed_;
    message.status_fresh = status_fresh();
    message.last_transport_error = last_transport_error_;
    message.last_protocol_error = last_protocol_error_;
    message.session_id = session_id_;
    message.arm_requested = arm_requested_;
    message.authority_armed =
        status_fresh() && system_status_.active_session_id == session_id_ &&
        (system_status_.motion_flags & kMotionAuthorityArmed) != 0U;
    message.command_fresh = local_command_fresh();
    message.last_stm32_frame_age_ms = age_ms(last_any_frame_received_);
    message.last_system_status_age_ms = age_ms(last_system_status_received_);
    message.last_wheel_state_age_ms = age_ms(last_wheel_received_);
    message.last_imu_data_age_ms = age_ms(last_imu_received_);
    message.last_battery_state_age_ms = age_ms(last_battery_received_);
    message.rx_frame_count = rx_frame_count_;
    message.tx_frame_count = tx_frame_count_;
    message.rx_error_frame_count = rx_error_frame_count_;
    message.rx_socket_failure_count = rx_socket_failure_count_;
    message.tx_failure_count = tx_failure_count_;
    message.protocol_reject_count = protocol_reject_count_;
    message.version_mismatch_count = version_mismatch_count_;
    message.sequence_discontinuity_count = sequence_discontinuity_count_;
    message.received_protocol_major = received_protocol_major_;
    message.received_protocol_minor = received_protocol_minor_;
    if (have_system_status_) {
      message.firmware_major = system_status_.firmware_major;
      message.firmware_minor = system_status_.firmware_minor;
      message.firmware_patch = system_status_.firmware_patch;
      message.system_state = system_status_.system_state;
      message.motion_flags = system_status_.motion_flags;
      message.communication_flags = system_status_.communication_flags;
      message.fault_flags = system_status_.fault_flags;
      message.reset_reason = system_status_.reset_reason;
      message.active_session_id = system_status_.active_session_id;
      message.last_command_sequence = system_status_.last_command_sequence;
      message.last_heartbeat_sequence = system_status_.last_heartbeat_sequence;
    }
    message.last_tx_command_sequence = last_transmitted_command_sequence_;
    message.last_tx_heartbeat_sequence = last_transmitted_heartbeat_sequence_;
    message.last_tx_authority_sequence = last_transmitted_authority_sequence_;
    message.command_ack_consistent = command_ack_consistent_;
    message.heartbeat_ack_consistent = heartbeat_ack_consistent_;
    message.stm32_critical_fault = stm32_critical_fault();
    message.last_zero_command_tx_stamp = last_zero_command_tx_stamp_;
    message.last_disarm_tx_stamp = last_disarm_tx_stamp_;
    status_publisher_->publish(message);
  }

  void publish_diagnostics() {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "robot_stm32_bridge/Protocol_1_0";
    status.hardware_id = can_interface_;
    const bool telemetry_streams_fresh =
        age_ms(last_wheel_received_) <= 200U &&
        age_ms(last_imu_received_) <= 100U &&
        age_ms(last_battery_received_) <= 1500U;
    if (connected() && !stm32_critical_fault() && telemetry_streams_fresh) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = arm_requested_ ? "connected; motion explicitly requested"
                                      : "connected; DISARMED";
    } else if (socket_fd_ >= 0 && status_fresh() && protocol_valid_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "connected with degraded/fault state";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message =
          last_protocol_error_.empty()
              ? (last_transport_error_.empty() ? "STM32 status unavailable"
                                               : last_transport_error_)
              : last_protocol_error_;
    }

    status.values.push_back(key_value("can_interface", can_interface_));
    status.values.push_back(
        key_value("socket_open", socket_fd_ >= 0 ? "true" : "false"));
    status.values.push_back(key_value(
        "can_health", can_healthy() ? "Error Active" : "not healthy"));
    status.values.push_back(
        key_value("can_error_seen", can_error_observed_ ? "true" : "false"));
    status.values.push_back(
        key_value("can_bus_off", bus_off_observed_ ? "true" : "false"));
    status.values.push_back(key_value("protocol_version", "1.0 exact"));
    status.values.push_back(
        key_value("received_protocol_version",
                  decimal_string(received_protocol_major_) + "." +
                      decimal_string(received_protocol_minor_)));
    status.values.push_back(
        key_value("session_id", hex_string(session_id_, 8U)));
    status.values.push_back(
        key_value("arm_requested", arm_requested_ ? "true" : "false"));
    status.values.push_back(
        key_value("command_fresh", local_command_fresh() ? "true" : "false"));
    status.values.push_back(
        key_value("last_stm32_frame_age_ms",
                  decimal_string(age_ms(last_any_frame_received_))));
    status.values.push_back(
        key_value("system_status_age_ms",
                  decimal_string(age_ms(last_system_status_received_))));
    status.values.push_back(key_value(
        "wheel_state_age_ms", decimal_string(age_ms(last_wheel_received_))));
    status.values.push_back(key_value(
        "imu_data_age_ms", decimal_string(age_ms(last_imu_received_))));
    status.values.push_back(
        key_value("battery_state_age_ms",
                  decimal_string(age_ms(last_battery_received_))));
    status.values.push_back(
        key_value("rx_frames", decimal_string(rx_frame_count_)));
    status.values.push_back(
        key_value("tx_frames", decimal_string(tx_frame_count_)));
    status.values.push_back(
        key_value("rx_error_frames", decimal_string(rx_error_frame_count_)));
    status.values.push_back(key_value(
        "rx_socket_failures", decimal_string(rx_socket_failure_count_)));
    status.values.push_back(
        key_value("tx_failures", decimal_string(tx_failure_count_)));
    status.values.push_back(
        key_value("protocol_rejects", decimal_string(protocol_reject_count_)));
    status.values.push_back(key_value("version_mismatches",
                                      decimal_string(version_mismatch_count_)));
    status.values.push_back(
        key_value("sequence_discontinuities",
                  decimal_string(sequence_discontinuity_count_)));
    if (have_system_status_) {
      status.values.push_back(
          key_value("firmware_version",
                    decimal_string(system_status_.firmware_major) + "." +
                        decimal_string(system_status_.firmware_minor) + "." +
                        decimal_string(system_status_.firmware_patch)));
      status.values.push_back(key_value(
          "stm32_state", system_state_name(system_status_.system_state)));
      status.values.push_back(key_value(
          "motion_flags", hex_string(system_status_.motion_flags, 4U)));
      status.values.push_back(
          key_value("communication_flags",
                    hex_string(system_status_.communication_flags, 4U)));
      status.values.push_back(
          key_value("fault_flags", hex_string(system_status_.fault_flags, 8U)));
      status.values.push_back(
          key_value("fault_names", fault_names(system_status_.fault_flags)));
      status.values.push_back(key_value(
          "reset_reason", hex_string(system_status_.reset_reason, 8U)));
      status.values.push_back(
          key_value("active_session_id",
                    hex_string(system_status_.active_session_id, 8U)));
      status.values.push_back(
          key_value("heartbeat_fresh",
                    (system_status_.motion_flags & kMotionHeartbeatFresh) != 0U
                        ? "true"
                        : "false"));
      status.values.push_back(
          key_value("authority_armed",
                    (system_status_.motion_flags & kMotionAuthorityArmed) != 0U
                        ? "true"
                        : "false"));
      status.values.push_back(key_value(
          "stm32_command_fresh",
          (system_status_.motion_flags & kMotionCommandFresh) != 0U ? "true"
                                                                    : "false"));
      status.values.push_back(
          key_value("motor_authorized",
                    (system_status_.motion_flags & kMotionMotorAuthorized) != 0U
                        ? "true"
                        : "false"));
      status.values.push_back(key_value(
          "motor_stby_enabled",
          (system_status_.motion_flags & kMotionStbyEnabled) != 0U ? "true"
                                                                   : "false"));
      status.values.push_back(
          key_value("last_command_ack",
                    decimal_string(system_status_.last_command_sequence)));
      status.values.push_back(
          key_value("last_heartbeat_ack",
                    decimal_string(system_status_.last_heartbeat_sequence)));
      status.values.push_back(
          key_value("last_command_tx",
                    decimal_string(last_transmitted_command_sequence_)));
      status.values.push_back(
          key_value("last_heartbeat_tx",
                    decimal_string(last_transmitted_heartbeat_sequence_)));
      status.values.push_back(
          key_value("last_authority_tx",
                    decimal_string(last_transmitted_authority_sequence_)));
      status.values.push_back(
          key_value("command_ack_consistent",
                    command_ack_consistent_ ? "true" : "false"));
      status.values.push_back(
          key_value("heartbeat_ack_consistent",
                    heartbeat_ack_consistent_ ? "true" : "false"));
      status.values.push_back(
          key_value("battery_raw_adc", decimal_string(last_battery_raw_adc_)));
      status.values.push_back(
          key_value("battery_flags", hex_string(last_battery_flags_, 4U)));
    }
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  std::string can_interface_;
  std::string cmd_vel_topic_;
  std::string imu_frame_id_;
  int command_timeout_ms_{100};
  int status_timeout_ms_{350};
  int reconnect_period_ms_{1000};
  double unsupported_twist_epsilon_{1.0e-9};

  int socket_fd_{-1};
  const std::chrono::steady_clock::time_point process_started_;
  std::chrono::steady_clock::time_point next_reconnect_{};
  std::string last_transport_error_;
  std::string last_protocol_error_;
  bool transport_fault_active_{false};
  bool reconnect_inhibited_{false};
  bool protocol_fault_latched_{false};
  bool protocol_valid_{false};
  bool can_error_observed_{false};
  bool bus_off_observed_{false};
  bool status_timeout_latched_{false};

  uint32_t session_id_{0U};
  uint16_t heartbeat_sequence_{0U};
  uint16_t authority_sequence_{0U};
  uint16_t command_sequence_{0U};
  uint16_t last_transmitted_command_sequence_{0U};
  uint16_t last_transmitted_heartbeat_sequence_{0U};
  uint16_t last_transmitted_authority_sequence_{0U};
  bool heartbeat_sent_for_session_{false};
  bool command_sent_for_session_{false};
  bool command_ack_consistent_{false};
  bool heartbeat_ack_consistent_{false};
  bool sequence_ack_fault_latched_{false};
  bool disarmed_sent_for_session_{false};
  bool session_acknowledged_{false};
  bool arm_requested_{false};
  std::chrono::steady_clock::time_point arm_request_started_{};

  bool have_cmd_vel_{false};
  double last_linear_velocity_{0.0};
  double last_angular_velocity_{0.0};
  std::chrono::steady_clock::time_point last_cmd_vel_received_{};
  std::chrono::steady_clock::time_point last_safe_transmission_{};

  bool have_system_status_{false};
  bool have_stm32_timestamp_{false};
  uint32_t last_stm32_timestamp_ms_{0U};
  protocol::SystemStatus system_status_{};
  uint8_t received_protocol_major_{0U};
  uint8_t received_protocol_minor_{0U};
  std::chrono::steady_clock::time_point last_any_frame_received_{};
  std::chrono::steady_clock::time_point last_system_status_received_{};
  std::chrono::steady_clock::time_point last_wheel_received_{};
  std::chrono::steady_clock::time_point last_imu_received_{};
  std::chrono::steady_clock::time_point last_battery_received_{};
  std::unordered_map<uint32_t, SequenceState> sequence_states_;

  uint64_t rx_frame_count_{0U};
  uint64_t tx_frame_count_{0U};
  uint64_t rx_error_frame_count_{0U};
  uint64_t rx_socket_failure_count_{0U};
  uint64_t tx_failure_count_{0U};
  uint64_t protocol_reject_count_{0U};
  uint64_t version_mismatch_count_{0U};
  uint64_t sequence_discontinuity_count_{0U};
  uint16_t last_battery_raw_adc_{0U};
  uint16_t last_battery_flags_{0U};
  uint16_t observed_stm32_communication_latches_{0U};
  bool stm32_version_mismatch_latch_handled_{false};
  builtin_interfaces::msg::Time last_zero_command_tx_stamp_{};
  builtin_interfaces::msg::Time last_disarm_tx_stamp_{};

  rclcpp::Publisher<msg::BridgeStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<msg::WheelState>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr
      battery_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      cmd_vel_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr authority_timer_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

} // namespace robot_stm32_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<robot_stm32_bridge::BridgeNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_stm32_bridge"), "%s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
