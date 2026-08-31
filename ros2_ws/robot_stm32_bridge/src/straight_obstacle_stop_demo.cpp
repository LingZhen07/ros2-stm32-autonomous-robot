#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_stm32_bridge/msg/bridge_status.hpp"
#include "robot_stm32_bridge/msg/demo_status.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_stm32_bridge {
namespace {

using namespace std::chrono_literals;

constexpr uint16_t kMotionMotorAuthorized = 1U << 7U;
constexpr uint16_t kMotionStbyEnabled = 1U << 8U;
constexpr uint16_t kMotionBodyCommandReady = 1U << 15U;
constexpr double kPi = 3.14159265358979323846;

int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool stamp_is_zero(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec == 0 && stamp.nanosec == 0U;
}

double elapsed_ms(const builtin_interfaces::msg::Time &start,
                  const builtin_interfaces::msg::Time &end) {
  if (stamp_is_zero(start) || stamp_is_zero(end)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return static_cast<double>(stamp_nanoseconds(end) -
                             stamp_nanoseconds(start)) /
         1.0e6;
}

double angular_difference(const double first, const double second) {
  return std::atan2(std::sin(first - second), std::cos(first - second));
}

} // namespace

class StraightObstacleStopDemo final : public rclcpp::Node {
public:
  StraightObstacleStopDemo() : Node("straight_obstacle_stop_demo") {
    forward_speed_mps_ = declare_parameter<double>("forward_speed_mps", 0.30);
    stop_distance_m_ = declare_parameter<double>("stop_distance_m", 0.60);
    front_sector_deg_ = declare_parameter<double>("front_sector_deg", 30.0);
    front_center_deg_ = declare_parameter<double>("front_center_deg", 180.0);
    scan_timeout_ms_ = declare_parameter<int>("scan_timeout_ms", 400);
    bridge_timeout_ms_ = declare_parameter<int>("bridge_timeout_ms", 300);
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    cmd_vel_topic_ =
        declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    bridge_status_topic_ =
        declare_parameter<std::string>("bridge_status_topic", "/stm32/status");
    arm_service_name_ = declare_parameter<std::string>(
        "arm_service", "/robot_stm32_bridge/arm");
    disarm_service_name_ = declare_parameter<std::string>(
        "disarm_service", "/robot_stm32_bridge/disarm");

    if (!std::isfinite(forward_speed_mps_) || forward_speed_mps_ <= 0.0 ||
        forward_speed_mps_ > 0.30 || !std::isfinite(stop_distance_m_) ||
        stop_distance_m_ <= 0.0 || !std::isfinite(front_sector_deg_) ||
        front_sector_deg_ <= 0.0 || front_sector_deg_ > 360.0 ||
        !std::isfinite(front_center_deg_) || scan_timeout_ms_ < 100 ||
        bridge_timeout_ms_ < 100) {
      throw std::invalid_argument("invalid demo parameters: use 0 < "
                                  "forward_speed_mps <= 0.30, positive "
                                  "stop_distance_m, 0 < front_sector_deg <= "
                                  "360, and timeouts >= 100 ms");
    }

    command_publisher_ =
        create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    status_publisher_ = create_publisher<msg::DemoStatus>("~/status", 10);
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&StraightObstacleStopDemo::on_scan, this,
                  std::placeholders::_1));
    bridge_subscription_ = create_subscription<msg::BridgeStatus>(
        bridge_status_topic_, rclcpp::QoS(10),
        std::bind(&StraightObstacleStopDemo::on_bridge_status, this,
                  std::placeholders::_1));

    arm_client_ = create_client<std_srvs::srv::Trigger>(arm_service_name_);
    disarm_client_ =
        create_client<std_srvs::srv::Trigger>(disarm_service_name_);
    start_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start", std::bind(&StraightObstacleStopDemo::on_start, this,
                             std::placeholders::_1, std::placeholders::_2));
    stop_service_ = create_service<std_srvs::srv::Trigger>(
        "~/stop", std::bind(&StraightObstacleStopDemo::on_stop, this,
                            std::placeholders::_1, std::placeholders::_2));

    control_timer_ = create_wall_timer(
        20ms, std::bind(&StraightObstacleStopDemo::control_tick, this));
    status_timer_ = create_wall_timer(
        100ms, std::bind(&StraightObstacleStopDemo::publish_status, this));

    reason_ = "STOPPED; waiting for explicit ~/start";
    publish_zero_twist(false);
    RCLCPP_INFO(get_logger(),
                "Obstacle-stop demo STOPPED: speed %.3f m/s, stop %.3f m, sector "
                "%.1f deg centered %.1f deg in LaserScan frame",
                forward_speed_mps_, stop_distance_m_, front_sector_deg_,
                front_center_deg_);
  }

  ~StraightObstacleStopDemo() override { publish_zero_twist(false); }

private:
  enum class State : uint8_t {
    kStopped = msg::DemoStatus::STOPPED,
    kRunning = msg::DemoStatus::RUNNING,
  };

  const char *state_name() const {
    switch (state_) {
    case State::kStopped:
      return "STOPPED";
    case State::kRunning:
      return "RUNNING";
    default:
      return "INVALID";
    }
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

  bool scan_fresh() const {
    return have_scan_ && age_ms(last_scan_received_) <=
                             static_cast<uint32_t>(scan_timeout_ms_);
  }

  bool bridge_status_fresh() const {
    return have_bridge_status_ && age_ms(last_bridge_status_received_) <=
                                      static_cast<uint32_t>(bridge_timeout_ms_);
  }

  bool bridge_healthy() const {
    return bridge_status_fresh() && bridge_status_.connected &&
           bridge_status_.protocol_valid && bridge_status_.can_healthy &&
           bridge_status_.status_fresh && !bridge_status_.stm32_critical_fault;
  }

  bool motion_path_ready() const {
    const bool state_allows_motion =
        bridge_status_.system_state == msg::BridgeStatus::SYSTEM_SAFE ||
        bridge_status_.system_state == msg::BridgeStatus::SYSTEM_READY ||
        bridge_status_.system_state == msg::BridgeStatus::SYSTEM_ACTIVE;
    return bridge_healthy() && state_allows_motion &&
           (bridge_status_.motion_flags & kMotionBodyCommandReady) != 0U;
  }

  void transition(const State next, const std::string &reason) {
    if (state_ != next || reason_ != reason) {
      RCLCPP_INFO(get_logger(), "Demo %s -> %s: %s", state_name(),
                  next == State::kRunning ? "RUNNING" : "STOPPED",
                  reason.c_str());
    }
    state_ = next;
    reason_ = reason;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (scan->ranges.empty() || !std::isfinite(scan->angle_min) ||
        !std::isfinite(scan->angle_increment) ||
        scan->angle_increment == 0.0F || !std::isfinite(scan->range_min) ||
        !std::isfinite(scan->range_max) || scan->range_min < 0.0F ||
        scan->range_max <= scan->range_min) {
      latch_stopped("invalid LaserScan metadata");
      return;
    }

    const double center = front_center_deg_ * kPi / 180.0;
    const double half_width = front_sector_deg_ * kPi / 360.0;
    float minimum = std::numeric_limits<float>::infinity();
    std::size_t valid_samples = 0U;
    for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
      const double angle = static_cast<double>(scan->angle_min) +
                           static_cast<double>(index) *
                               static_cast<double>(scan->angle_increment);
      if (std::abs(angular_difference(angle, center)) > half_width) {
        continue;
      }
      const float range = scan->ranges[index];
      if (std::isinf(range) && range > 0.0F) {
        ++valid_samples;
        continue;
      }
      if (!std::isfinite(range) || range < scan->range_min ||
          range > scan->range_max) {
        continue;
      }
      minimum = std::min(minimum, range);
      ++valid_samples;
    }

    last_scan_received_ = std::chrono::steady_clock::now();
    have_scan_ = valid_samples > 0U;
    minimum_front_range_m_ = minimum;
    if (valid_samples == 0U) {
      if (start_pending_ || state_ == State::kRunning) {
        latch_stopped("no valid LaserScan samples in configured front sector");
      }
      return;
    }

    if ((start_pending_ || state_ == State::kRunning) &&
        minimum_front_range_m_ <= static_cast<float>(stop_distance_m_)) {
      obstacle_stop();
    }
  }

  void on_bridge_status(const msg::BridgeStatus::SharedPtr status) {
    bridge_status_ = *status;
    have_bridge_status_ = true;
    last_bridge_status_received_ = std::chrono::steady_clock::now();

    if (state_ == State::kStopped && !stamp_is_zero(obstacle_detected_stamp_)) {
      if (stamp_is_zero(authority_withdrawn_tx_stamp_) &&
          stamp_nanoseconds(status->last_disarm_tx_stamp) >=
              stamp_nanoseconds(obstacle_detected_stamp_)) {
        authority_withdrawn_tx_stamp_ = status->last_disarm_tx_stamp;
      }
      const bool stm32_safe =
          !status->authority_armed &&
          (status->motion_flags & kMotionMotorAuthorized) == 0U &&
          (status->motion_flags & kMotionStbyEnabled) == 0U;
      if (stm32_safe && stamp_is_zero(stm32_stop_confirmed_stamp_)) {
        stm32_stop_confirmed_stamp_ = status->header.stamp;
        RCLCPP_WARN(get_logger(), "STM32 status confirms authority, motor "
                                  "authorization, and STBY are clear");
      }
    }
  }

  void on_start(const std_srvs::srv::Trigger::Request::SharedPtr,
                std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (start_pending_ || state_ == State::kRunning) {
      response->success = false;
      response->message = "demo START is pending or already RUNNING";
      return;
    }
    if (!scan_fresh()) {
      response->success = false;
      response->message = "fresh /scan is required";
      return;
    }
    if (std::isfinite(minimum_front_range_m_) &&
        minimum_front_range_m_ <= static_cast<float>(stop_distance_m_)) {
      response->success = false;
      response->message = "front sector is blocked or has no valid range";
      return;
    }
    if (!bridge_healthy()) {
      response->success = false;
      response->message =
          "healthy Protocol 1.0 bridge/STM32 status is required";
      return;
    }
    if (!motion_path_ready()) {
      response->success = false;
      response->message =
          "STM32 BODY_COMMAND_READY/runtime motion gates are not ready";
      return;
    }
    if (!arm_client_->service_is_ready()) {
      response->success = false;
      response->message = "bridge arm service is unavailable";
      return;
    }

    clear_stop_observability();
    start_pending_ = true;
    arm_response_received_ = false;
    arm_response_success_ = false;
    publish_zero_twist(false);
    transition(State::kStopped,
               "explicit START accepted; waiting for authority acknowledgement");
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    arm_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto result = future.get();
          arm_response_received_ = true;
          arm_response_success_ = result->success;
          arm_response_message_ = result->message;
        });
    response->success = true;
    response->message = "START accepted; waiting for bridge authority";
  }

  void on_stop(const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
    publish_zero_twist(false);
    start_pending_ = false;
    request_disarm(true);
    transition(State::kStopped, "explicit stop; STOPPED latched");
    response->success = true;
    response->message =
        "zero Twist published, disarm requested, STOPPED latched";
  }

  void publish_zero_twist(const bool record_obstacle_stop) {
    geometry_msgs::msg::Twist command;
    command_publisher_->publish(command);
    if (record_obstacle_stop && stamp_is_zero(zero_twist_published_stamp_)) {
      zero_twist_published_stamp_ = now();
    }
  }

  void publish_drive_twist() {
    geometry_msgs::msg::Twist command;
    command.linear.x = forward_speed_mps_;
    command.angular.z = 0.0;
    command_publisher_->publish(command);
  }

  void request_disarm(const bool record_obstacle_stop) {
    if (record_obstacle_stop && stamp_is_zero(disarm_requested_stamp_)) {
      disarm_requested_stamp_ = now();
    }
    last_disarm_attempt_ = std::chrono::steady_clock::now();
    if (!disarm_client_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Bridge disarm service is unavailable");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    disarm_client_->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto result = future.get();
          if (!result->success) {
            RCLCPP_ERROR(get_logger(), "Bridge disarm reported: %s",
                         result->message.c_str());
          }
        });
  }

  void obstacle_stop() {
    if (state_ == State::kStopped && !start_pending_) {
      return;
    }
    obstacle_detected_stamp_ = now();
    publish_zero_twist(true);
    start_pending_ = false;
    request_disarm(true);
    transition(State::kStopped,
               "front obstacle threshold reached; STOPPED latched");
    RCLCPP_WARN(get_logger(),
                "Obstacle %.3f m <= %.3f m: zero Twist and disarm issued",
                minimum_front_range_m_, stop_distance_m_);
  }

  void latch_stopped(const std::string &reason) {
    const bool withdrawal_needed =
        start_pending_ || state_ == State::kRunning;
    start_pending_ = false;
    publish_zero_twist(false);
    if (withdrawal_needed) {
      request_disarm(false);
    }
    transition(State::kStopped, reason + "; STOPPED latched");
  }

  void clear_stop_observability() {
    obstacle_detected_stamp_ = builtin_interfaces::msg::Time();
    zero_twist_published_stamp_ = builtin_interfaces::msg::Time();
    disarm_requested_stamp_ = builtin_interfaces::msg::Time();
    authority_withdrawn_tx_stamp_ = builtin_interfaces::msg::Time();
    stm32_stop_confirmed_stamp_ = builtin_interfaces::msg::Time();
  }

  void control_tick() {
    if (!startup_disarm_requested_ && disarm_client_->service_is_ready()) {
      startup_disarm_requested_ = true;
      request_disarm(false);
    }

    if (start_pending_ || state_ == State::kRunning) {
      if (!scan_fresh()) {
        latch_stopped("LaserScan timeout");
      } else if (!bridge_status_fresh()) {
        latch_stopped("bridge status timeout");
      } else if (!motion_path_ready()) {
        latch_stopped("CAN/protocol/STM32 health lost");
      }
    }

    if (start_pending_) {
      if (arm_response_received_ && !arm_response_success_) {
        latch_stopped("bridge arm rejected: " + arm_response_message_);
      } else if (arm_response_received_ && bridge_status_.authority_armed) {
        start_pending_ = false;
        transition(State::kRunning,
                   "explicit START and STM32 authority acknowledgement observed");
      }
    } else if (state_ == State::kRunning &&
               !bridge_status_.authority_armed) {
      latch_stopped("motion authority unexpectedly cleared");
    }

    if (state_ == State::kRunning) {
      publish_drive_twist();
    } else {
      publish_zero_twist(false);
    }

    if (state_ == State::kStopped && !start_pending_ && have_bridge_status_ &&
        (bridge_status_.authority_armed || bridge_status_.arm_requested) &&
        age_ms(last_disarm_attempt_) >= 100U) {
      request_disarm(false);
    }
  }

  void publish_status() {
    msg::DemoStatus message;
    message.header.stamp = now();
    message.state = static_cast<uint8_t>(state_);
    message.state_name = state_name();
    message.reason = reason_;
    message.scan_fresh = scan_fresh();
    message.bridge_connected =
        bridge_status_fresh() && bridge_status_.connected;
    message.bridge_healthy = bridge_healthy();
    message.authority_armed =
        bridge_status_fresh() && bridge_status_.authority_armed;
    message.minimum_front_range_m = minimum_front_range_m_;
    message.obstacle_detected_stamp = obstacle_detected_stamp_;
    message.zero_twist_published_stamp = zero_twist_published_stamp_;
    message.disarm_requested_stamp = disarm_requested_stamp_;
    message.authority_withdrawn_tx_stamp = authority_withdrawn_tx_stamp_;
    message.stm32_stop_confirmed_stamp = stm32_stop_confirmed_stamp_;
    message.detection_to_zero_twist_ms =
        elapsed_ms(obstacle_detected_stamp_, zero_twist_published_stamp_);
    message.detection_to_disarm_request_ms =
        elapsed_ms(obstacle_detected_stamp_, disarm_requested_stamp_);
    message.detection_to_authority_withdrawal_ms =
        elapsed_ms(obstacle_detected_stamp_, authority_withdrawn_tx_stamp_);
    message.detection_to_stm32_confirmation_ms =
        elapsed_ms(obstacle_detected_stamp_, stm32_stop_confirmed_stamp_);
    status_publisher_->publish(message);
  }

  double forward_speed_mps_{0.30};
  double stop_distance_m_{0.60};
  double front_sector_deg_{30.0};
  double front_center_deg_{180.0};
  int scan_timeout_ms_{400};
  int bridge_timeout_ms_{300};
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string bridge_status_topic_;
  std::string arm_service_name_;
  std::string disarm_service_name_;

  State state_{State::kStopped};
  std::string reason_;
  bool have_scan_{false};
  bool have_bridge_status_{false};
  bool startup_disarm_requested_{false};
  bool start_pending_{false};
  bool arm_response_received_{false};
  bool arm_response_success_{false};
  std::string arm_response_message_;
  float minimum_front_range_m_{std::numeric_limits<float>::infinity()};
  msg::BridgeStatus bridge_status_{};
  std::chrono::steady_clock::time_point last_scan_received_{};
  std::chrono::steady_clock::time_point last_bridge_status_received_{};
  std::chrono::steady_clock::time_point last_disarm_attempt_{};

  builtin_interfaces::msg::Time obstacle_detected_stamp_{};
  builtin_interfaces::msg::Time zero_twist_published_stamp_{};
  builtin_interfaces::msg::Time disarm_requested_stamp_{};
  builtin_interfaces::msg::Time authority_withdrawn_tx_stamp_{};
  builtin_interfaces::msg::Time stm32_stop_confirmed_stamp_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<msg::DemoStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
  rclcpp::Subscription<msg::BridgeStatus>::SharedPtr bridge_subscription_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disarm_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

} // namespace robot_stm32_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<robot_stm32_bridge::StraightObstacleStopDemo>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("straight_obstacle_stop_demo"), "%s",
                 exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
