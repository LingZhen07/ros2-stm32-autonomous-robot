#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_stm32_bridge/msg/bridge_status.hpp"
#include "robot_stm32_bridge/msg/demo_status.hpp"
#include "robot_stm32_bridge/straight_obstacle_stop_demo.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_stm32_bridge {
namespace {

using namespace std::chrono_literals;

constexpr uint16_t kMotionMotorAuthorized = 1U << 7U;
constexpr uint16_t kMotionStbyEnabled = 1U << 8U;
constexpr uint16_t kMotionBodyCommandReady = 1U << 15U;
constexpr double kPi = 3.14159265358979323846;

enum class ScanMode { kFresh, kPaused, kStale, kZero, kFuture };

class SafetyGateHarness final : public rclcpp::Node {
public:
  SafetyGateHarness() : Node("obstacle_stop_scan_freshness_harness") {
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        "/test_obstacle_stop/scan", rclcpp::SensorDataQoS());
    bridge_publisher_ = create_publisher<msg::BridgeStatus>(
        "/test_obstacle_stop/bridge_status", 10);
    demo_status_subscription_ = create_subscription<msg::DemoStatus>(
        "/straight_obstacle_stop_demo/status", 10,
        [this](const msg::DemoStatus::SharedPtr status) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_demo_status_ = *status;
          have_demo_status_ = true;
        });
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        "/test_obstacle_stop/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr command) {
          std::lock_guard<std::mutex> lock(data_mutex_);
          latest_command_ = *command;
          have_command_ = true;
        });

    arm_service_ = create_service<std_srvs::srv::Trigger>(
        "/test_obstacle_stop/arm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          armed_.store(true);
          response->success = true;
          response->message = "test authority armed";
        });
    disarm_service_ = create_service<std_srvs::srv::Trigger>(
        "/test_obstacle_stop/disarm",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          armed_.store(false);
          disarm_count_.fetch_add(1U);
          response->success = true;
          response->message = "test authority withdrawn";
        });
    start_client_ =
        create_client<std_srvs::srv::Trigger>(
            "/straight_obstacle_stop_demo/start");

    scan_timer_ = create_wall_timer(20ms, [this]() { publish_scan(); });
    bridge_timer_ =
        create_wall_timer(20ms, [this]() { publish_bridge_status(); });
  }

  void set_scan_mode(const ScanMode mode) { scan_mode_.store(mode); }

  uint32_t disarm_count() const { return disarm_count_.load(); }

  bool wait_for_demo_status(
      const std::function<bool(const msg::DemoStatus &)> &predicate,
      const std::chrono::milliseconds timeout = 3000ms) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (have_demo_status_ && predicate(latest_demo_status_)) {
          return true;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    return have_demo_status_ && predicate(latest_demo_status_);
  }

  bool wait_for_command(const double linear_x,
                        const std::chrono::milliseconds timeout = 1000ms) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (have_command_ &&
            std::abs(latest_command_.linear.x - linear_x) < 1.0e-9) {
          return true;
        }
      }
      std::this_thread::sleep_for(5ms);
    }
    return false;
  }

  std_srvs::srv::Trigger::Response::SharedPtr explicit_start() {
    if (!start_client_->wait_for_service(2s)) {
      return nullptr;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = start_client_->async_send_request(request);
    if (future.wait_for(2s) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

private:
  void publish_scan() {
    const ScanMode mode = scan_mode_.load();
    if (mode == ScanMode::kPaused) {
      return;
    }

    sensor_msgs::msg::LaserScan scan;
    const rclcpp::Time current_time = now();
    if (mode == ScanMode::kFresh) {
      scan.header.stamp = current_time;
    } else if (mode == ScanMode::kStale) {
      scan.header.stamp =
          current_time - rclcpp::Duration::from_seconds(1.0);
    } else if (mode == ScanMode::kFuture) {
      scan.header.stamp =
          current_time + rclcpp::Duration::from_seconds(1.0);
    }
    scan.header.frame_id = "test_laser";
    scan.angle_min = static_cast<float>(kPi);
    scan.angle_max = static_cast<float>(kPi + 0.1);
    scan.angle_increment = 0.1F;
    scan.range_min = 0.1F;
    scan.range_max = 12.0F;
    scan.ranges = {2.0F};
    scan_publisher_->publish(scan);
  }

  void publish_bridge_status() {
    msg::BridgeStatus status;
    status.header.stamp = now();
    status.connected = true;
    status.protocol_valid = true;
    status.can_healthy = true;
    status.status_fresh = true;
    status.system_state = msg::BridgeStatus::SYSTEM_SAFE;
    status.authority_armed = armed_.load();
    status.motion_flags = kMotionBodyCommandReady;
    if (status.authority_armed) {
      status.motion_flags |= kMotionMotorAuthorized | kMotionStbyEnabled;
    }
    bridge_publisher_->publish(status);
  }

  std::atomic<ScanMode> scan_mode_{ScanMode::kFresh};
  std::atomic<bool> armed_{false};
  std::atomic<uint32_t> disarm_count_{0U};

  mutable std::mutex data_mutex_;
  bool have_demo_status_{false};
  bool have_command_{false};
  msg::DemoStatus latest_demo_status_{};
  geometry_msgs::msg::Twist latest_command_{};

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::Publisher<msg::BridgeStatus>::SharedPtr bridge_publisher_;
  rclcpp::Subscription<msg::DemoStatus>::SharedPtr demo_status_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      command_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr bridge_timer_;
};

class ObstacleStopScanFreshnessTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    harness_ = std::make_shared<SafetyGateHarness>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("scan_timeout_ms", 100),
        rclcpp::Parameter("bridge_timeout_ms", 100),
        rclcpp::Parameter("scan_topic", "/test_obstacle_stop/scan"),
        rclcpp::Parameter("cmd_vel_topic", "/test_obstacle_stop/cmd_vel"),
        rclcpp::Parameter("bridge_status_topic",
                          "/test_obstacle_stop/bridge_status"),
        rclcpp::Parameter("arm_service", "/test_obstacle_stop/arm"),
        rclcpp::Parameter("disarm_service", "/test_obstacle_stop/disarm"),
    });
    demo_ = make_straight_obstacle_stop_demo(options);
    executor_.add_node(harness_);
    executor_.add_node(demo_);
    spin_thread_ = std::thread([this]() { executor_.spin(); });
  }

  void TearDown() override {
    executor_.cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_.remove_node(demo_);
    executor_.remove_node(harness_);
    demo_.reset();
    harness_.reset();
  }

  void expect_running_after_explicit_start() {
    ASSERT_TRUE(harness_->wait_for_demo_status(
        [](const msg::DemoStatus &status) {
          return status.scan_fresh && status.bridge_healthy;
        }));
    const auto response = harness_->explicit_start();
    ASSERT_NE(response, nullptr);
    ASSERT_TRUE(response->success) << response->message;
    ASSERT_TRUE(harness_->wait_for_demo_status(
        [](const msg::DemoStatus &status) {
          return status.state == msg::DemoStatus::RUNNING;
        }));
    ASSERT_TRUE(harness_->wait_for_command(0.30));
  }

  void expect_latched_stop(const ScanMode failure_mode,
                           const std::string &expected_reason) {
    const uint32_t disarms_before = harness_->disarm_count();
    harness_->set_scan_mode(failure_mode);
    ASSERT_TRUE(harness_->wait_for_demo_status(
        [&expected_reason](const msg::DemoStatus &status) {
          return status.state == msg::DemoStatus::STOPPED &&
                 status.reason.find(expected_reason) != std::string::npos;
        }));
    ASSERT_TRUE(harness_->wait_for_command(0.0));
    ASSERT_TRUE(wait_until([this, disarms_before]() {
      return harness_->disarm_count() > disarms_before;
    }));

    harness_->set_scan_mode(ScanMode::kFresh);
    ASSERT_TRUE(harness_->wait_for_demo_status(
        [](const msg::DemoStatus &status) { return status.scan_fresh; }));
    std::this_thread::sleep_for(200ms);
    ASSERT_TRUE(harness_->wait_for_demo_status(
        [](const msg::DemoStatus &status) {
          return status.state == msg::DemoStatus::STOPPED;
        }));
    ASSERT_TRUE(harness_->wait_for_command(0.0));
  }

  bool wait_until(const std::function<bool()> &predicate,
                  const std::chrono::milliseconds timeout = 1000ms) const {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return predicate();
  }

  rclcpp::executors::MultiThreadedExecutor executor_;
  std::shared_ptr<SafetyGateHarness> harness_;
  std::shared_ptr<rclcpp::Node> demo_;
  std::thread spin_thread_;
};

TEST_F(ObstacleStopScanFreshnessTest,
       EveryScanFreshnessFailureUsesTheLatchedStopPath) {
  expect_running_after_explicit_start();

  expect_latched_stop(ScanMode::kPaused, "scan receipt timeout");
  expect_running_after_explicit_start();

  expect_latched_stop(ScanMode::kStale, "scan source timestamp stale");
  expect_running_after_explicit_start();

  expect_latched_stop(ScanMode::kZero, "scan source timestamp invalid");
  expect_running_after_explicit_start();

  expect_latched_stop(ScanMode::kFuture, "scan source timestamp invalid");
  expect_running_after_explicit_start();
}

} // namespace
} // namespace robot_stm32_bridge
