#include <gtest/gtest.h>

#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "rcl/time.h"
#include "rclcpp/time.hpp"
#include "robot_stm32_bridge/scan_validity.hpp"

namespace robot_stm32_bridge {
namespace {

constexpr uint32_t kTimeoutMs = 400U;
constexpr int64_t kFutureToleranceNs = 50LL * 1000000LL;

builtin_interfaces::msg::Time stamp(const int32_t seconds,
                                    const uint32_t nanoseconds = 0U) {
  builtin_interfaces::msg::Time result;
  result.sec = seconds;
  result.nanosec = nanoseconds;
  return result;
}

TEST(ScanValidityTest, RequiresFreshReceiptAndFreshSourceTimestamp) {
  const rclcpp::Time now(10, 0U, RCL_ROS_TIME);

  EXPECT_EQ(evaluate_scan_validity(true, 20U, kTimeoutMs,
                                   stamp(9, 800000000U), now,
                                   kFutureToleranceNs),
            ScanValidity::kValid);
  EXPECT_EQ(evaluate_scan_validity(true, 401U, kTimeoutMs, stamp(10), now,
                                   kFutureToleranceNs),
            ScanValidity::kReceiptTimeout);
  EXPECT_EQ(evaluate_scan_validity(true, 20U, kTimeoutMs,
                                   stamp(9, 599000000U), now,
                                   kFutureToleranceNs),
            ScanValidity::kSourceTimestampStale);
  EXPECT_EQ(evaluate_scan_validity(true, 20U, kTimeoutMs, stamp(0), now,
                                   kFutureToleranceNs),
            ScanValidity::kSourceTimestampInvalid);
  EXPECT_EQ(evaluate_scan_validity(true, 20U, kTimeoutMs,
                                   stamp(10, 51000000U), now,
                                   kFutureToleranceNs),
            ScanValidity::kSourceTimestampInvalid);
}

TEST(ScanValidityTest, AllowsOnlyTheExplicitMinorFutureTolerance) {
  const rclcpp::Time now(10, 0U, RCL_ROS_TIME);

  EXPECT_EQ(evaluate_scan_validity(true, 20U, kTimeoutMs,
                                   stamp(10, 50000000U), now,
                                   kFutureToleranceNs),
            ScanValidity::kValid);
  EXPECT_EQ(evaluate_scan_validity(true, 20U, kTimeoutMs, stamp(10),
                                   rclcpp::Time(0, 0U, RCL_ROS_TIME),
                                   kFutureToleranceNs),
            ScanValidity::kSourceTimestampInvalid);
}

} // namespace
} // namespace robot_stm32_bridge
