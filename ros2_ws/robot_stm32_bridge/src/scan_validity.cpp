#include "robot_stm32_bridge/scan_validity.hpp"

#include <exception>

namespace robot_stm32_bridge {

ScanValidity evaluate_scan_validity(
    const bool have_usable_scan, const uint32_t receipt_age_ms,
    const uint32_t timeout_ms,
    const builtin_interfaces::msg::Time &source_stamp,
    const rclcpp::Time &ros_now, const int64_t future_tolerance_ns) {
  if (!have_usable_scan || receipt_age_ms > timeout_ms) {
    return ScanValidity::kReceiptTimeout;
  }
  if ((source_stamp.sec == 0 && source_stamp.nanosec == 0U) ||
      source_stamp.sec < 0 || source_stamp.nanosec >= 1000000000U ||
      ros_now.nanoseconds() <= 0 || future_tolerance_ns < 0) {
    return ScanValidity::kSourceTimestampInvalid;
  }

  try {
    const rclcpp::Time source_time(source_stamp, ros_now.get_clock_type());
    const int64_t source_age_ns = (ros_now - source_time).nanoseconds();
    if (source_age_ns < -future_tolerance_ns) {
      return ScanValidity::kSourceTimestampInvalid;
    }
    if (source_age_ns > static_cast<int64_t>(timeout_ms) * 1000000LL) {
      return ScanValidity::kSourceTimestampStale;
    }
  } catch (const std::exception &) {
    return ScanValidity::kSourceTimestampInvalid;
  }

  return ScanValidity::kValid;
}

const char *scan_validity_reason(const ScanValidity validity) {
  switch (validity) {
  case ScanValidity::kValid:
    return "scan valid";
  case ScanValidity::kReceiptTimeout:
    return "scan receipt timeout";
  case ScanValidity::kSourceTimestampStale:
    return "scan source timestamp stale";
  case ScanValidity::kSourceTimestampInvalid:
    return "scan source timestamp invalid";
  default:
    return "scan source timestamp invalid";
  }
}

} // namespace robot_stm32_bridge
