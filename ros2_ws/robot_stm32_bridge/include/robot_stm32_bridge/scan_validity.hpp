#pragma once

#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/time.hpp"

namespace robot_stm32_bridge {

enum class ScanValidity {
  kValid,
  kReceiptTimeout,
  kSourceTimestampStale,
  kSourceTimestampInvalid,
};

ScanValidity evaluate_scan_validity(
    bool have_usable_scan, uint32_t receipt_age_ms, uint32_t timeout_ms,
    const builtin_interfaces::msg::Time &source_stamp,
    const rclcpp::Time &ros_now, int64_t future_tolerance_ns);

const char *scan_validity_reason(ScanValidity validity);

} // namespace robot_stm32_bridge
