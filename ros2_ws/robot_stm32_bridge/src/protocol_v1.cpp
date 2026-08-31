#include "robot_stm32_bridge/protocol_v1.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <type_traits>

namespace robot_stm32_bridge::protocol_v1 {
namespace {

template <typename UnsignedT>
UnsignedT read_unsigned(const uint8_t *data, const std::size_t offset) {
  static_assert(std::is_unsigned<UnsignedT>::value,
                "wire helper requires an unsigned type");
  UnsignedT value = 0;
  for (std::size_t index = 0; index < sizeof(UnsignedT); ++index) {
    value |= static_cast<UnsignedT>(data[offset + index]) << (8U * index);
  }
  return value;
}

template <typename SignedT, typename UnsignedT>
SignedT read_signed(const uint8_t *data, const std::size_t offset) {
  static_assert(sizeof(SignedT) == sizeof(UnsignedT),
                "signed and unsigned widths differ");
  const UnsignedT wire = read_unsigned<UnsignedT>(data, offset);
  SignedT value{};
  std::memcpy(&value, &wire, sizeof(value));
  return value;
}

template <typename UnsignedT, std::size_t Size>
void write_unsigned(std::array<uint8_t, Size> &payload,
                    const std::size_t offset, UnsignedT value) {
  static_assert(std::is_unsigned<UnsignedT>::value,
                "wire helper requires an unsigned type");
  for (std::size_t index = 0; index < sizeof(UnsignedT); ++index) {
    payload[offset + index] =
        static_cast<uint8_t>((value >> (8U * index)) & 0xFFU);
  }
}

template <typename SignedT, typename UnsignedT, std::size_t Size>
void write_signed(std::array<uint8_t, Size> &payload, const std::size_t offset,
                  SignedT value) {
  static_assert(sizeof(SignedT) == sizeof(UnsignedT),
                "signed and unsigned widths differ");
  UnsignedT wire{};
  std::memcpy(&wire, &value, sizeof(wire));
  write_unsigned(payload, offset, wire);
}

DecodeResult success() { return {}; }

DecodeResult failure(const DecodeError error, const std::string &reason) {
  return DecodeResult{error, reason};
}

DecodeResult validate_header(const uint8_t *data, const std::size_t length,
                             const std::size_t expected_length,
                             const char *frame_name) {
  if (data == nullptr || length != expected_length) {
    std::ostringstream stream;
    stream << frame_name << " length " << length << " does not equal "
           << expected_length;
    return failure(DecodeError::kWrongLength, stream.str());
  }
  if (data[0] != kProtocolMajor || data[1] != kProtocolMinor) {
    std::ostringstream stream;
    stream << frame_name << " version " << static_cast<unsigned>(data[0]) << '.'
           << static_cast<unsigned>(data[1]) << " is not 1.0";
    return failure(DecodeError::kVersionMismatch, stream.str());
  }
  return success();
}

DecodeResult velocity_to_wire(const double value, const char *name,
                              int16_t &output) {
  if (!std::isfinite(value)) {
    return failure(DecodeError::kInvalidValue,
                   std::string(name) + " is not finite");
  }
  const double scaled = std::round(value / 0.001);
  if (scaled < -32767.0 || scaled > 32767.0) {
    return failure(DecodeError::kInvalidValue,
                   std::string(name) +
                       " is outside Protocol 1.0 serialization range");
  }
  output = static_cast<int16_t>(scaled);
  return success();
}

} // namespace

CommandPayload encode_host_heartbeat(const uint16_t sequence,
                                     const uint32_t session_id,
                                     const uint32_t host_uptime_ms,
                                     const bool bridge_ready) {
  if (session_id == 0U) {
    throw std::invalid_argument("HOST_HEARTBEAT session_id zero is invalid");
  }
  CommandPayload payload{};
  payload[0] = kProtocolMajor;
  payload[1] = kProtocolMinor;
  write_unsigned(payload, 2U, sequence);
  write_unsigned(payload, 4U, session_id);
  write_unsigned(payload, 8U, host_uptime_ms);
  write_unsigned(payload, 12U, static_cast<uint16_t>(bridge_ready ? 1U : 0U));
  return payload;
}

CommandPayload encode_motion_authority(const uint16_t sequence,
                                       const uint32_t session_id,
                                       const bool armed,
                                       const uint32_t host_uptime_ms) {
  if (session_id == 0U) {
    throw std::invalid_argument("MOTION_AUTHORITY session_id zero is invalid");
  }
  CommandPayload payload{};
  payload[0] = kProtocolMajor;
  payload[1] = kProtocolMinor;
  write_unsigned(payload, 2U, sequence);
  write_unsigned(payload, 4U, session_id);
  payload[8] = armed ? 1U : 0U;
  write_unsigned(payload, 12U, host_uptime_ms);
  return payload;
}

DecodeResult encode_motion_command(const uint16_t sequence,
                                   const uint32_t session_id,
                                   const bool body_velocity,
                                   const double linear_velocity_mps,
                                   const double angular_velocity_radps,
                                   CommandPayload &payload) {
  payload.fill(0U);
  if (session_id == 0U) {
    return failure(DecodeError::kInvalidValue, "session_id zero is invalid");
  }

  int16_t linear_wire = 0;
  int16_t angular_wire = 0;
  if (body_velocity) {
    const auto linear_result =
        velocity_to_wire(linear_velocity_mps, "linear velocity", linear_wire);
    if (!linear_result) {
      return linear_result;
    }
    const auto angular_result = velocity_to_wire(
        angular_velocity_radps, "angular velocity", angular_wire);
    if (!angular_result) {
      return angular_result;
    }
  } else if (linear_velocity_mps != 0.0 || angular_velocity_radps != 0.0) {
    return failure(DecodeError::kInvalidValue,
                   "DISABLED command requires exactly zero velocities");
  }

  payload[0] = kProtocolMajor;
  payload[1] = kProtocolMinor;
  write_unsigned(payload, 2U, sequence);
  write_unsigned(payload, 4U, session_id);
  payload[8] = body_velocity ? 1U : 0U;
  write_signed<int16_t, uint16_t>(payload, 10U, linear_wire);
  write_signed<int16_t, uint16_t>(payload, 12U, angular_wire);
  return success();
}

DecodeResult decode_system_status(const uint8_t *data, const std::size_t length,
                                  SystemStatus &output) {
  const auto header =
      validate_header(data, length, kSystemStatusPayloadSize, "SYSTEM_STATUS");
  if (!header) {
    return header;
  }

  output.protocol_major = data[0];
  output.protocol_minor = data[1];
  output.sequence = read_unsigned<uint16_t>(data, 2U);
  output.timestamp_ms = read_unsigned<uint32_t>(data, 4U);
  output.firmware_major = data[8];
  output.firmware_minor = data[9];
  output.firmware_patch = data[10];
  output.system_state = data[11];
  output.motion_flags = read_unsigned<uint16_t>(data, 12U);
  output.communication_flags = read_unsigned<uint16_t>(data, 14U);
  output.fault_flags = read_unsigned<uint32_t>(data, 16U);
  output.reset_reason = read_unsigned<uint32_t>(data, 20U);
  output.active_session_id = read_unsigned<uint32_t>(data, 24U);
  output.last_command_sequence = read_unsigned<uint16_t>(data, 28U);
  output.last_heartbeat_sequence = read_unsigned<uint16_t>(data, 30U);

  if (output.system_state > 5U) {
    return failure(DecodeError::kInvalidValue,
                   "SYSTEM_STATUS system_state is invalid");
  }
  if ((output.communication_flags & 0xC000U) != 0U) {
    return failure(DecodeError::kReservedField,
                   "SYSTEM_STATUS communication reserved bits are nonzero");
  }
  if ((output.fault_flags & 0xFFFFF000UL) != 0U) {
    return failure(DecodeError::kReservedField,
                   "SYSTEM_STATUS fault reserved bits are nonzero");
  }
  return success();
}

DecodeResult decode_wheel_state(const uint8_t *data, const std::size_t length,
                                WheelState &output) {
  const auto header =
      validate_header(data, length, kWheelStatePayloadSize, "WHEEL_STATE");
  if (!header) {
    return header;
  }

  output.sequence = read_unsigned<uint16_t>(data, 2U);
  output.timestamp_ms = read_unsigned<uint32_t>(data, 4U);
  output.left_position_counts = read_signed<int64_t, uint64_t>(data, 8U);
  output.right_position_counts = read_signed<int64_t, uint64_t>(data, 16U);
  output.left_measured_cps = read_signed<int32_t, uint32_t>(data, 24U);
  output.right_measured_cps = read_signed<int32_t, uint32_t>(data, 28U);
  output.left_target_cps = read_signed<int32_t, uint32_t>(data, 32U);
  output.right_target_cps = read_signed<int32_t, uint32_t>(data, 36U);
  output.left_controller_output = read_signed<int16_t, uint16_t>(data, 40U);
  output.right_controller_output = read_signed<int16_t, uint16_t>(data, 42U);
  output.encoder_1_raw_counter = read_unsigned<uint16_t>(data, 44U);
  output.encoder_2_raw_counter = read_unsigned<uint16_t>(data, 46U);
  output.encoder_1_raw_cps = read_signed<int32_t, uint32_t>(data, 48U);
  output.encoder_2_raw_cps = read_signed<int32_t, uint32_t>(data, 52U);
  output.flags = read_unsigned<uint16_t>(data, 56U);
  output.encoder_1_age_ms = read_unsigned<uint16_t>(data, 58U);
  output.encoder_2_age_ms = read_unsigned<uint16_t>(data, 60U);
  output.sample_period_ms = read_unsigned<uint16_t>(data, 62U);

  if ((output.flags & 0xC000U) != 0U) {
    return failure(DecodeError::kReservedField,
                   "WHEEL_STATE reserved flag bits are nonzero");
  }
  const auto effort_valid = [](const int16_t value) {
    return value == std::numeric_limits<int16_t>::min() ||
           (value >= -10000 && value <= 10000);
  };
  if (!effort_valid(output.left_controller_output) ||
      !effort_valid(output.right_controller_output)) {
    return failure(DecodeError::kInvalidValue,
                   "WHEEL_STATE controller effort is invalid");
  }
  return success();
}

DecodeResult decode_imu_data(const uint8_t *data, const std::size_t length,
                             ImuData &output) {
  const auto header =
      validate_header(data, length, kImuDataPayloadSize, "IMU_DATA");
  if (!header) {
    return header;
  }

  output.sequence = read_unsigned<uint16_t>(data, 2U);
  output.sample_timestamp_ms = read_unsigned<uint32_t>(data, 4U);
  output.flags = read_unsigned<uint16_t>(data, 8U);
  output.who_am_i = data[10];
  output.imu_state = data[11];
  output.accel_x = read_signed<int32_t, uint32_t>(data, 12U);
  output.accel_y = read_signed<int32_t, uint32_t>(data, 16U);
  output.accel_z = read_signed<int32_t, uint32_t>(data, 20U);
  output.gyro_x = read_signed<int32_t, uint32_t>(data, 24U);
  output.gyro_y = read_signed<int32_t, uint32_t>(data, 28U);
  output.gyro_z = read_signed<int32_t, uint32_t>(data, 32U);
  output.sample_age_ms = read_unsigned<uint16_t>(data, 36U);
  output.int1_count = read_unsigned<uint32_t>(data, 40U);
  output.int2_count = read_unsigned<uint32_t>(data, 44U);

  if ((output.flags & 0xFF80U) != 0U ||
      read_unsigned<uint16_t>(data, 38U) != 0U) {
    return failure(DecodeError::kReservedField,
                   "IMU_DATA reserved field is nonzero");
  }
  if (output.imu_state > 3U) {
    return failure(DecodeError::kInvalidValue, "IMU_DATA imu_state is invalid");
  }
  if ((output.flags & (1U << 2U)) != 0U && output.who_am_i != 0x47U) {
    return failure(DecodeError::kInvalidValue,
                   "IMU_DATA WHO_AM_I flag contradicts value");
  }
  const auto invalid_i32 = std::numeric_limits<int32_t>::min();
  if ((output.flags & (1U << 5U)) != 0U &&
      (output.accel_x == invalid_i32 || output.accel_y == invalid_i32 ||
       output.accel_z == invalid_i32)) {
    return failure(DecodeError::kInvalidValue,
                   "IMU_DATA valid acceleration contains sentinel");
  }
  if ((output.flags & (1U << 6U)) != 0U &&
      (output.gyro_x == invalid_i32 || output.gyro_y == invalid_i32 ||
       output.gyro_z == invalid_i32)) {
    return failure(DecodeError::kInvalidValue,
                   "IMU_DATA valid gyroscope contains sentinel");
  }
  return success();
}

DecodeResult decode_battery_state(const uint8_t *data, const std::size_t length,
                                  BatteryState &output) {
  const auto header =
      validate_header(data, length, kBatteryStatePayloadSize, "BATTERY_STATE");
  if (!header) {
    return header;
  }

  output.sequence = read_unsigned<uint16_t>(data, 2U);
  output.sample_timestamp_ms = read_unsigned<uint32_t>(data, 4U);
  output.battery_voltage_mv = read_unsigned<uint16_t>(data, 8U);
  output.raw_adc = read_unsigned<uint16_t>(data, 10U);
  output.flags = read_unsigned<uint16_t>(data, 12U);
  output.sample_age_ms = read_unsigned<uint16_t>(data, 14U);

  if ((output.flags & 0xFFF8U) != 0U) {
    return failure(DecodeError::kReservedField,
                   "BATTERY_STATE reserved flag bits are nonzero");
  }
  if (output.raw_adc > 4095U) {
    return failure(DecodeError::kInvalidValue,
                   "BATTERY_STATE raw ADC is outside 12-bit range");
  }
  if ((output.flags & 1U) != 0U && output.battery_voltage_mv == 0xFFFFU) {
    return failure(DecodeError::kInvalidValue,
                   "BATTERY_STATE valid voltage contains sentinel");
  }
  return success();
}

} // namespace robot_stm32_bridge::protocol_v1
