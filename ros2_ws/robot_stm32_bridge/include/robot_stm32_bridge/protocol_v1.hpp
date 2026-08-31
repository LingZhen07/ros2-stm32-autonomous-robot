#ifndef ROBOT_STM32_BRIDGE__PROTOCOL_V1_HPP_
#define ROBOT_STM32_BRIDGE__PROTOCOL_V1_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace robot_stm32_bridge::protocol_v1 {

constexpr uint8_t kProtocolMajor = 1U;
constexpr uint8_t kProtocolMinor = 0U;

constexpr uint32_t kMotionAuthorityCanId = 0x080U;
constexpr uint32_t kMotionCommandCanId = 0x081U;
constexpr uint32_t kHostHeartbeatCanId = 0x082U;
constexpr uint32_t kSystemStatusCanId = 0x180U;
constexpr uint32_t kWheelStateCanId = 0x181U;
constexpr uint32_t kImuDataCanId = 0x182U;
constexpr uint32_t kBatteryStateCanId = 0x183U;

constexpr std::size_t kCommandPayloadSize = 16U;
constexpr std::size_t kSystemStatusPayloadSize = 32U;
constexpr std::size_t kWheelStatePayloadSize = 64U;
constexpr std::size_t kImuDataPayloadSize = 48U;
constexpr std::size_t kBatteryStatePayloadSize = 16U;

using CommandPayload = std::array<uint8_t, kCommandPayloadSize>;

enum class DecodeError {
  kNone,
  kWrongLength,
  kVersionMismatch,
  kReservedField,
  kInvalidValue,
};

struct DecodeResult {
  DecodeError error{DecodeError::kNone};
  std::string reason;

  explicit operator bool() const { return error == DecodeError::kNone; }
};

struct SystemStatus {
  uint8_t protocol_major{0U};
  uint8_t protocol_minor{0U};
  uint16_t sequence{0U};
  uint32_t timestamp_ms{0U};
  uint8_t firmware_major{0U};
  uint8_t firmware_minor{0U};
  uint8_t firmware_patch{0U};
  uint8_t system_state{0U};
  uint16_t motion_flags{0U};
  uint16_t communication_flags{0U};
  uint32_t fault_flags{0U};
  uint32_t reset_reason{0U};
  uint32_t active_session_id{0U};
  uint16_t last_command_sequence{0U};
  uint16_t last_heartbeat_sequence{0U};
};

struct WheelState {
  uint16_t sequence{0U};
  uint32_t timestamp_ms{0U};
  int64_t left_position_counts{0};
  int64_t right_position_counts{0};
  int32_t left_measured_cps{0};
  int32_t right_measured_cps{0};
  int32_t left_target_cps{0};
  int32_t right_target_cps{0};
  int16_t left_controller_output{0};
  int16_t right_controller_output{0};
  uint16_t encoder_1_raw_counter{0U};
  uint16_t encoder_2_raw_counter{0U};
  int32_t encoder_1_raw_cps{0};
  int32_t encoder_2_raw_cps{0};
  uint16_t flags{0U};
  uint16_t encoder_1_age_ms{0U};
  uint16_t encoder_2_age_ms{0U};
  uint16_t sample_period_ms{0U};
};

struct ImuData {
  uint16_t sequence{0U};
  uint32_t sample_timestamp_ms{0U};
  uint16_t flags{0U};
  uint8_t who_am_i{0U};
  uint8_t imu_state{0U};
  int32_t accel_x{0};
  int32_t accel_y{0};
  int32_t accel_z{0};
  int32_t gyro_x{0};
  int32_t gyro_y{0};
  int32_t gyro_z{0};
  uint16_t sample_age_ms{0U};
  uint32_t int1_count{0U};
  uint32_t int2_count{0U};
};

struct BatteryState {
  uint16_t sequence{0U};
  uint32_t sample_timestamp_ms{0U};
  uint16_t battery_voltage_mv{0U};
  uint16_t raw_adc{0U};
  uint16_t flags{0U};
  uint16_t sample_age_ms{0U};
};

CommandPayload encode_host_heartbeat(uint16_t sequence, uint32_t session_id,
                                     uint32_t host_uptime_ms,
                                     bool bridge_ready);

CommandPayload encode_motion_authority(uint16_t sequence, uint32_t session_id,
                                       bool armed, uint32_t host_uptime_ms);

DecodeResult encode_motion_command(uint16_t sequence, uint32_t session_id,
                                   bool body_velocity,
                                   double linear_velocity_mps,
                                   double angular_velocity_radps,
                                   CommandPayload &payload);

DecodeResult decode_system_status(const uint8_t *data, std::size_t length,
                                  SystemStatus &output);
DecodeResult decode_wheel_state(const uint8_t *data, std::size_t length,
                                WheelState &output);
DecodeResult decode_imu_data(const uint8_t *data, std::size_t length,
                             ImuData &output);
DecodeResult decode_battery_state(const uint8_t *data, std::size_t length,
                                  BatteryState &output);

} // namespace robot_stm32_bridge::protocol_v1

#endif // ROBOT_STM32_BRIDGE__PROTOCOL_V1_HPP_
