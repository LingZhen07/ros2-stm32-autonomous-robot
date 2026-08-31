#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "robot_stm32_bridge/protocol_v1.hpp"

namespace protocol = robot_stm32_bridge::protocol_v1;

TEST(ProtocolV1, EncodesGoldenHeartbeat) {
  constexpr std::array<uint8_t, 16> expected{0x01, 0x00, 0x34, 0x12, 0xD4, 0xC3,
                                             0xB2, 0xA1, 0x40, 0xE2, 0x01, 0x00,
                                             0x01, 0x00, 0x00, 0x00};
  EXPECT_EQ(protocol::encode_host_heartbeat(0x1234, 0xA1B2C3D4, 123456, true),
            expected);
}

TEST(ProtocolV1, EncodesGoldenDisarmedAuthority) {
  constexpr std::array<uint8_t, 16> expected{0x01, 0x00, 0x35, 0x12, 0xD4, 0xC3,
                                             0xB2, 0xA1, 0x00, 0x00, 0x00, 0x00,
                                             0xA4, 0xE2, 0x01, 0x00};
  EXPECT_EQ(
      protocol::encode_motion_authority(0x1235, 0xA1B2C3D4, false, 123556),
      expected);
}

TEST(ProtocolV1, EncodesGoldenBodyVelocity) {
  constexpr std::array<uint8_t, 16> expected{0x01, 0x00, 0x36, 0x12, 0xD4, 0xC3,
                                             0xB2, 0xA1, 0x01, 0x00, 0xFA, 0x00,
                                             0x0C, 0xFE, 0x00, 0x00};
  protocol::CommandPayload payload{};
  ASSERT_TRUE(protocol::encode_motion_command(0x1236, 0xA1B2C3D4, true, 0.250,
                                              -0.500, payload));
  EXPECT_EQ(payload, expected);
}

TEST(ProtocolV1, RejectsUnrepresentableVelocityAndDisabledMotion) {
  protocol::CommandPayload payload{};
  EXPECT_THROW(protocol::encode_host_heartbeat(1, 0, 0, true),
               std::invalid_argument);
  EXPECT_THROW(protocol::encode_motion_authority(1, 0, false, 0),
               std::invalid_argument);
  EXPECT_FALSE(
      protocol::encode_motion_command(1, 1, true, 32.768, 0.0, payload));
  EXPECT_FALSE(
      protocol::encode_motion_command(1, 1, false, 0.001, 0.0, payload));
  EXPECT_FALSE(protocol::encode_motion_command(1, 0, true, 0.0, 0.0, payload));
}

TEST(ProtocolV1, DecodesGoldenSystemStatus) {
  constexpr std::array<uint8_t, 32> payload{
      0x01, 0x00, 0x02, 0x01, 0xE8, 0x03, 0x00, 0x00, 0x00, 0x04, 0x00,
      0x04, 0xFF, 0xFF, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x00, 0xD4, 0xC3, 0xB2, 0xA1, 0x36, 0x12, 0x34, 0x12};
  protocol::SystemStatus status;
  ASSERT_TRUE(
      protocol::decode_system_status(payload.data(), payload.size(), status));
  EXPECT_EQ(status.sequence, 0x0102);
  EXPECT_EQ(status.timestamp_ms, 1000U);
  EXPECT_EQ(status.firmware_minor, 4U);
  EXPECT_EQ(status.system_state, 4U);
  EXPECT_EQ(status.motion_flags, 0xFFFFU);
  EXPECT_EQ(status.communication_flags, 0x3F00U);
  EXPECT_EQ(status.active_session_id, 0xA1B2C3D4U);
  EXPECT_EQ(status.last_command_sequence, 0x1236U);
}

TEST(ProtocolV1, DecodesGoldenWheelState) {
  constexpr std::array<uint8_t, 64> payload{
      0x01, 0x00, 0x01, 0x20, 0xE8, 0x03, 0x00, 0x00, 0x39, 0x30, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x39, 0x30, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xB0, 0x04, 0x00, 0x00, 0x9C, 0x04, 0x00, 0x00, 0xB0,
      0x04, 0x00, 0x00, 0xB0, 0x04, 0x00, 0x00, 0xC4, 0x09, 0x60, 0x09,
      0x31, 0xD4, 0x39, 0x30, 0x50, 0xFB, 0xFF, 0xFF, 0x9C, 0x04, 0x00,
      0x00, 0x3F, 0x3F, 0x05, 0x00, 0x05, 0x00, 0x0A, 0x00};
  protocol::WheelState state;
  ASSERT_TRUE(
      protocol::decode_wheel_state(payload.data(), payload.size(), state));
  EXPECT_EQ(state.left_position_counts, 12345);
  EXPECT_EQ(state.left_measured_cps, 1200);
  EXPECT_EQ(state.encoder_1_raw_cps, -1200);
  EXPECT_EQ(state.flags, 0x3F3FU);
}

TEST(ProtocolV1, DecodesGoldenImuData) {
  constexpr std::array<uint8_t, 48> payload{
      0x01, 0x00, 0x01, 0x30, 0xE8, 0x03, 0x00, 0x00, 0x6F, 0x00, 0x47, 0x02,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x7F, 0x01, 0x00,
      0xE8, 0x03, 0x00, 0x00, 0x30, 0xF8, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
      0x05, 0x00, 0x00, 0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  protocol::ImuData data;
  ASSERT_TRUE(protocol::decode_imu_data(payload.data(), payload.size(), data));
  EXPECT_EQ(data.who_am_i, 0x47U);
  EXPECT_EQ(data.accel_z, 98067);
  EXPECT_EQ(data.gyro_x, 1000);
  EXPECT_EQ(data.gyro_y, -2000);
}

TEST(ProtocolV1, DecodesGoldenBatteryState) {
  constexpr std::array<uint8_t, 16> payload{0x01, 0x00, 0x01, 0x40, 0x84, 0x03,
                                            0x00, 0x00, 0x39, 0x30, 0xC4, 0x09,
                                            0x07, 0x00, 0x64, 0x00};
  protocol::BatteryState state;
  ASSERT_TRUE(
      protocol::decode_battery_state(payload.data(), payload.size(), state));
  EXPECT_EQ(state.battery_voltage_mv, 12345U);
  EXPECT_EQ(state.raw_adc, 2500U);
  EXPECT_EQ(state.flags, 7U);
}

TEST(ProtocolV1, RejectsEnvelopeViolations) {
  std::array<uint8_t, 32> payload{};
  payload[0] = 2U;
  protocol::SystemStatus status;
  auto result =
      protocol::decode_system_status(payload.data(), payload.size(), status);
  EXPECT_EQ(result.error, protocol::DecodeError::kVersionMismatch);

  payload[0] = 1U;
  payload[1] = 0U;
  payload[11] = 2U;
  payload[15] = 0x40U;
  result =
      protocol::decode_system_status(payload.data(), payload.size(), status);
  EXPECT_EQ(result.error, protocol::DecodeError::kReservedField);
}
