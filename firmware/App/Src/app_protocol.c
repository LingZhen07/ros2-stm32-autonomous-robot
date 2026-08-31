#include "app_protocol.h"

#include "app_config.h"
#include "app_command.h"
#include "app_control.h"
#include "app_drivetrain.h"

#include <math.h>
#include <string.h>

_Static_assert(APP_PROTOCOL_SYSTEM_STATUS_LENGTH == 32U, "Protocol v1 system DLC changed");
_Static_assert(APP_PROTOCOL_WHEEL_STATE_LENGTH == 64U, "Protocol v1 wheel DLC changed");
_Static_assert(APP_PROTOCOL_IMU_DATA_LENGTH == 48U, "Protocol v1 IMU DLC changed");
_Static_assert(APP_PROTOCOL_BATTERY_STATE_LENGTH == 16U, "Protocol v1 battery DLC changed");

static uint16_t AppProtocol_ReadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int16_t AppProtocol_ReadI16(const uint8_t *data)
{
  return (int16_t)AppProtocol_ReadU16(data);
}

static uint32_t AppProtocol_ReadU32(const uint8_t *data)
{
  return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void AppProtocol_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
}

static void AppProtocol_WriteI16(uint8_t *data, int16_t value)
{
  AppProtocol_WriteU16(data, (uint16_t)value);
}

static void AppProtocol_WriteU32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}

static void AppProtocol_WriteI32(uint8_t *data, int32_t value)
{
  AppProtocol_WriteU32(data, (uint32_t)value);
}

static void AppProtocol_WriteU64(uint8_t *data, uint64_t value)
{
  for (uint32_t index = 0U; index < 8U; ++index)
  {
    data[index] = (uint8_t)(value >> (index * 8U));
  }
}

static void AppProtocol_WriteI64(uint8_t *data, int64_t value)
{
  AppProtocol_WriteU64(data, (uint64_t)value);
}

static int32_t AppProtocol_ScaleI32(float value, float multiplier, bool valid)
{
  double scaled;

  if (!valid || !isfinite(value) || !isfinite(multiplier))
  {
    return APP_PROTOCOL_INVALID_I32;
  }
  scaled = (double)value * (double)multiplier;
  if ((scaled < -2147483647.0) || (scaled > 2147483647.0))
  {
    return APP_PROTOCOL_INVALID_I32;
  }
  scaled += (scaled >= 0.0) ? 0.5 : -0.5;
  return (int32_t)scaled;
}

static int16_t AppProtocol_ScaleI16(float value, float multiplier, bool valid)
{
  double scaled;

  if (!valid || !isfinite(value) || !isfinite(multiplier))
  {
    return APP_PROTOCOL_INVALID_I16;
  }
  scaled = (double)value * (double)multiplier;
  if ((scaled < -32767.0) || (scaled > 32767.0))
  {
    return APP_PROTOCOL_INVALID_I16;
  }
  scaled += (scaled >= 0.0) ? 0.5 : -0.5;
  return (int16_t)scaled;
}

static uint16_t AppProtocol_SaturateU16(uint32_t value)
{
  return (value > UINT16_MAX) ? UINT16_MAX : (uint16_t)value;
}

static void AppProtocol_WriteHeader(uint8_t *data, uint16_t sequence)
{
  data[0] = APP_PROTOCOL_VERSION_MAJOR;
  data[1] = APP_PROTOCOL_VERSION_MINOR;
  AppProtocol_WriteU16(&data[2], sequence);
}

static AppProtocolDecodeResult AppProtocol_CheckHeader(const uint8_t *data,
                                                       size_t length,
                                                       size_t expected_length)
{
  if ((data == NULL) || (length != expected_length))
  {
    return APP_PROTOCOL_DECODE_WRONG_LENGTH;
  }
  if ((data[0] != APP_PROTOCOL_VERSION_MAJOR) ||
      (data[1] != APP_PROTOCOL_VERSION_MINOR))
  {
    return APP_PROTOCOL_DECODE_VERSION_MISMATCH;
  }
  return APP_PROTOCOL_DECODE_OK;
}

AppProtocolDecodeResult AppProtocol_DecodeHostHeartbeat(
  const uint8_t *data, size_t length, AppProtocolHostHeartbeat *heartbeat)
{
  AppProtocolDecodeResult result = AppProtocol_CheckHeader(
    data, length, APP_PROTOCOL_HOST_HEARTBEAT_LENGTH);
  uint16_t flags;

  if ((result != APP_PROTOCOL_DECODE_OK) || (heartbeat == NULL))
  {
    return (heartbeat == NULL) ? APP_PROTOCOL_DECODE_INVALID_VALUE : result;
  }
  flags = AppProtocol_ReadU16(&data[12]);
  if (((flags & 0xFFFEU) != 0U) || (AppProtocol_ReadU16(&data[14]) != 0U))
  {
    return APP_PROTOCOL_DECODE_RESERVED_NONZERO;
  }
  heartbeat->sequence = AppProtocol_ReadU16(&data[2]);
  heartbeat->session_id = AppProtocol_ReadU32(&data[4]);
  heartbeat->host_uptime_ms = AppProtocol_ReadU32(&data[8]);
  heartbeat->bridge_ready = ((flags & 0x0001U) != 0U);
  if (heartbeat->session_id == 0U)
  {
    return APP_PROTOCOL_DECODE_INVALID_VALUE;
  }
  return APP_PROTOCOL_DECODE_OK;
}

AppProtocolDecodeResult AppProtocol_DecodeMotionAuthority(
  const uint8_t *data, size_t length, AppProtocolMotionAuthority *authority)
{
  AppProtocolDecodeResult result = AppProtocol_CheckHeader(
    data, length, APP_PROTOCOL_AUTHORITY_LENGTH);

  if ((result != APP_PROTOCOL_DECODE_OK) || (authority == NULL))
  {
    return (authority == NULL) ? APP_PROTOCOL_DECODE_INVALID_VALUE : result;
  }
  if ((data[9] != 0U) || (AppProtocol_ReadU16(&data[10]) != 0U))
  {
    return APP_PROTOCOL_DECODE_RESERVED_NONZERO;
  }
  if (data[8] > (uint8_t)APP_PROTOCOL_AUTHORITY_ARMED)
  {
    return APP_PROTOCOL_DECODE_INVALID_VALUE;
  }
  authority->sequence = AppProtocol_ReadU16(&data[2]);
  authority->session_id = AppProtocol_ReadU32(&data[4]);
  authority->state = (AppProtocolAuthorityState)data[8];
  authority->host_uptime_ms = AppProtocol_ReadU32(&data[12]);
  if (authority->session_id == 0U)
  {
    return APP_PROTOCOL_DECODE_INVALID_VALUE;
  }
  return APP_PROTOCOL_DECODE_OK;
}

AppProtocolDecodeResult AppProtocol_DecodeMotionCommand(
  const uint8_t *data, size_t length, AppProtocolMotionCommand *command)
{
  AppProtocolDecodeResult result = AppProtocol_CheckHeader(
    data, length, APP_PROTOCOL_MOTION_COMMAND_LENGTH);
  int16_t linear_wire;
  int16_t angular_wire;

  if ((result != APP_PROTOCOL_DECODE_OK) || (command == NULL))
  {
    return (command == NULL) ? APP_PROTOCOL_DECODE_INVALID_VALUE : result;
  }
  if ((data[9] != 0U) || (AppProtocol_ReadU16(&data[14]) != 0U))
  {
    return APP_PROTOCOL_DECODE_RESERVED_NONZERO;
  }
  if (data[8] > (uint8_t)APP_PROTOCOL_COMMAND_BODY_VELOCITY)
  {
    return APP_PROTOCOL_DECODE_UNSUPPORTED_MODE;
  }
  linear_wire = AppProtocol_ReadI16(&data[10]);
  angular_wire = AppProtocol_ReadI16(&data[12]);
  if ((linear_wire == APP_PROTOCOL_INVALID_I16) ||
      (angular_wire == APP_PROTOCOL_INVALID_I16))
  {
    return APP_PROTOCOL_DECODE_INVALID_VALUE;
  }
  if ((data[8] == (uint8_t)APP_PROTOCOL_COMMAND_DISABLED) &&
      ((linear_wire != 0) || (angular_wire != 0)))
  {
    return APP_PROTOCOL_DECODE_INVALID_VALUE;
  }
  command->sequence = AppProtocol_ReadU16(&data[2]);
  command->session_id = AppProtocol_ReadU32(&data[4]);
  command->mode = (AppProtocolCommandMode)data[8];
  command->linear_velocity_mps = (float)linear_wire * 0.001f;
  command->angular_velocity_radps = (float)angular_wire * 0.001f;
  if (command->session_id == 0U)
  {
    return APP_PROTOCOL_DECODE_INVALID_VALUE;
  }
  return APP_PROTOCOL_DECODE_OK;
}

void AppProtocol_EncodeSystemStatus(const AppTelemetrySnapshot *telemetry,
                                    const AppCanSnapshot *communication,
                                    uint16_t sequence,
                                    uint8_t data[APP_PROTOCOL_SYSTEM_STATUS_LENGTH])
{
  uint16_t motion_flags = 0U;
  uint16_t communication_flags = 0U;

  if ((telemetry == NULL) || (communication == NULL) || (data == NULL))
  {
    return;
  }
  memset(data, 0, APP_PROTOCOL_SYSTEM_STATUS_LENGTH);
  AppProtocol_WriteHeader(data, sequence);
  AppProtocol_WriteU32(&data[4], telemetry->runtime_ms);
  data[8] = APP_VERSION_MAJOR;
  data[9] = APP_VERSION_MINOR;
  data[10] = APP_VERSION_PATCH;
  data[11] = (uint8_t)telemetry->state.system_state;

  motion_flags |= communication->initialized ? (1U << 0) : 0U;
  motion_flags |= communication->controller_error_active ? (1U << 1) : 0U;
  motion_flags |= communication->heartbeat_fresh ? (1U << 2) : 0U;
  motion_flags |= communication->authority_disarmed_seen ? (1U << 3) : 0U;
  motion_flags |= communication->authority_armed ? (1U << 4) : 0U;
  {
    AppCommandSnapshot command;
    AppCommand_GetSnapshot(telemetry->runtime_ms, &command);
    motion_flags |= command.valid ? (1U << 5) : 0U;
    motion_flags |= command.fresh ? (1U << 6) : 0U;
  }
  motion_flags |= telemetry->motor.authorized ? (1U << 7) : 0U;
  motion_flags |= telemetry->motor.standby_asserted ? (1U << 8) : 0U;
  motion_flags |= telemetry->supervisor.critical_tasks_healthy ? (1U << 9) : 0U;
  motion_flags |= telemetry->supervisor.watchdog_refresh_allowed ? (1U << 10) : 0U;
  motion_flags |= telemetry->imu.valid ? (1U << 11) : 0U;
  motion_flags |= telemetry->encoder_1.valid ? (1U << 12) : 0U;
  motion_flags |= telemetry->encoder_2.valid ? (1U << 13) : 0U;
  motion_flags |= telemetry->battery.valid ? (1U << 14) : 0U;
  motion_flags |= (AppControl_AllConfigured() && AppDrivetrain_BodyCommandReady())
    ? (1U << 15) : 0U;
  AppProtocol_WriteU16(&data[12], motion_flags);

  communication_flags |= communication->error_warning ? (1U << 0) : 0U;
  communication_flags |= communication->error_passive ? (1U << 1) : 0U;
  communication_flags |= communication->bus_off ? (1U << 2) : 0U;
  communication_flags |= communication->rx_software_overflow_latched ? (1U << 3) : 0U;
  communication_flags |= communication->rx_hardware_lost_latched ? (1U << 4) : 0U;
  communication_flags |= communication->tx_failure_latched ? (1U << 5) : 0U;
  communication_flags |= communication->protocol_rejection_latched ? (1U << 6) : 0U;
  communication_flags |= communication->version_mismatch_latched ? (1U << 7) : 0U;
  communication_flags |= communication->session_active ? (1U << 8) : 0U;
  communication_flags |= communication->bridge_ready ? (1U << 9) : 0U;
  communication_flags |= (1U << 10); /* Protocol v1 always uses CAN FD with BRS. */
  communication_flags |= communication->command_sequence_seen ? (1U << 11) : 0U;
  communication_flags |= communication->heartbeat_sequence_seen ? (1U << 12) : 0U;
  communication_flags |= communication->authority_fresh ? (1U << 13) : 0U;
  AppProtocol_WriteU16(&data[14], communication_flags);
  AppProtocol_WriteU32(&data[16], telemetry->state.fault_flags);
  AppProtocol_WriteU32(&data[20], telemetry->state.reset_reason);
  AppProtocol_WriteU32(&data[24], communication->active_session_id);
  AppProtocol_WriteU16(&data[28], communication->command_sequence_seen
    ? communication->last_command_sequence : APP_PROTOCOL_INVALID_U16);
  AppProtocol_WriteU16(&data[30], communication->heartbeat_sequence_seen
    ? communication->last_heartbeat_sequence : APP_PROTOCOL_INVALID_U16);
}

void AppProtocol_EncodeWheelState(const AppTelemetrySnapshot *telemetry,
                                  uint16_t sequence,
                                  uint8_t data[APP_PROTOCOL_WHEEL_STATE_LENGTH])
{
  int64_t left_position = APP_PROTOCOL_INVALID_I64;
  int64_t right_position = APP_PROTOCOL_INVALID_I64;
  float left_rate = 0.0f;
  float right_rate = 0.0f;
  bool mapping_valid;
  uint16_t flags = 0U;

  if ((telemetry == NULL) || (data == NULL))
  {
    return;
  }
  mapping_valid = AppDrivetrain_MapEncoderPositions(
                    telemetry->encoder_1.accumulated_counts,
                    telemetry->encoder_2.accumulated_counts,
                    &left_position, &right_position) &&
                  AppDrivetrain_MapEncoderRates(
                    telemetry->encoder_1.filtered_counts_per_second,
                    telemetry->encoder_2.filtered_counts_per_second,
                    &left_rate, &right_rate);

  memset(data, 0, APP_PROTOCOL_WHEEL_STATE_LENGTH);
  AppProtocol_WriteHeader(data, sequence);
  AppProtocol_WriteU32(&data[4], telemetry->runtime_ms);
  AppProtocol_WriteI64(&data[8], mapping_valid ? left_position : APP_PROTOCOL_INVALID_I64);
  AppProtocol_WriteI64(&data[16], mapping_valid ? right_position : APP_PROTOCOL_INVALID_I64);
  AppProtocol_WriteI32(&data[24], AppProtocol_ScaleI32(left_rate, 1.0f, mapping_valid));
  AppProtocol_WriteI32(&data[28], AppProtocol_ScaleI32(right_rate, 1.0f, mapping_valid));
  AppProtocol_WriteI32(&data[32], AppProtocol_ScaleI32(
    telemetry->left_controller.target, 1.0f, telemetry->left_controller.configured));
  AppProtocol_WriteI32(&data[36], AppProtocol_ScaleI32(
    telemetry->right_controller.target, 1.0f, telemetry->right_controller.configured));
  AppProtocol_WriteI16(&data[40], AppProtocol_ScaleI16(
    telemetry->left_controller.output, 10000.0f, telemetry->left_controller.configured));
  AppProtocol_WriteI16(&data[42], AppProtocol_ScaleI16(
    telemetry->right_controller.output, 10000.0f, telemetry->right_controller.configured));
  AppProtocol_WriteU16(&data[44], telemetry->encoder_1.raw_counter);
  AppProtocol_WriteU16(&data[46], telemetry->encoder_2.raw_counter);
  AppProtocol_WriteI32(&data[48], AppProtocol_ScaleI32(
    telemetry->encoder_1.filtered_counts_per_second, 1.0f, telemetry->encoder_1.valid));
  AppProtocol_WriteI32(&data[52], AppProtocol_ScaleI32(
    telemetry->encoder_2.filtered_counts_per_second, 1.0f, telemetry->encoder_2.valid));

  flags |= mapping_valid ? (1U << 0) : 0U;
  flags |= telemetry->encoder_1.valid ? (1U << 1) : 0U;
  flags |= telemetry->encoder_2.valid ? (1U << 2) : 0U;
  flags |= telemetry->left_controller.configured ? (1U << 3) : 0U;
  flags |= telemetry->right_controller.configured ? (1U << 4) : 0U;
  flags |= (telemetry->left_controller.enabled && telemetry->right_controller.enabled)
    ? (1U << 5) : 0U;
  flags |= telemetry->left_controller.saturated ? (1U << 6) : 0U;
  flags |= telemetry->right_controller.saturated ? (1U << 7) : 0U;
  flags |= telemetry->motor.authorized ? (1U << 8) : 0U;
  flags |= telemetry->motor.standby_asserted ? (1U << 9) : 0U;
  flags |= telemetry->left_controller.configured ? (1U << 10) : 0U;
  flags |= telemetry->right_controller.configured ? (1U << 11) : 0U;
  flags |= (AppControl_AllConfigured() && AppDrivetrain_BodyCommandReady())
    ? (1U << 12) : 0U;
  flags |= (1U << 13); /* Left/right forward encoder signs are hardware-verified. */
  AppProtocol_WriteU16(&data[56], flags);
  AppProtocol_WriteU16(&data[58], AppProtocol_SaturateU16(telemetry->encoder_1.sample_age_ms));
  AppProtocol_WriteU16(&data[60], AppProtocol_SaturateU16(telemetry->encoder_2.sample_age_ms));
  AppProtocol_WriteU16(&data[62], APP_MOTOR_CONTROL_PERIOD_MS);
}

void AppProtocol_EncodeImuData(const AppTelemetrySnapshot *telemetry,
                               uint16_t sequence,
                               uint8_t data[APP_PROTOCOL_IMU_DATA_LENGTH])
{
  uint16_t flags = 0U;

  if ((telemetry == NULL) || (data == NULL))
  {
    return;
  }
  memset(data, 0, APP_PROTOCOL_IMU_DATA_LENGTH);
  AppProtocol_WriteHeader(data, sequence);
  AppProtocol_WriteU32(&data[4], telemetry->imu.timestamp_ms);
  flags |= telemetry->imu.valid ? (1U << 0) : 0U;
  flags |= telemetry->imu.data_ready ? (1U << 1) : 0U;
  flags |= (telemetry->imu.who_am_i == APP_IMU_WHO_AM_I_EXPECTED) ? (1U << 2) : 0U;
  flags |= (telemetry->imu.interrupt_1_count > 0U) ? (1U << 3) : 0U;
  flags |= (telemetry->imu.interrupt_2_count > 0U) ? (1U << 4) : 0U;
  flags |= telemetry->imu.valid ? (1U << 5) : 0U;
  flags |= telemetry->imu.valid ? (1U << 6) : 0U;
  AppProtocol_WriteU16(&data[8], flags);
  data[10] = telemetry->imu.who_am_i;
  data[11] = (uint8_t)telemetry->imu.state;
  for (uint32_t axis = 0U; axis < 3U; ++axis)
  {
    AppProtocol_WriteI32(&data[12U + (axis * 4U)], AppProtocol_ScaleI32(
      telemetry->imu.acceleration_mps2[axis], 10000.0f, telemetry->imu.valid));
    AppProtocol_WriteI32(&data[24U + (axis * 4U)], AppProtocol_ScaleI32(
      telemetry->imu.angular_velocity_radps[axis], 100000.0f, telemetry->imu.valid));
  }
  AppProtocol_WriteU16(&data[36], AppProtocol_SaturateU16(telemetry->imu.sample_age_ms));
  AppProtocol_WriteU32(&data[40], telemetry->imu.interrupt_1_count);
  AppProtocol_WriteU32(&data[44], telemetry->imu.interrupt_2_count);
}

void AppProtocol_EncodeBatteryState(const AppTelemetrySnapshot *telemetry,
                                    uint16_t sequence,
                                    uint8_t data[APP_PROTOCOL_BATTERY_STATE_LENGTH])
{
  uint16_t voltage_mv = APP_PROTOCOL_INVALID_U16;
  uint16_t flags = 0U;
  float scaled_voltage;

  if ((telemetry == NULL) || (data == NULL))
  {
    return;
  }
  if (telemetry->battery.valid && isfinite(telemetry->battery.filtered_battery_voltage))
  {
    scaled_voltage = telemetry->battery.filtered_battery_voltage * 1000.0f;
    if ((scaled_voltage >= 0.0f) && (scaled_voltage < (float)UINT16_MAX))
    {
      voltage_mv = (uint16_t)(scaled_voltage + 0.5f);
      flags |= (1U << 0);
    }
  }
  flags |= telemetry->battery.adc_calibrated ? (1U << 1) : 0U;
  flags |= telemetry->battery.divider_calibrated ? (1U << 2) : 0U;

  memset(data, 0, APP_PROTOCOL_BATTERY_STATE_LENGTH);
  AppProtocol_WriteHeader(data, sequence);
  AppProtocol_WriteU32(&data[4], telemetry->battery.timestamp_ms);
  AppProtocol_WriteU16(&data[8], voltage_mv);
  AppProtocol_WriteU16(&data[10], telemetry->battery.raw_adc);
  AppProtocol_WriteU16(&data[12], flags);
  AppProtocol_WriteU16(&data[14], AppProtocol_SaturateU16(telemetry->battery.sample_age_ms));
}
