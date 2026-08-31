#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include "app_can.h"
#include "app_telemetry.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_PROTOCOL_VERSION_MAJOR             1U
#define APP_PROTOCOL_VERSION_MINOR             0U

#define APP_PROTOCOL_CAN_ID_MOTION_AUTHORITY   0x080U
#define APP_PROTOCOL_CAN_ID_MOTION_COMMAND     0x081U
#define APP_PROTOCOL_CAN_ID_HOST_HEARTBEAT     0x082U
#define APP_PROTOCOL_CAN_ID_SYSTEM_STATUS      0x180U
#define APP_PROTOCOL_CAN_ID_WHEEL_STATE        0x181U
#define APP_PROTOCOL_CAN_ID_IMU_DATA           0x182U
#define APP_PROTOCOL_CAN_ID_BATTERY_STATE      0x183U

#define APP_PROTOCOL_AUTHORITY_LENGTH          16U
#define APP_PROTOCOL_MOTION_COMMAND_LENGTH     16U
#define APP_PROTOCOL_HOST_HEARTBEAT_LENGTH     16U
#define APP_PROTOCOL_SYSTEM_STATUS_LENGTH      32U
#define APP_PROTOCOL_WHEEL_STATE_LENGTH        64U
#define APP_PROTOCOL_IMU_DATA_LENGTH           48U
#define APP_PROTOCOL_BATTERY_STATE_LENGTH      16U

#define APP_PROTOCOL_INVALID_I16               INT16_MIN
#define APP_PROTOCOL_INVALID_I32               INT32_MIN
#define APP_PROTOCOL_INVALID_I64               INT64_MIN
#define APP_PROTOCOL_INVALID_U16               UINT16_MAX

typedef enum
{
  APP_PROTOCOL_DECODE_OK = 0,
  APP_PROTOCOL_DECODE_WRONG_LENGTH,
  APP_PROTOCOL_DECODE_VERSION_MISMATCH,
  APP_PROTOCOL_DECODE_RESERVED_NONZERO,
  APP_PROTOCOL_DECODE_INVALID_VALUE,
  APP_PROTOCOL_DECODE_UNSUPPORTED_MODE
} AppProtocolDecodeResult;

typedef enum
{
  APP_PROTOCOL_AUTHORITY_DISARMED = 0,
  APP_PROTOCOL_AUTHORITY_ARMED = 1
} AppProtocolAuthorityState;

typedef enum
{
  APP_PROTOCOL_COMMAND_DISABLED = 0,
  APP_PROTOCOL_COMMAND_BODY_VELOCITY = 1
} AppProtocolCommandMode;

typedef struct
{
  uint16_t sequence;
  uint32_t session_id;
  uint32_t host_uptime_ms;
  bool bridge_ready;
} AppProtocolHostHeartbeat;

typedef struct
{
  uint16_t sequence;
  uint32_t session_id;
  AppProtocolAuthorityState state;
  uint32_t host_uptime_ms;
} AppProtocolMotionAuthority;

typedef struct
{
  uint16_t sequence;
  uint32_t session_id;
  AppProtocolCommandMode mode;
  float linear_velocity_mps;
  float angular_velocity_radps;
} AppProtocolMotionCommand;

AppProtocolDecodeResult AppProtocol_DecodeHostHeartbeat(
  const uint8_t *data, size_t length, AppProtocolHostHeartbeat *heartbeat);
AppProtocolDecodeResult AppProtocol_DecodeMotionAuthority(
  const uint8_t *data, size_t length, AppProtocolMotionAuthority *authority);
AppProtocolDecodeResult AppProtocol_DecodeMotionCommand(
  const uint8_t *data, size_t length, AppProtocolMotionCommand *command);

void AppProtocol_EncodeSystemStatus(const AppTelemetrySnapshot *telemetry,
                                    const AppCanSnapshot *communication,
                                    uint16_t sequence,
                                    uint8_t data[APP_PROTOCOL_SYSTEM_STATUS_LENGTH]);
void AppProtocol_EncodeWheelState(const AppTelemetrySnapshot *telemetry,
                                  uint16_t sequence,
                                  uint8_t data[APP_PROTOCOL_WHEEL_STATE_LENGTH]);
void AppProtocol_EncodeImuData(const AppTelemetrySnapshot *telemetry,
                               uint16_t sequence,
                               uint8_t data[APP_PROTOCOL_IMU_DATA_LENGTH]);
void AppProtocol_EncodeBatteryState(const AppTelemetrySnapshot *telemetry,
                                    uint16_t sequence,
                                    uint8_t data[APP_PROTOCOL_BATTERY_STATE_LENGTH]);

#endif /* APP_PROTOCOL_H */
