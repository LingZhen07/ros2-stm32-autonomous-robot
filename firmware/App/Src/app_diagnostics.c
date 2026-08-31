#include "app_diagnostics.h"

#include "app_can.h"
#include "app_command.h"
#include "app_config.h"
#include "app_control.h"
#include "app_drivetrain.h"
#include "app_encoder.h"
#include "app_motor.h"
#include "app_platform.h"
#include "app_protocol.h"
#include "app_safety.h"
#include "app_state.h"
#include "app_telemetry.h"
#include "cmsis_os2.h"
#include "usart.h"

#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
  uint16_t length;
  char text[APP_DIAGNOSTIC_OUTPUT_LENGTH];
} AppDiagnosticBlock;

typedef enum
{
  APP_DIAGNOSTIC_WATCH_OFF = 0,
  APP_DIAGNOSTIC_WATCH_ENCODER,
  APP_DIAGNOSTIC_WATCH_IMU,
  APP_DIAGNOSTIC_WATCH_CAN
} AppDiagnosticWatchSource;

typedef struct
{
  uint32_t flag;
  const char *name;
} AppDiagnosticFaultInfo;

static const AppDiagnosticFaultInfo g_fault_info[] = {
  {APP_FAULT_IMU_INITIALIZATION, "IMU_INITIALIZATION"},
  {APP_FAULT_COMMAND_TIMEOUT, "COMMAND_TIMEOUT"},
  {APP_FAULT_INVALID_MOTOR_COMMAND, "INVALID_MOTOR_COMMAND"},
  {APP_FAULT_SUPERVISOR, "SUPERVISOR"},
  {APP_FAULT_ENCODER_VALIDITY, "ENCODER_VALIDITY"},
  {APP_FAULT_BATTERY_MEASUREMENT, "BATTERY_MEASUREMENT"},
  {APP_FAULT_CONTROL_SATURATION, "CONTROL_SATURATION"},
  {APP_FAULT_CONTROL_ABNORMAL, "CONTROL_ABNORMAL"},
  {APP_FAULT_INTERNAL_CONFIGURATION, "INTERNAL_CONFIGURATION"},
  {APP_FAULT_RTOS_STACK_OVERFLOW, "RTOS_STACK_OVERFLOW"},
  {APP_FAULT_RTOS_MALLOC_FAILURE, "RTOS_MALLOC_FAILURE"},
  {APP_FAULT_FDCAN_COMMUNICATION, "FDCAN_COMMUNICATION"}
};

static osMessageQueueId_t g_rx_queue;
static osMessageQueueId_t g_tx_queue;
static uint8_t g_rx_byte;
static char g_rx_line[APP_DIAGNOSTIC_INPUT_LENGTH];
static uint32_t g_rx_length;
static AppDiagnosticBlock g_response;
static AppDiagnosticBlock g_active_tx;
static volatile bool g_tx_busy;
static volatile bool g_rx_overflow_pending;
static volatile uint32_t g_rx_dropped_bytes;
static volatile uint32_t g_uart_error_count;
static uint32_t g_dropped_blocks;
static bool g_response_truncated;
static bool g_discard_input_line;
static bool g_line_too_long_pending;
static AppDiagnosticWatchSource g_watch_source;
static uint32_t g_next_watch_ms;

static void AppDiagnostics_ResponseReset(void)
{
  memset(&g_response, 0, sizeof(g_response));
  g_response_truncated = false;
}

static void AppDiagnostics_ResponseAppend(const char *format, ...)
{
  va_list args;
  size_t available;
  int written;

  if ((format == NULL) || g_response_truncated)
  {
    return;
  }
  available = sizeof(g_response.text) - g_response.length;
  if (available <= 1U)
  {
    g_response_truncated = true;
    return;
  }

  va_start(args, format);
  written = vsnprintf(&g_response.text[g_response.length], available, format, args);
  va_end(args);
  if (written < 0)
  {
    g_response_truncated = true;
  }
  else if ((size_t)written >= available)
  {
    g_response.length = (uint16_t)(sizeof(g_response.text) - 1U);
    g_response.text[g_response.length] = '\0';
    g_response_truncated = true;
  }
  else
  {
    g_response.length = (uint16_t)(g_response.length + (uint16_t)written);
  }
}

static void AppDiagnostics_ResponseStart(const char *line)
{
  AppDiagnostics_ResponseReset();
  AppDiagnostics_ResponseAppend("\r\n> %s\r\n\r\n", (line != NULL) ? line : "");
}

static bool AppDiagnostics_QueueCurrentResponse(bool low_priority)
{
  static const char truncated[] = "\r\nERROR: response truncated\r\n\r\n> ";

  if (g_response_truncated)
  {
    const size_t marker_length = sizeof(truncated) - 1U;
    const size_t offset = sizeof(g_response.text) - marker_length - 1U;
    memcpy(&g_response.text[offset], truncated, marker_length + 1U);
    g_response.length = (uint16_t)(offset + marker_length);
  }
  if ((g_tx_queue == NULL) ||
      (low_priority && (osMessageQueueGetSpace(g_tx_queue) <= 1U)) ||
      (osMessageQueuePut(g_tx_queue, &g_response, 0U, 0U) != osOK))
  {
    g_dropped_blocks++;
    return false;
  }
  return true;
}

static void AppDiagnostics_ResponseFinish(void)
{
  AppDiagnostics_ResponseAppend("\r\n> ");
  (void)AppDiagnostics_QueueCurrentResponse(false);
}

static void AppDiagnostics_I64ToText(int64_t value, char text[22])
{
  char reverse[21];
  uint32_t length = 0U;
  uint32_t output = 0U;
  uint64_t magnitude = (value < 0) ? ((uint64_t)(-(value + 1)) + 1U) : (uint64_t)value;

  do
  {
    reverse[length++] = (char)('0' + (magnitude % 10U));
    magnitude /= 10U;
  } while (magnitude != 0U);

  if (value < 0)
  {
    text[output++] = '-';
  }
  while (length > 0U)
  {
    text[output++] = reverse[--length];
  }
  text[output] = '\0';
}

static uint32_t AppDiagnostics_Power10(uint8_t decimals)
{
  uint32_t value = 1U;
  for (uint8_t index = 0U; index < decimals; ++index)
  {
    value *= 10U;
  }
  return value;
}

static void AppDiagnostics_AppendFixed(float value, uint8_t decimals)
{
  uint32_t scale;
  double scaled;
  int32_t rounded;
  uint32_t magnitude;

  if (!isfinite(value) || (decimals > 6U))
  {
    AppDiagnostics_ResponseAppend("INVALID");
    return;
  }
  scale = AppDiagnostics_Power10(decimals);
  scaled = (double)value * (double)scale;
  if ((scaled < (double)INT32_MIN) || (scaled > (double)INT32_MAX))
  {
    AppDiagnostics_ResponseAppend("OUT_OF_RANGE");
    return;
  }
  rounded = (int32_t)(scaled + ((scaled >= 0.0) ? 0.5 : -0.5));
  magnitude = (rounded < 0) ? (uint32_t)(-(int64_t)rounded) : (uint32_t)rounded;

  if (decimals == 0U)
  {
    AppDiagnostics_ResponseAppend("%s%lu", (rounded < 0) ? "-" : "",
                                  (unsigned long)magnitude);
  }
  else
  {
    AppDiagnostics_ResponseAppend("%s%lu.%0*lu", (rounded < 0) ? "-" : "",
                                  (unsigned long)(magnitude / scale), (int)decimals,
                                  (unsigned long)(magnitude % scale));
  }
}

static bool AppDiagnostics_ParseFloat(const char *text, float *value)
{
  char *end = NULL;
  float parsed;

  if ((text == NULL) || (value == NULL))
  {
    return false;
  }
  parsed = strtof(text, &end);
  if ((end == text) || (*end != '\0') || !isfinite(parsed))
  {
    return false;
  }
  *value = parsed;
  return true;
}

static bool AppDiagnostics_ParseU32(const char *text, uint32_t *value)
{
  char *end = NULL;
  unsigned long parsed;

  if ((text == NULL) || (value == NULL) || (*text == '-'))
  {
    return false;
  }
  parsed = strtoul(text, &end, 10);
  if ((end == text) || (*end != '\0') || (parsed > UINT32_MAX))
  {
    return false;
  }
  *value = (uint32_t)parsed;
  return true;
}

static const char *AppDiagnostics_CanStateName(const AppCanSnapshot *can)
{
  if ((can == NULL) || !can->initialized) { return "NOT_INITIALIZED"; }
  if (can->bus_off) { return "BUS_OFF"; }
  if (can->error_passive) { return "ERROR_PASSIVE"; }
  if (can->error_warning) { return "ERROR_WARNING"; }
  return "ERROR_ACTIVE";
}

static const char *AppDiagnostics_ImuStateName(AppImuState state)
{
  switch (state)
  {
    case APP_IMU_STATE_INITIALIZING: return "INITIALIZING";
    case APP_IMU_STATE_READY: return "READY";
    case APP_IMU_STATE_FAULT: return "FAULT";
    case APP_IMU_STATE_NOT_INITIALIZED:
    default: return "NOT_INITIALIZED";
  }
}

static const char *AppDiagnostics_AuthorityName(const AppCanSnapshot *can,
                                                 const AppCommandSnapshot *command)
{
  if ((can != NULL) && can->authority_armed) { return "CAN"; }
  if ((command != NULL) && command->arm_requested) { return "UART"; }
  return "NONE";
}

static const char *AppDiagnostics_FirstFaultName(uint32_t faults)
{
  for (uint32_t index = 0U; index < (sizeof(g_fault_info) / sizeof(g_fault_info[0])); ++index)
  {
    if ((faults & g_fault_info[index].flag) != 0U)
    {
      return g_fault_info[index].name;
    }
  }
  return "UNKNOWN";
}

static void AppDiagnostics_AppendFaultNames(uint32_t faults)
{
  bool first = true;

  if (faults == 0U)
  {
    AppDiagnostics_ResponseAppend("NONE");
    return;
  }
  for (uint32_t index = 0U; index < (sizeof(g_fault_info) / sizeof(g_fault_info[0])); ++index)
  {
    if ((faults & g_fault_info[index].flag) != 0U)
    {
      AppDiagnostics_ResponseAppend("%s%s", first ? "" : " ",
                                    g_fault_info[index].name);
      first = false;
    }
  }
  if (first)
  {
    AppDiagnostics_ResponseAppend("UNKNOWN");
  }
}

static void AppDiagnostics_PrintStatus(uint32_t now_ms)
{
  AppTelemetrySnapshot telemetry;
  AppCommandSnapshot command;
  AppCanSnapshot can;
  const bool encoders_valid = AppEncoder_AllValid(now_ms);

  AppTelemetry_GetSnapshot(now_ms, &telemetry);
  AppCommand_GetSnapshot(now_ms, &command);
  AppCan_GetSnapshot(now_ms, &can);

  AppDiagnostics_ResponseAppend("FW        : %s\r\n", APP_VERSION_STRING);
  AppDiagnostics_ResponseAppend("State     : %s\r\n",
                                AppState_SystemStateName(telemetry.state.system_state));
  AppDiagnostics_ResponseAppend("Fault     : ");
  if (telemetry.state.fault_flags == 0U)
  {
    AppDiagnostics_ResponseAppend("NONE\r\n");
  }
  else
  {
    AppDiagnostics_ResponseAppend("0x%08lX\r\n",
                                  (unsigned long)telemetry.state.fault_flags);
  }
  AppDiagnostics_ResponseAppend("Motor     : %s\r\n",
                                telemetry.motor.standby_asserted ? "ENABLED" : "DISABLED");
  AppDiagnostics_ResponseAppend("STBY      : %s\r\n",
                                telemetry.motor.standby_asserted ? "HIGH" : "LOW");
  AppDiagnostics_ResponseAppend("Encoder   : %s\r\n", encoders_valid ? "OK" : "INVALID");
  AppDiagnostics_ResponseAppend("IMU       : %s\r\n", telemetry.imu.valid ? "OK" : "INVALID");
  AppDiagnostics_ResponseAppend("CAN       : %s\r\n", AppDiagnostics_CanStateName(&can));
  AppDiagnostics_ResponseAppend("Authority : %s\r\n",
                                AppDiagnostics_AuthorityName(&can, &command));
  AppDiagnostics_ResponseAppend("Body Cmd  : %s\r\n",
                                (AppControl_AllConfigured() &&
                                 AppDrivetrain_BodyCommandReady()) ? "READY" : "NOT READY");
}

static void AppDiagnostics_PrintEncoderLine(const char *name, int64_t total,
                                            float cps, bool valid, uint32_t age_ms)
{
  char total_text[22];
  AppDiagnostics_I64ToText(total, total_text);
  AppDiagnostics_ResponseAppend("%-5s : total=%s cps=", name, total_text);
  AppDiagnostics_AppendFixed(cps, 1U);
  AppDiagnostics_ResponseAppend(" valid=%s age=%lu ms\r\n", valid ? "yes" : "no",
                                (unsigned long)age_ms);
}

static void AppDiagnostics_PrintEncoder(uint32_t now_ms)
{
  AppEncoderSnapshot encoder_1;
  AppEncoderSnapshot encoder_2;
  AppDrivetrainConfig config;
  int64_t left_total;
  int64_t right_total;
  float left_cps;
  float right_cps;
  bool left_valid;
  bool right_valid;
  uint32_t left_age;
  uint32_t right_age;

  AppEncoder_GetSnapshot(APP_ENCODER_1, now_ms, &encoder_1);
  AppEncoder_GetSnapshot(APP_ENCODER_2, now_ms, &encoder_2);
  AppDrivetrain_GetConfig(&config);
  if (!AppDrivetrain_MapEncoderPositions(encoder_1.accumulated_counts,
                                         encoder_2.accumulated_counts,
                                         &left_total, &right_total) ||
      !AppDrivetrain_MapEncoderRates(encoder_1.filtered_counts_per_second,
                                    encoder_2.filtered_counts_per_second,
                                    &left_cps, &right_cps))
  {
    AppDiagnostics_PrintEncoderLine("Enc1", encoder_1.accumulated_counts,
                                    encoder_1.filtered_counts_per_second,
                                    encoder_1.valid, encoder_1.sample_age_ms);
    AppDiagnostics_PrintEncoderLine("Enc2", encoder_2.accumulated_counts,
                                    encoder_2.filtered_counts_per_second,
                                    encoder_2.valid, encoder_2.sample_age_ms);
    AppDiagnostics_ResponseAppend("Mapping: INVALID\r\n");
    return;
  }

  if (config.encoder_1_side == APP_WHEEL_RIGHT)
  {
    right_valid = encoder_1.valid;
    right_age = encoder_1.sample_age_ms;
    left_valid = encoder_2.valid;
    left_age = encoder_2.sample_age_ms;
  }
  else
  {
    left_valid = encoder_1.valid;
    left_age = encoder_1.sample_age_ms;
    right_valid = encoder_2.valid;
    right_age = encoder_2.sample_age_ms;
  }
  AppDiagnostics_PrintEncoderLine("Right", right_total, right_cps, right_valid, right_age);
  AppDiagnostics_PrintEncoderLine("Left", left_total, left_cps, left_valid, left_age);
}

static void AppDiagnostics_PrintImu(uint32_t now_ms)
{
  AppTelemetrySnapshot telemetry;
  AppTelemetry_GetSnapshot(now_ms, &telemetry);

  AppDiagnostics_ResponseAppend("State : %s\r\n", AppDiagnostics_ImuStateName(telemetry.imu.state));
  AppDiagnostics_ResponseAppend("WHO   : 0x%02X\r\n", telemetry.imu.who_am_i);
  AppDiagnostics_ResponseAppend("Valid : %s age=%lu ms\r\n",
                                telemetry.imu.valid ? "yes" : "no",
                                (unsigned long)telemetry.imu.sample_age_ms);
  AppDiagnostics_ResponseAppend("Accel : x=");
  AppDiagnostics_AppendFixed(telemetry.imu.acceleration_mps2[0], 3U);
  AppDiagnostics_ResponseAppend(" y=");
  AppDiagnostics_AppendFixed(telemetry.imu.acceleration_mps2[1], 3U);
  AppDiagnostics_ResponseAppend(" z=");
  AppDiagnostics_AppendFixed(telemetry.imu.acceleration_mps2[2], 3U);
  AppDiagnostics_ResponseAppend(" m/s^2\r\nGyro  : x=");
  AppDiagnostics_AppendFixed(telemetry.imu.angular_velocity_radps[0], 3U);
  AppDiagnostics_ResponseAppend(" y=");
  AppDiagnostics_AppendFixed(telemetry.imu.angular_velocity_radps[1], 3U);
  AppDiagnostics_ResponseAppend(" z=");
  AppDiagnostics_AppendFixed(telemetry.imu.angular_velocity_radps[2], 3U);
  AppDiagnostics_ResponseAppend(" rad/s\r\n");
}

static void AppDiagnostics_PrintBattery(uint32_t now_ms)
{
  AppTelemetrySnapshot telemetry;
  AppTelemetry_GetSnapshot(now_ms, &telemetry);

  AppDiagnostics_ResponseAppend("Valid   : %s age=%lu ms\r\n",
                                telemetry.battery.valid ? "yes" : "no",
                                (unsigned long)telemetry.battery.sample_age_ms);
  AppDiagnostics_ResponseAppend("ADC     : %u counts / ", telemetry.battery.raw_adc);
  AppDiagnostics_AppendFixed(telemetry.battery.adc_voltage, 3U);
  AppDiagnostics_ResponseAppend(" V\r\nBattery : ");
  AppDiagnostics_AppendFixed(telemetry.battery.filtered_battery_voltage, 3U);
  AppDiagnostics_ResponseAppend(" V (%s divider)\r\n",
                                telemetry.battery.divider_calibrated ? "verified" : "commissioning");
}

static void AppDiagnostics_AppendCanAge(const char *label, bool seen, bool fresh,
                                        uint32_t age_ms)
{
  if (!seen)
  {
    AppDiagnostics_ResponseAppend("%-7s : NONE\r\n", label);
  }
  else
  {
    AppDiagnostics_ResponseAppend("%-7s : %s age=%lu ms\r\n", label,
                                  fresh ? "FRESH" : "STALE", (unsigned long)age_ms);
  }
}

static void AppDiagnostics_PrintCan(uint32_t now_ms)
{
  AppCanSnapshot can;
  AppCan_GetSnapshot(now_ms, &can);

  AppDiagnostics_ResponseAppend("State   : %s\r\n", AppDiagnostics_CanStateName(&can));
  AppDiagnostics_ResponseAppend("RX/TX   : %lu / %lu\r\n",
                                (unsigned long)can.rx_frames, (unsigned long)can.tx_frames);
  AppDiagnostics_ResponseAppend("Reject  : %lu  SeqErr=%lu\r\n",
                                (unsigned long)can.rx_rejected,
                                (unsigned long)(can.rx_duplicate + can.rx_out_of_order));
  AppDiagnostics_ResponseAppend("Overflow: %lu  TXFail=%lu  BusOff=%lu\r\n",
                                (unsigned long)can.rx_overflow,
                                (unsigned long)can.tx_failures,
                                (unsigned long)can.bus_off_count);
  if (can.session_active)
  {
    AppDiagnostics_ResponseAppend("Session : 0x%08lX\r\n",
                                  (unsigned long)can.active_session_id);
  }
  else
  {
    AppDiagnostics_ResponseAppend("Session : NONE\r\n");
  }
  AppDiagnostics_AppendCanAge("HB", can.heartbeat_sequence_seen,
                              can.heartbeat_fresh, can.heartbeat_age_ms);
  if (can.authority_sequence_seen)
  {
    const char *state = can.authority_armed ? "ARMED" :
                        (can.authority_fresh ? "DISARMED" : "STALE");
    AppDiagnostics_ResponseAppend("Auth    : %s age=%lu ms\r\n", state,
                                  (unsigned long)can.authority_age_ms);
  }
  else
  {
    AppDiagnostics_ResponseAppend("Auth    : NONE\r\n");
  }
  AppDiagnostics_AppendCanAge("Cmd", can.command_sequence_seen,
                              can.command_sequence_seen &&
                              (can.command_age_ms <= APP_COMMAND_DEFAULT_TIMEOUT_MS),
                              can.command_age_ms);
}

static void AppDiagnostics_PrintFault(void)
{
  const uint32_t faults = AppState_GetFaultFlags();
  AppDiagnostics_ResponseAppend("0x%08lX ", (unsigned long)faults);
  AppDiagnostics_AppendFaultNames(faults);
  AppDiagnostics_ResponseAppend("\r\n");
}

static void AppDiagnostics_PrintHelp(void)
{
  AppDiagnostics_ResponseAppend("help\r\n");
  AppDiagnostics_ResponseAppend("status\r\n");
  AppDiagnostics_ResponseAppend("encoder | imu | battery | can\r\n");
  AppDiagnostics_ResponseAppend("fault | clear\r\n");
  AppDiagnostics_ResponseAppend("arm | disarm | stop\r\n");
  AppDiagnostics_ResponseAppend("motor <a> <b> <duration_ms>\r\n");
  AppDiagnostics_ResponseAppend("watch encoder|imu|can | watch off\r\n");
  AppDiagnostics_ResponseAppend("motor: finite effort -1..+1, duration %u..%u ms\r\n",
                                APP_MOTOR_CONTROL_PERIOD_MS, APP_COMMAND_MAX_TIMEOUT_MS);
}

static bool AppDiagnostics_UartMotionAvailable(uint32_t now_ms)
{
  AppCanSnapshot can;
  AppCan_GetSnapshot(now_ms, &can);
  return !can.authority_armed;
}

static void AppDiagnostics_AppendSyntax(const char *usage)
{
  AppDiagnostics_ResponseAppend("ERROR: syntax\r\nUsage: %s\r\n", usage);
}

static void AppDiagnostics_HandleClear(void)
{
  uint32_t faults;
  (void)AppSafety_ClearRecoverableFaults();
  faults = AppState_GetFaultFlags();
  if (faults == 0U)
  {
    AppDiagnostics_ResponseAppend("OK\r\n");
  }
  else if ((faults & APP_FAULT_ENCODER_VALIDITY) != 0U)
  {
    AppDiagnostics_ResponseAppend("REJECTED: ENCODER_VALIDITY still active\r\n");
  }
  else
  {
    AppDiagnostics_ResponseAppend("REJECTED: %s still active\r\n",
                                  AppDiagnostics_FirstFaultName(faults));
  }
}

static void AppDiagnostics_HandleArm(uint32_t now_ms)
{
  if (!AppDiagnostics_UartMotionAvailable(now_ms))
  {
    AppDiagnostics_ResponseAppend("REJECTED: CAN authority active\r\n");
  }
  else if (AppState_HasCriticalFault())
  {
    AppDiagnostics_ResponseAppend("REJECTED: critical fault active\r\n");
  }
  else
  {
    AppCommand_RequestArm();
    AppDiagnostics_ResponseAppend("OK\r\n");
  }
}

static void AppDiagnostics_HandleStop(void)
{
  AppCommand_Stop();
  AppControl_Reset();
  AppMotor_ForceSafe();
  AppDiagnostics_ResponseAppend("OK\r\n");
}

static void AppDiagnostics_HandleMotor(uint32_t now_ms)
{
  char *a_text = strtok(NULL, " \t");
  char *b_text = strtok(NULL, " \t");
  char *duration_text = strtok(NULL, " \t");
  AppCommandSnapshot command;
  float motor_a;
  float motor_b;
  uint32_t duration_ms;

  if (!AppDiagnostics_ParseFloat(a_text, &motor_a) ||
      !AppDiagnostics_ParseFloat(b_text, &motor_b) ||
      !AppDiagnostics_ParseU32(duration_text, &duration_ms) ||
      (strtok(NULL, " \t") != NULL))
  {
    AppDiagnostics_AppendSyntax("motor <a> <b> <duration_ms>");
    return;
  }
  if ((motor_a < -1.0f) || (motor_a > 1.0f) ||
      (motor_b < -1.0f) || (motor_b > 1.0f))
  {
    AppDiagnostics_ResponseAppend("REJECTED: effort must be -1..+1\r\n");
    return;
  }
  if ((duration_ms < APP_MOTOR_CONTROL_PERIOD_MS) ||
      (duration_ms > APP_COMMAND_MAX_TIMEOUT_MS))
  {
    AppDiagnostics_ResponseAppend("REJECTED: duration must be %u..%u ms\r\n",
                                  APP_MOTOR_CONTROL_PERIOD_MS,
                                  APP_COMMAND_MAX_TIMEOUT_MS);
    return;
  }
  if (!AppDiagnostics_UartMotionAvailable(now_ms))
  {
    AppDiagnostics_ResponseAppend("REJECTED: CAN authority active\r\n");
    return;
  }
  if (AppState_HasCriticalFault())
  {
    AppDiagnostics_ResponseAppend("REJECTED: critical fault active\r\n");
    return;
  }
  AppCommand_GetSnapshot(now_ms, &command);
  if (!command.arm_requested)
  {
    AppDiagnostics_ResponseAppend("REJECTED: arm required\r\n");
    return;
  }
  if (!AppCommand_SubmitMotorEffort(APP_COMMAND_SOURCE_DIAGNOSTIC,
                                    motor_a, motor_b, duration_ms))
  {
    AppDiagnostics_ResponseAppend("REJECTED: command validation failed\r\n");
    return;
  }
  AppDiagnostics_ResponseAppend("OK: motor command accepted (%lu ms)\r\n",
                                (unsigned long)duration_ms);
}

static const char *AppDiagnostics_WatchName(AppDiagnosticWatchSource source)
{
  switch (source)
  {
    case APP_DIAGNOSTIC_WATCH_ENCODER: return "encoder";
    case APP_DIAGNOSTIC_WATCH_IMU: return "imu";
    case APP_DIAGNOSTIC_WATCH_CAN: return "can";
    case APP_DIAGNOSTIC_WATCH_OFF:
    default: return "off";
  }
}

static void AppDiagnostics_HandleWatch(uint32_t now_ms)
{
  char *source = strtok(NULL, " \t");

  if ((source == NULL) || (strtok(NULL, " \t") != NULL))
  {
    AppDiagnostics_AppendSyntax("watch encoder|imu|can | watch off");
    return;
  }
  if (strcmp(source, "off") == 0)
  {
    g_watch_source = APP_DIAGNOSTIC_WATCH_OFF;
    AppDiagnostics_ResponseAppend("OK\r\n");
    return;
  }
  if (strcmp(source, "encoder") == 0)
  {
    g_watch_source = APP_DIAGNOSTIC_WATCH_ENCODER;
  }
  else if (strcmp(source, "imu") == 0)
  {
    g_watch_source = APP_DIAGNOSTIC_WATCH_IMU;
  }
  else if (strcmp(source, "can") == 0)
  {
    g_watch_source = APP_DIAGNOSTIC_WATCH_CAN;
  }
  else
  {
    AppDiagnostics_AppendSyntax("watch encoder|imu|can | watch off");
    return;
  }
  g_next_watch_ms = now_ms + (1000U / APP_DIAGNOSTIC_WATCH_DEFAULT_HZ);
  AppDiagnostics_ResponseAppend("OK: watch %s at %u Hz\r\n",
                                AppDiagnostics_WatchName(g_watch_source),
                                APP_DIAGNOSTIC_WATCH_DEFAULT_HZ);
}

static void AppDiagnostics_HandleLine(char *line, uint32_t now_ms)
{
  char display_line[APP_DIAGNOSTIC_INPUT_LENGTH];
  char *command;

  strncpy(display_line, line, sizeof(display_line) - 1U);
  display_line[sizeof(display_line) - 1U] = '\0';
  command = strtok(line, " \t");
  if ((command == NULL) || (*command == '\0'))
  {
    return;
  }
  AppDiagnostics_ResponseStart(display_line);

  if (strcmp(command, "help") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintHelp(); }
    else { AppDiagnostics_AppendSyntax("help"); }
  }
  else if (strcmp(command, "status") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintStatus(now_ms); }
    else { AppDiagnostics_AppendSyntax("status"); }
  }
  else if (strcmp(command, "encoder") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintEncoder(now_ms); }
    else { AppDiagnostics_AppendSyntax("encoder"); }
  }
  else if (strcmp(command, "imu") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintImu(now_ms); }
    else { AppDiagnostics_AppendSyntax("imu"); }
  }
  else if (strcmp(command, "battery") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintBattery(now_ms); }
    else { AppDiagnostics_AppendSyntax("battery"); }
  }
  else if (strcmp(command, "can") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintCan(now_ms); }
    else { AppDiagnostics_AppendSyntax("can"); }
  }
  else if (strcmp(command, "fault") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_PrintFault(); }
    else { AppDiagnostics_AppendSyntax("fault"); }
  }
  else if (strcmp(command, "clear") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_HandleClear(); }
    else { AppDiagnostics_AppendSyntax("clear"); }
  }
  else if (strcmp(command, "arm") == 0)
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_HandleArm(now_ms); }
    else { AppDiagnostics_AppendSyntax("arm"); }
  }
  else if ((strcmp(command, "disarm") == 0) || (strcmp(command, "stop") == 0))
  {
    if (strtok(NULL, " \t") == NULL) { AppDiagnostics_HandleStop(); }
    else { AppDiagnostics_AppendSyntax(command); }
  }
  else if (strcmp(command, "motor") == 0)
  {
    AppDiagnostics_HandleMotor(now_ms);
  }
  else if (strcmp(command, "watch") == 0)
  {
    AppDiagnostics_HandleWatch(now_ms);
  }
  else
  {
    AppDiagnostics_ResponseAppend("ERROR: unknown command '%s'\r\nHint : type 'help'\r\n",
                                  command);
  }
  AppDiagnostics_ResponseFinish();
}

static bool AppDiagnostics_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void AppDiagnostics_EmitWatch(uint32_t now_ms)
{
  AppDiagnostics_ResponseReset();
  AppDiagnostics_ResponseAppend("\r\n[watch %s]\r\n",
                                AppDiagnostics_WatchName(g_watch_source));
  switch (g_watch_source)
  {
    case APP_DIAGNOSTIC_WATCH_ENCODER: AppDiagnostics_PrintEncoder(now_ms); break;
    case APP_DIAGNOSTIC_WATCH_IMU: AppDiagnostics_PrintImu(now_ms); break;
    case APP_DIAGNOSTIC_WATCH_CAN: AppDiagnostics_PrintCan(now_ms); break;
    case APP_DIAGNOSTIC_WATCH_OFF:
    default: return;
  }
  AppDiagnostics_ResponseAppend("\r\n> ");
  (void)AppDiagnostics_QueueCurrentResponse(true);
}

static void AppDiagnostics_PumpTx(void)
{
  uint32_t key = AppPlatform_IrqLock();
  if (g_tx_busy)
  {
    AppPlatform_IrqUnlock(key);
    return;
  }
  g_tx_busy = true;
  AppPlatform_IrqUnlock(key);

  if (osMessageQueueGet(g_tx_queue, &g_active_tx, NULL, 0U) != osOK)
  {
    g_tx_busy = false;
    return;
  }
  if ((g_active_tx.length == 0U) ||
      (HAL_UART_Transmit_IT(&huart2, (uint8_t *)g_active_tx.text,
                           g_active_tx.length) != HAL_OK))
  {
    g_tx_busy = false;
    g_dropped_blocks++;
  }
}

bool AppDiagnostics_Init(void)
{
  g_rx_queue = osMessageQueueNew(APP_DIAGNOSTIC_RX_QUEUE_DEPTH, sizeof(uint8_t), NULL);
  g_tx_queue = osMessageQueueNew(APP_DIAGNOSTIC_TX_QUEUE_DEPTH,
                                 sizeof(AppDiagnosticBlock), NULL);
  g_rx_length = 0U;
  g_tx_busy = false;
  g_rx_overflow_pending = false;
  g_rx_dropped_bytes = 0U;
  g_uart_error_count = 0U;
  g_dropped_blocks = 0U;
  g_response_truncated = false;
  g_discard_input_line = false;
  g_line_too_long_pending = false;
  g_watch_source = APP_DIAGNOSTIC_WATCH_OFF;
  g_next_watch_ms = 0U;
  if ((g_rx_queue == NULL) || (g_tx_queue == NULL)) { return false; }
  if (HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1U) != HAL_OK) { return false; }

  AppDiagnostics_ResponseReset();
  AppDiagnostics_ResponseAppend("RobotProject STM32 boot\r\n");
  AppDiagnostics_ResponseAppend("FW %s / Protocol %u.%u\r\n",
                                APP_VERSION_STRING,
                                APP_PROTOCOL_VERSION_MAJOR,
                                APP_PROTOCOL_VERSION_MINOR);
  AppDiagnostics_ResponseAppend("Quiet console; type 'help'\r\n\r\n> ");
  return AppDiagnostics_QueueCurrentResponse(false);
}

void AppDiagnostics_Process(uint32_t now_ms)
{
  uint8_t byte;
  bool command_processed = false;

  if (g_rx_overflow_pending)
  {
    const uint32_t key = AppPlatform_IrqLock();
    g_rx_overflow_pending = false;
    AppPlatform_IrqUnlock(key);
    g_rx_length = 0U;
    g_discard_input_line = true;
    AppDiagnostics_ResponseStart("<input overflow>");
    AppDiagnostics_ResponseAppend("ERROR: UART input overflow; resend command\r\n");
    AppDiagnostics_ResponseFinish();
    command_processed = true;
  }

  while (osMessageQueueGet(g_rx_queue, &byte, NULL, 0U) == osOK)
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (g_discard_input_line)
      {
        g_discard_input_line = false;
        if (g_line_too_long_pending)
        {
          AppDiagnostics_ResponseStart("<line too long>");
          AppDiagnostics_ResponseAppend("ERROR: command too long\r\n");
          AppDiagnostics_ResponseFinish();
          g_line_too_long_pending = false;
          command_processed = true;
        }
      }
      else if (g_rx_length > 0U)
      {
        g_rx_line[g_rx_length] = '\0';
        AppDiagnostics_HandleLine(g_rx_line, now_ms);
        g_rx_length = 0U;
        command_processed = true;
      }
    }
    else if (!g_discard_input_line && (byte >= 0x20U) && (byte <= 0x7EU))
    {
      if (g_rx_length < (sizeof(g_rx_line) - 1U))
      {
        g_rx_line[g_rx_length++] = (char)byte;
      }
      else
      {
        g_rx_length = 0U;
        g_discard_input_line = true;
        g_line_too_long_pending = true;
      }
    }
  }

  if (!command_processed && (g_watch_source != APP_DIAGNOSTIC_WATCH_OFF) &&
      AppDiagnostics_TimeReached(now_ms, g_next_watch_ms))
  {
    AppDiagnostics_EmitWatch(now_ms);
    g_next_watch_ms = now_ms + (1000U / APP_DIAGNOSTIC_WATCH_DEFAULT_HZ);
  }
  AppDiagnostics_PumpTx();
}

void AppDiagnostics_UartRxComplete(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2) && (g_rx_queue != NULL))
  {
    if (osMessageQueuePut(g_rx_queue, &g_rx_byte, 0U, 0U) != osOK)
    {
      g_rx_dropped_bytes++;
      g_rx_overflow_pending = true;
    }
    (void)HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1U);
  }
}

void AppDiagnostics_UartTxComplete(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    g_tx_busy = false;
  }
}

void AppDiagnostics_UartError(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2))
  {
    g_uart_error_count++;
    g_tx_busy = false;
    (void)HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1U);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  AppDiagnostics_UartRxComplete(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  AppDiagnostics_UartTxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  AppDiagnostics_UartError(huart);
}
