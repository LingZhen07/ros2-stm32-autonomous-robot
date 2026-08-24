#include "app_diagnostics.h"

#include "app_battery.h"
#include "app_command.h"
#include "app_config.h"
#include "app_control.h"
#include "app_drivetrain.h"
#include "app_platform.h"
#include "app_safety.h"
#include "app_state.h"
#include "app_telemetry.h"
#include "cmsis_os2.h"
#include "usart.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
  char text[APP_DIAGNOSTIC_LINE_LENGTH];
} AppDiagnosticLine;

static osMessageQueueId_t g_rx_queue;
static osMessageQueueId_t g_tx_queue;
static uint8_t g_rx_byte;
static char g_rx_line[APP_DIAGNOSTIC_LINE_LENGTH];
static uint32_t g_rx_length;
static AppDiagnosticLine g_active_tx;
static volatile bool g_tx_busy;
static volatile uint32_t g_dropped_lines;

static int32_t AppDiagnostics_Scaled(float value, float scale)
{
  if (!isfinite(value))
  {
    return 0;
  }
  return (int32_t)(value * scale);
}

static bool AppDiagnostics_Queue(const char *format, ...)
{
  AppDiagnosticLine line;
  va_list args;
  int length;

  if ((g_tx_queue == NULL) || (format == NULL))
  {
    return false;
  }

  va_start(args, format);
  length = vsnprintf(line.text, sizeof(line.text), format, args);
  va_end(args);
  if (length < 0)
  {
    return false;
  }

  line.text[sizeof(line.text) - 1U] = '\0';
  if (osMessageQueuePut(g_tx_queue, &line, 0U, 0U) != osOK)
  {
    g_dropped_lines++;
    return false;
  }
  return true;
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

  if ((text == NULL) || (value == NULL))
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

static void AppDiagnostics_PrintStatus(void)
{
  AppTelemetrySnapshot telemetry;
  AppCommandSnapshot command;
  const uint32_t now_ms = osKernelGetTickCount();

  AppTelemetry_GetSnapshot(now_ms, &telemetry);
  AppCommand_GetSnapshot(now_ms, &command);
  (void)AppDiagnostics_Queue(
    "fw=%s state=%s fault=0x%08lx reset=0x%08lx runtime_ms=%lu\r\n",
    APP_VERSION_STRING, AppState_SystemStateName(telemetry.state.system_state),
    (unsigned long)telemetry.state.fault_flags, (unsigned long)telemetry.state.reset_reason,
    (unsigned long)telemetry.runtime_ms);
  (void)AppDiagnostics_Queue(
    "safety arm=%u cmd=%u mode=%u fresh=%u seq=%lu motor_en=%u effort_milli=%ld,%ld\r\n",
    command.arm_requested ? 1U : 0U, command.valid ? 1U : 0U, (unsigned int)command.mode,
    command.fresh ? 1U : 0U, (unsigned long)command.sequence,
    telemetry.motor.standby_asserted ? 1U : 0U,
    (long)AppDiagnostics_Scaled(telemetry.motor.motor_a_effort, 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.motor.motor_b_effort, 1000.0f));
  (void)AppDiagnostics_Queue(
    "battery valid=%u raw=%u adc_mv=%ld estimate_mv=%ld filtered_mv=%ld divider_verified=%u\r\n",
    telemetry.battery.valid ? 1U : 0U, telemetry.battery.raw_adc,
    (long)AppDiagnostics_Scaled(telemetry.battery.adc_voltage, 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.battery.estimated_battery_voltage, 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.battery.filtered_battery_voltage, 1000.0f),
    telemetry.battery.divider_calibrated ? 1U : 0U);
  (void)AppDiagnostics_Queue(
    "encoder e1_raw=%u e1_cps=%ld e1_valid=%u e2_raw=%u e2_cps=%ld e2_valid=%u\r\n",
    telemetry.encoder_1.raw_counter,
    (long)AppDiagnostics_Scaled(telemetry.encoder_1.filtered_counts_per_second, 1.0f),
    telemetry.encoder_1.valid ? 1U : 0U, telemetry.encoder_2.raw_counter,
    (long)AppDiagnostics_Scaled(telemetry.encoder_2.filtered_counts_per_second, 1.0f),
    telemetry.encoder_2.valid ? 1U : 0U);
  (void)AppDiagnostics_Queue(
    "imu state=%u who=0x%02x valid=%u accel_milli=%ld,%ld,%ld gyro_milli=%ld,%ld,%ld irq=%lu,%lu\r\n",
    (unsigned int)telemetry.imu.state, telemetry.imu.who_am_i, telemetry.imu.valid ? 1U : 0U,
    (long)AppDiagnostics_Scaled(telemetry.imu.acceleration_mps2[0], 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.imu.acceleration_mps2[1], 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.imu.acceleration_mps2[2], 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.imu.angular_velocity_radps[0], 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.imu.angular_velocity_radps[1], 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.imu.angular_velocity_radps[2], 1000.0f),
    (unsigned long)telemetry.imu.interrupt_1_count,
    (unsigned long)telemetry.imu.interrupt_2_count);
  (void)AppDiagnostics_Queue(
    "controller left_cfg=%u target=%ld measured=%ld out_milli=%ld right_cfg=%u target=%ld measured=%ld out_milli=%ld\r\n",
    telemetry.left_controller.configured ? 1U : 0U,
    (long)AppDiagnostics_Scaled(telemetry.left_controller.target, 1.0f),
    (long)AppDiagnostics_Scaled(telemetry.left_controller.measured, 1.0f),
    (long)AppDiagnostics_Scaled(telemetry.left_controller.output, 1000.0f),
    telemetry.right_controller.configured ? 1U : 0U,
    (long)AppDiagnostics_Scaled(telemetry.right_controller.target, 1.0f),
    (long)AppDiagnostics_Scaled(telemetry.right_controller.measured, 1.0f),
    (long)AppDiagnostics_Scaled(telemetry.right_controller.output, 1000.0f));
  (void)AppDiagnostics_Queue(
    "supervisor healthy=%u iwdg_feed=%u last_feed_ms=%lu dropped_logs=%lu\r\n",
    telemetry.supervisor.critical_tasks_healthy ? 1U : 0U,
    telemetry.supervisor.watchdog_refresh_allowed ? 1U : 0U,
    (unsigned long)telemetry.supervisor.last_watchdog_refresh_ms,
    (unsigned long)g_dropped_lines);
}

static void AppDiagnostics_PrintDrivetrain(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);
  (void)AppDiagnostics_Queue(
    "drive wheel_ready=%u body_ready=%u motor_sides=%u,%u encoder_sides=%u,%u signs=%d,%d,%d,%d\r\n",
    AppDrivetrain_WheelControlReady() ? 1U : 0U,
    AppDrivetrain_BodyCommandReady() ? 1U : 0U,
    (unsigned int)config.motor_a_side, (unsigned int)config.motor_b_side,
    (unsigned int)config.encoder_1_side, (unsigned int)config.encoder_2_side,
    config.motor_a_forward_sign, config.motor_b_forward_sign,
    config.encoder_1_forward_sign, config.encoder_2_forward_sign);
  (void)AppDiagnostics_Queue(
    "drive scale_um=%ld track_um=%ld encoder_counts_milli=%ld gear_milli=%ld\r\n",
    (long)AppDiagnostics_Scaled(config.effective_wheel_radius_m, 1000000.0f),
    (long)AppDiagnostics_Scaled(config.wheel_track_m, 1000000.0f),
    (long)AppDiagnostics_Scaled(config.encoder_counts_per_motor_revolution, 1000.0f),
    (long)AppDiagnostics_Scaled(config.motor_to_wheel_gear_ratio, 1000.0f));
}

static void AppDiagnostics_HandlePid(char *side_text)
{
  char *kp_text = strtok(NULL, " \t");
  char *ki_text = strtok(NULL, " \t");
  char *kd_text = strtok(NULL, " \t");
  char *integrator_text = strtok(NULL, " \t");
  char *output_text = strtok(NULL, " \t");
  AppControllerConfig config;
  AppControllerId id;

  if ((side_text == NULL) || (strcmp(side_text, "left") != 0 && strcmp(side_text, "right") != 0) ||
      !AppDiagnostics_ParseFloat(kp_text, &config.kp) ||
      !AppDiagnostics_ParseFloat(ki_text, &config.ki) ||
      !AppDiagnostics_ParseFloat(kd_text, &config.kd) ||
      !AppDiagnostics_ParseFloat(integrator_text, &config.integrator_limit) ||
      !AppDiagnostics_ParseFloat(output_text, &config.output_limit))
  {
    (void)AppDiagnostics_Queue("ERR usage: pid left|right kp ki kd integrator_limit output_limit\r\n");
    return;
  }
  config.configured = true;
  id = (strcmp(side_text, "left") == 0) ? APP_CONTROLLER_LEFT : APP_CONTROLLER_RIGHT;
  (void)AppDiagnostics_Queue(AppControl_SetConfig(id, &config) ? "OK pid configured\r\n" :
                                                               "ERR invalid pid configuration\r\n");
}

static void AppDiagnostics_HandleDrive(char *subcommand)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  if ((subcommand != NULL) && (strcmp(subcommand, "map") == 0))
  {
    char *motor_a_side = strtok(NULL, " \t");
    char *encoder_1_side = strtok(NULL, " \t");
    char *motor_a_sign = strtok(NULL, " \t");
    char *motor_b_sign = strtok(NULL, " \t");
    char *encoder_1_sign = strtok(NULL, " \t");
    char *encoder_2_sign = strtok(NULL, " \t");
    long signs[4];
    char *end = NULL;

    if ((motor_a_side == NULL) || (encoder_1_side == NULL) ||
        ((strcmp(motor_a_side, "left") != 0) && (strcmp(motor_a_side, "right") != 0)) ||
        ((strcmp(encoder_1_side, "left") != 0) && (strcmp(encoder_1_side, "right") != 0)) ||
        (motor_a_sign == NULL) || (motor_b_sign == NULL) ||
        (encoder_1_sign == NULL) || (encoder_2_sign == NULL))
    {
      (void)AppDiagnostics_Queue("ERR usage: drive map motorA_side encoder1_side ma mb e1 e2 (signs +/-1)\r\n");
      return;
    }
    signs[0] = strtol(motor_a_sign, &end, 10);
    if (*end != '\0') { goto invalid_map; }
    signs[1] = strtol(motor_b_sign, &end, 10);
    if (*end != '\0') { goto invalid_map; }
    signs[2] = strtol(encoder_1_sign, &end, 10);
    if (*end != '\0') { goto invalid_map; }
    signs[3] = strtol(encoder_2_sign, &end, 10);
    if (*end != '\0') { goto invalid_map; }
    for (uint32_t index = 0U; index < 4U; ++index)
    {
      if ((signs[index] != -1L) && (signs[index] != 1L)) { goto invalid_map; }
    }

    config.motor_a_side = (strcmp(motor_a_side, "left") == 0) ? APP_WHEEL_LEFT : APP_WHEEL_RIGHT;
    config.motor_b_side = (config.motor_a_side == APP_WHEEL_LEFT) ? APP_WHEEL_RIGHT : APP_WHEEL_LEFT;
    config.encoder_1_side = (strcmp(encoder_1_side, "left") == 0) ? APP_WHEEL_LEFT : APP_WHEEL_RIGHT;
    config.encoder_2_side = (config.encoder_1_side == APP_WHEEL_LEFT) ? APP_WHEEL_RIGHT : APP_WHEEL_LEFT;
    config.motor_a_forward_sign = (int8_t)signs[0];
    config.motor_b_forward_sign = (int8_t)signs[1];
    config.encoder_1_forward_sign = (int8_t)signs[2];
    config.encoder_2_forward_sign = (int8_t)signs[3];
    (void)AppDrivetrain_SetConfig(&config);
    (void)AppDiagnostics_Queue("OK drivetrain mapping set in volatile commissioning configuration\r\n");
    return;

invalid_map:
    (void)AppDiagnostics_Queue("ERR invalid drivetrain mapping\r\n");
    return;
  }

  if ((subcommand != NULL) && (strcmp(subcommand, "scale") == 0))
  {
    char *radius = strtok(NULL, " \t");
    char *track = strtok(NULL, " \t");
    char *counts = strtok(NULL, " \t");
    char *gear = strtok(NULL, " \t");
    if (!AppDiagnostics_ParseFloat(radius, &config.effective_wheel_radius_m) ||
        !AppDiagnostics_ParseFloat(track, &config.wheel_track_m) ||
        !AppDiagnostics_ParseFloat(counts, &config.encoder_counts_per_motor_revolution) ||
        !AppDiagnostics_ParseFloat(gear, &config.motor_to_wheel_gear_ratio) ||
        (config.effective_wheel_radius_m <= 0.0f) || (config.wheel_track_m <= 0.0f) ||
        (config.encoder_counts_per_motor_revolution <= 0.0f) ||
        (config.motor_to_wheel_gear_ratio <= 0.0f))
    {
      (void)AppDiagnostics_Queue("ERR usage: drive scale radius_m track_m decoded_counts_per_motor_rev gear_ratio\r\n");
      return;
    }
    (void)AppDrivetrain_SetConfig(&config);
    (void)AppDiagnostics_Queue("OK drivetrain scale set in volatile commissioning configuration\r\n");
    return;
  }

  AppDiagnostics_PrintDrivetrain();
}

static void AppDiagnostics_HandleLine(char *line)
{
  char *command = strtok(line, " \t");

  if ((command == NULL) || (*command == '\0'))
  {
    return;
  }
  if (strcmp(command, "status") == 0)
  {
    AppDiagnostics_PrintStatus();
  }
  else if (strcmp(command, "arm") == 0)
  {
    if (AppState_HasCriticalFault())
    {
      (void)AppDiagnostics_Queue("ERR clear critical fault before arm\r\n");
    }
    else
    {
      AppCommand_RequestArm();
      (void)AppDiagnostics_Queue("OK arm requested; fresh valid command still required\r\n");
    }
  }
  else if ((strcmp(command, "disarm") == 0) || (strcmp(command, "stop") == 0))
  {
    AppCommand_Stop();
    (void)AppDiagnostics_Queue("OK motor domain disarmed\r\n");
  }
  else if (strcmp(command, "clear") == 0)
  {
    (void)AppDiagnostics_Queue(AppSafety_ClearRecoverableFaults() ? "OK recoverable faults cleared\r\n" :
                                                                   "ERR non-recoverable fault remains\r\n");
  }
  else if (strcmp(command, "motor") == 0)
  {
    float motor_a;
    float motor_b;
    uint32_t timeout_ms = APP_COMMAND_DEFAULT_TIMEOUT_MS;
    char *a_text = strtok(NULL, " \t");
    char *b_text = strtok(NULL, " \t");
    char *timeout_text = strtok(NULL, " \t");
    if (!AppDiagnostics_ParseFloat(a_text, &motor_a) ||
        !AppDiagnostics_ParseFloat(b_text, &motor_b) ||
        ((timeout_text != NULL) && !AppDiagnostics_ParseU32(timeout_text, &timeout_ms)))
    {
      (void)AppDiagnostics_Queue("ERR usage: motor effortA effortB [timeout_ms], range -1..1\r\n");
    }
    else
    {
      (void)AppDiagnostics_Queue(
        AppCommand_SubmitMotorEffort(APP_COMMAND_SOURCE_DIAGNOSTIC, motor_a, motor_b, timeout_ms)
          ? "OK motor effort accepted by shared command path\r\n"
          : "ERR motor effort rejected\r\n");
    }
  }
  else if (strcmp(command, "wheel") == 0)
  {
    float left;
    float right;
    uint32_t timeout_ms = APP_COMMAND_DEFAULT_TIMEOUT_MS;
    char *left_text = strtok(NULL, " \t");
    char *right_text = strtok(NULL, " \t");
    char *timeout_text = strtok(NULL, " \t");
    if (!AppDiagnostics_ParseFloat(left_text, &left) ||
        !AppDiagnostics_ParseFloat(right_text, &right) ||
        ((timeout_text != NULL) && !AppDiagnostics_ParseU32(timeout_text, &timeout_ms)))
    {
      (void)AppDiagnostics_Queue("ERR usage: wheel left_cps right_cps [timeout_ms]\r\n");
    }
    else
    {
      (void)AppDiagnostics_Queue(
        AppCommand_SubmitWheelRate(APP_COMMAND_SOURCE_DIAGNOSTIC, left, right, timeout_ms)
          ? "OK wheel command accepted; configuration guards apply\r\n"
          : "ERR wheel command rejected\r\n");
    }
  }
  else if (strcmp(command, "body") == 0)
  {
    float linear;
    float angular;
    uint32_t timeout_ms = APP_COMMAND_DEFAULT_TIMEOUT_MS;
    char *linear_text = strtok(NULL, " \t");
    char *angular_text = strtok(NULL, " \t");
    char *timeout_text = strtok(NULL, " \t");
    if (!AppDiagnostics_ParseFloat(linear_text, &linear) ||
        !AppDiagnostics_ParseFloat(angular_text, &angular) ||
        ((timeout_text != NULL) && !AppDiagnostics_ParseU32(timeout_text, &timeout_ms)))
    {
      (void)AppDiagnostics_Queue("ERR usage: body linear_mps angular_radps [timeout_ms]\r\n");
    }
    else
    {
      (void)AppDiagnostics_Queue(
        AppCommand_SubmitBodyVelocity(APP_COMMAND_SOURCE_DIAGNOSTIC, linear, angular, timeout_ms)
          ? "OK body command accepted; geometry guards apply\r\n"
          : "ERR body command rejected\r\n");
    }
  }
  else if (strcmp(command, "pid") == 0)
  {
    AppDiagnostics_HandlePid(strtok(NULL, " \t"));
  }
  else if (strcmp(command, "drive") == 0)
  {
    AppDiagnostics_HandleDrive(strtok(NULL, " \t"));
  }
  else if ((strcmp(command, "encoder") == 0) || (strcmp(command, "imu") == 0) ||
           (strcmp(command, "battery") == 0) || (strcmp(command, "controller") == 0))
  {
    AppDiagnostics_PrintStatus();
  }
  else if (strcmp(command, "help") == 0)
  {
    (void)AppDiagnostics_Queue("cmd: status arm disarm stop clear motor wheel body pid drive encoder imu battery controller help\r\n");
  }
  else
  {
    (void)AppDiagnostics_Queue("ERR unknown command; use help\r\n");
  }
}

static void AppDiagnostics_PumpTx(void)
{
  uint32_t key;
  size_t length;

  key = AppPlatform_IrqLock();
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
  length = strnlen(g_active_tx.text, sizeof(g_active_tx.text));
  if ((length == 0U) || (HAL_UART_Transmit_IT(&huart2, (uint8_t *)g_active_tx.text,
                                             (uint16_t)length) != HAL_OK))
  {
    g_tx_busy = false;
    g_dropped_lines++;
  }
}

bool AppDiagnostics_Init(void)
{
  g_rx_queue = osMessageQueueNew(APP_DIAGNOSTIC_RX_QUEUE_DEPTH, sizeof(uint8_t), NULL);
  g_tx_queue = osMessageQueueNew(APP_DIAGNOSTIC_TX_QUEUE_DEPTH,
                                 sizeof(AppDiagnosticLine), NULL);
  g_rx_length = 0U;
  g_tx_busy = false;
  g_dropped_lines = 0U;
  if ((g_rx_queue == NULL) || (g_tx_queue == NULL))
  {
    return false;
  }
  if (HAL_UART_Receive_IT(&huart2, &g_rx_byte, 1U) != HAL_OK)
  {
    return false;
  }
  (void)AppDiagnostics_Queue("RobotProject STM32 boot\r\n");
  (void)AppDiagnostics_Queue("firmware=%s state=INIT type=help for commands\r\n", APP_VERSION_STRING);
  return true;
}

void AppDiagnostics_Process(void)
{
  uint8_t byte;

  while (osMessageQueueGet(g_rx_queue, &byte, NULL, 0U) == osOK)
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (g_rx_length > 0U)
      {
        g_rx_line[g_rx_length] = '\0';
        AppDiagnostics_HandleLine(g_rx_line);
        g_rx_length = 0U;
      }
    }
    else if ((byte >= 0x20U) && (byte <= 0x7EU))
    {
      if (g_rx_length < (sizeof(g_rx_line) - 1U))
      {
        g_rx_line[g_rx_length++] = (char)byte;
      }
      else
      {
        g_rx_length = 0U;
        (void)AppDiagnostics_Queue("ERR input line too long\r\n");
      }
    }
  }
  AppDiagnostics_PumpTx();
}

void AppDiagnostics_EmitPeriodic(uint32_t now_ms)
{
  AppTelemetrySnapshot telemetry;
  AppTelemetry_GetSnapshot(now_ms, &telemetry);
  (void)AppDiagnostics_Queue(
    "tick=%lu state=%s fault=0x%08lx batt_mv=%ld enc_cps=%ld,%ld imu=%u motor=%u wd=%u\r\n",
    (unsigned long)now_ms, AppState_SystemStateName(telemetry.state.system_state),
    (unsigned long)telemetry.state.fault_flags,
    (long)AppDiagnostics_Scaled(telemetry.battery.filtered_battery_voltage, 1000.0f),
    (long)AppDiagnostics_Scaled(telemetry.encoder_1.filtered_counts_per_second, 1.0f),
    (long)AppDiagnostics_Scaled(telemetry.encoder_2.filtered_counts_per_second, 1.0f),
    telemetry.imu.valid ? 1U : 0U, telemetry.motor.standby_asserted ? 1U : 0U,
    telemetry.supervisor.watchdog_refresh_allowed ? 1U : 0U);
}

void AppDiagnostics_UartRxComplete(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART2) && (g_rx_queue != NULL))
  {
    (void)osMessageQueuePut(g_rx_queue, &g_rx_byte, 0U, 0U);
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
