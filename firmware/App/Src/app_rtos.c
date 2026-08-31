#include "app_rtos.h"

#include "app_battery.h"
#include "app_can.h"
#include "app_command.h"
#include "app_config.h"
#include "app_control.h"
#include "app_diagnostics.h"
#include "app_drivetrain.h"
#include "app_encoder.h"
#include "app_imu.h"
#include "app_motor.h"
#include "app_platform.h"
#include "app_safety.h"
#include "app_state.h"
#include "app_supervisor.h"
#include "cmsis_os2.h"

#include <stdint.h>

static osThreadId_t g_motor_task;
static osThreadId_t g_imu_task;
static osThreadId_t g_communication_task;
static osThreadId_t g_telemetry_task;

static void AppRtos_MotorTask(void *argument);
static void AppRtos_ImuTask(void *argument);
static void AppRtos_CommunicationTask(void *argument);
static void AppRtos_TelemetryTask(void *argument);

static const osThreadAttr_t g_motor_task_attributes = {
  .name = "MotorControl",
  .priority = osPriorityHigh,
  .stack_size = 1024U
};

static const osThreadAttr_t g_imu_task_attributes = {
  .name = "Imu",
  .priority = osPriorityAboveNormal,
  .stack_size = 1024U
};

static const osThreadAttr_t g_communication_task_attributes = {
  .name = "Communication",
  .priority = osPriorityNormal,
  .stack_size = 2048U
};

static const osThreadAttr_t g_telemetry_task_attributes = {
  .name = "Telemetry",
  .priority = osPriorityBelowNormal,
  .stack_size = 1024U
};

bool AppRtos_InitObjects(void)
{
  if (!AppDiagnostics_Init())
  {
    AppState_SetFault(APP_FAULT_RTOS_MALLOC_FAILURE);
    return false;
  }
  if (!AppCan_Init())
  {
    return false;
  }
  return true;
}

bool AppRtos_CreateThreads(void)
{
  g_motor_task = osThreadNew(AppRtos_MotorTask, NULL, &g_motor_task_attributes);
  g_imu_task = osThreadNew(AppRtos_ImuTask, NULL, &g_imu_task_attributes);
  g_communication_task = osThreadNew(AppRtos_CommunicationTask, NULL,
                                      &g_communication_task_attributes);
  g_telemetry_task = osThreadNew(AppRtos_TelemetryTask, NULL, &g_telemetry_task_attributes);
  AppImu_RegisterTask(g_imu_task);
  AppCan_RegisterTask(g_communication_task);

  if ((g_motor_task == NULL) || (g_imu_task == NULL) ||
      (g_communication_task == NULL) || (g_telemetry_task == NULL))
  {
    AppSafety_ForceFault(APP_FAULT_RTOS_MALLOC_FAILURE);
    return false;
  }
  return true;
}

static void AppRtos_MotorTask(void *argument)
{
  uint32_t next_tick = osKernelGetTickCount();
  (void)argument;

  for (;;)
  {
    AppCommandSnapshot command;
    AppEncoderSnapshot encoder_1;
    AppEncoderSnapshot encoder_2;
    float left_measured;
    float right_measured;
    float left_target;
    float right_target;
    float left_output;
    float right_output;
    float motor_a;
    float motor_b;
    const uint32_t now_ms = HAL_GetTick();

    AppEncoder_Sample(now_ms);
    if (!AppSafety_ValidateActiveCommand(now_ms, &command))
    {
      AppControl_Reset();
      AppMotor_ForceSafe();
    }
    else if (command.mode == APP_COMMAND_MODE_MOTOR_EFFORT)
    {
      (void)AppMotor_ApplyEffort(command.first, command.second);
    }
    else
    {
      AppEncoder_GetSnapshot(APP_ENCODER_1, now_ms, &encoder_1);
      AppEncoder_GetSnapshot(APP_ENCODER_2, now_ms, &encoder_2);
      if (!encoder_1.valid || !encoder_2.valid ||
          !AppDrivetrain_MapEncoderRates(encoder_1.filtered_counts_per_second,
                                         encoder_2.filtered_counts_per_second,
                                         &left_measured, &right_measured))
      {
        AppSafety_ForceFault(APP_FAULT_ENCODER_VALIDITY | APP_FAULT_CONTROL_ABNORMAL);
      }
      else
      {
        left_target = command.first;
        right_target = command.second;
        if ((command.mode == APP_COMMAND_MODE_BODY_VELOCITY) &&
            !AppDrivetrain_BodyToEncoderRates(command.first, command.second,
                                              &left_target, &right_target))
        {
          AppSafety_ForceFault(APP_FAULT_INVALID_MOTOR_COMMAND);
        }
        else if (!AppControl_Update(left_target, right_target, left_measured, right_measured,
                                    (float)APP_MOTOR_CONTROL_PERIOD_MS / 1000.0f,
                                    &left_output, &right_output) ||
                 !AppDrivetrain_MapWheelEfforts(left_output, right_output,
                                                &motor_a, &motor_b) ||
                 !AppMotor_ApplyEffort(motor_a, motor_b))
        {
          AppSafety_ForceFault(APP_FAULT_CONTROL_ABNORMAL);
        }
      }
    }

    AppSupervisor_ReportTask(APP_RUNTIME_TASK_MOTOR, now_ms);
    next_tick += APP_MOTOR_CONTROL_PERIOD_MS;
    (void)osDelayUntil(next_tick);
  }
}

static void AppRtos_ImuTask(void *argument)
{
  (void)argument;
  AppImu_RegisterTask(osThreadGetId());
  for (;;)
  {
    uint32_t now_ms;
    (void)osThreadFlagsWait(0x7FFFFFFFUL, osFlagsWaitAny, APP_IMU_TASK_WAIT_MS);
    now_ms = HAL_GetTick();
    (void)AppImu_Service(now_ms);
    AppSupervisor_ReportTask(APP_RUNTIME_TASK_IMU, now_ms);
  }
}

static void AppRtos_CommunicationTask(void *argument)
{
  (void)argument;

  for (;;)
  {
    const uint32_t now_ms = HAL_GetTick();
    AppCan_Process(now_ms);
    AppDiagnostics_Process(now_ms);
    AppSupervisor_ReportTask(APP_RUNTIME_TASK_COMMUNICATION, now_ms);
    (void)osThreadFlagsWait(APP_CAN_TASK_FLAG_EVENT, osFlagsWaitAny,
                            APP_COMMUNICATION_PERIOD_MS);
  }
}

static void AppRtos_TelemetryTask(void *argument)
{
  uint32_t next_tick = osKernelGetTickCount();
  uint32_t last_battery_ms = 0U;
  (void)argument;

  for (;;)
  {
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - last_battery_ms) >= APP_BATTERY_SAMPLE_PERIOD_MS)
    {
      (void)AppBattery_Sample(now_ms);
      last_battery_ms = now_ms;
    }
    AppSupervisor_ReportTask(APP_RUNTIME_TASK_TELEMETRY, now_ms);
    next_tick += APP_TELEMETRY_PERIOD_MS;
    (void)osDelayUntil(next_tick);
  }
}

void AppRtos_SupervisorTask(void *argument)
{
  uint32_t next_tick = osKernelGetTickCount();
  (void)argument;

  for (;;)
  {
    AppSupervisor_RunCycle(HAL_GetTick());
    next_tick += APP_SUPERVISOR_PERIOD_MS;
    (void)osDelayUntil(next_tick);
  }
}

void AppRtos_AssertFailed(const char *file, uint32_t line)
{
  (void)file;
  (void)line;
  AppMotor_EmergencySafe();
  AppState_SetFault(APP_FAULT_INTERNAL_CONFIGURATION);
  __disable_irq();
  for (;;)
  {
  }
}
