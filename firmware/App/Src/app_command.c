#include "app_command.h"

#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"

#include <math.h>
#include <string.h>

static AppCommandSnapshot g_command;

static bool AppCommand_Submit(AppCommandSource source, AppCommandMode mode,
                              float first, float second, uint32_t timeout_ms)
{
  uint32_t key;

  if ((source == APP_COMMAND_SOURCE_NONE) || (mode == APP_COMMAND_MODE_NONE) ||
      !isfinite(first) || !isfinite(second) || (timeout_ms == 0U) ||
      (timeout_ms > APP_COMMAND_MAX_TIMEOUT_MS))
  {
    AppState_SetFault(APP_FAULT_INVALID_MOTOR_COMMAND);
    return false;
  }

  key = AppPlatform_IrqLock();
  g_command.source = source;
  g_command.mode = mode;
  g_command.valid = true;
  g_command.fresh = true;
  g_command.timed_out = false;
  g_command.sequence++;
  g_command.received_ms = HAL_GetTick();
  g_command.timeout_ms = timeout_ms;
  g_command.first = first;
  g_command.second = second;
  AppPlatform_IrqUnlock(key);
  return true;
}

void AppCommand_Init(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  memset(&g_command, 0, sizeof(g_command));
  g_command.source = APP_COMMAND_SOURCE_NONE;
  g_command.mode = APP_COMMAND_MODE_NONE;
  g_command.timeout_ms = APP_COMMAND_DEFAULT_TIMEOUT_MS;
  AppPlatform_IrqUnlock(key);
}

void AppCommand_RequestArm(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  g_command.arm_requested = true;
  AppPlatform_IrqUnlock(key);
}

void AppCommand_Disarm(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  g_command.arm_requested = false;
  g_command.valid = false;
  g_command.fresh = false;
  g_command.timed_out = false;
  g_command.mode = APP_COMMAND_MODE_NONE;
  g_command.first = 0.0f;
  g_command.second = 0.0f;
  AppPlatform_IrqUnlock(key);
}

void AppCommand_Stop(void)
{
  AppCommand_Disarm();
}

bool AppCommand_SubmitMotorEffort(AppCommandSource source, float motor_a, float motor_b,
                                 uint32_t timeout_ms)
{
  if ((motor_a < -1.0f) || (motor_a > 1.0f) || (motor_b < -1.0f) || (motor_b > 1.0f))
  {
    AppState_SetFault(APP_FAULT_INVALID_MOTOR_COMMAND);
    return false;
  }
  return AppCommand_Submit(source, APP_COMMAND_MODE_MOTOR_EFFORT,
                           motor_a, motor_b, timeout_ms);
}

bool AppCommand_SubmitWheelRate(AppCommandSource source, float left_cps, float right_cps,
                               uint32_t timeout_ms)
{
  return AppCommand_Submit(source, APP_COMMAND_MODE_WHEEL_RATE,
                           left_cps, right_cps, timeout_ms);
}

bool AppCommand_SubmitBodyVelocity(AppCommandSource source, float linear_mps,
                                  float angular_radps, uint32_t timeout_ms)
{
  return AppCommand_Submit(source, APP_COMMAND_MODE_BODY_VELOCITY,
                           linear_mps, angular_radps, timeout_ms);
}

void AppCommand_GetSnapshot(uint32_t now_ms, AppCommandSnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_command;
  AppPlatform_IrqUnlock(key);

  if (snapshot->valid)
  {
    snapshot->timed_out = (AppPlatform_ElapsedMs(now_ms, snapshot->received_ms) >
                           snapshot->timeout_ms);
    snapshot->fresh = !snapshot->timed_out;
  }
  else
  {
    snapshot->fresh = false;
    snapshot->timed_out = false;
  }
}
