#include "app_safety.h"

#include "app_control.h"
#include "app_drivetrain.h"
#include "app_encoder.h"
#include "app_motor.h"
#include "app_platform.h"
#include "app_state.h"

static volatile bool g_initialization_complete;
static volatile bool g_motion_authorized;

void AppSafety_Init(void)
{
  g_initialization_complete = false;
  g_motion_authorized = false;
  AppMotor_ForceSafe();
}

void AppSafety_SetInitializationComplete(bool complete)
{
  g_initialization_complete = complete;
}

bool AppSafety_CommandModeReady(const AppCommandSnapshot *command)
{
  if ((command == NULL) || !command->valid)
  {
    return false;
  }

  switch (command->mode)
  {
    case APP_COMMAND_MODE_MOTOR_EFFORT:
      return true;

    case APP_COMMAND_MODE_WHEEL_RATE:
      return AppControl_AllConfigured() && AppDrivetrain_WheelControlReady();

    case APP_COMMAND_MODE_BODY_VELOCITY:
      return AppControl_AllConfigured() && AppDrivetrain_BodyCommandReady();

    case APP_COMMAND_MODE_NONE:
    default:
      return false;
  }
}

void AppSafety_ForceFault(uint32_t fault_flags)
{
  AppState_SetFault(fault_flags);
  g_motion_authorized = false;
  AppCommand_Disarm();
  AppControl_Reset();
  AppMotor_ForceSafe();
  AppState_SetSystemState(APP_SYSTEM_FAULT, HAL_GetTick());
}

void AppSafety_Update(uint32_t now_ms)
{
  AppCommandSnapshot command;

  if (!g_initialization_complete)
  {
    g_motion_authorized = false;
    AppMotor_ForceSafe();
    AppState_SetSystemState(APP_SYSTEM_INIT, now_ms);
    return;
  }

  if (AppState_HasCriticalFault())
  {
    AppSafety_ForceFault(0U);
    return;
  }

  AppCommand_GetSnapshot(now_ms, &command);
  if (!command.arm_requested)
  {
    g_motion_authorized = false;
    AppMotor_ForceSafe();
    AppState_SetSystemState(APP_SYSTEM_READY, now_ms);
    return;
  }

  if (!command.valid)
  {
    g_motion_authorized = false;
    AppMotor_ForceSafe();
    AppState_SetSystemState(APP_SYSTEM_READY, now_ms);
    return;
  }

  if (command.timed_out)
  {
    AppSafety_ForceFault(APP_FAULT_COMMAND_TIMEOUT);
    return;
  }

  if (!AppSafety_CommandModeReady(&command))
  {
    AppSafety_ForceFault(APP_FAULT_INVALID_MOTOR_COMMAND);
    return;
  }

  g_motion_authorized = true;
  AppMotor_SetAuthorized(true);
  AppState_SetSystemState(APP_SYSTEM_ACTIVE, now_ms);
}

bool AppSafety_ClearRecoverableFaults(void)
{
  uint32_t recoverable = APP_FAULT_COMMAND_TIMEOUT |
                         APP_FAULT_INVALID_MOTOR_COMMAND |
                         APP_FAULT_CONTROL_SATURATION;

  if (AppEncoder_AllValid(HAL_GetTick()))
  {
    recoverable |= APP_FAULT_ENCODER_VALIDITY;
  }

  AppCommand_Disarm();
  AppMotor_ForceSafe();
  AppControl_Reset();
  AppState_ClearFault(recoverable);
  g_motion_authorized = false;

  if (AppState_HasCriticalFault())
  {
    AppState_SetSystemState(APP_SYSTEM_FAULT, HAL_GetTick());
    return false;
  }

  AppState_SetSystemState(APP_SYSTEM_SAFE, HAL_GetTick());
  return true;
}

bool AppSafety_MotionAuthorized(void)
{
  return g_motion_authorized && (AppState_GetSystemState() == APP_SYSTEM_ACTIVE) &&
         !AppState_HasCriticalFault();
}

bool AppSafety_ValidateActiveCommand(uint32_t now_ms, AppCommandSnapshot *command)
{
  if (command == NULL)
  {
    AppSafety_ForceFault(APP_FAULT_INTERNAL_CONFIGURATION);
    return false;
  }

  AppCommand_GetSnapshot(now_ms, command);
  if (!AppSafety_MotionAuthorized())
  {
    return false;
  }
  if (!command->valid || !command->fresh || command->timed_out)
  {
    AppSafety_ForceFault(APP_FAULT_COMMAND_TIMEOUT);
    return false;
  }
  if (!AppSafety_CommandModeReady(command))
  {
    AppSafety_ForceFault(APP_FAULT_INVALID_MOTOR_COMMAND);
    return false;
  }
  return true;
}
