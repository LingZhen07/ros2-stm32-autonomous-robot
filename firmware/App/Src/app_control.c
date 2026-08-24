#include "app_control.h"

#include "app_platform.h"
#include "app_state.h"

#include <math.h>
#include <string.h>

typedef struct
{
  AppControllerConfig config;
  AppControllerSnapshot snapshot;
  float previous_error;
} AppController;

static AppController g_controllers[APP_CONTROLLER_COUNT];

static float AppControl_Clamp(float value, float limit, bool *saturated)
{
  if (value > limit)
  {
    *saturated = true;
    return limit;
  }
  if (value < -limit)
  {
    *saturated = true;
    return -limit;
  }
  return value;
}

static bool AppControl_ConfigValid(const AppControllerConfig *config)
{
  return (config != NULL) && config->configured &&
         isfinite(config->kp) && isfinite(config->ki) && isfinite(config->kd) &&
         isfinite(config->integrator_limit) && (config->integrator_limit >= 0.0f) &&
         isfinite(config->output_limit) && (config->output_limit > 0.0f) &&
         (config->output_limit <= 1.0f);
}

static bool AppControl_UpdateOne(AppController *controller, float target, float measured,
                                 float dt_seconds, float *output)
{
  float error;
  float derivative;
  float candidate_integrator;
  float unclamped;
  bool saturated = false;

  if (!AppControl_ConfigValid(&controller->config) || !isfinite(target) ||
      !isfinite(measured) || !isfinite(dt_seconds) || (dt_seconds <= 0.0f))
  {
    return false;
  }

  error = target - measured;
  derivative = (error - controller->previous_error) / dt_seconds;
  candidate_integrator = controller->snapshot.integrator + (error * dt_seconds);
  if (candidate_integrator > controller->config.integrator_limit)
  {
    candidate_integrator = controller->config.integrator_limit;
  }
  else if (candidate_integrator < -controller->config.integrator_limit)
  {
    candidate_integrator = -controller->config.integrator_limit;
  }

  unclamped = (controller->config.kp * error) +
              (controller->config.ki * candidate_integrator) +
              (controller->config.kd * derivative);
  *output = AppControl_Clamp(unclamped, controller->config.output_limit, &saturated);

  if (!saturated || ((*output > 0.0f) && (error < 0.0f)) ||
      ((*output < 0.0f) && (error > 0.0f)))
  {
    controller->snapshot.integrator = candidate_integrator;
  }

  controller->snapshot.target = target;
  controller->snapshot.measured = measured;
  controller->snapshot.error = error;
  controller->snapshot.output = *output;
  controller->snapshot.enabled = true;
  controller->snapshot.saturated = saturated;
  controller->snapshot.configured = true;
  controller->previous_error = error;
  return isfinite(*output);
}

void AppControl_Init(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  memset(g_controllers, 0, sizeof(g_controllers));
  AppPlatform_IrqUnlock(key);
}

bool AppControl_SetConfig(AppControllerId id, const AppControllerConfig *config)
{
  uint32_t key;

  if ((id >= APP_CONTROLLER_COUNT) || !AppControl_ConfigValid(config))
  {
    return false;
  }
  key = AppPlatform_IrqLock();
  g_controllers[id].config = *config;
  memset(&g_controllers[id].snapshot, 0, sizeof(g_controllers[id].snapshot));
  g_controllers[id].snapshot.configured = true;
  g_controllers[id].previous_error = 0.0f;
  AppPlatform_IrqUnlock(key);
  return true;
}

void AppControl_GetConfig(AppControllerId id, AppControllerConfig *config)
{
  uint32_t key;

  if ((id >= APP_CONTROLLER_COUNT) || (config == NULL))
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *config = g_controllers[id].config;
  AppPlatform_IrqUnlock(key);
}

bool AppControl_AllConfigured(void)
{
  return AppControl_ConfigValid(&g_controllers[APP_CONTROLLER_LEFT].config) &&
         AppControl_ConfigValid(&g_controllers[APP_CONTROLLER_RIGHT].config);
}

void AppControl_Reset(void)
{
  uint32_t key = AppPlatform_IrqLock();
  for (uint32_t index = 0U; index < APP_CONTROLLER_COUNT; ++index)
  {
    memset(&g_controllers[index].snapshot, 0, sizeof(g_controllers[index].snapshot));
    g_controllers[index].snapshot.configured = g_controllers[index].config.configured;
    g_controllers[index].previous_error = 0.0f;
  }
  AppPlatform_IrqUnlock(key);
}

bool AppControl_Update(float left_target, float right_target,
                       float left_measured, float right_measured,
                       float dt_seconds, float *left_output, float *right_output)
{
  bool left_ok;
  bool right_ok;
  uint32_t key;

  if ((left_output == NULL) || (right_output == NULL))
  {
    return false;
  }

  key = AppPlatform_IrqLock();
  left_ok = AppControl_UpdateOne(&g_controllers[APP_CONTROLLER_LEFT], left_target,
                                 left_measured, dt_seconds, left_output);
  right_ok = AppControl_UpdateOne(&g_controllers[APP_CONTROLLER_RIGHT], right_target,
                                  right_measured, dt_seconds, right_output);
  AppPlatform_IrqUnlock(key);
  if (!left_ok || !right_ok)
  {
    AppState_SetFault(APP_FAULT_CONTROL_ABNORMAL);
    return false;
  }
  if (g_controllers[APP_CONTROLLER_LEFT].snapshot.saturated ||
      g_controllers[APP_CONTROLLER_RIGHT].snapshot.saturated)
  {
    AppState_SetFault(APP_FAULT_CONTROL_SATURATION);
  }
  return true;
}

void AppControl_GetSnapshot(AppControllerId id, AppControllerSnapshot *snapshot)
{
  uint32_t key;

  if ((id >= APP_CONTROLLER_COUNT) || (snapshot == NULL))
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_controllers[id].snapshot;
  AppPlatform_IrqUnlock(key);
}
