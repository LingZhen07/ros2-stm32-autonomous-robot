#include "app_telemetry.h"

#include <string.h>

void AppTelemetry_GetSnapshot(uint32_t now_ms, AppTelemetrySnapshot *snapshot)
{
  if (snapshot == NULL)
  {
    return;
  }

  memset(snapshot, 0, sizeof(*snapshot));
  snapshot->runtime_ms = now_ms;
  AppState_GetSnapshot(&snapshot->state);
  AppEncoder_GetSnapshot(APP_ENCODER_1, now_ms, &snapshot->encoder_1);
  AppEncoder_GetSnapshot(APP_ENCODER_2, now_ms, &snapshot->encoder_2);
  AppControl_GetSnapshot(APP_CONTROLLER_LEFT, &snapshot->left_controller);
  AppControl_GetSnapshot(APP_CONTROLLER_RIGHT, &snapshot->right_controller);
  AppMotor_GetSnapshot(&snapshot->motor);
  AppImu_GetSnapshot(now_ms, &snapshot->imu);
  AppBattery_GetSnapshot(now_ms, &snapshot->battery);
  AppSupervisor_GetSnapshot(now_ms, &snapshot->supervisor);
}
