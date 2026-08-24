#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include "app_battery.h"
#include "app_control.h"
#include "app_encoder.h"
#include "app_imu.h"
#include "app_motor.h"
#include "app_state.h"
#include "app_supervisor.h"

#include <stdint.h>

typedef struct
{
  uint32_t runtime_ms;
  AppStateSnapshot state;
  AppEncoderSnapshot encoder_1;
  AppEncoderSnapshot encoder_2;
  AppControllerSnapshot left_controller;
  AppControllerSnapshot right_controller;
  AppMotorSnapshot motor;
  AppImuSnapshot imu;
  AppBatterySnapshot battery;
  AppSupervisorSnapshot supervisor;
} AppTelemetrySnapshot;

void AppTelemetry_GetSnapshot(uint32_t now_ms, AppTelemetrySnapshot *snapshot);

#endif /* APP_TELEMETRY_H */
