#include "app_drivetrain.h"

#include "app_platform.h"

#include <math.h>
#include <string.h>

#define APP_TWO_PI 6.2831853071795864769f

static AppDrivetrainConfig g_config;

static bool AppDrivetrain_SignValid(int8_t sign)
{
  return ((sign == -1) || (sign == 1));
}

static bool AppDrivetrain_SidesValid(AppWheelSide first, AppWheelSide second)
{
  return (((first == APP_WHEEL_LEFT) && (second == APP_WHEEL_RIGHT)) ||
          ((first == APP_WHEEL_RIGHT) && (second == APP_WHEEL_LEFT)));
}

void AppDrivetrain_Init(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  memset(&g_config, 0, sizeof(g_config));
  g_config.effective_wheel_radius_m = NAN;
  g_config.wheel_track_m = NAN;
  g_config.encoder_counts_per_motor_revolution = NAN;
  g_config.motor_to_wheel_gear_ratio = NAN;
  AppPlatform_IrqUnlock(key);
}

bool AppDrivetrain_SetConfig(const AppDrivetrainConfig *config)
{
  uint32_t key;

  if (config == NULL)
  {
    return false;
  }

  key = AppPlatform_IrqLock();
  g_config = *config;
  AppPlatform_IrqUnlock(key);
  return true;
}

void AppDrivetrain_GetConfig(AppDrivetrainConfig *config)
{
  uint32_t key;

  if (config == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *config = g_config;
  AppPlatform_IrqUnlock(key);
}

bool AppDrivetrain_WheelControlReady(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  return AppDrivetrain_SidesValid(config.motor_a_side, config.motor_b_side) &&
         AppDrivetrain_SidesValid(config.encoder_1_side, config.encoder_2_side) &&
         AppDrivetrain_SignValid(config.motor_a_forward_sign) &&
         AppDrivetrain_SignValid(config.motor_b_forward_sign) &&
         AppDrivetrain_SignValid(config.encoder_1_forward_sign) &&
         AppDrivetrain_SignValid(config.encoder_2_forward_sign);
}

bool AppDrivetrain_BodyCommandReady(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  return AppDrivetrain_WheelControlReady() &&
         isfinite(config.effective_wheel_radius_m) && (config.effective_wheel_radius_m > 0.0f) &&
         isfinite(config.wheel_track_m) && (config.wheel_track_m > 0.0f) &&
         isfinite(config.encoder_counts_per_motor_revolution) &&
         (config.encoder_counts_per_motor_revolution > 0.0f) &&
         isfinite(config.motor_to_wheel_gear_ratio) && (config.motor_to_wheel_gear_ratio > 0.0f);
}

bool AppDrivetrain_MapEncoderRates(float encoder_1_cps, float encoder_2_cps,
                                   float *left_cps, float *right_cps)
{
  AppDrivetrainConfig config;

  if ((left_cps == NULL) || (right_cps == NULL) || !AppDrivetrain_WheelControlReady())
  {
    return false;
  }
  AppDrivetrain_GetConfig(&config);

  if (config.encoder_1_side == APP_WHEEL_LEFT)
  {
    *left_cps = encoder_1_cps * (float)config.encoder_1_forward_sign;
    *right_cps = encoder_2_cps * (float)config.encoder_2_forward_sign;
  }
  else
  {
    *right_cps = encoder_1_cps * (float)config.encoder_1_forward_sign;
    *left_cps = encoder_2_cps * (float)config.encoder_2_forward_sign;
  }
  return true;
}

bool AppDrivetrain_MapWheelEfforts(float left_effort, float right_effort,
                                  float *motor_a_effort, float *motor_b_effort)
{
  AppDrivetrainConfig config;

  if ((motor_a_effort == NULL) || (motor_b_effort == NULL) ||
      !isfinite(left_effort) || !isfinite(right_effort) ||
      !AppDrivetrain_WheelControlReady())
  {
    return false;
  }
  AppDrivetrain_GetConfig(&config);

  *motor_a_effort = ((config.motor_a_side == APP_WHEEL_LEFT) ? left_effort : right_effort) *
                    (float)config.motor_a_forward_sign;
  *motor_b_effort = ((config.motor_b_side == APP_WHEEL_LEFT) ? left_effort : right_effort) *
                    (float)config.motor_b_forward_sign;
  return true;
}

bool AppDrivetrain_BodyToEncoderRates(float linear_mps, float angular_radps,
                                     float *left_cps, float *right_cps)
{
  AppDrivetrainConfig config;
  float left_mps;
  float right_mps;
  float counts_per_wheel_revolution;

  if ((left_cps == NULL) || (right_cps == NULL) ||
      !isfinite(linear_mps) || !isfinite(angular_radps) ||
      !AppDrivetrain_BodyCommandReady())
  {
    return false;
  }
  AppDrivetrain_GetConfig(&config);

  left_mps = linear_mps - (angular_radps * config.wheel_track_m * 0.5f);
  right_mps = linear_mps + (angular_radps * config.wheel_track_m * 0.5f);
  counts_per_wheel_revolution = config.encoder_counts_per_motor_revolution *
                                config.motor_to_wheel_gear_ratio;
  *left_cps = (left_mps / (APP_TWO_PI * config.effective_wheel_radius_m)) *
              counts_per_wheel_revolution;
  *right_cps = (right_mps / (APP_TWO_PI * config.effective_wheel_radius_m)) *
               counts_per_wheel_revolution;
  return isfinite(*left_cps) && isfinite(*right_cps);
}
