#include "app_drivetrain.h"

#include "app_config.h"
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

static bool AppDrivetrain_OptionalPositive(float value)
{
  return isnan(value) || (isfinite(value) && (value > 0.0f));
}

void AppDrivetrain_Init(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  memset(&g_config, 0, sizeof(g_config));
  g_config.effective_wheel_radius_m = APP_DRIVETRAIN_WHEEL_RADIUS_M;
  g_config.wheel_track_m = APP_DRIVETRAIN_WHEEL_TRACK_M;
  g_config.left_counts_per_wheel_revolution = APP_DRIVETRAIN_LEFT_COUNTS_PER_WHEEL_REV;
  g_config.right_counts_per_wheel_revolution = APP_DRIVETRAIN_RIGHT_COUNTS_PER_WHEEL_REV;
  g_config.max_body_linear_speed_mps = APP_DRIVETRAIN_MAX_BODY_LINEAR_SPEED_MPS;
  g_config.max_body_angular_speed_radps = APP_DRIVETRAIN_MAX_BODY_ANGULAR_SPEED_RADPS;
  g_config.max_wheel_rate_cps = APP_DRIVETRAIN_MAX_WHEEL_RATE_CPS;
  /* Verified 2026-08-29: Motor A is right, Motor B is left. */
  g_config.motor_a_side = APP_WHEEL_RIGHT;
  g_config.motor_b_side = APP_WHEEL_LEFT;
  /* Verified forward command polarity for both TB6612 channels. */
  g_config.motor_a_forward_sign = -1;
  g_config.motor_b_forward_sign = -1;
  /* Verified 2026-08-29: Encoder 1 is right, Encoder 2 is left. */
  g_config.encoder_1_side = APP_WHEEL_RIGHT;
  g_config.encoder_2_side = APP_WHEEL_LEFT;
  /* Verified raw signs: left-forward is negative; right-forward is positive. */
  g_config.left_encoder_forward_sign = -1;
  g_config.right_encoder_forward_sign = 1;
  AppPlatform_IrqUnlock(key);
}

bool AppDrivetrain_SetConfig(const AppDrivetrainConfig *config)
{
  uint32_t key;

  if ((config == NULL) ||
      !AppDrivetrain_OptionalPositive(config->effective_wheel_radius_m) ||
      !AppDrivetrain_OptionalPositive(config->wheel_track_m) ||
      !AppDrivetrain_OptionalPositive(config->left_counts_per_wheel_revolution) ||
      !AppDrivetrain_OptionalPositive(config->right_counts_per_wheel_revolution) ||
      !AppDrivetrain_OptionalPositive(config->max_body_linear_speed_mps) ||
      !AppDrivetrain_OptionalPositive(config->max_body_angular_speed_radps) ||
      !AppDrivetrain_OptionalPositive(config->max_wheel_rate_cps) ||
      (config->motor_a_side != APP_WHEEL_RIGHT) ||
      (config->motor_b_side != APP_WHEEL_LEFT) ||
      (config->encoder_1_side != APP_WHEEL_RIGHT) ||
      (config->encoder_2_side != APP_WHEEL_LEFT) ||
      (config->motor_a_forward_sign != -1) ||
      (config->motor_b_forward_sign != -1) ||
      (config->left_encoder_forward_sign != -1) ||
      (config->right_encoder_forward_sign != 1))
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

bool AppDrivetrain_EncoderMappingReady(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  return (config.encoder_1_side == APP_WHEEL_RIGHT) &&
         (config.encoder_2_side == APP_WHEEL_LEFT) &&
         (config.left_encoder_forward_sign == -1) &&
         (config.right_encoder_forward_sign == 1);
}

bool AppDrivetrain_MotorMappingReady(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  return AppDrivetrain_SidesValid(config.motor_a_side, config.motor_b_side) &&
         AppDrivetrain_SignValid(config.motor_a_forward_sign) &&
         AppDrivetrain_SignValid(config.motor_b_forward_sign);
}

bool AppDrivetrain_WheelControlReady(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  return AppDrivetrain_EncoderMappingReady() && AppDrivetrain_MotorMappingReady() &&
         isfinite(config.max_wheel_rate_cps) && (config.max_wheel_rate_cps > 0.0f);
}

bool AppDrivetrain_BodyCommandReady(void)
{
  AppDrivetrainConfig config;
  AppDrivetrain_GetConfig(&config);

  return AppDrivetrain_WheelControlReady() &&
         isfinite(config.effective_wheel_radius_m) && (config.effective_wheel_radius_m > 0.0f) &&
         isfinite(config.wheel_track_m) && (config.wheel_track_m > 0.0f) &&
         isfinite(config.left_counts_per_wheel_revolution) &&
         (config.left_counts_per_wheel_revolution > 0.0f) &&
         isfinite(config.right_counts_per_wheel_revolution) &&
         (config.right_counts_per_wheel_revolution > 0.0f) &&
         isfinite(config.max_body_linear_speed_mps) &&
         (config.max_body_linear_speed_mps > 0.0f) &&
         isfinite(config.max_body_angular_speed_radps) &&
         (config.max_body_angular_speed_radps > 0.0f);
}

bool AppDrivetrain_MapEncoderRates(float encoder_1_cps, float encoder_2_cps,
                                   float *left_cps, float *right_cps)
{
  AppDrivetrainConfig config;

  if ((left_cps == NULL) || (right_cps == NULL) || !AppDrivetrain_EncoderMappingReady())
  {
    return false;
  }
  AppDrivetrain_GetConfig(&config);

  if (config.encoder_1_side == APP_WHEEL_LEFT)
  {
    *left_cps = encoder_1_cps * (float)config.left_encoder_forward_sign;
    *right_cps = encoder_2_cps * (float)config.right_encoder_forward_sign;
  }
  else
  {
    *right_cps = encoder_1_cps * (float)config.right_encoder_forward_sign;
    *left_cps = encoder_2_cps * (float)config.left_encoder_forward_sign;
  }
  return true;
}

bool AppDrivetrain_MapEncoderPositions(int64_t encoder_1_counts, int64_t encoder_2_counts,
                                       int64_t *left_counts, int64_t *right_counts)
{
  AppDrivetrainConfig config;

  if ((left_counts == NULL) || (right_counts == NULL) ||
      !AppDrivetrain_EncoderMappingReady())
  {
    return false;
  }
  AppDrivetrain_GetConfig(&config);

  if (config.encoder_1_side == APP_WHEEL_LEFT)
  {
    *left_counts = encoder_1_counts * (int64_t)config.left_encoder_forward_sign;
    *right_counts = encoder_2_counts * (int64_t)config.right_encoder_forward_sign;
  }
  else
  {
    *right_counts = encoder_1_counts * (int64_t)config.right_encoder_forward_sign;
    *left_counts = encoder_2_counts * (int64_t)config.left_encoder_forward_sign;
  }
  return true;
}

bool AppDrivetrain_MapWheelEfforts(float left_effort, float right_effort,
                                  float *motor_a_effort, float *motor_b_effort)
{
  AppDrivetrainConfig config;

  if ((motor_a_effort == NULL) || (motor_b_effort == NULL) ||
      !isfinite(left_effort) || !isfinite(right_effort) ||
      !AppDrivetrain_MotorMappingReady())
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

  if ((left_cps == NULL) || (right_cps == NULL) ||
      !isfinite(linear_mps) || !isfinite(angular_radps) ||
      !AppDrivetrain_BodyCommandReady())
  {
    return false;
  }
  AppDrivetrain_GetConfig(&config);

  if ((fabsf(linear_mps) > config.max_body_linear_speed_mps) ||
      (fabsf(angular_radps) > config.max_body_angular_speed_radps))
  {
    return false;
  }

  left_mps = linear_mps - (angular_radps * config.wheel_track_m * 0.5f);
  right_mps = linear_mps + (angular_radps * config.wheel_track_m * 0.5f);
  *left_cps = (left_mps / (APP_TWO_PI * config.effective_wheel_radius_m)) *
              config.left_counts_per_wheel_revolution;
  *right_cps = (right_mps / (APP_TWO_PI * config.effective_wheel_radius_m)) *
               config.right_counts_per_wheel_revolution;
  return isfinite(*left_cps) && isfinite(*right_cps) &&
         (fabsf(*left_cps) <= config.max_wheel_rate_cps) &&
         (fabsf(*right_cps) <= config.max_wheel_rate_cps);
}
