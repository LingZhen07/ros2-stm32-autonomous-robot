#ifndef APP_DRIVETRAIN_H
#define APP_DRIVETRAIN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_WHEEL_UNKNOWN = 0,
  APP_WHEEL_LEFT,
  APP_WHEEL_RIGHT
} AppWheelSide;

typedef struct
{
  float effective_wheel_radius_m;
  float wheel_track_m;
  float encoder_counts_per_motor_revolution;
  float motor_to_wheel_gear_ratio;
  AppWheelSide motor_a_side;
  AppWheelSide motor_b_side;
  AppWheelSide encoder_1_side;
  AppWheelSide encoder_2_side;
  int8_t motor_a_forward_sign;
  int8_t motor_b_forward_sign;
  int8_t encoder_1_forward_sign;
  int8_t encoder_2_forward_sign;
} AppDrivetrainConfig;

void AppDrivetrain_Init(void);
bool AppDrivetrain_SetConfig(const AppDrivetrainConfig *config);
void AppDrivetrain_GetConfig(AppDrivetrainConfig *config);
bool AppDrivetrain_WheelControlReady(void);
bool AppDrivetrain_BodyCommandReady(void);
bool AppDrivetrain_MapEncoderRates(float encoder_1_cps, float encoder_2_cps,
                                   float *left_cps, float *right_cps);
bool AppDrivetrain_MapWheelEfforts(float left_effort, float right_effort,
                                  float *motor_a_effort, float *motor_b_effort);
bool AppDrivetrain_BodyToEncoderRates(float linear_mps, float angular_radps,
                                     float *left_cps, float *right_cps);

#endif /* APP_DRIVETRAIN_H */
