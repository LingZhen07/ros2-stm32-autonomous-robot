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
  float left_counts_per_wheel_revolution;
  float right_counts_per_wheel_revolution;
  float max_body_linear_speed_mps;
  float max_body_angular_speed_radps;
  float max_wheel_rate_cps;
  AppWheelSide motor_a_side;
  AppWheelSide motor_b_side;
  AppWheelSide encoder_1_side;
  AppWheelSide encoder_2_side;
  int8_t motor_a_forward_sign;
  int8_t motor_b_forward_sign;
  int8_t left_encoder_forward_sign;
  int8_t right_encoder_forward_sign;
} AppDrivetrainConfig;

void AppDrivetrain_Init(void);
bool AppDrivetrain_SetConfig(const AppDrivetrainConfig *config);
void AppDrivetrain_GetConfig(AppDrivetrainConfig *config);
bool AppDrivetrain_EncoderMappingReady(void);
bool AppDrivetrain_MotorMappingReady(void);
bool AppDrivetrain_WheelControlReady(void);
bool AppDrivetrain_BodyCommandReady(void);
bool AppDrivetrain_MapEncoderRates(float encoder_1_cps, float encoder_2_cps,
                                   float *left_cps, float *right_cps);
bool AppDrivetrain_MapEncoderPositions(int64_t encoder_1_counts, int64_t encoder_2_counts,
                                       int64_t *left_counts, int64_t *right_counts);
bool AppDrivetrain_MapWheelEfforts(float left_effort, float right_effort,
                                  float *motor_a_effort, float *motor_b_effort);
bool AppDrivetrain_BodyToEncoderRates(float linear_mps, float angular_radps,
                                     float *left_cps, float *right_cps);

#endif /* APP_DRIVETRAIN_H */
