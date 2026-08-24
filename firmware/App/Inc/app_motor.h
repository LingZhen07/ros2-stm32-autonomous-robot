#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_MOTOR_STOP_COAST = 0,
  APP_MOTOR_STOP_SHORT_BRAKE
} AppMotorStopMode;

typedef struct
{
  bool initialized;
  bool authorized;
  bool standby_asserted;
  float motor_a_effort;
  float motor_b_effort;
  uint16_t motor_a_duty_counts;
  uint16_t motor_b_duty_counts;
  AppMotorStopMode stop_mode;
} AppMotorSnapshot;

bool AppMotor_Init(void);
void AppMotor_SetAuthorized(bool authorized);
bool AppMotor_ApplyEffort(float motor_a, float motor_b);
void AppMotor_Stop(AppMotorStopMode mode);
void AppMotor_ForceSafe(void);
void AppMotor_EmergencySafe(void);
void AppMotor_GetSnapshot(AppMotorSnapshot *snapshot);

#endif /* APP_MOTOR_H */
