#ifndef APP_COMMAND_H
#define APP_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_COMMAND_SOURCE_NONE = 0,
  APP_COMMAND_SOURCE_DIAGNOSTIC,
  APP_COMMAND_SOURCE_FDCAN
} AppCommandSource;

typedef enum
{
  APP_COMMAND_MODE_NONE = 0,
  APP_COMMAND_MODE_MOTOR_EFFORT,
  APP_COMMAND_MODE_WHEEL_RATE,
  APP_COMMAND_MODE_BODY_VELOCITY
} AppCommandMode;

typedef struct
{
  AppCommandSource source;
  AppCommandMode mode;
  bool arm_requested;
  bool valid;
  bool fresh;
  bool timed_out;
  uint32_t sequence;
  uint32_t received_ms;
  uint32_t timeout_ms;
  float first;
  float second;
} AppCommandSnapshot;

void AppCommand_Init(void);
void AppCommand_RequestArm(void);
void AppCommand_Disarm(void);
void AppCommand_Stop(void);
bool AppCommand_SubmitMotorEffort(AppCommandSource source, float motor_a, float motor_b,
                                 uint32_t timeout_ms);
bool AppCommand_SubmitWheelRate(AppCommandSource source, float left_cps, float right_cps,
                               uint32_t timeout_ms);
bool AppCommand_SubmitBodyVelocity(AppCommandSource source, float linear_mps,
                                  float angular_radps, uint32_t timeout_ms);
void AppCommand_GetSnapshot(uint32_t now_ms, AppCommandSnapshot *snapshot);

#endif /* APP_COMMAND_H */
