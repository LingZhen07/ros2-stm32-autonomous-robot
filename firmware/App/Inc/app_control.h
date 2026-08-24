#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include <stdbool.h>

typedef enum
{
  APP_CONTROLLER_LEFT = 0,
  APP_CONTROLLER_RIGHT,
  APP_CONTROLLER_COUNT
} AppControllerId;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integrator_limit;
  float output_limit;
  bool configured;
} AppControllerConfig;

typedef struct
{
  float target;
  float measured;
  float error;
  float output;
  float integrator;
  bool enabled;
  bool saturated;
  bool configured;
} AppControllerSnapshot;

void AppControl_Init(void);
bool AppControl_SetConfig(AppControllerId id, const AppControllerConfig *config);
void AppControl_GetConfig(AppControllerId id, AppControllerConfig *config);
bool AppControl_AllConfigured(void);
void AppControl_Reset(void);
bool AppControl_Update(float left_target, float right_target,
                       float left_measured, float right_measured,
                       float dt_seconds, float *left_output, float *right_output);
void AppControl_GetSnapshot(AppControllerId id, AppControllerSnapshot *snapshot);

#endif /* APP_CONTROL_H */
