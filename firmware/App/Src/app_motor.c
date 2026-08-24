#include "app_motor.h"

#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"
#include "main.h"
#include "tim.h"

#include <math.h>
#include <string.h>

static AppMotorSnapshot g_motor;

static float AppMotor_ClampEffort(float effort)
{
  if (effort > 1.0f)
  {
    return 1.0f;
  }
  if (effort < -1.0f)
  {
    return -1.0f;
  }
  return effort;
}

static uint16_t AppMotor_EffortToDuty(float effort)
{
  const float magnitude = (effort < 0.0f) ? -effort : effort;
  return (uint16_t)(magnitude * (float)(APP_MOTOR_PWM_PERIOD_COUNTS - 1UL));
}

static void AppMotor_SetDirection(GPIO_TypeDef *port, uint16_t pin_1, uint16_t pin_2,
                                  float effort, AppMotorStopMode stop_mode)
{
  if (effort > 0.0f)
  {
    HAL_GPIO_WritePin(port, pin_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(port, pin_2, GPIO_PIN_RESET);
  }
  else if (effort < 0.0f)
  {
    HAL_GPIO_WritePin(port, pin_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(port, pin_2, GPIO_PIN_SET);
  }
  else if (stop_mode == APP_MOTOR_STOP_SHORT_BRAKE)
  {
    HAL_GPIO_WritePin(port, pin_1 | pin_2, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(port, pin_1 | pin_2, GPIO_PIN_RESET);
  }
}

bool AppMotor_Init(void)
{
  memset(&g_motor, 0, sizeof(g_motor));
  AppMotor_EmergencySafe();

  if ((htim1.Init.Prescaler != 0U) ||
      ((htim1.Init.Period + 1UL) != APP_MOTOR_PWM_PERIOD_COUNTS))
  {
    AppState_SetFault(APP_FAULT_INTERNAL_CONFIGURATION);
    return false;
  }

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  if ((HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK))
  {
    AppMotor_EmergencySafe();
    AppState_SetFault(APP_FAULT_INTERNAL_CONFIGURATION);
    return false;
  }

  g_motor.initialized = true;
  g_motor.stop_mode = APP_MOTOR_STOP_COAST;
  AppMotor_ForceSafe();
  return true;
}

void AppMotor_SetAuthorized(bool authorized)
{
  const uint32_t key = AppPlatform_IrqLock();
  g_motor.authorized = authorized && g_motor.initialized;
  AppPlatform_IrqUnlock(key);
  if (!authorized)
  {
    AppMotor_ForceSafe();
  }
}

bool AppMotor_ApplyEffort(float motor_a, float motor_b)
{
  uint16_t duty_a;
  uint16_t duty_b;
  uint32_t key;

  if (!g_motor.initialized || !g_motor.authorized ||
      !isfinite(motor_a) || !isfinite(motor_b))
  {
    AppState_SetFault(APP_FAULT_INVALID_MOTOR_COMMAND);
    AppMotor_ForceSafe();
    return false;
  }

  motor_a = AppMotor_ClampEffort(motor_a);
  motor_b = AppMotor_ClampEffort(motor_b);
  duty_a = AppMotor_EffortToDuty(motor_a);
  duty_b = AppMotor_EffortToDuty(motor_b);

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
  AppMotor_SetDirection(GPIOB, AIN1_Pin, AIN2_Pin, motor_a, APP_MOTOR_STOP_COAST);
  AppMotor_SetDirection(GPIOB, BIN1_Pin, BIN2_Pin, motor_b, APP_MOTOR_STOP_COAST);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_b);
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

  key = AppPlatform_IrqLock();
  g_motor.motor_a_effort = motor_a;
  g_motor.motor_b_effort = motor_b;
  g_motor.motor_a_duty_counts = duty_a;
  g_motor.motor_b_duty_counts = duty_b;
  g_motor.standby_asserted = true;
  g_motor.stop_mode = APP_MOTOR_STOP_COAST;
  AppPlatform_IrqUnlock(key);
  return true;
}

void AppMotor_Stop(AppMotorStopMode mode)
{
  const uint16_t duty = (mode == APP_MOTOR_STOP_SHORT_BRAKE)
    ? (uint16_t)(APP_MOTOR_PWM_PERIOD_COUNTS - 1UL) : 0U;
  uint32_t key;

  AppMotor_SetDirection(GPIOB, AIN1_Pin, AIN2_Pin, 0.0f, mode);
  AppMotor_SetDirection(GPIOB, BIN1_Pin, BIN2_Pin, 0.0f, mode);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty);
  if (g_motor.authorized && g_motor.initialized)
  {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
  }

  key = AppPlatform_IrqLock();
  g_motor.motor_a_effort = 0.0f;
  g_motor.motor_b_effort = 0.0f;
  g_motor.motor_a_duty_counts = duty;
  g_motor.motor_b_duty_counts = duty;
  g_motor.stop_mode = mode;
  g_motor.standby_asserted = g_motor.authorized && g_motor.initialized;
  AppPlatform_IrqUnlock(key);
}

void AppMotor_ForceSafe(void)
{
  uint32_t key;

  AppMotor_EmergencySafe();
  key = AppPlatform_IrqLock();
  g_motor.authorized = false;
  g_motor.standby_asserted = false;
  g_motor.motor_a_effort = 0.0f;
  g_motor.motor_b_effort = 0.0f;
  g_motor.motor_a_duty_counts = 0U;
  g_motor.motor_b_duty_counts = 0U;
  g_motor.stop_mode = APP_MOTOR_STOP_COAST;
  AppPlatform_IrqUnlock(key);
}

void AppMotor_EmergencySafe(void)
{
  GPIOC->BSRR = ((uint32_t)STBY_Pin << 16U);
  GPIOB->BSRR = ((uint32_t)(AIN1_Pin | AIN2_Pin | BIN1_Pin | BIN2_Pin) << 16U);
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
}

void AppMotor_GetSnapshot(AppMotorSnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_motor;
  AppPlatform_IrqUnlock(key);
}
