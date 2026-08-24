#include "app_runtime.h"

#include "adc.h"
#include "app_battery.h"
#include "app_command.h"
#include "app_config.h"
#include "app_control.h"
#include "app_drivetrain.h"
#include "app_encoder.h"
#include "app_imu.h"
#include "app_motor.h"
#include "app_safety.h"
#include "app_state.h"
#include "app_supervisor.h"
#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

static bool AppRuntime_GeneratedBaselineValid(void)
{
  const bool safe_gpio =
    (HAL_GPIO_ReadPin(IMU_CS_GPIO_Port, IMU_CS_Pin) == GPIO_PIN_SET) &&
    (HAL_GPIO_ReadPin(STBY_GPIO_Port, STBY_Pin) == GPIO_PIN_RESET) &&
    (HAL_GPIO_ReadPin(AIN1_GPIO_Port, AIN1_Pin) == GPIO_PIN_RESET) &&
    (HAL_GPIO_ReadPin(AIN2_GPIO_Port, AIN2_Pin) == GPIO_PIN_RESET) &&
    (HAL_GPIO_ReadPin(BIN1_GPIO_Port, BIN1_Pin) == GPIO_PIN_RESET) &&
    (HAL_GPIO_ReadPin(BIN2_GPIO_Port, BIN2_Pin) == GPIO_PIN_RESET);
  const bool pwm_safe = (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1) == 0U) &&
                         (__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2) == 0U) &&
                         (htim1.Init.Prescaler == 0U) &&
                         ((htim1.Init.Period + 1UL) == APP_MOTOR_PWM_PERIOD_COUNTS);
  const bool encoders_16_bit = (htim2.Init.Period == APP_ENCODER_COUNTER_PERIOD) &&
                               (htim3.Init.Period == APP_ENCODER_COUNTER_PERIOD);
  const bool uart_valid = (huart2.Init.BaudRate == 115200U) &&
                          (huart2.Init.WordLength == UART_WORDLENGTH_8B) &&
                          (huart2.Init.Parity == UART_PARITY_NONE) &&
                          (huart2.Init.StopBits == UART_STOPBITS_1);
  const bool spi_valid = (hspi1.Init.Mode == SPI_MODE_MASTER) &&
                         (hspi1.Init.CLKPolarity == SPI_POLARITY_LOW) &&
                         (hspi1.Init.CLKPhase == SPI_PHASE_1EDGE) &&
                         (hspi1.Init.NSS == SPI_NSS_SOFT);
  const bool fdcan_valid = (hfdcan1.Init.NominalPrescaler == 17U) &&
                           (hfdcan1.Init.NominalTimeSeg1 == 15U) &&
                           (hfdcan1.Init.NominalTimeSeg2 == 4U);

  return (HAL_RCC_GetSysClockFreq() == APP_SYSCLK_HZ) && safe_gpio && pwm_safe &&
         encoders_16_bit && uart_valid && spi_valid && fdcan_valid;
}

void AppRuntime_EarlyInit(void)
{
  AppState_EarlyInit();
  AppState_SetSystemState(APP_SYSTEM_BOOT, HAL_GetTick());
  __HAL_DBGMCU_FREEZE_IWDG();
}

bool AppRuntime_InitHardware(void)
{
  bool motor_ok;
  bool encoder_ok;

  AppState_SetSystemState(APP_SYSTEM_INIT, HAL_GetTick());
  AppCommand_Init();
  AppDrivetrain_Init();
  AppControl_Init();
  AppSafety_Init();
  AppSupervisor_Init();

  if (!AppRuntime_GeneratedBaselineValid())
  {
    AppState_SetFault(APP_FAULT_INTERNAL_CONFIGURATION);
  }

  motor_ok = AppMotor_Init();
  (void)AppBattery_Init();
  encoder_ok = AppEncoder_Init();
  (void)AppImu_Init();

  AppSafety_SetInitializationComplete(true);
  if (!motor_ok)
  {
    AppSafety_ForceFault(APP_FAULT_INTERNAL_CONFIGURATION);
  }
  else
  {
    AppState_SetSystemState(APP_SYSTEM_SAFE, HAL_GetTick());
  }
  return motor_ok && encoder_ok && !AppState_HasCriticalFault();
}
