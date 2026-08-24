#include "app_battery.h"

#include "adc.h"
#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"

#include <math.h>
#include <string.h>

static AppBatterySnapshot g_battery;

bool AppBattery_Init(void)
{
  memset(&g_battery, 0, sizeof(g_battery));
  g_battery.adc_reference_voltage = APP_ADC_REFERENCE_VOLTS_DEFAULT;
  g_battery.divider_ratio = APP_BATTERY_DIVIDER_RATIO_DEFAULT;
  g_battery.divider_calibrated = false;

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  {
    AppState_SetFault(APP_FAULT_BATTERY_MEASUREMENT);
    return false;
  }
  g_battery.adc_calibrated = true;
  return true;
}

bool AppBattery_Sample(uint32_t now_ms)
{
  uint32_t raw;
  float adc_voltage;
  float battery_voltage;
  uint32_t key;

  if (!g_battery.adc_calibrated || (HAL_ADC_Start(&hadc1) != HAL_OK) ||
      (HAL_ADC_PollForConversion(&hadc1, 5U) != HAL_OK))
  {
    (void)HAL_ADC_Stop(&hadc1);
    AppState_SetFault(APP_FAULT_BATTERY_MEASUREMENT);
    g_battery.valid = false;
    return false;
  }

  raw = HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  if (raw > 4095U)
  {
    AppState_SetFault(APP_FAULT_BATTERY_MEASUREMENT);
    g_battery.valid = false;
    return false;
  }

  adc_voltage = ((float)raw / APP_ADC_FULL_SCALE_COUNTS) * g_battery.adc_reference_voltage;
  battery_voltage = adc_voltage * g_battery.divider_ratio;

  key = AppPlatform_IrqLock();
  g_battery.raw_adc = (uint16_t)raw;
  g_battery.adc_voltage = adc_voltage;
  g_battery.estimated_battery_voltage = battery_voltage;
  if (!g_battery.valid)
  {
    g_battery.filtered_battery_voltage = battery_voltage;
  }
  else
  {
    g_battery.filtered_battery_voltage += APP_BATTERY_FILTER_ALPHA *
      (battery_voltage - g_battery.filtered_battery_voltage);
  }
  g_battery.timestamp_ms = now_ms;
  g_battery.sample_age_ms = 0U;
  g_battery.valid = true;
  AppPlatform_IrqUnlock(key);
  AppState_ClearFault(APP_FAULT_BATTERY_MEASUREMENT);
  return true;
}

bool AppBattery_SetCalibration(float adc_reference_voltage, float divider_ratio,
                               bool physically_verified)
{
  uint32_t key;

  if (!isfinite(adc_reference_voltage) || !isfinite(divider_ratio) ||
      (adc_reference_voltage <= 0.0f) || (divider_ratio <= 0.0f))
  {
    return false;
  }

  key = AppPlatform_IrqLock();
  g_battery.adc_reference_voltage = adc_reference_voltage;
  g_battery.divider_ratio = divider_ratio;
  g_battery.divider_calibrated = physically_verified;
  g_battery.valid = false;
  AppPlatform_IrqUnlock(key);
  return true;
}

void AppBattery_GetSnapshot(uint32_t now_ms, AppBatterySnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_battery;
  AppPlatform_IrqUnlock(key);
  snapshot->sample_age_ms = AppPlatform_ElapsedMs(now_ms, snapshot->timestamp_ms);
}
