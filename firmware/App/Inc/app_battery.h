#ifndef APP_BATTERY_H
#define APP_BATTERY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t raw_adc;
  float adc_voltage;
  float estimated_battery_voltage;
  float filtered_battery_voltage;
  float adc_reference_voltage;
  float divider_ratio;
  uint32_t timestamp_ms;
  uint32_t sample_age_ms;
  bool adc_calibrated;
  bool divider_calibrated;
  bool valid;
} AppBatterySnapshot;

bool AppBattery_Init(void);
bool AppBattery_Sample(uint32_t now_ms);
bool AppBattery_SetCalibration(float adc_reference_voltage, float divider_ratio,
                               bool physically_verified);
void AppBattery_GetSnapshot(uint32_t now_ms, AppBatterySnapshot *snapshot);

#endif /* APP_BATTERY_H */
