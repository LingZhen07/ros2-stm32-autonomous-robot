#ifndef APP_IMU_H
#define APP_IMU_H

#include "cmsis_os2.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_IMU_STATE_NOT_INITIALIZED = 0,
  APP_IMU_STATE_INITIALIZING,
  APP_IMU_STATE_READY,
  APP_IMU_STATE_FAULT
} AppImuState;

typedef struct
{
  AppImuState state;
  uint8_t who_am_i;
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float acceleration_mps2[3];
  float angular_velocity_radps[3];
  uint32_t interrupt_1_count;
  uint32_t interrupt_2_count;
  uint32_t timestamp_ms;
  uint32_t sample_age_ms;
  bool data_ready;
  bool valid;
} AppImuSnapshot;

bool AppImu_Init(void);
void AppImu_RegisterTask(osThreadId_t task_handle);
bool AppImu_Service(uint32_t now_ms);
void AppImu_GetSnapshot(uint32_t now_ms, AppImuSnapshot *snapshot);
void AppImu_NotifyExti(uint16_t gpio_pin);

#endif /* APP_IMU_H */
