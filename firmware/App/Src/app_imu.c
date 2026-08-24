#include "app_imu.h"

#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"
#include "main.h"
#include "spi.h"

#include <string.h>

#define ICM42688_REG_DEVICE_CONFIG   0x11U
#define ICM42688_REG_INT_CONFIG      0x14U
#define ICM42688_REG_ACCEL_DATA_X1   0x1FU
#define ICM42688_REG_INT_STATUS      0x2DU
#define ICM42688_REG_PWR_MGMT0       0x4EU
#define ICM42688_REG_GYRO_CONFIG0    0x4FU
#define ICM42688_REG_ACCEL_CONFIG0   0x50U
#define ICM42688_REG_INT_SOURCE0     0x65U
#define ICM42688_REG_WHO_AM_I        0x75U
#define ICM42688_REG_BANK_SEL        0x76U

#define ICM42688_SPI_READ            0x80U
#define ICM42688_SOFT_RESET          0x01U
#define ICM42688_INT_ACTIVE_HIGH_PP  0x1BU
#define ICM42688_UI_DATA_READY_INT1  0x08U
#define ICM42688_INT_STATUS_DRDY     0x08U
#define ICM42688_ACCEL_4G_100HZ      0x48U
#define ICM42688_GYRO_500DPS_100HZ   0x48U
#define ICM42688_ACCEL_GYRO_LOW_NOISE 0x0FU
#define APP_IMU_THREAD_FLAG_DATA_READY (1UL << 0)

#define APP_STANDARD_GRAVITY_MPS2 9.80665f
#define APP_DEGREES_TO_RADIANS    0.01745329251994329577f

static AppImuSnapshot g_imu;
static osThreadId_t g_imu_task;
static volatile bool g_irq_pending;
static uint32_t g_last_init_attempt_ms;
static uint8_t g_consecutive_errors;

static bool AppImu_ReadRegisters(uint8_t address, uint8_t *data, uint16_t length)
{
  uint8_t command = address | ICM42688_SPI_READ;
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(&hspi1, &command, 1U, 10U);
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi1, data, length, 10U);
  }
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return (status == HAL_OK);
}

static bool AppImu_WriteRegister(uint8_t address, uint8_t value)
{
  uint8_t data[2] = {(uint8_t)(address & (uint8_t)~ICM42688_SPI_READ), value};
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
  status = HAL_SPI_Transmit(&hspi1, data, sizeof(data), 10U);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  return (status == HAL_OK);
}

static bool AppImu_InitializeDevice(void)
{
  uint8_t who_am_i = 0U;

  g_imu.state = APP_IMU_STATE_INITIALIZING;
  g_last_init_attempt_ms = HAL_GetTick();
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(3U);

  if (!AppImu_WriteRegister(ICM42688_REG_BANK_SEL, 0U) ||
      !AppImu_WriteRegister(ICM42688_REG_DEVICE_CONFIG, ICM42688_SOFT_RESET))
  {
    goto failure;
  }
  HAL_Delay(3U);

  if (!AppImu_ReadRegisters(ICM42688_REG_WHO_AM_I, &who_am_i, 1U))
  {
    goto failure;
  }
  g_imu.who_am_i = who_am_i;
  if (who_am_i != APP_IMU_WHO_AM_I_EXPECTED)
  {
    goto failure;
  }

  if (!AppImu_WriteRegister(ICM42688_REG_INT_CONFIG, ICM42688_INT_ACTIVE_HIGH_PP) ||
      !AppImu_WriteRegister(ICM42688_REG_GYRO_CONFIG0, ICM42688_GYRO_500DPS_100HZ) ||
      !AppImu_WriteRegister(ICM42688_REG_ACCEL_CONFIG0, ICM42688_ACCEL_4G_100HZ) ||
      !AppImu_WriteRegister(ICM42688_REG_PWR_MGMT0, ICM42688_ACCEL_GYRO_LOW_NOISE))
  {
    goto failure;
  }
  HAL_Delay(50U);
  if (!AppImu_WriteRegister(ICM42688_REG_INT_SOURCE0, ICM42688_UI_DATA_READY_INT1))
  {
    goto failure;
  }

  g_imu.state = APP_IMU_STATE_READY;
  g_imu.valid = false;
  g_consecutive_errors = 0U;
  AppState_ClearFault(APP_FAULT_IMU_INITIALIZATION);
  return true;

failure:
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  g_imu.state = APP_IMU_STATE_FAULT;
  g_imu.valid = false;
  AppState_SetFault(APP_FAULT_IMU_INITIALIZATION);
  return false;
}

bool AppImu_Init(void)
{
  memset(&g_imu, 0, sizeof(g_imu));
  g_imu.state = APP_IMU_STATE_NOT_INITIALIZED;
  g_imu_task = NULL;
  g_irq_pending = false;
  g_last_init_attempt_ms = 0U;
  g_consecutive_errors = 0U;
  return AppImu_InitializeDevice();
}

void AppImu_RegisterTask(osThreadId_t task_handle)
{
  g_imu_task = task_handle;
}

bool AppImu_Service(uint32_t now_ms)
{
  uint8_t status = 0U;
  uint8_t data[12];
  bool data_ready;

  if (g_imu.state != APP_IMU_STATE_READY)
  {
    if (AppPlatform_ElapsedMs(now_ms, g_last_init_attempt_ms) >= APP_IMU_RETRY_PERIOD_MS)
    {
      return AppImu_InitializeDevice();
    }
    return false;
  }

  data_ready = g_irq_pending;
  if (!AppImu_ReadRegisters(ICM42688_REG_INT_STATUS, &status, 1U))
  {
    goto sample_failure;
  }
  data_ready = data_ready || ((status & ICM42688_INT_STATUS_DRDY) != 0U);
  if (!data_ready)
  {
    return true;
  }

  g_irq_pending = false;
  if (!AppImu_ReadRegisters(ICM42688_REG_ACCEL_DATA_X1, data, sizeof(data)))
  {
    goto sample_failure;
  }

  {
    const uint32_t key = AppPlatform_IrqLock();
    for (uint32_t axis = 0U; axis < 3U; ++axis)
    {
      const uint32_t accel_offset = axis * 2U;
      const uint32_t gyro_offset = 6U + (axis * 2U);
      g_imu.accel_raw[axis] = (int16_t)(((uint16_t)data[accel_offset] << 8U) |
                                        data[accel_offset + 1U]);
      g_imu.gyro_raw[axis] = (int16_t)(((uint16_t)data[gyro_offset] << 8U) |
                                       data[gyro_offset + 1U]);
      g_imu.acceleration_mps2[axis] =
        ((float)g_imu.accel_raw[axis] / APP_IMU_ACCEL_LSB_PER_G) * APP_STANDARD_GRAVITY_MPS2;
      g_imu.angular_velocity_radps[axis] =
        ((float)g_imu.gyro_raw[axis] / APP_IMU_GYRO_LSB_PER_DPS) * APP_DEGREES_TO_RADIANS;
    }
    g_imu.timestamp_ms = now_ms;
    g_imu.sample_age_ms = 0U;
    g_imu.data_ready = true;
    g_imu.valid = true;
    AppPlatform_IrqUnlock(key);
  }
  g_consecutive_errors = 0U;
  return true;

sample_failure:
  if (++g_consecutive_errors >= 3U)
  {
    g_imu.state = APP_IMU_STATE_FAULT;
    g_imu.valid = false;
    g_last_init_attempt_ms = now_ms;
    AppState_SetFault(APP_FAULT_IMU_INITIALIZATION);
  }
  return false;
}

void AppImu_GetSnapshot(uint32_t now_ms, AppImuSnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_imu;
  AppPlatform_IrqUnlock(key);
  snapshot->sample_age_ms = AppPlatform_ElapsedMs(now_ms, snapshot->timestamp_ms);
}

void AppImu_NotifyExti(uint16_t gpio_pin)
{
  if (gpio_pin == IMU_INT1_Pin)
  {
    g_imu.interrupt_1_count++;
    g_irq_pending = true;
    if (g_imu_task != NULL)
    {
      (void)osThreadFlagsSet(g_imu_task, APP_IMU_THREAD_FLAG_DATA_READY);
    }
  }
  else if (gpio_pin == IMU_INT2_Pin)
  {
    g_imu.interrupt_2_count++;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  AppImu_NotifyExti(GPIO_Pin);
}
