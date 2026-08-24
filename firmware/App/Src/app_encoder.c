#include "app_encoder.h"

#include "app_config.h"
#include "app_platform.h"
#include "app_state.h"
#include "tim.h"

#include <string.h>

typedef struct
{
  TIM_HandleTypeDef *timer;
  uint16_t previous_counter;
  AppEncoderSnapshot snapshot;
} AppEncoderChannel;

static AppEncoderChannel g_encoders[APP_ENCODER_COUNT];
static uint32_t g_last_sample_ms;

bool AppEncoder_Init(void)
{
  memset(g_encoders, 0, sizeof(g_encoders));
  g_encoders[APP_ENCODER_1].timer = &htim2;
  g_encoders[APP_ENCODER_2].timer = &htim3;

  if ((htim2.Init.Period != APP_ENCODER_COUNTER_PERIOD) ||
      (htim3.Init.Period != APP_ENCODER_COUNTER_PERIOD))
  {
    AppState_SetFault(APP_FAULT_INTERNAL_CONFIGURATION | APP_FAULT_ENCODER_VALIDITY);
    return false;
  }
  if ((HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) ||
      (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK))
  {
    AppState_SetFault(APP_FAULT_ENCODER_VALIDITY);
    return false;
  }

  for (uint32_t index = 0U; index < APP_ENCODER_COUNT; ++index)
  {
    g_encoders[index].previous_counter =
      (uint16_t)__HAL_TIM_GET_COUNTER(g_encoders[index].timer);
    g_encoders[index].snapshot.raw_counter = g_encoders[index].previous_counter;
    g_encoders[index].snapshot.timestamp_ms = HAL_GetTick();
    g_encoders[index].snapshot.valid = true;
  }
  g_last_sample_ms = HAL_GetTick();
  AppState_ClearFault(APP_FAULT_ENCODER_VALIDITY);
  return true;
}

void AppEncoder_Sample(uint32_t now_ms)
{
  const uint32_t elapsed_ms = AppPlatform_ElapsedMs(now_ms, g_last_sample_ms);

  if (elapsed_ms == 0U)
  {
    return;
  }

  for (uint32_t index = 0U; index < APP_ENCODER_COUNT; ++index)
  {
    AppEncoderChannel *channel = &g_encoders[index];
    const uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(channel->timer);
    const int16_t delta = (int16_t)(uint16_t)(current - channel->previous_counter);
    const float rate = ((float)delta * 1000.0f) / (float)elapsed_ms;
    const uint32_t key = AppPlatform_IrqLock();

    channel->snapshot.raw_counter = current;
    channel->snapshot.delta_counts = delta;
    channel->snapshot.counts_per_interval = delta;
    channel->snapshot.accumulated_counts += (int64_t)delta;
    channel->snapshot.counts_per_second = rate;
    if (channel->snapshot.timestamp_ms == 0U)
    {
      channel->snapshot.filtered_counts_per_second = rate;
    }
    else
    {
      channel->snapshot.filtered_counts_per_second +=
        APP_ENCODER_FILTER_ALPHA * (rate - channel->snapshot.filtered_counts_per_second);
    }
    channel->snapshot.direction = (delta > 0) ? 1 : ((delta < 0) ? -1 : 0);
    channel->snapshot.timestamp_ms = now_ms;
    channel->snapshot.sample_age_ms = 0U;
    channel->snapshot.valid = (elapsed_ms <= (APP_MOTOR_CONTROL_PERIOD_MS * 5U));
    channel->previous_counter = current;
    AppPlatform_IrqUnlock(key);
  }

  g_last_sample_ms = now_ms;
  if ((elapsed_ms > (APP_MOTOR_CONTROL_PERIOD_MS * 5U)))
  {
    AppState_SetFault(APP_FAULT_ENCODER_VALIDITY);
  }
}

void AppEncoder_GetSnapshot(AppEncoderId id, uint32_t now_ms, AppEncoderSnapshot *snapshot)
{
  uint32_t key;

  if ((id >= APP_ENCODER_COUNT) || (snapshot == NULL))
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_encoders[id].snapshot;
  AppPlatform_IrqUnlock(key);
  snapshot->sample_age_ms = AppPlatform_ElapsedMs(now_ms, snapshot->timestamp_ms);
}
