#ifndef APP_ENCODER_H
#define APP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_ENCODER_1 = 0,
  APP_ENCODER_2,
  APP_ENCODER_COUNT
} AppEncoderId;

typedef struct
{
  uint16_t raw_counter;
  int16_t delta_counts;
  int16_t counts_per_interval;
  int64_t accumulated_counts;
  float counts_per_second;
  float filtered_counts_per_second;
  int8_t direction;
  uint32_t timestamp_ms;
  uint32_t sample_age_ms;
  bool valid;
} AppEncoderSnapshot;

bool AppEncoder_Init(void);
void AppEncoder_Sample(uint32_t now_ms);
void AppEncoder_GetSnapshot(AppEncoderId id, uint32_t now_ms, AppEncoderSnapshot *snapshot);

#endif /* APP_ENCODER_H */
