#ifndef APP_PLATFORM_H
#define APP_PLATFORM_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

static inline uint32_t AppPlatform_IrqLock(void)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static inline void AppPlatform_IrqUnlock(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static inline uint32_t AppPlatform_ElapsedMs(uint32_t now_ms, uint32_t then_ms)
{
  return now_ms - then_ms;
}

static inline bool AppPlatform_DeadlineReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

#endif /* APP_PLATFORM_H */
