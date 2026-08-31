#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_SYSTEM_BOOT = 0,
  APP_SYSTEM_INIT,
  APP_SYSTEM_SAFE,
  APP_SYSTEM_READY,
  APP_SYSTEM_ACTIVE,
  APP_SYSTEM_FAULT
} AppSystemState;

typedef enum
{
  APP_FAULT_NONE                    = 0U,
  APP_FAULT_IMU_INITIALIZATION      = (1UL << 0),
  APP_FAULT_COMMAND_TIMEOUT         = (1UL << 1),
  APP_FAULT_INVALID_MOTOR_COMMAND   = (1UL << 2),
  APP_FAULT_SUPERVISOR              = (1UL << 3),
  APP_FAULT_ENCODER_VALIDITY        = (1UL << 4),
  APP_FAULT_BATTERY_MEASUREMENT     = (1UL << 5),
  APP_FAULT_CONTROL_SATURATION      = (1UL << 6),
  APP_FAULT_CONTROL_ABNORMAL        = (1UL << 7),
  APP_FAULT_INTERNAL_CONFIGURATION  = (1UL << 8),
  APP_FAULT_RTOS_STACK_OVERFLOW     = (1UL << 9),
  APP_FAULT_RTOS_MALLOC_FAILURE     = (1UL << 10),
  APP_FAULT_FDCAN_COMMUNICATION     = (1UL << 11)
} AppFaultFlag;

#define APP_FAULT_CRITICAL_MASK \
  ((uint32_t)APP_FAULT_COMMAND_TIMEOUT | (uint32_t)APP_FAULT_INVALID_MOTOR_COMMAND | \
   (uint32_t)APP_FAULT_SUPERVISOR | (uint32_t)APP_FAULT_CONTROL_ABNORMAL | \
   (uint32_t)APP_FAULT_INTERNAL_CONFIGURATION | (uint32_t)APP_FAULT_RTOS_STACK_OVERFLOW | \
   (uint32_t)APP_FAULT_RTOS_MALLOC_FAILURE | (uint32_t)APP_FAULT_FDCAN_COMMUNICATION)

typedef enum
{
  APP_RESET_REASON_NONE      = 0U,
  APP_RESET_REASON_PIN       = (1UL << 0),
  APP_RESET_REASON_SOFTWARE  = (1UL << 1),
  APP_RESET_REASON_IWDG      = (1UL << 2),
  APP_RESET_REASON_WWDG      = (1UL << 3),
  APP_RESET_REASON_BROWNOUT  = (1UL << 4),
  APP_RESET_REASON_LOW_POWER = (1UL << 5)
} AppResetReason;

typedef struct
{
  AppSystemState system_state;
  uint32_t fault_flags;
  uint32_t reset_reason;
  uint32_t state_changed_ms;
} AppStateSnapshot;

void AppState_EarlyInit(void);
void AppState_SetSystemState(AppSystemState state, uint32_t now_ms);
AppSystemState AppState_GetSystemState(void);
void AppState_SetFault(uint32_t flags);
void AppState_ClearFault(uint32_t flags);
uint32_t AppState_GetFaultFlags(void);
bool AppState_HasCriticalFault(void);
void AppState_GetSnapshot(AppStateSnapshot *snapshot);
const char *AppState_SystemStateName(AppSystemState state);

#endif /* APP_STATE_H */
