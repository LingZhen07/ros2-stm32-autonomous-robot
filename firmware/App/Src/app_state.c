#include "app_state.h"

#include "app_platform.h"

static volatile AppSystemState g_system_state = APP_SYSTEM_BOOT;
static volatile uint32_t g_fault_flags;
static volatile uint32_t g_reset_reason;
static volatile uint32_t g_state_changed_ms;

void AppState_EarlyInit(void)
{
  uint32_t reason = APP_RESET_REASON_NONE;

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
  {
    reason |= APP_RESET_REASON_PIN;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
  {
    reason |= APP_RESET_REASON_SOFTWARE;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
  {
    reason |= APP_RESET_REASON_IWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET)
  {
    reason |= APP_RESET_REASON_WWDG;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
  {
    reason |= APP_RESET_REASON_BROWNOUT;
  }
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET)
  {
    reason |= APP_RESET_REASON_LOW_POWER;
  }

  g_reset_reason = reason;
  g_fault_flags = APP_FAULT_NONE;
  g_system_state = APP_SYSTEM_BOOT;
  g_state_changed_ms = HAL_GetTick();
  __HAL_RCC_CLEAR_RESET_FLAGS();
}

void AppState_SetSystemState(AppSystemState state, uint32_t now_ms)
{
  const uint32_t key = AppPlatform_IrqLock();
  if (g_system_state != state)
  {
    g_system_state = state;
    g_state_changed_ms = now_ms;
  }
  AppPlatform_IrqUnlock(key);
}

AppSystemState AppState_GetSystemState(void)
{
  return g_system_state;
}

void AppState_SetFault(uint32_t flags)
{
  const uint32_t key = AppPlatform_IrqLock();
  g_fault_flags |= flags;
  AppPlatform_IrqUnlock(key);
}

void AppState_ClearFault(uint32_t flags)
{
  const uint32_t key = AppPlatform_IrqLock();
  g_fault_flags &= ~flags;
  AppPlatform_IrqUnlock(key);
}

uint32_t AppState_GetFaultFlags(void)
{
  return g_fault_flags;
}

bool AppState_HasCriticalFault(void)
{
  return ((g_fault_flags & APP_FAULT_CRITICAL_MASK) != 0U);
}

void AppState_GetSnapshot(AppStateSnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }

  key = AppPlatform_IrqLock();
  snapshot->system_state = g_system_state;
  snapshot->fault_flags = g_fault_flags;
  snapshot->reset_reason = g_reset_reason;
  snapshot->state_changed_ms = g_state_changed_ms;
  AppPlatform_IrqUnlock(key);
}

const char *AppState_SystemStateName(AppSystemState state)
{
  switch (state)
  {
    case APP_SYSTEM_BOOT:   return "BOOT";
    case APP_SYSTEM_INIT:   return "INIT";
    case APP_SYSTEM_SAFE:   return "SAFE";
    case APP_SYSTEM_READY:  return "READY";
    case APP_SYSTEM_ACTIVE: return "ACTIVE";
    case APP_SYSTEM_FAULT:  return "FAULT";
    default:                return "UNKNOWN";
  }
}
