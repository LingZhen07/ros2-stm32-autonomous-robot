#include "app_supervisor.h"

#include "app_config.h"
#include "app_platform.h"
#include "app_safety.h"
#include "app_state.h"
#include "iwdg.h"

#include <string.h>

static volatile uint32_t g_heartbeats[APP_RUNTIME_TASK_COUNT];
static volatile bool g_reported[APP_RUNTIME_TASK_COUNT];
static volatile bool g_critical_tasks_healthy;
static volatile bool g_watchdog_refresh_allowed;
static volatile uint32_t g_last_watchdog_refresh_ms;
static uint32_t g_supervisor_started_ms;

void AppSupervisor_Init(void)
{
  const uint32_t key = AppPlatform_IrqLock();
  memset((void *)g_heartbeats, 0, sizeof(g_heartbeats));
  memset((void *)g_reported, 0, sizeof(g_reported));
  g_critical_tasks_healthy = false;
  g_watchdog_refresh_allowed = false;
  g_last_watchdog_refresh_ms = 0U;
  g_supervisor_started_ms = HAL_GetTick();
  AppPlatform_IrqUnlock(key);
}

void AppSupervisor_ReportTask(AppRuntimeTaskId task_id, uint32_t now_ms)
{
  uint32_t key;

  if (task_id >= APP_RUNTIME_TASK_COUNT)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  g_heartbeats[task_id] = now_ms;
  g_reported[task_id] = true;
  AppPlatform_IrqUnlock(key);
}

void AppSupervisor_RunCycle(uint32_t now_ms)
{
  bool motor_ok;
  bool communication_ok;
  const uint32_t watchdog_inhibit_faults = APP_FAULT_SUPERVISOR |
                                            APP_FAULT_RTOS_STACK_OVERFLOW |
                                            APP_FAULT_RTOS_MALLOC_FAILURE;

  motor_ok = g_reported[APP_RUNTIME_TASK_MOTOR] &&
             (AppPlatform_ElapsedMs(now_ms, g_heartbeats[APP_RUNTIME_TASK_MOTOR]) <=
              APP_MOTOR_HEARTBEAT_TIMEOUT_MS);
  communication_ok = g_reported[APP_RUNTIME_TASK_COMMUNICATION] &&
                     (AppPlatform_ElapsedMs(now_ms, g_heartbeats[APP_RUNTIME_TASK_COMMUNICATION]) <=
                      APP_COMMUNICATION_HEARTBEAT_TIMEOUT_MS);

  g_critical_tasks_healthy = motor_ok && communication_ok;
  g_watchdog_refresh_allowed = g_critical_tasks_healthy &&
                               ((AppState_GetFaultFlags() & watchdog_inhibit_faults) == 0U);

  if (!g_critical_tasks_healthy &&
      (AppPlatform_ElapsedMs(now_ms, g_supervisor_started_ms) > APP_SUPERVISOR_STARTUP_GRACE_MS))
  {
    AppSafety_ForceFault(APP_FAULT_SUPERVISOR);
    g_watchdog_refresh_allowed = false;
    return;
  }

  AppSafety_Update(now_ms);
  if (g_watchdog_refresh_allowed && (hiwdg.Instance == IWDG))
  {
    if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK)
    {
      g_last_watchdog_refresh_ms = now_ms;
    }
    else
    {
      g_watchdog_refresh_allowed = false;
      AppSafety_ForceFault(APP_FAULT_SUPERVISOR);
    }
  }
}

void AppSupervisor_GetSnapshot(uint32_t now_ms, AppSupervisorSnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  snapshot->critical_tasks_healthy = g_critical_tasks_healthy;
  snapshot->watchdog_refresh_allowed = g_watchdog_refresh_allowed;
  snapshot->last_watchdog_refresh_ms = g_last_watchdog_refresh_ms;
  snapshot->motor_heartbeat_age_ms = g_reported[APP_RUNTIME_TASK_MOTOR]
    ? AppPlatform_ElapsedMs(now_ms, g_heartbeats[APP_RUNTIME_TASK_MOTOR]) : UINT32_MAX;
  snapshot->communication_heartbeat_age_ms = g_reported[APP_RUNTIME_TASK_COMMUNICATION]
    ? AppPlatform_ElapsedMs(now_ms, g_heartbeats[APP_RUNTIME_TASK_COMMUNICATION]) : UINT32_MAX;
  AppPlatform_IrqUnlock(key);
}
