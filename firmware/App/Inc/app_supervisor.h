#ifndef APP_SUPERVISOR_H
#define APP_SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
  APP_RUNTIME_TASK_MOTOR = 0,
  APP_RUNTIME_TASK_COMMUNICATION,
  APP_RUNTIME_TASK_IMU,
  APP_RUNTIME_TASK_TELEMETRY,
  APP_RUNTIME_TASK_COUNT
} AppRuntimeTaskId;

typedef struct
{
  bool critical_tasks_healthy;
  bool watchdog_refresh_allowed;
  uint32_t last_watchdog_refresh_ms;
  uint32_t motor_heartbeat_age_ms;
  uint32_t communication_heartbeat_age_ms;
} AppSupervisorSnapshot;

void AppSupervisor_Init(void);
void AppSupervisor_ReportTask(AppRuntimeTaskId task_id, uint32_t now_ms);
void AppSupervisor_RunCycle(uint32_t now_ms);
void AppSupervisor_GetSnapshot(uint32_t now_ms, AppSupervisorSnapshot *snapshot);

#endif /* APP_SUPERVISOR_H */
