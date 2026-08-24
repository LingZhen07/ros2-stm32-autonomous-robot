#ifndef APP_SAFETY_H
#define APP_SAFETY_H

#include "app_command.h"

#include <stdbool.h>
#include <stdint.h>

void AppSafety_Init(void);
void AppSafety_SetInitializationComplete(bool complete);
void AppSafety_Update(uint32_t now_ms);
void AppSafety_ForceFault(uint32_t fault_flags);
bool AppSafety_ClearRecoverableFaults(void);
bool AppSafety_MotionAuthorized(void);
bool AppSafety_CommandModeReady(const AppCommandSnapshot *command);
bool AppSafety_ValidateActiveCommand(uint32_t now_ms, AppCommandSnapshot *command);

#endif /* APP_SAFETY_H */
