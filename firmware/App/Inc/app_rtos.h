#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdbool.h>
#include <stdint.h>

bool AppRtos_InitObjects(void);
bool AppRtos_CreateThreads(void);
void AppRtos_SupervisorTask(void *argument);
void AppRtos_AssertFailed(const char *file, uint32_t line);

#endif /* APP_RTOS_H */
