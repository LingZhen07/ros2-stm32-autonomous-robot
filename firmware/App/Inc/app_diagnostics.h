#ifndef APP_DIAGNOSTICS_H
#define APP_DIAGNOSTICS_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

bool AppDiagnostics_Init(void);
void AppDiagnostics_Process(void);
void AppDiagnostics_EmitPeriodic(uint32_t now_ms);
void AppDiagnostics_UartRxComplete(UART_HandleTypeDef *huart);
void AppDiagnostics_UartTxComplete(UART_HandleTypeDef *huart);
void AppDiagnostics_UartError(UART_HandleTypeDef *huart);

#endif /* APP_DIAGNOSTICS_H */
