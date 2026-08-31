#ifndef APP_CAN_H
#define APP_CAN_H

#include "cmsis_os2.h"
#include "fdcan.h"

#include <stdbool.h>
#include <stdint.h>

#define APP_CAN_TASK_FLAG_EVENT (1UL << 0)

typedef struct
{
  bool initialized;
  bool controller_error_active;
  bool error_warning;
  bool error_passive;
  bool bus_off;
  bool heartbeat_fresh;
  bool bridge_ready;
  bool session_active;
  bool authority_disarmed_seen;
  bool authority_fresh;
  bool authority_armed;
  bool rx_software_overflow_latched;
  bool rx_hardware_lost_latched;
  bool tx_failure_latched;
  bool protocol_rejection_latched;
  bool version_mismatch_latched;
  uint32_t active_session_id;
  uint16_t last_command_sequence;
  uint16_t last_authority_sequence;
  uint16_t last_heartbeat_sequence;
  bool command_sequence_seen;
  bool authority_sequence_seen;
  bool heartbeat_sequence_seen;
  uint32_t command_age_ms;
  uint32_t authority_age_ms;
  uint32_t heartbeat_age_ms;
  uint32_t rx_frames;
  uint32_t rx_accepted;
  uint32_t rx_rejected;
  uint32_t rx_duplicate;
  uint32_t rx_out_of_order;
  uint32_t rx_overflow;
  uint32_t tx_frames;
  uint32_t tx_failures;
  uint32_t bus_off_count;
} AppCanSnapshot;

bool AppCan_Init(void);
void AppCan_RegisterTask(osThreadId_t task_handle);
void AppCan_Process(uint32_t now_ms);
void AppCan_GetSnapshot(uint32_t now_ms, AppCanSnapshot *snapshot);

void AppCan_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupt_flags);
void AppCan_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupt_flags);
void AppCan_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);

#endif /* APP_CAN_H */
