#include "app_can.h"

#include "app_command.h"
#include "app_config.h"
#include "app_control.h"
#include "app_drivetrain.h"
#include "app_motor.h"
#include "app_platform.h"
#include "app_protocol.h"
#include "app_safety.h"
#include "app_state.h"
#include "app_telemetry.h"

#include <string.h>

typedef struct
{
  uint32_t identifier;
  uint32_t received_ms;
  uint8_t length;
  uint8_t data[64];
  uint32_t id_type;
  uint32_t frame_type;
  uint32_t fd_format;
  uint32_t bit_rate_switch;
} AppCanRxFrame;

typedef struct
{
  bool seen;
  uint16_t last;
} AppCanSequenceTracker;

typedef enum
{
  APP_CAN_SEQUENCE_NEW = 0,
  APP_CAN_SEQUENCE_DUPLICATE,
  APP_CAN_SEQUENCE_OUT_OF_ORDER
} AppCanSequenceResult;

static AppCanRxFrame g_rx_queue[APP_CAN_RX_QUEUE_DEPTH];
static volatile uint8_t g_rx_head;
static volatile uint8_t g_rx_tail;
static volatile bool g_rx_overflow_pending;
static volatile bool g_bus_off_pending;
static volatile bool g_rx_hardware_lost_pending;
static osThreadId_t g_communication_task;
static AppCanSnapshot g_state;
static AppCanSequenceTracker g_heartbeat_sequence;
static AppCanSequenceTracker g_authority_sequence;
static AppCanSequenceTracker g_command_sequence;
static uint32_t g_last_heartbeat_ms;
static uint32_t g_last_authority_ms;
static uint32_t g_last_command_ms;
static uint32_t g_next_bus_recovery_ms;
static uint32_t g_next_system_status_ms;
static uint32_t g_next_wheel_state_ms;
static uint32_t g_next_imu_ms;
static uint32_t g_next_battery_ms;
static uint16_t g_system_status_sequence;
static uint16_t g_wheel_state_sequence;
static uint16_t g_imu_sequence;
static uint16_t g_battery_sequence;

static uint8_t AppCan_DlcToLength(uint32_t dlc)
{
  static const uint8_t lengths[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
  };
  return (dlc < 16U) ? lengths[dlc] : 0U;
}

static uint32_t AppCan_LengthToDlc(uint8_t length)
{
  switch (length)
  {
    case 16U: return FDCAN_DLC_BYTES_16;
    case 32U: return FDCAN_DLC_BYTES_32;
    case 48U: return FDCAN_DLC_BYTES_48;
    case 64U: return FDCAN_DLC_BYTES_64;
    default:  return FDCAN_DLC_BYTES_0;
  }
}

static bool AppCan_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return ((int32_t)(now_ms - deadline_ms) >= 0);
}

static void AppCan_AdvanceDeadline(uint32_t *deadline_ms, uint32_t period_ms,
                                   uint32_t now_ms)
{
  do
  {
    *deadline_ms += period_ms;
  } while (AppCan_TimeReached(now_ms, *deadline_ms));
}

static AppCanSequenceResult AppCan_CheckSequence(AppCanSequenceTracker *tracker,
                                                 uint16_t sequence)
{
  uint16_t delta;

  if (!tracker->seen)
  {
    tracker->seen = true;
    tracker->last = sequence;
    return APP_CAN_SEQUENCE_NEW;
  }
  delta = (uint16_t)(sequence - tracker->last);
  if (delta == 0U)
  {
    return APP_CAN_SEQUENCE_DUPLICATE;
  }
  if (delta >= 0x8000U)
  {
    return APP_CAN_SEQUENCE_OUT_OF_ORDER;
  }
  tracker->last = sequence;
  return APP_CAN_SEQUENCE_NEW;
}

static bool AppCan_FdcanOwnsMotion(uint32_t now_ms)
{
  AppCommandSnapshot command;
  AppCommand_GetSnapshot(now_ms, &command);
  return g_state.authority_armed ||
         ((command.source == APP_COMMAND_SOURCE_FDCAN) &&
          (command.valid || command.arm_requested));
}

static void AppCan_ForceFdcanSafe(uint32_t now_ms, uint32_t fault_flags)
{
  const bool owned_motion = AppCan_FdcanOwnsMotion(now_ms);

  g_state.authority_armed = false;
  g_state.authority_fresh = false;
  g_state.authority_disarmed_seen = false;
  if (owned_motion)
  {
    if (fault_flags != 0U)
    {
      AppSafety_ForceFault(fault_flags);
    }
    else
    {
      AppCommand_Disarm();
      AppControl_Reset();
      AppMotor_ForceSafe();
    }
  }
}

static void AppCan_ResetSession(uint32_t now_ms, bool fault_if_moving)
{
  AppCan_ForceFdcanSafe(now_ms, fault_if_moving
    ? (APP_FAULT_COMMAND_TIMEOUT | APP_FAULT_FDCAN_COMMUNICATION) : 0U);
  g_state.session_active = false;
  g_state.bridge_ready = false;
  g_state.heartbeat_fresh = false;
  g_state.active_session_id = 0U;
  memset(&g_heartbeat_sequence, 0, sizeof(g_heartbeat_sequence));
  memset(&g_authority_sequence, 0, sizeof(g_authority_sequence));
  memset(&g_command_sequence, 0, sizeof(g_command_sequence));
  g_state.command_sequence_seen = false;
  g_state.authority_sequence_seen = false;
  g_state.heartbeat_sequence_seen = false;
}

static void AppCan_RecordSequenceResult(AppCanSequenceResult result)
{
  if (result == APP_CAN_SEQUENCE_DUPLICATE)
  {
    g_state.rx_duplicate++;
  }
  else if (result == APP_CAN_SEQUENCE_OUT_OF_ORDER)
  {
    g_state.rx_out_of_order++;
  }
}

static void AppCan_RejectControl(uint32_t now_ms, AppProtocolDecodeResult reason)
{
  g_state.rx_rejected++;
  g_state.protocol_rejection_latched = true;
  if (reason == APP_PROTOCOL_DECODE_VERSION_MISMATCH)
  {
    g_state.version_mismatch_latched = true;
  }
  if (AppCan_FdcanOwnsMotion(now_ms))
  {
    AppCan_ForceFdcanSafe(now_ms, APP_FAULT_INVALID_MOTOR_COMMAND);
  }
}

static bool AppCan_FrameEnvelopeValid(const AppCanRxFrame *frame)
{
  return (frame->id_type == FDCAN_STANDARD_ID) &&
         (frame->frame_type == FDCAN_DATA_FRAME) &&
         (frame->fd_format == FDCAN_FD_CAN) &&
         (frame->bit_rate_switch == FDCAN_BRS_ON);
}

static void AppCan_HandleHeartbeat(const AppCanRxFrame *frame)
{
  AppProtocolHostHeartbeat heartbeat;
  AppProtocolDecodeResult decode = AppProtocol_DecodeHostHeartbeat(
    frame->data, frame->length, &heartbeat);
  AppCanSequenceResult sequence_result;

  if (decode != APP_PROTOCOL_DECODE_OK)
  {
    AppCan_RejectControl(frame->received_ms, decode);
    return;
  }

  if (!g_state.session_active || (heartbeat.session_id != g_state.active_session_id))
  {
    const bool moving = AppCan_FdcanOwnsMotion(frame->received_ms);
    AppCan_ResetSession(frame->received_ms, moving);
    g_state.session_active = true;
    g_state.active_session_id = heartbeat.session_id;
  }

  sequence_result = AppCan_CheckSequence(&g_heartbeat_sequence, heartbeat.sequence);
  if (sequence_result != APP_CAN_SEQUENCE_NEW)
  {
    AppCan_RecordSequenceResult(sequence_result);
    return;
  }

  g_last_heartbeat_ms = frame->received_ms;
  g_state.last_heartbeat_sequence = heartbeat.sequence;
  g_state.heartbeat_sequence_seen = true;
  g_state.heartbeat_fresh = true;
  g_state.bridge_ready = heartbeat.bridge_ready;
  if (!heartbeat.bridge_ready)
  {
    AppCan_ForceFdcanSafe(frame->received_ms, 0U);
  }
  g_state.rx_accepted++;
}

static void AppCan_HandleAuthority(const AppCanRxFrame *frame)
{
  AppProtocolMotionAuthority authority;
  AppProtocolDecodeResult decode = AppProtocol_DecodeMotionAuthority(
    frame->data, frame->length, &authority);
  AppCanSequenceResult sequence_result;

  if (decode != APP_PROTOCOL_DECODE_OK)
  {
    AppCan_RejectControl(frame->received_ms, decode);
    return;
  }
  if (!g_state.session_active || !g_state.heartbeat_fresh || !g_state.bridge_ready ||
      (authority.session_id != g_state.active_session_id))
  {
    AppCan_RejectControl(frame->received_ms, APP_PROTOCOL_DECODE_INVALID_VALUE);
    return;
  }
  sequence_result = AppCan_CheckSequence(&g_authority_sequence, authority.sequence);
  if (sequence_result != APP_CAN_SEQUENCE_NEW)
  {
    AppCan_RecordSequenceResult(sequence_result);
    return;
  }

  if (authority.state == APP_PROTOCOL_AUTHORITY_DISARMED)
  {
    g_state.authority_armed = false;
    g_state.authority_disarmed_seen = true;
    AppCommand_Disarm();
    AppControl_Reset();
    AppMotor_ForceSafe();
    if (!g_state.bus_off && !g_rx_overflow_pending && !g_rx_hardware_lost_pending)
    {
      AppState_ClearFault(APP_FAULT_FDCAN_COMMUNICATION);
      (void)AppSafety_ClearRecoverableFaults();
    }
  }
  else if (!g_state.authority_disarmed_seen)
  {
    AppCan_RejectControl(frame->received_ms, APP_PROTOCOL_DECODE_INVALID_VALUE);
    return;
  }
  else
  {
    g_state.authority_armed = true;
  }
  g_last_authority_ms = frame->received_ms;
  g_state.last_authority_sequence = authority.sequence;
  g_state.authority_sequence_seen = true;
  g_state.authority_fresh = true;
  g_state.rx_accepted++;
}

static void AppCan_HandleMotionCommand(const AppCanRxFrame *frame)
{
  AppProtocolMotionCommand command;
  AppProtocolDecodeResult decode = AppProtocol_DecodeMotionCommand(
    frame->data, frame->length, &command);
  AppCanSequenceResult sequence_result;
  float left_target_cps;
  float right_target_cps;

  if (decode != APP_PROTOCOL_DECODE_OK)
  {
    AppCan_RejectControl(frame->received_ms, decode);
    return;
  }
  if (!g_state.session_active || !g_state.heartbeat_fresh || !g_state.bridge_ready ||
      (command.session_id != g_state.active_session_id))
  {
    AppCan_RejectControl(frame->received_ms, APP_PROTOCOL_DECODE_INVALID_VALUE);
    return;
  }
  sequence_result = AppCan_CheckSequence(&g_command_sequence, command.sequence);
  if (sequence_result != APP_CAN_SEQUENCE_NEW)
  {
    AppCan_RecordSequenceResult(sequence_result);
    return;
  }
  if (command.mode == APP_PROTOCOL_COMMAND_DISABLED)
  {
    AppCommand_Disarm();
    AppControl_Reset();
    AppMotor_ForceSafe();
  }
  else if (!g_state.authority_armed || !g_state.authority_disarmed_seen ||
           !AppControl_AllConfigured() ||
           !AppDrivetrain_BodyToEncoderRates(command.linear_velocity_mps,
                                              command.angular_velocity_radps,
                                              &left_target_cps,
                                              &right_target_cps) ||
           !AppCommand_SubmitBodyVelocity(APP_COMMAND_SOURCE_FDCAN,
                                          command.linear_velocity_mps,
                                          command.angular_velocity_radps,
                                          APP_COMMAND_DEFAULT_TIMEOUT_MS))
  {
    AppCan_RejectControl(frame->received_ms, APP_PROTOCOL_DECODE_INVALID_VALUE);
    return;
  }
  else
  {
    AppCommand_RequestArm();
  }
  g_last_command_ms = frame->received_ms;
  g_state.last_command_sequence = command.sequence;
  g_state.command_sequence_seen = true;
  g_state.rx_accepted++;
}

static void AppCan_HandleFrame(const AppCanRxFrame *frame)
{
  if (!AppCan_FrameEnvelopeValid(frame))
  {
    AppCan_RejectControl(frame->received_ms, APP_PROTOCOL_DECODE_INVALID_VALUE);
    return;
  }

  switch (frame->identifier)
  {
    case APP_PROTOCOL_CAN_ID_HOST_HEARTBEAT:
      AppCan_HandleHeartbeat(frame);
      break;

    case APP_PROTOCOL_CAN_ID_MOTION_AUTHORITY:
      AppCan_HandleAuthority(frame);
      break;

    case APP_PROTOCOL_CAN_ID_MOTION_COMMAND:
      AppCan_HandleMotionCommand(frame);
      break;

    default:
      g_state.rx_rejected++;
      break;
  }
}

static bool AppCan_Dequeue(AppCanRxFrame *frame)
{
  uint32_t key;

  if (frame == NULL)
  {
    return false;
  }
  key = AppPlatform_IrqLock();
  if (g_rx_tail == g_rx_head)
  {
    AppPlatform_IrqUnlock(key);
    return false;
  }
  *frame = g_rx_queue[g_rx_tail];
  g_rx_tail = (uint8_t)((g_rx_tail + 1U) % APP_CAN_RX_QUEUE_DEPTH);
  AppPlatform_IrqUnlock(key);
  return true;
}

static bool AppCan_Send(uint32_t identifier, uint8_t length, const uint8_t *data)
{
  FDCAN_TxHeaderTypeDef header = {0};

  if ((data == NULL) || (AppCan_LengthToDlc(length) == FDCAN_DLC_BYTES_0) ||
      (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U))
  {
    g_state.tx_failures++;
    g_state.tx_failure_latched = true;
    return false;
  }
  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = AppCan_LengthToDlc(length);
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_ON;
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0U;
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, data) != HAL_OK)
  {
    g_state.tx_failures++;
    g_state.tx_failure_latched = true;
    return false;
  }
  g_state.tx_frames++;
  return true;
}

static void AppCan_SendScheduledTelemetry(uint32_t now_ms)
{
  const bool system_due = AppCan_TimeReached(now_ms, g_next_system_status_ms);
  const bool wheel_due = AppCan_TimeReached(now_ms, g_next_wheel_state_ms);
  const bool imu_due = AppCan_TimeReached(now_ms, g_next_imu_ms);
  const bool battery_due = AppCan_TimeReached(now_ms, g_next_battery_ms);
  AppTelemetrySnapshot telemetry;
  AppCanSnapshot communication;
  uint8_t data[64];

  if (!system_due && !wheel_due && !imu_due && !battery_due)
  {
    return;
  }
  AppTelemetry_GetSnapshot(now_ms, &telemetry);
  AppCan_GetSnapshot(now_ms, &communication);

  if (system_due)
  {
    AppProtocol_EncodeSystemStatus(&telemetry, &communication,
                                   g_system_status_sequence, data);
    if (AppCan_Send(APP_PROTOCOL_CAN_ID_SYSTEM_STATUS,
                    APP_PROTOCOL_SYSTEM_STATUS_LENGTH, data))
    {
      g_system_status_sequence++;
    }
    AppCan_AdvanceDeadline(&g_next_system_status_ms,
                           APP_CAN_SYSTEM_STATUS_PERIOD_MS, now_ms);
  }
  if (wheel_due)
  {
    AppProtocol_EncodeWheelState(&telemetry, g_wheel_state_sequence, data);
    if (AppCan_Send(APP_PROTOCOL_CAN_ID_WHEEL_STATE,
                    APP_PROTOCOL_WHEEL_STATE_LENGTH, data))
    {
      g_wheel_state_sequence++;
    }
    AppCan_AdvanceDeadline(&g_next_wheel_state_ms, APP_CAN_WHEEL_STATE_PERIOD_MS, now_ms);
  }
  if (imu_due)
  {
    AppProtocol_EncodeImuData(&telemetry, g_imu_sequence, data);
    if (AppCan_Send(APP_PROTOCOL_CAN_ID_IMU_DATA,
                    APP_PROTOCOL_IMU_DATA_LENGTH, data))
    {
      g_imu_sequence++;
    }
    AppCan_AdvanceDeadline(&g_next_imu_ms, APP_CAN_IMU_PERIOD_MS, now_ms);
  }
  if (battery_due)
  {
    AppProtocol_EncodeBatteryState(&telemetry, g_battery_sequence, data);
    if (AppCan_Send(APP_PROTOCOL_CAN_ID_BATTERY_STATE,
                    APP_PROTOCOL_BATTERY_STATE_LENGTH, data))
    {
      g_battery_sequence++;
    }
    AppCan_AdvanceDeadline(&g_next_battery_ms, APP_CAN_BATTERY_PERIOD_MS, now_ms);
  }
}

static void AppCan_ProcessHealth(uint32_t now_ms)
{
  FDCAN_ProtocolStatusTypeDef protocol_status = {0};
  AppCommandSnapshot command;

  if (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &protocol_status) == HAL_OK)
  {
    g_state.error_warning = (protocol_status.Warning != 0U);
    g_state.error_passive = (protocol_status.ErrorPassive != 0U);
    g_state.bus_off = (protocol_status.BusOff != 0U);
    g_state.controller_error_active = !g_state.error_passive && !g_state.bus_off;
  }

  if (g_bus_off_pending || g_state.bus_off)
  {
    g_bus_off_pending = false;
    if (AppCan_FdcanOwnsMotion(now_ms))
    {
      AppCan_ForceFdcanSafe(now_ms, APP_FAULT_FDCAN_COMMUNICATION);
    }
    if (AppCan_TimeReached(now_ms, g_next_bus_recovery_ms))
    {
      if ((HAL_FDCAN_Stop(&hfdcan1) == HAL_OK) &&
          (HAL_FDCAN_Start(&hfdcan1) == HAL_OK))
      {
        g_state.bus_off = false;
      }
      g_next_bus_recovery_ms = now_ms + APP_CAN_BUS_RECOVERY_PERIOD_MS;
    }
  }

  if (g_rx_overflow_pending || g_rx_hardware_lost_pending)
  {
    if (AppCan_FdcanOwnsMotion(now_ms))
    {
      AppCan_ForceFdcanSafe(now_ms, APP_FAULT_FDCAN_COMMUNICATION);
    }
    g_rx_overflow_pending = false;
    g_rx_hardware_lost_pending = false;
  }

  if (g_state.session_active)
  {
    g_state.heartbeat_fresh =
      (AppPlatform_ElapsedMs(now_ms, g_last_heartbeat_ms) <=
       APP_CAN_HOST_HEARTBEAT_TIMEOUT_MS);
    if (!g_state.heartbeat_fresh)
    {
      AppCan_ResetSession(now_ms, AppCan_FdcanOwnsMotion(now_ms));
    }
  }

  if (g_state.authority_fresh &&
      (AppPlatform_ElapsedMs(now_ms, g_last_authority_ms) >
       APP_CAN_AUTHORITY_TIMEOUT_MS))
  {
    if (g_state.authority_armed)
    {
      AppCan_ForceFdcanSafe(now_ms,
                            APP_FAULT_COMMAND_TIMEOUT | APP_FAULT_FDCAN_COMMUNICATION);
    }
    else
    {
      g_state.authority_fresh = false;
      g_state.authority_disarmed_seen = false;
    }
  }

  AppCommand_GetSnapshot(now_ms, &command);
  if (g_state.authority_armed && (command.source == APP_COMMAND_SOURCE_FDCAN) &&
      command.timed_out)
  {
    AppCan_ForceFdcanSafe(now_ms, APP_FAULT_COMMAND_TIMEOUT);
  }
}

bool AppCan_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};
  const uint32_t notifications = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                 FDCAN_IT_RX_FIFO0_FULL |
                                 FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                 FDCAN_IT_ERROR_WARNING |
                                 FDCAN_IT_ERROR_PASSIVE |
                                 FDCAN_IT_BUS_OFF |
                                 FDCAN_IT_ARB_PROTOCOL_ERROR |
                                 FDCAN_IT_DATA_PROTOCOL_ERROR;
  const uint32_t now_ms = HAL_GetTick();

  memset(&g_state, 0, sizeof(g_state));
  memset(&g_heartbeat_sequence, 0, sizeof(g_heartbeat_sequence));
  memset(&g_authority_sequence, 0, sizeof(g_authority_sequence));
  memset(&g_command_sequence, 0, sizeof(g_command_sequence));
  g_rx_head = 0U;
  g_rx_tail = 0U;
  g_rx_overflow_pending = false;
  g_bus_off_pending = false;
  g_rx_hardware_lost_pending = false;
  g_communication_task = NULL;
  g_next_bus_recovery_ms = now_ms + APP_CAN_BUS_RECOVERY_PERIOD_MS;
  g_next_wheel_state_ms = now_ms + 5U;
  g_next_imu_ms = now_ms + 10U;
  g_next_system_status_ms = now_ms + 15U;
  g_next_battery_ms = now_ms + 25U;
  g_system_status_sequence = 0U;
  g_wheel_state_sequence = 0U;
  g_imu_sequence = 0U;
  g_battery_sequence = 0U;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0U;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = APP_PROTOCOL_CAN_ID_MOTION_AUTHORITY;
  filter.FilterID2 = APP_PROTOCOL_CAN_ID_HOST_HEARTBEAT;

  if ((HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) ||
      (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) ||
      (HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0,
                                       FDCAN_RX_FIFO_BLOCKING) != HAL_OK) ||
      (HAL_FDCAN_ActivateNotification(&hfdcan1, notifications, 0U) != HAL_OK) ||
      (HAL_FDCAN_Start(&hfdcan1) != HAL_OK))
  {
    AppState_SetFault(APP_FAULT_FDCAN_COMMUNICATION);
    return false;
  }
  g_state.initialized = true;
  g_state.controller_error_active = true;
  return true;
}

void AppCan_RegisterTask(osThreadId_t task_handle)
{
  g_communication_task = task_handle;
}

void AppCan_Process(uint32_t now_ms)
{
  AppCanRxFrame frame;

  while (AppCan_Dequeue(&frame))
  {
    AppCan_HandleFrame(&frame);
  }
  AppCan_ProcessHealth(now_ms);
  if (g_state.initialized && !g_state.bus_off)
  {
    AppCan_SendScheduledTelemetry(now_ms);
  }
}

void AppCan_GetSnapshot(uint32_t now_ms, AppCanSnapshot *snapshot)
{
  uint32_t key;

  if (snapshot == NULL)
  {
    return;
  }
  key = AppPlatform_IrqLock();
  *snapshot = g_state;
  AppPlatform_IrqUnlock(key);
  if (snapshot->session_active)
  {
    snapshot->heartbeat_age_ms = AppPlatform_ElapsedMs(now_ms, g_last_heartbeat_ms);
    snapshot->heartbeat_fresh =
      (snapshot->heartbeat_age_ms <= APP_CAN_HOST_HEARTBEAT_TIMEOUT_MS);
  }
  else
  {
    snapshot->heartbeat_age_ms = UINT32_MAX;
  }
  if (snapshot->authority_fresh)
  {
    snapshot->authority_age_ms = AppPlatform_ElapsedMs(now_ms, g_last_authority_ms);
    snapshot->authority_fresh =
      (snapshot->authority_age_ms <= APP_CAN_AUTHORITY_TIMEOUT_MS);
  }
  else
  {
    snapshot->authority_age_ms = snapshot->authority_sequence_seen
      ? AppPlatform_ElapsedMs(now_ms, g_last_authority_ms) : UINT32_MAX;
  }
  snapshot->command_age_ms = snapshot->command_sequence_seen
    ? AppPlatform_ElapsedMs(now_ms, g_last_command_ms) : UINT32_MAX;
}

void AppCan_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupt_flags)
{
  FDCAN_RxHeaderTypeDef header;
  AppCanRxFrame frame;

  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))
  {
    return;
  }
  if ((interrupt_flags & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    g_state.rx_hardware_lost_latched = true;
    g_state.rx_overflow++;
    g_rx_hardware_lost_pending = true;
  }

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U)
  {
    uint8_t next_head;
    memset(&header, 0, sizeof(header));
    memset(&frame, 0, sizeof(frame));
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK)
    {
      break;
    }
    frame.identifier = header.Identifier;
    frame.received_ms = HAL_GetTick();
    frame.length = AppCan_DlcToLength(header.DataLength);
    frame.id_type = header.IdType;
    frame.frame_type = header.RxFrameType;
    frame.fd_format = header.FDFormat;
    frame.bit_rate_switch = header.BitRateSwitch;
    g_state.rx_frames++;

    next_head = (uint8_t)((g_rx_head + 1U) % APP_CAN_RX_QUEUE_DEPTH);
    if (next_head == g_rx_tail)
    {
      g_state.rx_software_overflow_latched = true;
      g_state.rx_overflow++;
      g_rx_overflow_pending = true;
    }
    else
    {
      g_rx_queue[g_rx_head] = frame;
      __DMB();
      g_rx_head = next_head;
    }
  }
  if (g_communication_task != NULL)
  {
    (void)osThreadFlagsSet(g_communication_task, APP_CAN_TASK_FLAG_EVENT);
  }
}

void AppCan_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t interrupt_flags)
{
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))
  {
    return;
  }
  if ((interrupt_flags & FDCAN_IT_BUS_OFF) != 0U)
  {
    g_state.bus_off = true;
    g_state.bus_off_count++;
    g_bus_off_pending = true;
  }
  if (g_communication_task != NULL)
  {
    (void)osThreadFlagsSet(g_communication_task, APP_CAN_TASK_FLAG_EVENT);
  }
}

void AppCan_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  if ((hfdcan == NULL) || (hfdcan->Instance != FDCAN1))
  {
    return;
  }
  if ((hfdcan->ErrorCode & (HAL_FDCAN_ERROR_FIFO_FULL |
                            HAL_FDCAN_ERROR_RAM_ACCESS)) != 0U)
  {
    g_state.tx_failure_latched = true;
    g_state.tx_failures++;
  }
  if (g_communication_task != NULL)
  {
    (void)osThreadFlagsSet(g_communication_task, APP_CAN_TASK_FLAG_EVENT);
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  AppCan_RxFifo0Callback(hfdcan, RxFifo0ITs);
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  AppCan_ErrorStatusCallback(hfdcan, ErrorStatusITs);
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  AppCan_ErrorCallback(hfdcan);
}
