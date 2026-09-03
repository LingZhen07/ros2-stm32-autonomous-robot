#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Development firmware identity for the integrated drivetrain-control baseline. */
#define APP_VERSION_MAJOR                       0U
#define APP_VERSION_MINOR                       5U
#define APP_VERSION_PATCH                       4U
#define APP_VERSION_STRING                      "0.5.4"

/* Clock and peripheral invariants frozen by Pin Allocation v1. */
#define APP_SYSCLK_HZ                           170000000UL
#define APP_MOTOR_PWM_HZ                        10000UL
#define APP_MOTOR_PWM_PERIOD_COUNTS             17000UL
#define APP_ENCODER_COUNTER_PERIOD              65535UL

/*
 * User-measured commissioning geometry (2026-08-28). These are intentionally
 * not final effective calibration values until travel/rotation validation.
 */
#define APP_DRIVETRAIN_WHEEL_RADIUS_M           0.023f
#define APP_DRIVETRAIN_WHEEL_TRACK_M            0.125f

/*
 * Direct 10-wheel-revolution measurements (2026-08-29). Keep the sides
 * independent during commissioning; do not replace them with an average.
 */
#define APP_DRIVETRAIN_LEFT_COUNTS_PER_WHEEL_REV  1060.8f
#define APP_DRIVETRAIN_RIGHT_COUNTS_PER_WHEEL_REV 1059.5f

/*
 * Production drivetrain commissioning envelope. These values are deliberately below a
 * claimed hardware maximum: installed-motor current/speed and TB6612 module
 * thermal margin have not yet been measured under robot load.
 */
#define APP_DRIVETRAIN_MAX_BODY_LINEAR_SPEED_MPS   0.30f
#define APP_DRIVETRAIN_MAX_BODY_ANGULAR_SPEED_RADPS 1.50f
#define APP_DRIVETRAIN_MAX_WHEEL_RATE_CPS          3000.0f

/*
 * Initial closed-loop commissioning parameters. Left/right values remain
 * independent so physical tuning does not require a control-architecture
 * change. Target slew is about 0.60 m/s^2 at the measured encoder scales.
 */
#define APP_CONTROL_LEFT_KP                     0.00020f
#define APP_CONTROL_LEFT_KI                     0.00060f
#define APP_CONTROL_LEFT_KD                     0.0f
#define APP_CONTROL_RIGHT_KP                    0.00020f
#define APP_CONTROL_RIGHT_KI                    0.00060f
#define APP_CONTROL_RIGHT_KD                    0.0f
#define APP_CONTROL_INTEGRATOR_LIMIT            700.0f
#define APP_CONTROL_OUTPUT_LIMIT                0.60f
#define APP_CONTROL_TARGET_RAMP_RATE_CPS2       4400.0f

/* Commissioning scheduling defaults. They are centralized for measurement-led tuning. */
#define APP_SUPERVISOR_PERIOD_MS                20U
#define APP_MOTOR_CONTROL_PERIOD_MS             10U
#define APP_IMU_TASK_WAIT_MS                    20U
#define APP_IMU_RETRY_PERIOD_MS                 1000U
#define APP_COMMUNICATION_PERIOD_MS             5U
#define APP_BATTERY_SAMPLE_PERIOD_MS            100U
#define APP_TELEMETRY_PERIOD_MS                 50U

/* Protocol v1 production CAN scheduling and supervision. */
#define APP_CAN_SYSTEM_STATUS_PERIOD_MS         100U
#define APP_CAN_WHEEL_STATE_PERIOD_MS           20U
#define APP_CAN_IMU_PERIOD_MS                   10U
#define APP_CAN_BATTERY_PERIOD_MS               500U
#define APP_CAN_HOST_HEARTBEAT_TIMEOUT_MS       500U
#define APP_CAN_AUTHORITY_TIMEOUT_MS            500U
#define APP_CAN_BUS_RECOVERY_PERIOD_MS          1000U
#define APP_CAN_RX_QUEUE_DEPTH                  8U

#define APP_COMMAND_DEFAULT_TIMEOUT_MS          250U
#define APP_COMMAND_MAX_TIMEOUT_MS              1000U
#define APP_SUPERVISOR_STARTUP_GRACE_MS         1000U
#define APP_MOTOR_HEARTBEAT_TIMEOUT_MS          100U
#define APP_COMMUNICATION_HEARTBEAT_TIMEOUT_MS  500U

/* IWDG: nominal 4 s at the nominal 32 kHz LSI (prescaler 64, reload 1999). */
#define APP_WATCHDOG_NOMINAL_TIMEOUT_MS         4000U

/* Uncalibrated commissioning defaults; no battery safety threshold uses these values. */
#define APP_ADC_FULL_SCALE_COUNTS               4095.0f
#define APP_ADC_REFERENCE_VOLTS_DEFAULT         3.3f
#define APP_BATTERY_DIVIDER_RATIO_DEFAULT       11.0f
#define APP_BATTERY_FILTER_ALPHA                0.10f
#define APP_ENCODER_FILTER_ALPHA                0.25f
#define APP_ENCODER_SAMPLE_VALIDITY_LIMIT_MS    (APP_MOTOR_CONTROL_PERIOD_MS * 5U)

#define APP_IMU_WHO_AM_I_EXPECTED               0x47U
#define APP_IMU_ACCEL_LSB_PER_G                  8192.0f
#define APP_IMU_GYRO_LSB_PER_DPS                 65.5f

#define APP_DIAGNOSTIC_INPUT_LENGTH             64U
#define APP_DIAGNOSTIC_OUTPUT_LENGTH            1024U
#define APP_DIAGNOSTIC_RX_QUEUE_DEPTH           64U
#define APP_DIAGNOSTIC_TX_QUEUE_DEPTH           3U
#define APP_DIAGNOSTIC_WATCH_DEFAULT_HZ         1U

#endif /* APP_CONFIG_H */
