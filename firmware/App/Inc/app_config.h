#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* Development firmware identity for the integrated M1-M3 baseline. */
#define APP_VERSION_MAJOR                       0U
#define APP_VERSION_MINOR                       3U
#define APP_VERSION_PATCH                       0U
#define APP_VERSION_STRING                      "0.3.0"

/* Clock and peripheral invariants frozen by Pin Allocation v1. */
#define APP_SYSCLK_HZ                           170000000UL
#define APP_MOTOR_PWM_HZ                        10000UL
#define APP_MOTOR_PWM_PERIOD_COUNTS             17000UL
#define APP_ENCODER_COUNTER_PERIOD              65535UL

/* Commissioning scheduling defaults. They are centralized for measurement-led tuning. */
#define APP_SUPERVISOR_PERIOD_MS                20U
#define APP_MOTOR_CONTROL_PERIOD_MS             10U
#define APP_IMU_TASK_WAIT_MS                    20U
#define APP_IMU_RETRY_PERIOD_MS                 1000U
#define APP_COMMUNICATION_PERIOD_MS             5U
#define APP_BATTERY_SAMPLE_PERIOD_MS            100U
#define APP_TELEMETRY_PERIOD_MS                 50U
#define APP_DIAGNOSTIC_PERIOD_MS                1000U

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

#define APP_IMU_WHO_AM_I_EXPECTED               0x47U
#define APP_IMU_ACCEL_LSB_PER_G                  8192.0f
#define APP_IMU_GYRO_LSB_PER_DPS                 65.5f

#define APP_DIAGNOSTIC_LINE_LENGTH              192U
#define APP_DIAGNOSTIC_RX_QUEUE_DEPTH           64U
#define APP_DIAGNOSTIC_TX_QUEUE_DEPTH           8U

#endif /* APP_CONFIG_H */
