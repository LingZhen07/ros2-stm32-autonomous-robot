# STM32 Production Firmware M1-M5

[中文](firmware.zh-CN.md)

## Status

| Milestone | Status | Evidence boundary |
|---|---|---|
| M1 MCU foundation | `VERIFIED` | Real STM32 execution, SWD, USART2, clock/startup behavior |
| M2 peripheral foundation | `VERIFIED` | Real ADC, encoder, ICM-42688-P, TIM1 PWM behavior |
| M3 real-time control foundation | `VERIFIED` | FreeRTOS, safety, watchdog architecture, TB6612 path, real motor rotation |
| M4 Protocol v1 + firmware FDCAN | `FROZEN / PHYSICALLY INTEGRATED` | Shared contract, firmware integration, and real Orange Pi CAN3 link accepted |
| M5 real-hardware integration | `VERIFIED / PASS` | BODY_COMMAND_READY, Motion Authority, 0.30 m/s closed-loop straight motion, LiDAR stop, authority withdrawal, and STM32 safe stop accepted |

M1-M5 real-hardware acceptance is complete. Protocol 1.0 remains frozen and production transport
remains Orange Pi CAN3 / SocketCAN `can3`.

## MCU baseline

| Item | Frozen configuration |
|---|---|
| Target | STM32G474RET6, LQFP64; DeveBox STM32G474R Ver:20 |
| Clock | 8 MHz HSE, PLL M=2/N=85/R=2, 170 MHz SYSCLK/HCLK/PCLK1/PCLK2 |
| Debug/time base | SWD retained; TIM6 HAL 1 ms time base; SysTick FreeRTOS 1 ms tick |
| PWM | TIM1 CH1/CH2, 10 kHz, PSC=0, ARR=16999, startup CCR=0 |
| Encoders | TIM2/TIM3 hardware encoder mode, both 16-bit range 0..65535 |
| IMU | SPI1 mode 0, ICM-42688-P 100 Hz baseline, EXTI task signaling |
| Battery | ADC1 IN6, calibrated ADC path, software-triggered long sample |
| Diagnostics | USART2 115200 8N1 permanent block-serialized CLI; quiet by default |
| Watchdog | IWDG nominal 4 s; Supervisor-only refresh; debug halt freeze |
| FDCAN | 500 kbit/s nominal, 2 Mbit/s data, FD+BRS, Standard IDs |

The frozen pin map and safe output levels remain in [hardware.md](hardware.md).

## Verified hardware acceptance

The first integrated board session established real evidence for:

- STM32 execution and USART2 bytes;
- physical encoder acquisition;
- ICM-42688-P communication and real inertial samples;
- battery ADC measurement path;
- TIM1 PWM generation;
- TB6612 motor-control electrical path and real motor rotation.

The verified encoder sign observation is:

```text
left wheel forward  -> raw CPS < 0
right wheel forward -> raw CPS > 0
logical forward     -> normalized rate > 0 on both sides
```

The correction is owned by `app_drivetrain`, not the timer encoder driver. Commissioning has now
verified Encoder 1 as the right wheel with forward raw sign `+1`, and Encoder 2 as the left wheel
with forward raw sign `-1`. Raw Encoder 1/2 diagnostics remain unchanged; logical wheel state and
Protocol `0x181` telemetry use right=`+raw`, left=`-raw`. Real commissioning also verified Motor A
as the right wheel and Motor B as the left wheel. Both motor channels require logical-forward sign
`-1`; the normalization remains above the motor GPIO/PWM driver.

## Production source architecture

CubeMX-owned initialization stays under `firmware/Core`; project-owned code stays under
`firmware/App`.

| Module | Responsibility |
|---|---|
| `app_config` | Firmware identity, scheduling, safety and communication constants |
| `app_state` / `app_safety` | BOOT/INIT/SAFE/READY/ACTIVE/FAULT and centralized safe convergence |
| `app_supervisor` | Critical task heartbeat checks and sole watchdog feed ownership |
| `app_command` | Shared validated command, source, timestamp and local freshness |
| `app_drivetrain` | Wheel/motor mapping, verified logical signs, calibration guards, differential drive |
| `app_control` | Independent wheel PI/PID-compatible control, clamp and anti-windup |
| `app_motor` | TB6612 direction/PWM/STBY ownership and emergency-safe output |
| `app_encoder` | Wrap-safe 16-bit delta, cumulative counts and raw/filtered CPS |
| `app_imu` | ICM register driver, WHO_AM_I, data acquisition and interrupt state |
| `app_battery` | ADC calibration, divider model, filtered estimate and calibration validity |
| `app_telemetry` | Coherent sensor/control/safety snapshot |
| `app_protocol` | Protocol v1 explicit little-endian decode/encode; no packed struct casts |
| `app_can` | FDCAN filters, ISR RX ring, session/sequence/operational-limit validation, CAN health and TX schedule |
| `app_diagnostics` | Lightweight nonblocking block-serialized UART console, 1 Hz watch and link diagnostics |
| `app_rtos` | Compact task model and deterministic ownership |

Firmware identity is `0.5.4`; shared wire protocol identity remains `1.0`.

## FreeRTOS and interrupt architecture

| Task | Priority | Stack | Baseline behavior |
|---|---:|---:|---|
| SupervisorTask | Realtime | 2048 B | 20 ms safety/watchdog cycle |
| MotorControl | High | 1024 B | 10 ms encoder/control/output path |
| Imu | AboveNormal | 1024 B | EXTI wake with 20 ms fallback |
| Communication | Normal | 2048 B | event wake or 5 ms maximum wait; UART + FDCAN task-context processing |
| Telemetry | BelowNormal | 1024 B | 50 ms service; battery acquisition scheduling only |

FDCAN1_IT0 is priority 5, equal to
`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`. The ISR drains the three-element hardware FIFO
into a bounded eight-slot project ring, records overflow/error events, and signals
CommunicationTask. It does not parse Protocol v1 or change motor targets. Protocol validation and
telemetry serialization run in task context.

## Safety and CAN authority

The production state model remains:

```text
BOOT -> INIT -> SAFE -> READY -> ACTIVE
                     \          /
                      -> FAULT <-
```

Power-up and every fault path enforce STBY LOW, direction LOW, and zero PWM. CAN cannot bypass
the existing supervisor, internal command model, drivetrain guards, wheel controller, or motor
layer.

M5 validates each CAN `BODY_VELOCITY` against configured controllers, drivetrain readiness, and
local operating limits before accepting it into the shared command model. An incomplete or
out-of-range active command is rejected and converges through the existing safety supervisor; it
is never clamped into a different motion.

Protocol v1 requires:

```text
fresh heartbeat
-> explicit DISARMED handshake
-> explicit ARMED authority
-> fresh BODY_VELOCITY command
-> all drivetrain/controller/safety gates
-> ACTIVE
```

The motion command timeout is 250 ms; host heartbeat and motion-authority timeouts are 500 ms.
All are measured from STM32 local reception time. Timeout, session replacement, bus-off, or RX loss clears CAN
authority and never reuses prior motion. Recovery requires a new DISARMED/ARMED transaction.

A disabled/disarmed command snapshot is intentionally not timed out. `COMMAND_TIMEOUT` applies
only when an established motion/control session loses required freshness, such as a valid motion
command expiring or armed CAN authority/heartbeat timing out. The boot/SAFE state cannot
manufacture the fault merely because no command exists.

Encoder acquisition validity is independent of drivetrain and controller readiness. The
`ENCODER_VALIDITY` bit is asserted for timer-start failure, an invalid sample, or either sample
older than 50 ms. It clears automatically only when both encoder timers initialized and both
channels again have valid samples no older than 50 ms. Unknown Motor A/B mapping/polarity,
unconfigured gains/limits, and `BODY_COMMAND_READY=false` do not assert this bit.

FDCAN communication is a centralized fault bit when loss affects CAN-owned motion. Warning and
Error Passive remain observable without manufacturing a second safety state machine. Bus-off
recovery is attempted every 1 s, but hardware recovery never restores authorization.

USART2 exposes compact system/sensor/fault information plus RX/TX/reject/sequence/overflow/
bus-off counters, local heartbeat/authority/command ages, session ID, authority and motor enable.
The console emits no unsolicited periodic telemetry after its boot block. Its only watch sources
are encoder, IMU and CAN, all fixed at 1 Hz. Complete response blocks are serialized and UART
transmission never blocks MotorControlTask. The complete command reference is
[STM32 USART2 Production CLI](uart_cli.md).

## Protocol v1 transport

| Item | Value |
|---|---|
| Nominal timing | prescaler 17, SEG1 15, SEG2 4, SJW 4; 500 kbit/s, 80% sample point |
| Data timing | prescaler 5, SEG1 13, SEG2 3, SJW 3; 2 Mbit/s, 82.3529% sample point |
| RX IDs | `0x080` authority, `0x081` motion, `0x082` heartbeat |
| TX IDs | `0x180` system, `0x181` wheel, `0x182` IMU, `0x183` battery |
| Integrity | CAN FD link CRC only; no application CRC |
| RX policy | exact filter, FD+BRS envelope, length/version/reserved/session/sequence validation |
| TX policy | CommunicationTask owns deterministic scheduling; modules only update telemetry state |

The complete contract, field offsets, golden frames, ROS conversion, and fault meanings are in
[Protocol v1](../interfaces/protocol_v1.md).

## Drivetrain configuration boundary

Centralized firmware defaults now contain the user-measured commissioning geometry:

| Value | Commissioning setting | Status |
|---|---:|---|
| Wheel radius | 0.023 m | `MEASURED / COMMISSIONING` |
| Wheel track | 0.125 m | `MEASURED / COMMISSIONING` |
| Half track used by differential drive | 0.0625 m | `DERIVED` |
| Geometric circumference | 0.1445132621 m | `DERIVED` |
| Encoder 1 ownership / forward sign | right wheel / `+1` | `VERIFIED` |
| Encoder 2 ownership / forward sign | left wheel / `-1` | `VERIFIED` |
| Motor A ownership / logical-forward sign | right wheel / `-1` | `VERIFIED` |
| Motor B ownership / logical-forward sign | left wheel / `-1` | `VERIFIED` |
| Right counts per wheel revolution | 1059.5 | `MEASURED / COMMISSIONING` |
| Left counts per wheel revolution | 1060.8 | `MEASURED / COMMISSIONING` |
| Right meters / radians per count | 0.0001363976 m / 0.005930331 rad | `DERIVED` |
| Left meters / radians per count | 0.0001362305 m / 0.005923063 rad | `DERIVED` |
| Maximum body linear speed | 0.30 m/s | `COMMISSIONING LIMIT` |
| Maximum body angular speed | 1.50 rad/s | `COMMISSIONING LIMIT` |
| Maximum normalized wheel rate | 3000 count/s (about 0.409 m/s) | `COMMISSIONING LIMIT` |
| Wheel target slew | 4400 count/s^2 (about 0.60 m/s^2) | `COMMISSIONING LIMIT` |
| Motor effort / PWM output | +/-0.60 normalized | `COMMISSIONING LIMIT` |

The implemented differential-drive relationship is therefore:

```text
v_left  = v - omega * 0.0625
v_right = v + omega * 0.0625
```

The wheel scale came from ten complete forward revolutions on each physical wheel: Encoder 1
accumulated +10,595 counts and Encoder 2 accumulated -10,608 counts. The approximately 0.123%
side-to-side difference is deliberately preserved; firmware does not average the two calibration
values. Radius, track, and encoder scales are still commissioning values and must be validated by
real straight-line travel and rotational motion.

The initial independent left/right PI configurations are `Kp=0.00020`, `Ki=0.00060`, `Kd=0`, an
integrator limit of 700 count-seconds, and output limit 0.60. These are deliberately equal initial
commissioning values, not physically tuned gains. The controller clamps non-finite/out-of-range
output, has conditional-integration anti-windup, and applies the configured target slew only while
an active command is valid. Any safety convergence resets the controllers and directly forces the
motor-safe state; withdrawal is not delayed by the ramp.

At initialization, BODY_COMMAND_READY is true only when the verified mapping/scales/geometry,
finite operating envelope, and both valid controller configurations are present. Protocol `0x180`
bit 15, `0x181` bit 12, UART `status`, and the Safety guard use this complete readiness condition.
An out-of-range BODY_VELOCITY is rejected rather than clamped.

The 0.30 m/s ceiling is a firmware commissioning limit, not a verified motor or robot maximum.
The exact installed-motor loaded and stall current, attainable loaded wheel speed, and TB6612
carrier/module thermal margin have not been measured; without those quantities a higher sustained
limit cannot be justified. The 0.60 output clamp limits duty request but is not a current limit
because this hardware has no current feedback in the firmware path. Encoder PPR/CPR definition and
gear-ratio decomposition remain unknown but are not required by the direct output-wheel count
scales. No Nav2 footprint geometry is reused as drivetrain geometry.

## Telemetry and ROS boundary

Firmware transmits normalized cumulative wheel counts/rates, the frozen raw Encoder 1/2 diagnostic
fields, controller target/output, IMU SI measurements, battery estimate/validity, safety state,
fault word, supervisor/watchdog health, STM32 monotonic time, and sequences. Raw values also remain
available through local UART diagnostics; they coexist with, and are not substituted for, the
logical Protocol `0x181` fields. Firmware does not create ROS odometry, ROS time, orientation,
covariance, or TF.

The Orange Pi owns `nav_msgs/msg/Odometry` and the later deliberate single authority for
`odom -> base_link`. Metric odometry must wait for measured drivetrain scale and real motion
consistency evidence.

## Build and acceptance boundary

On 2026-08-29, clean Debug and Release cross-builds with Arm GNU Toolchain completed with zero
compiler/linker warnings and produced ELF, HEX, and BIN images:

| Build | FLASH | RAM | Result |
|---|---:|---:|---|
| Debug | 105,544 B / 512 KiB (20.13%) | 42,848 B / 128 KiB (32.69%) | PASS |
| Release | 67,936 B / 512 KiB (12.96%) | 42,848 B / 128 KiB (32.69%) | PASS |

The physical cross-domain link is accepted through Orange Pi CAN3 / SocketCAN `can3`: real
FD+BRS frames `0x080`, `0x082`, and `0x180..0x183` were observed at 500 kbit/s nominal and 2 Mbit/s
data. Explicit Linux sample points 80% / 82.5% produced Error Active with zero TX/RX errors and
zero bus-off events. Protocol 1.0, the firmware timing, and the robot's default DISARMED behavior
are unchanged.

The final M5 real-hardware demo passed with firmware `0.5.4`: BODY_COMMAND_READY and Motion Authority
were confirmed, closed-loop straight motion ran at the current 0.30 m/s commissioning limit, and a
real RPLIDAR A1 `/scan` obstacle inside the 30° frontal sector at 0.60 m caused zero velocity,
authority withdrawal, and STM32 safe stop. STOPPED remained latched after obstacle removal and a new
explicit user START was required before motion could resume. The accepted commissioning speed is
not a claim of physical maximum speed.

Observed obstacle-detection intervals were approximately 0.075 ms to the zero velocity command,
1.276 ms to Motion Authority withdrawal, and 30.8 ms to STM32 stop confirmation. These are successful
demo observations, not guaranteed worst-case safety limits.
