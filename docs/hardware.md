# Hardware Baseline and STM32 Pin Map

[中文版本](hardware.zh-CN.md)

This document records the persistent hardware baseline for `ros2-stm32-autonomous-robot`.

## 1. Hardware Overview

| Domain | Component | Role |
|---|---|---|
| High computing | Orange Pi AI Pro 8GB | ROS 2, perception, SLAM, Nav2, bridge |
| AI acceleration | Ascend 310B4 | YOLOv8n inference |
| Real-time control | STM32G474RET6 | Safety, motor control, encoder, IMU, ADC, CAN |
| MCU carrier | DeveBox STM32G474R Ver:20 | STM32 development board |
| Motor drive | TB6612-based dual DC motor driver | Dual brushed-DC H-bridge |
| IMU | ICM-42688-P | 6-axis inertial sensing |
| CAN physical layer | TJA1042T(K)/3 | CAN transceiver |
| LiDAR | RPLIDAR A1 | 2D LaserScan for SLAM/Nav2 |
| RGB-D camera | Astra RGB-D | RGB and depth sensing |

## 2. High-Computing Platform

| Item | Value |
|---|---|
| Board | Orange Pi AI Pro 8GB |
| OS | Ubuntu 22.04.5 LTS |
| Architecture | aarch64 |
| Kernel | Vendor Linux 5.10 BSP |
| ROS | ROS 2 Humble |
| NPU | Ascend 310B4 |
| ROS workspace | `/data/ros2_ws` |
| Project data | `/data/projects` |

The high-computing domain supports non-actuating Navigation v1.

## 3. STM32 Platform

| Item | Value |
|---|---|
| MCU | STM32G474RET6 |
| Package | LQFP64 |
| Core | Arm Cortex-M4 + FPU |
| Development board | DeveBox STM32G474R Ver:20 |
| HSE | 8 MHz |
| SYSCLK target | 170 MHz |
| Primary datasheet | ST DS12288 |
| Reference manual | ST RM0440 |

### SWD reservation

| Signal | Pin |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |
| Reset | NRST |
| Reference power | 3V3 |
| Ground | GND |

SWD remains reserved throughout bring-up.

## 4. STM32 Pin Map v1

| External function | MCU pin | STM32 function | AF / mode | Owner | Startup / note |
|---|---|---|---|---|---|
| Encoder 1 A | PA0 | TIM2_CH1 | AF1 | TIM2 | Pull-up baseline |
| Encoder 1 B | PA1 | TIM2_CH2 | AF1 | TIM2 | Pull-up baseline |
| Debug UART TX | PA2 | USART2_TX | AF7 | USART2 | 115200 8N1 baseline |
| Debug UART RX | PA3 | USART2_RX | AF7 | USART2 | 115200 8N1 baseline |
| IMU CS | PA4 | GPIO Output | GPIO | SPI1 device select | HIGH |
| IMU SCLK | PA5 | SPI1_SCK | AF5 | SPI1 | ICM-42688-P |
| IMU MISO | PA6 | SPI1_MISO | AF5 | SPI1 | ICM-42688-P |
| IMU MOSI | PA7 | SPI1_MOSI | AF5 | SPI1 | ICM-42688-P |
| Motor A PWM | PA8 | TIM1_CH1 | AF6 | TIM1 | 10 kHz target; duty 0 at start |
| Motor B PWM | PA9 | TIM1_CH2 | AF6 | TIM1 | 10 kHz target; duty 0 at start |
| CAN RX | PA11 | FDCAN1_RX | AF9 | FDCAN1 | Allocated |
| CAN TX | PA12 | FDCAN1_TX | AF9 | FDCAN1 | Allocated |
| SWDIO | PA13 | SWDIO | Debug | SWD | Reserved |
| SWCLK | PA14 | SWCLK | Debug | SWD | Reserved |
| Battery ADC | PC0 | ADC1_IN6 | Analog | ADC1 | No Pull |
| IMU INT1 | PC4 | EXTI4 | Rising-edge baseline | EXTI | Pull-down baseline |
| IMU INT2 | PC5 | EXTI5 | Rising-edge baseline | EXTI9_5 IRQ | Pull-down baseline |
| Encoder 2 A | PC6 | TIM3_CH1 | AF2 | TIM3 | Pull-up baseline |
| Encoder 2 B | PC7 | TIM3_CH2 | AF2 | TIM3 | Pull-up baseline |
| TB6612 STBY | PC8 | GPIO Output | GPIO | Motor safety | LOW |
| TB6612 AIN1 | PB12 | GPIO Output | GPIO | Motor A direction | LOW |
| TB6612 AIN2 | PB13 | GPIO Output | GPIO | Motor A direction | LOW |
| TB6612 BIN1 | PB14 | GPIO Output | GPIO | Motor B direction | LOW |
| TB6612 BIN2 | PB15 | GPIO Output | GPIO | Motor B direction | LOW |

Pin changes require a demonstrated hardware or CubeMX conflict.

## 5. Peripheral Ownership

| Peripheral | Channels / pins | Responsibility | Baseline |
|---|---|---|---|
| TIM1 | CH1 PA8, CH2 PA9 | Synchronized motor PWM | 10 kHz |
| TIM2 | CH1 PA0, CH2 PA1 | Encoder 1 quadrature decoding | Hardware encoder mode; configured 16-bit range |
| TIM3 | CH1 PC6, CH2 PC7 | Encoder 2 quadrature decoding | Hardware encoder mode; configured 16-bit range |
| SPI1 | PA5, PA6, PA7 + PA4 CS | ICM-42688-P | SPI |
| ADC1 | PC0 / IN6 | Battery measurement | Single channel |
| USART2 | PA2, PA3 | Debug / bring-up | 115200 8N1 |
| FDCAN1 | PA11, PA12 | CAN FD transport | 500 kbit/s nominal, 2 Mbit/s data, FD+BRS |
| EXTI | PC4, PC5 | IMU interrupts | Rising-edge baseline |
| SWD | PA13, PA14 | Debug / flashing | Reserved |

Quadrature decoding uses TIM2/TIM3 hardware encoder mode. Both counters are configured to the 0..65535 range in the current firmware baseline. Motor PWM channels share the TIM1 time base.

### Encoder mapping, sign, and scale

Real hardware commissioning established the physical wheel ownership and raw direction after ten
complete forward wheel revolutions:

| Physical wheel | Encoder | Ten-revolution raw total | Raw forward sign | Logical normalization |
|---|---|---:|---:|---|
| Right | Encoder 1 / TIM2 | +10,595 counts | `+1` | logical = `+raw` |
| Left | Encoder 2 / TIM3 | -10,608 counts | `-1` | logical = `-raw` |

Normalization belongs above the low-level timer driver. The raw counters, cumulative raw UART
diagnostics, and Protocol `0x181` raw diagnostic fields are unchanged. Motor ownership and polarity
were measured separately; they were not inferred from the encoder mapping.

## 6. Motor Driver

### Hardware

| Item | Value |
|---|---|
| Driver | TB6612-based dual DC motor driver |
| Motor supply | 12 V battery |
| Channels | Motor A + Motor B |
| Standby control | PC8 / `STBY` |
| PWM | TIM1 CH1 + CH2 |
| Motor A physical ownership / forward sign | right wheel / `-1` |
| Motor B physical ownership / forward sign | left wheel / `-1` |

### Control signals

| Signal | MCU pin | Function |
|---|---|---|
| PWMA | PA8 | Motor A PWM |
| AIN1 | PB12 | Motor A direction |
| AIN2 | PB13 | Motor A direction |
| PWMB | PA9 | Motor B PWM |
| BIN1 | PB14 | Motor B direction |
| BIN2 | PB15 | Motor B direction |
| STBY | PC8 | Motor-domain enable |

### Startup safety baseline

| Signal group | Startup state |
|---|---|
| STBY | LOW |
| AIN1 / AIN2 | LOW |
| BIN1 / BIN2 | LOW |
| TIM1 PWM duty | 0% |

The safety supervisor will control the later transition into an enabled motor state.

### Battery measurement

| Item | Value |
|---|---|
| Driver-board ADC output | `Vadc ≈ Vbattery / 11` |
| MCU input | PC0 |
| ADC channel | ADC1_IN6 |
| Calibration source | Multimeter reference + ADC reading |
| Safety thresholds | TBD after calibration |

## 7. IMU

| Item | Value |
|---|---|
| Device | TDK InvenSense ICM-42688-P |
| Sensor type | 6-axis accelerometer + gyroscope |
| Host interface | SPI |
| Maximum device SPI capability | 24 MHz |
| Observed behavior | WHO_AM_I response and real accelerometer/gyroscope samples |

### MCU mapping

| IMU signal | MCU pin | Function |
|---|---|---|
| CS | PA4 | GPIO Output |
| SCLK | PA5 | SPI1_SCK |
| MISO | PA6 | SPI1_MISO |
| MOSI | PA7 | SPI1_MOSI |
| INT1 | PC4 | EXTI4 |
| INT2 | PC5 | EXTI5 |

### Bring-up sequence

| Stage | Required evidence |
|---:|---|
| 1 | SPI register access |
| 2 | Valid `WHO_AM_I` |
| 3 | Real accelerometer / gyroscope samples |
| 4 | Measured interrupt behavior |
| 5 | DMA / FIFO added when measured throughput justifies it |

The breakout exposes both `VCC` and `3.3V`; their exact board-level roles are not yet confirmed.

## 8. CAN / CAN FD

| Layer | Component / setting |
|---|---|
| MCU controller | STM32 FDCAN1 |
| RX | PA11 / FDCAN1_RX |
| TX | PA12 / FDCAN1_TX |
| Orange Pi board TX | 40-pin header pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042T(K)/3 TXD |
| Orange Pi board RX | 40-pin header pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042T(K)/3 RXD |
| Orange Pi controller | CAN3 / `822d0000.mttcan`, `mttcan-id=3` |
| Linux interface | SocketCAN `can3` |
| Nominal bitrate | 500 kbit/s, 80% sample point |
| STM32 CAN FD data timing | 2 Mbit/s, 82.3529% sample point |
| Linux CAN FD data timing | 2 Mbit/s, explicit 82.5% sample point |
| Physical-layer transceiver | TJA1042T(K)/3 |
| Bus signals | CANH / CANL |
| Point-to-point wiring | CANH to CANH, CANL to CANL, common ground |
| Termination | One 120 ohm terminator at each physical end |
| Transport | CAN FD |

### Protocol 1.0 transport

| Phase | Prescaler | SEG1 | SEG2 | SJW | Result |
|---|---:|---:|---:|---:|---|
| Nominal | 17 | 15 | 4 | 4 | 500 kbit/s, 80% sample point |
| Data | 5 | 13 | 3 | 3 | 2 Mbit/s, 82.3529% sample point |

Protocol v1 uses Standard 11-bit IDs, CAN FD+BRS, explicit little-endian serialization, a 250 ms
motion-command timeout, and no application CRC beyond CAN FD link integrity. CAN IDs, layouts,
units, rates, authority/reconnection behavior, and ROS implementation requirements are defined in
[`interfaces/protocol_v1.md`](../interfaces/protocol_v1.md).

CANH/CANL continuity, common ground, and two-end 120 ohm termination were measured on the robot.
The Orange Pi CAN assignment is 40-pin header pin 36, GPIO2_17/CAN_TX3, to TJA1042T(K)/3 TXD and
40-pin header pin 11, GPIO2_18/CAN_RX3, from TJA1042T(K)/3 RXD. The
matching Huawei 25.2.0 CAN3 pinctrl values are TX `<0x40 0x1>` and RX `<0x44 0x1>`.

The board-specific signed Device Tree update is installed. The live system probes
`mttcan@3` at `822d0000.mttcan` with `mttcan-id=3` and exposes it as SocketCAN `can3`. The matching
live CAN3 pinmux and physical header mapping were observed directly. The former
`822c0000.mttcan` / `can2` investigation is historical only: CAN2 does not drive this wiring and is not an operational
fallback.

The Linux `drv_mttcan` defaults did not interoperate cleanly with the STM32 timing and produced
heavy CAN errors. Production startup therefore explicitly fixes nominal timing at 500 kbit/s / 80%
and data timing at 2 Mbit/s / 82.5%. With that configuration, the real link remained Error Active
with TX error 0, RX error 0, bus errors 0, and bus-off 0 during bidirectional Protocol 1.0 traffic.

Primary references: [Huawei 25.2.0 CAN device-tree/pinctrl procedure](https://www.hiascend.com/document/detail/zh/Atlas%20200I%20A2/2520/RC/driverdevelopmentguide/atlasdg_11_0065.html),
[Huawei 25.2.0 CAN register/pad map](https://www.hiascend.com/document/detail/zh/Atlas%20200I%20A2/2520/RC/driverdevelopmentguide/atlasdg_11_0064.html),
and [Huawei 40-pin interface map](https://www.hiascend.com/document/detail/zh/Atlas200IDKA2DeveloperKit/23.0.RC2/Hardware%20Interfaces/hiug/hiug_0024.html).
The robot uses a TJA1042T(K)/3 transceiver. Real Orange Pi <-> STM32 CAN FD+BRS traffic was observed;
software does not infer unrecorded carrier-board details.

## 9. Debug UART

| Item | Value |
|---|---|
| Peripheral | USART2 |
| TX | PA2 |
| RX | PA3 |
| Baud rate | 115200 |
| Data | 8 bits |
| Parity | None |
| Stop bits | 1 |
| Role | Bring-up, diagnostics, fallback communication |

## 10. Robot Geometry

### Coordinate frame

| Axis / origin | Definition |
|---|---|
| `base_link` origin | Midpoint of left/right drive-wheel axle |
| +X | Robot forward |
| +Y | Robot left |
| +Z | Robot up |

### Sensor transforms

| Transform | x [m] | y [m] | z [m] | roll | pitch | yaw |
|---|---:|---:|---:|---:|---:|---:|
| `base_link → camera_link` | +0.130 | 0.000 | +0.110 | 0 | 0 | 0 |
| `base_link → laser_frame` | +0.043 | 0.000 | +0.165 | 0 | 0 | π |

### Footprint

```yaml
footprint: "[[0.140, 0.080], [0.140, -0.080], [-0.070, -0.080], [-0.070, 0.080]]"
footprint_padding: 0.01
```

The drivetrain wheel track is an independent physical parameter; it is not derived from the
navigation footprint.

## 11. Drivetrain Measurements and Commissioning

| Parameter | Value / note |
|---|---|
| Geometric wheel radius | 0.023 m (`MEASURED / COMMISSIONING`) |
| Wheel-track distance | 0.125 m (`MEASURED / COMMISSIONING`) |
| Half track | 0.0625 m (`DERIVED` from measured track) |
| Geometric wheel circumference | 0.1445132621 m (`DERIVED`) |
| Right decoded counts per wheel revolution | 1059.5 (`MEASURED / COMMISSIONING`, Encoder 1) |
| Left decoded counts per wheel revolution | 1060.8 (`MEASURED / COMMISSIONING`, Encoder 2) |
| Right meters per count | approximately 0.0001363976 m/count (`DERIVED`) |
| Left meters per count | approximately 0.0001362305 m/count (`DERIVED`) |
| Right radians per count | approximately 0.005930331 rad/count (`DERIVED`) |
| Left radians per count | approximately 0.005923063 rad/count (`DERIVED`) |
| Encoder PPR / CPR definition | TBD; do not infer while direct measurement is available |
| Gear ratio | TBD; not required by the direct output-wheel count calibration |
| Motor A/B → left/right mapping | Motor A → right; Motor B → left |
| Motor forward sign | Motor A `-1`; Motor B `-1` |
| Encoder 1/2 → left/right mapping | Encoder 1 → right; Encoder 2 → left |
| Forward encoder sign | right raw `+1`; left raw `-1` |

The radius and track were physically measured by the user on 2026-08-28. They are commissioning
geometry, not calibrated effective values. Effective calibration requires measured straight-line
distance and rotational motion. On 2026-08-29 the user measured each
physical wheel over ten complete forward revolutions: `10595 / 10 = 1059.5` decoded counts/rev on
the right and `abs(-10608) / 10 = 1060.8` decoded counts/rev on the left. The difference is about
0.123%, so commissioning keeps independent left/right scale values rather than averaging them.
The derived values use `meters_per_count = (2 * pi * 0.023) / counts_per_wheel_rev` and
`radians_per_count = (2 * pi) / counts_per_wheel_rev`. They remain commissioning values until
straight-distance and rotational-motion validation is complete.

Firmware 0.5.4 uses a 0.30 m/s body-linear, 1.50 rad/s body-angular, 3000 count/s wheel-rate,
and 0.60 normalized-output commissioning envelope. These are not hardware ratings. Installed-motor
loaded/stall current, loaded attainable wheel speed, and TB6612 carrier/module thermal margin are
still unresolved physical measurements and prevent claiming a higher safe sustained limit.

## 12. Primary References

| Component | Primary reference |
|---|---|
| STM32G474 | STMicroelectronics DS12288 |
| STM32G4 peripherals | STMicroelectronics RM0440 |
| STM32G474 silicon notes | STM32G474 device errata |
| ICM-42688-P | TDK InvenSense DS-000347 |
| TB6612FNG | Toshiba TB6612FNG Datasheet |
| TJA1042T(K)/3 | NXP TJA1042 Datasheet |

Measured hardware behavior is the integration source of truth when module variants or board-level implementation details need confirmation.
