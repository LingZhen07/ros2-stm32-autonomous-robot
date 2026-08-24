# Hardware Baseline and STM32 Pin Map

[中文版本](hardware.zh-CN.md)

This document records the persistent hardware baseline for RobotProject.

## Status Convention

| Status | Meaning |
|---|---|
| `FROZEN` | Accepted configuration; retained until real evidence requires a change |
| `VERIFIED` | Supported by real hardware or accepted engineering evidence |
| `IN PROGRESS` | Active bring-up or integration |
| `PROPOSED` | Engineering candidate awaiting validation |
| `TBD` | Deliberately unresolved |

## 1. Hardware Overview

| Domain | Component | Role | Status |
|---|---|---|---|
| High computing | Orange Pi AI Pro 8GB | ROS 2, perception, SLAM, Nav2, bridge | VERIFIED |
| AI acceleration | Ascend 310B4 | YOLOv8n inference | VERIFIED / FROZEN |
| Real-time control | STM32G474RET6 | Safety, motor control, encoder, IMU, ADC, CAN | IN PROGRESS |
| MCU carrier | DeveBox STM32G474R Ver:20 | STM32 development board | VERIFIED |
| Motor drive | TB6612-based dual DC motor driver | Dual brushed-DC H-bridge | IN PROGRESS |
| IMU | ICM-42688-P | 6-axis inertial sensing | IN PROGRESS |
| CAN physical layer | TJA1042/TJA1043-family module | CAN transceiver | IN PROGRESS |
| LiDAR | RPLIDAR A1 | 2D LaserScan for SLAM/Nav2 | VERIFIED / FROZEN |
| RGB-D camera | Astra RGB-D | RGB and depth sensing | VERIFIED / FROZEN |

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

The high-computing domain is accepted through non-actuating Navigation v1.

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

## 4. Frozen STM32 Pin Map v1

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
| CAN RX | PA11 | FDCAN1_RX | AF9 | FDCAN1 | Frozen |
| CAN TX | PA12 | FDCAN1_TX | AF9 | FDCAN1 | Frozen |
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

**Pin Allocation v1 status:** `FROZEN`

Pin changes require a demonstrated hardware or CubeMX conflict.

## 5. Peripheral Ownership

| Peripheral | Channels / pins | Responsibility | Baseline |
|---|---|---|---|
| TIM1 | CH1 PA8, CH2 PA9 | Synchronized motor PWM | 10 kHz |
| TIM2 | CH1 PA0, CH2 PA1 | Encoder 1 quadrature decoding | Hardware encoder mode |
| TIM3 | CH1 PC6, CH2 PC7 | Encoder 2 quadrature decoding | Hardware encoder mode |
| SPI1 | PA5, PA6, PA7 + PA4 CS | ICM-42688-P | SPI |
| ADC1 | PC0 / IN6 | Battery measurement | Single channel |
| USART2 | PA2, PA3 | Debug / bring-up | 115200 8N1 |
| FDCAN1 | PA11, PA12 | CAN transport | 500 kbit/s nominal bring-up |
| EXTI | PC4, PC5 | IMU interrupts | Rising-edge baseline |
| SWD | PA13, PA14 | Debug / flashing | Reserved |

Quadrature decoding uses TIM2/TIM3 hardware encoder mode. Motor PWM channels share the TIM1 time base.

## 6. Motor Driver

### Hardware

| Item | Value |
|---|---|
| Driver | TB6612-based dual DC motor driver |
| Motor supply | 12 V battery |
| Channels | Motor A + Motor B |
| Standby control | PC8 / `STBY` |
| PWM | TIM1 CH1 + CH2 |

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
| Bring-up state | IN PROGRESS |

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

| Stage | Acceptance evidence |
|---:|---|
| 1 | SPI register access |
| 2 | Valid `WHO_AM_I` |
| 3 | Real accelerometer / gyroscope samples |
| 4 | Interrupt behavior verified |
| 5 | DMA / FIFO added when measured throughput justifies it |

The breakout exposes both `VCC` and `3.3V`. Their exact board-level roles remain a physical verification item.

## 8. CAN / CAN FD

| Layer | Component / setting |
|---|---|
| MCU controller | STM32 FDCAN1 |
| RX | PA11 / FDCAN1_RX |
| TX | PA12 / FDCAN1_TX |
| Nominal bring-up bitrate | 500 kbit/s |
| Physical-layer module | TJA1042/TJA1043 family |
| Bus signals | CANH / CANL |
| Production transport direction | CAN FD |

### Protocol items awaiting design

| Item | Status |
|---|---|
| CAN IDs | TBD |
| CAN FD data-phase bitrate | TBD |
| Payload packing | TBD |
| Scaling / units | TBD |
| CRC policy | TBD |
| Heartbeat format | TBD |
| Sequence format | TBD |
| Protocol version | TBD |

These items will move into `interfaces/` when the shared transport contract is deliberately designed.

The exact transceiver subvariant, termination arrangement, standby implementation, and logic-level arrangement remain hardware verification items.

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

## 10. Frozen Robot Geometry

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

The drivetrain wheel-track measurement remains an independent physical parameter.

## 11. Required Drivetrain Measurements

| Parameter | Status |
|---|---|
| Effective wheel radius | TBD |
| Wheel-track distance | TBD |
| Encoder PPR / CPR definition | TBD |
| Gear ratio | TBD |
| Motor A/B → left/right mapping | TBD |
| Encoder 1/2 → left/right mapping | TBD |
| Forward encoder sign | TBD |

These values will be recorded from real hardware before closed-loop wheel control and odometry acceptance.

## 12. Primary References

| Component | Primary reference |
|---|---|
| STM32G474 | STMicroelectronics DS12288 |
| STM32G4 peripherals | STMicroelectronics RM0440 |
| STM32G474 silicon notes | STM32G474 device errata |
| ICM-42688-P | TDK InvenSense DS-000347 |
| TB6612FNG | Toshiba TB6612FNG Datasheet |
| TJA1042 | NXP TJA1042 Datasheet |
| TJA1043 | NXP TJA1043 Datasheet |

Measured hardware behavior is the integration source of truth when module variants or board-level implementation details need confirmation.
