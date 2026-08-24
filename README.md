# RobotProject

[简体中文](README.zh-CN.md)

RobotProject is a heterogeneous autonomous mobile robot integrating **ROS 2, Ascend NPU, STM32 real-time control, LiDAR, RGB-D sensing, and CAN/CAN FD**.

The project follows an end-to-end robotics architecture from perception and planning to deterministic motor control and physical feedback.

## Project Snapshot

| Layer | Platform / Technology | Responsibility | Status |
|---|---|---|---|
| High-computing | Orange Pi AI Pro 8GB | ROS 2, perception, SLAM, Nav2, system bridge | Complete through Navigation v1 |
| AI acceleration | Ascend 310B4 | YOLOv8n inference | Frozen |
| Real-time control | STM32G474RET6 + FreeRTOS | Safety, motor control, encoder, IMU, ADC, CAN | Active development |
| Environment sensing | RPLIDAR A1 + Astra RGB-D | LaserScan, RGB, depth | Frozen |
| Motion actuation | TB6612 + dual DC motors | Differential-drive actuation | Bring-up pending |
| Feedback | Quadrature encoders + ICM-42688-P | Wheel state and inertial sensing | Bring-up pending |

## System Architecture

| Stage | Input | Processing / Owner | Output |
|---|---|---|---|
| Perception | RGB-D, LiDAR | Orange Pi + Ascend 310B4 | Obstacles, sensor data |
| Mapping | `/scan`, TF | Cartographer | `/map`, SLAM pose |
| Navigation | Goal, map, costmaps | Nav2 | `geometry_msgs/msg/Twist` |
| Transport | Body velocity command | Linux ↔ MCU bridge | CAN / CAN FD frames |
| Real-time control | Validated `v`, `ω` | STM32G474RET6 | Left/right wheel targets |
| Actuation | Wheel targets | PWM + TB6612 | Motor torque / robot motion |
| Feedback | Encoders, IMU, battery ADC | STM32G474RET6 | Telemetry and wheel state |
| Odometry | Wheel state | Orange Pi bridge | `nav_msgs/msg/Odometry`, `/odom` |
| Closed loop | `/odom`, LaserScan, costmaps | Nav2 controller | Continuous path following |

### Command path

| Step | Interface |
|---:|---|
| 1 | Navigation goal |
| 2 | Nav2 planner / controller |
| 3 | `geometry_msgs/msg/Twist` |
| 4 | Orange Pi transport bridge |
| 5 | CAN / CAN FD |
| 6 | STM32 command validation |
| 7 | Differential-drive wheel targets |
| 8 | Closed-loop wheel control |
| 9 | Physical robot motion |

### Feedback path

| Step | Interface |
|---:|---|
| 1 | Encoder counts + IMU + system telemetry |
| 2 | STM32 wheel-state estimation |
| 3 | CAN / CAN FD |
| 4 | Orange Pi transport bridge |
| 5 | `nav_msgs/msg/Odometry` |
| 6 | `/odom` and `odom → base_link` |
| 7 | Nav2 controller feedback |

## Current Status

| Subsystem | Status |
|---|---|
| Orange Pi / Ubuntu / vendor BSP | Complete |
| Ascend 310B4 baseline | Complete / Frozen |
| ROS 2 Humble foundation | Complete / Frozen |
| Astra RGB-D | Complete / Frozen |
| YOLOv8n Ascend inference | Working / Frozen |
| RPLIDAR A1 | Complete / Frozen |
| TF System v1 | Complete / Frozen |
| rosbag2 / diagnostics | Complete / Frozen |
| Cartographer LiDAR SLAM v1 | Complete / Frozen |
| Nav2 costmaps / global planning | Complete / Frozen |
| STM32 hardware definition | Complete |
| STM32 Pin Map v1 | Frozen |
| STM32CubeMX baseline | In Progress |
| Encoder / IMU / motor firmware | Planned |
| CAN / CAN FD integration | Planned |
| Real wheel odometry | Planned |
| Nav2 physical FollowPath | Planned |
| Closed-loop autonomous navigation | Target |

**Active engineering phase:** `STM32 REAL-TIME CONTROL DOMAIN`

## Hardware

### High-computing domain

| Component | Role |
|---|---|
| Orange Pi AI Pro 8GB | ROS 2 host and high-level compute |
| Ascend 310B4 | NPU acceleration |
| Astra RGB-D | RGB and depth sensing |
| RPLIDAR A1 | 2D LiDAR |

### Real-time domain

| Component | Role |
|---|---|
| STM32G474RET6 | Real-time controller |
| DeveBox STM32G474R Ver:20 | MCU development board |
| ICM-42688-P | 6-axis IMU |
| TB6612 dual DC motor driver | Motor H-bridge |
| TJA1042/TJA1043-family module | CAN physical layer |
| Quadrature encoders | Wheel feedback |
| 12 V battery | Motor supply |

Detailed hardware information: [Hardware Baseline and STM32 Pin Map](docs/hardware.md)

## STM32 Pin Map v1

| Function | Pin(s) | Peripheral |
|---|---|---|
| Encoder 1 A/B | PA0 / PA1 | TIM2 |
| Debug UART TX/RX | PA2 / PA3 | USART2 |
| IMU CS | PA4 | GPIO |
| IMU SPI | PA5 / PA6 / PA7 | SPI1 |
| Motor PWM A/B | PA8 / PA9 | TIM1 |
| CAN RX/TX | PA11 / PA12 | FDCAN1 |
| SWD | PA13 / PA14 | SWD |
| Battery ADC | PC0 | ADC1_IN6 |
| IMU INT1/INT2 | PC4 / PC5 | EXTI |
| Encoder 2 A/B | PC6 / PC7 | TIM3 |
| Motor STBY | PC8 | GPIO |
| Motor direction | PB12–PB15 | GPIO |

### Control baseline

| Parameter | Value |
|---|---:|
| HSE | 8 MHz |
| SYSCLK | 170 MHz |
| Motor PWM | 10 kHz |
| Debug UART | 115200 8N1 |
| CAN nominal bring-up bitrate | 500 kbit/s |

## Repository Structure

```text
RobotProject/
├── firmware/          STM32 firmware
├── ros2_ws/           ROS 2 packages and high-computing software
├── interfaces/        Linux ↔ STM32 communication contracts
├── docs/              Bilingual engineering knowledge base
├── AGENTS.md          Shared Codex engineering policy
├── CHANGELOG.md       Cross-domain engineering communication
├── README.md          English project overview
└── README.zh-CN.md    Chinese project overview
```

Two Codex CLI instances work in parallel:

| Agent | Primary scope | Runtime context |
|---|---|---|
| Firmware Codex | `firmware/` | Local STM32 development |
| ROS Codex | `ros2_ws/` | SSH to Orange Pi runtime |

Shared coordination uses `interfaces/`, `docs/`, and `CHANGELOG.md`.

## Engineering Workflow

| Phase | Acceptance principle |
|---|---|
| Inspect | Use real hardware and actual runtime state |
| Implement | Build the smallest valid subsystem |
| Verify | Collect real electrical / software evidence |
| Stabilize | Fix observed failures |
| Accept | Record measurable pass criteria |
| Freeze | Preserve working modules |
| Integrate | Advance to the next system boundary |

### Hardware evidence examples

| Subsystem | Acceptance evidence |
|---|---|
| Encoder | Physical wheel rotation produces consistent counts |
| IMU | Valid `WHO_AM_I` and real motion samples |
| PWM | Measured 10 kHz waveform and expected duty |
| CAN | Real transmitted and received frames |
| Motor | Controlled motion and verified safe stop |
| Odometry | Physical motion produces quantitatively consistent `/odom` |
| Navigation | Real goal reached with obstacle avoidance |

## Navigation and Odometry

Nav2 uses:

```text
linear.x  [m/s]
angular.z [rad/s]
```

The STM32 will validate the body command and convert it into differential-drive wheel targets.

Wheel feedback will return through the Linux bridge as:

```text
nav_msgs/msg/Odometry
```

The final system will maintain one authoritative publisher for:

```text
odom → base_link
```

during the transition from temporary SLAM-owned odometry to real wheel odometry.

## Development Roadmap

| Phase | Milestone |
|---:|---|
| 1 | STM32CubeMX baseline |
| 2 | SWD + safe GPIO startup |
| 3 | USART2 debug bring-up |
| 4 | Battery ADC verification |
| 5 | TIM2 / TIM3 encoder bring-up |
| 6 | ICM-42688-P SPI + interrupt bring-up |
| 7 | TIM1 10 kHz PWM verification |
| 8 | FreeRTOS minimal runtime |
| 9 | Safety supervisor + watchdog |
| 10 | Controlled motor test |
| 11 | Closed-loop wheel velocity |
| 12 | FDCAN physical bring-up |
| 13 | Shared CAN / CAN FD protocol v1 |
| 14 | Orange Pi ↔ STM32 bridge |
| 15 | Wheel odometry + real `/odom` |
| 16 | Nav2 Controller / FollowPath |
| 17 | Closed-loop autonomous navigation |
| 18 | Obstacle avoidance and target-reaching acceptance |

## Documentation

| Resource | Purpose |
|---|---|
| [Hardware Baseline / Pin Map](docs/hardware.md) | Hardware definitions and frozen MCU resource map |
| [硬件基线 / Pin Map](docs/hardware.zh-CN.md) | 中文硬件知识库 |
| `interfaces/` | Shared transport and protocol contracts |
| `CHANGELOG.md` | Cross-domain changes that require peer awareness |
| `AGENTS.md` | Mandatory Codex engineering rules |

## Project Focus

The project is designed to demonstrate practical engineering experience in:

- STM32G4 and FreeRTOS
- motor and encoder control
- IMU and ADC integration
- CAN / CAN FD
- embedded safety and watchdog design
- Embedded Linux + MCU heterogeneous systems
- ROS 2 and Nav2
- LiDAR SLAM
- Ascend NPU perception
- real wheel odometry
- end-to-end autonomous navigation

## License

TBD.
