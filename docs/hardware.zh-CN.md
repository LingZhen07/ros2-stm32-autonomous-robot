# 硬件基线与 STM32 Pin Map

[English Version](hardware.md)

本文档维护 RobotProject 的长期硬件基线。

## 状态定义

| 状态 | 含义 |
|---|---|
| `FROZEN` | 已验收配置；保持稳定，真实证据触发时再调整 |
| `VERIFIED` | 已有真实硬件或有效工程证据支持 |
| `IN PROGRESS` | 当前 Bring-up / Integration 项 |
| `PROPOSED` | 等待验证的工程候选 |
| `TBD` | 有意保留，等待后续实测或设计 |

## 1. 硬件概览

| 域 | 组件 | 职责 | 状态 |
|---|---|---|---|
| 高算力域 | Orange Pi AI Pro 8GB | ROS 2、感知、SLAM、Nav2、Bridge | VERIFIED |
| AI 加速 | Ascend 310B4 | YOLOv8n 推理 | VERIFIED / FROZEN |
| 实时控制域 | STM32G474RET6 | Safety、Motor、Encoder、IMU、ADC、CAN | IN PROGRESS |
| MCU 开发板 | DeveBox STM32G474R Ver:20 | STM32 开发载板 | VERIFIED |
| 电机驱动 | TB6612 双路 DC Motor Driver | 双路有刷直流电机 H Bridge | IN PROGRESS |
| IMU | ICM-42688-P | 6 轴惯性测量 | IN PROGRESS |
| CAN 物理层 | TJA1042/TJA1043 系列模块 | CAN Transceiver | IN PROGRESS |
| LiDAR | RPLIDAR A1 | SLAM/Nav2 的 2D LaserScan | VERIFIED / FROZEN |
| RGB-D Camera | Astra RGB-D | RGB 与 Depth | VERIFIED / FROZEN |

## 2. 高算力平台

| 项目 | 当前值 |
|---|---|
| Board | Orange Pi AI Pro 8GB |
| OS | Ubuntu 22.04.5 LTS |
| Architecture | aarch64 |
| Kernel | Vendor Linux 5.10 BSP |
| ROS | ROS 2 Humble |
| NPU | Ascend 310B4 |
| ROS Workspace | `/data/ros2_ws` |
| Project Data | `/data/projects` |

高算力域当前已验收到 Non-actuating Navigation v1。

## 3. STM32 平台

| 项目 | 当前值 |
|---|---|
| MCU | STM32G474RET6 |
| Package | LQFP64 |
| Core | Arm Cortex-M4 + FPU |
| Development Board | DeveBox STM32G474R Ver:20 |
| HSE | 8 MHz |
| SYSCLK Target | 170 MHz |
| Primary Datasheet | ST DS12288 |
| Reference Manual | ST RM0440 |

### SWD 保留

| 信号 | Pin |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |
| Reset | NRST |
| Reference Power | 3V3 |
| Ground | GND |

SWD 在整个 Bring-up 阶段保持可用。

## 4. STM32 Pin Map v1 — FROZEN

| 外部功能 | MCU Pin | STM32 功能 | AF / 模式 | 外设归属 | 启动状态 / 备注 |
|---|---|---|---|---|---|
| Encoder 1 A | PA0 | TIM2_CH1 | AF1 | TIM2 | Pull-up 基线 |
| Encoder 1 B | PA1 | TIM2_CH2 | AF1 | TIM2 | Pull-up 基线 |
| Debug UART TX | PA2 | USART2_TX | AF7 | USART2 | 115200 8N1 |
| Debug UART RX | PA3 | USART2_RX | AF7 | USART2 | 115200 8N1 |
| IMU CS | PA4 | GPIO Output | GPIO | SPI1 Device Select | HIGH |
| IMU SCLK | PA5 | SPI1_SCK | AF5 | SPI1 | ICM-42688-P |
| IMU MISO | PA6 | SPI1_MISO | AF5 | SPI1 | ICM-42688-P |
| IMU MOSI | PA7 | SPI1_MOSI | AF5 | SPI1 | ICM-42688-P |
| Motor A PWM | PA8 | TIM1_CH1 | AF6 | TIM1 | 10 kHz；启动 Duty 0 |
| Motor B PWM | PA9 | TIM1_CH2 | AF6 | TIM1 | 10 kHz；启动 Duty 0 |
| CAN RX | PA11 | FDCAN1_RX | AF9 | FDCAN1 | Frozen |
| CAN TX | PA12 | FDCAN1_TX | AF9 | FDCAN1 | Frozen |
| SWDIO | PA13 | SWDIO | Debug | SWD | Reserved |
| SWCLK | PA14 | SWCLK | Debug | SWD | Reserved |
| Battery ADC | PC0 | ADC1_IN6 | Analog | ADC1 | No Pull |
| IMU INT1 | PC4 | EXTI4 | Rising-edge 基线 | EXTI | Pull-down 基线 |
| IMU INT2 | PC5 | EXTI5 | Rising-edge 基线 | EXTI9_5 IRQ | Pull-down 基线 |
| Encoder 2 A | PC6 | TIM3_CH1 | AF2 | TIM3 | Pull-up 基线 |
| Encoder 2 B | PC7 | TIM3_CH2 | AF2 | TIM3 | Pull-up 基线 |
| TB6612 STBY | PC8 | GPIO Output | GPIO | Motor Safety | LOW |
| TB6612 AIN1 | PB12 | GPIO Output | GPIO | Motor A Direction | LOW |
| TB6612 AIN2 | PB13 | GPIO Output | GPIO | Motor A Direction | LOW |
| TB6612 BIN1 | PB14 | GPIO Output | GPIO | Motor B Direction | LOW |
| TB6612 BIN2 | PB15 | GPIO Output | GPIO | Motor B Direction | LOW |

**Pin Allocation v1 状态：** `FROZEN`

引脚调整以真实 Hardware / CubeMX 冲突证据为触发条件。

## 5. 外设归属

| Peripheral | Channel / Pin | 职责 | 基线 |
|---|---|---|---|
| TIM1 | CH1 PA8、CH2 PA9 | 同步 Motor PWM | 10 kHz |
| TIM2 | CH1 PA0、CH2 PA1 | Encoder 1 正交解码 | Hardware Encoder Mode |
| TIM3 | CH1 PC6、CH2 PC7 | Encoder 2 正交解码 | Hardware Encoder Mode |
| SPI1 | PA5、PA6、PA7 + PA4 CS | ICM-42688-P | SPI |
| ADC1 | PC0 / IN6 | Battery Measurement | Single Channel |
| USART2 | PA2、PA3 | Debug / Bring-up | 115200 8N1 |
| FDCAN1 | PA11、PA12 | CAN Transport | 500 kbit/s Nominal Bring-up |
| EXTI | PC4、PC5 | IMU Interrupt | Rising-edge Baseline |
| SWD | PA13、PA14 | Debug / Flash | Reserved |

TIM2/TIM3 使用 Hardware Encoder Mode 完成正交计数。TIM1 两路 PWM 共享同一 Time Base。

## 6. 电机驱动

### 硬件

| 项目 | 当前值 |
|---|---|
| Driver | TB6612 双路 DC Motor Driver |
| Motor Supply | 12 V Battery |
| Channel | Motor A + Motor B |
| Standby Control | PC8 / `STBY` |
| PWM | TIM1 CH1 + CH2 |

### 控制信号

| Signal | MCU Pin | Function |
|---|---|---|
| PWMA | PA8 | Motor A PWM |
| AIN1 | PB12 | Motor A Direction |
| AIN2 | PB13 | Motor A Direction |
| PWMB | PA9 | Motor B PWM |
| BIN1 | PB14 | Motor B Direction |
| BIN2 | PB15 | Motor B Direction |
| STBY | PC8 | Motor-domain Enable |

### 启动安全基线

| Signal Group | Startup State |
|---|---|
| STBY | LOW |
| AIN1 / AIN2 | LOW |
| BIN1 / BIN2 | LOW |
| TIM1 PWM Duty | 0% |

后续由 Safety Supervisor 管理 Motor Domain 的 Enable Transition。

### Battery Measurement

| 项目 | 当前值 |
|---|---|
| Driver-board ADC Output | `Vadc ≈ Vbattery / 11` |
| MCU Input | PC0 |
| ADC Channel | ADC1_IN6 |
| Calibration | Multimeter Reference + ADC Reading |
| Safety Threshold | TBD after Calibration |

## 7. IMU

| 项目 | 当前值 |
|---|---|
| Device | TDK InvenSense ICM-42688-P |
| Sensor Type | 6-axis Accelerometer + Gyroscope |
| Host Interface | SPI |
| Maximum Device SPI Capability | 24 MHz |
| Bring-up State | IN PROGRESS |

### MCU Mapping

| IMU Signal | MCU Pin | Function |
|---|---|---|
| CS | PA4 | GPIO Output |
| SCLK | PA5 | SPI1_SCK |
| MISO | PA6 | SPI1_MISO |
| MOSI | PA7 | SPI1_MOSI |
| INT1 | PC4 | EXTI4 |
| INT2 | PC5 | EXTI5 |

### Bring-up Sequence

| 阶段 | 验收证据 |
|---:|---|
| 1 | SPI Register Access |
| 2 | Valid `WHO_AM_I` |
| 3 | Real Accelerometer / Gyroscope Samples |
| 4 | Interrupt Behavior Verified |
| 5 | Measured Throughput 支持后引入 DMA / FIFO |

Breakout 同时暴露 `VCC` 与 `3.3V`。两者的板级功能保持 Physical Verification 状态。

## 8. CAN / CAN FD

| Layer | Component / Setting |
|---|---|
| MCU Controller | STM32 FDCAN1 |
| RX | PA11 / FDCAN1_RX |
| TX | PA12 / FDCAN1_TX |
| Nominal Bring-up Bitrate | 500 kbit/s |
| Physical Layer | TJA1042/TJA1043 Family |
| Bus Signal | CANH / CANL |
| Production Transport Direction | CAN FD |

### 待设计协议项

| 项目 | 状态 |
|---|---|
| CAN IDs | TBD |
| CAN FD Data-phase Bitrate | TBD |
| Payload Packing | TBD |
| Scaling / Units | TBD |
| CRC Policy | TBD |
| Heartbeat Format | TBD |
| Sequence Format | TBD |
| Protocol Version | TBD |

共享 Transport Contract 完成设计后，上述内容统一进入 `interfaces/`。

Transceiver 的精确 Subvariant、Termination、Standby 与 Logic-level Implementation 继续保持 Hardware Verification 状态。

## 9. Debug UART

| 项目 | 当前值 |
|---|---|
| Peripheral | USART2 |
| TX | PA2 |
| RX | PA3 |
| Baud Rate | 115200 |
| Data | 8 bits |
| Parity | None |
| Stop Bits | 1 |
| Role | Bring-up、Diagnostics、Fallback Communication |

## 10. 已冻结机器人几何

### Coordinate Frame

| Axis / Origin | Definition |
|---|---|
| `base_link` Origin | 左右驱动轮轴线中点 |
| +X | Robot Forward |
| +Y | Robot Left |
| +Z | Robot Up |

### Sensor Transform

| Transform | x [m] | y [m] | z [m] | roll | pitch | yaw |
|---|---:|---:|---:|---:|---:|---:|
| `base_link → camera_link` | +0.130 | 0.000 | +0.110 | 0 | 0 | 0 |
| `base_link → laser_frame` | +0.043 | 0.000 | +0.165 | 0 | 0 | π |

### Footprint

```yaml
footprint: "[[0.140, 0.080], [0.140, -0.080], [-0.070, -0.080], [-0.070, 0.080]]"
footprint_padding: 0.01
```

Drivetrain Wheel Track 保持独立 Physical Measurement 参数。

## 11. 待实测底盘参数

| 参数 | 状态 |
|---|---|
| Effective Wheel Radius | TBD |
| Wheel-track Distance | TBD |
| Encoder PPR / CPR Definition | TBD |
| Gear Ratio | TBD |
| Motor A/B → Left/Right Mapping | TBD |
| Encoder 1/2 → Left/Right Mapping | TBD |
| Forward Encoder Sign | TBD |

Closed-loop Wheel Control 与 Odometry 验收前，将以真实硬件测量结果填入这些参数。

## 12. 主要参考资料

| Component | Primary Reference |
|---|---|
| STM32G474 | STMicroelectronics DS12288 |
| STM32G4 Peripheral | STMicroelectronics RM0440 |
| STM32G474 Silicon Notes | STM32G474 Device Errata |
| ICM-42688-P | TDK InvenSense DS-000347 |
| TB6612FNG | Toshiba TB6612FNG Datasheet |
| TJA1042 | NXP TJA1042 Datasheet |
| TJA1043 | NXP TJA1043 Datasheet |

涉及模块版本与板级实现时，以原厂资料和真实硬件测量共同构成集成依据。
