# 硬件基线与 STM32 Pin Map

[English Version](hardware.md)

本文档维护 `ros2-stm32-autonomous-robot` 的长期硬件基线。

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
| 实时控制域 | STM32G474RET6 | Safety、Motor、Encoder、IMU、ADC、CAN | M1-M3 VERIFIED；M4 固件完成 |
| MCU 开发板 | DeveBox STM32G474R Ver:20 | STM32 开发载板 | VERIFIED |
| 电机驱动 | TB6612 双路 DC Motor Driver | 双路有刷直流电机 H Bridge | 已由受控真实转动 VERIFIED |
| IMU | ICM-42688-P | 6 轴惯性测量 | VERIFIED |
| CAN 物理层 | TJA1042T(K)/3 系列模块 | CAN Transceiver | 真实双向 CAN FD 已验证 |
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
| TIM2 | CH1 PA0、CH2 PA1 | Encoder 1 正交解码 | Hardware Encoder Mode；配置为 16 位范围 |
| TIM3 | CH1 PC6、CH2 PC7 | Encoder 2 正交解码 | Hardware Encoder Mode；配置为 16 位范围 |
| SPI1 | PA5、PA6、PA7 + PA4 CS | ICM-42688-P | SPI |
| ADC1 | PC0 / IN6 | Battery Measurement | Single Channel |
| USART2 | PA2、PA3 | Debug / Bring-up | 115200 8N1 |
| FDCAN1 | PA11、PA12 | CAN FD Transport | Nominal 500 kbit/s、Data 2 Mbit/s、FD+BRS |
| EXTI | PC4、PC5 | IMU Interrupt | Rising-edge Baseline |
| SWD | PA13、PA14 | Debug / Flash | Reserved |

TIM2/TIM3 使用 Hardware Encoder Mode 完成正交计数，当前固件基线将两者都配置为 0..65535 范围。TIM1 两路 PWM 共享同一 Time Base。

### 已验证 Encoder 映射、符号与比例

真实硬件标定通过每个车轮正向手动旋转完整 10 圈，确认了物理归属和 Raw 方向：

| 物理车轮 | Encoder | 10 圈 Raw Total | Raw 正向符号 | 逻辑归一化 |
|---|---|---:|---:|---|
| 右轮 | Encoder 1 / TIM2 | +10,595 counts | `+1`（`VERIFIED`） | logical = `+raw` |
| 左轮 | Encoder 2 / TIM3 | -10,608 counts | `-1`（`VERIFIED`） | logical = `-raw` |

归一化位于 Low-level Timer Driver 上层；Raw Counter、UART 累计 Raw 诊断值和冻结的 Protocol
`0x181` Raw Diagnostic Field 均保持不变。Motor 归属和 Polarity 已通过独立实测验证，并非从
Encoder Mapping 推断。

## 6. 电机驱动

### 硬件

| 项目 | 当前值 |
|---|---|
| Driver | TB6612 双路 DC Motor Driver |
| Motor Supply | 12 V Battery |
| Channel | Motor A + Motor B |
| Standby Control | PC8 / `STBY` |
| PWM | TIM1 CH1 + CH2 |
| Motor A 物理归属 / 正向符号 | 右轮 / `-1`（`VERIFIED`） |
| Motor B 物理归属 / 正向符号 | 左轮 / `-1`（`VERIFIED`） |

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
| Bring-up State | 已通过真实 WHO_AM_I、Sample 与硬件验收 `VERIFIED` |

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
| Orange Pi 板端 TX | 40Pin Pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042 TXD（`FROZEN`） |
| Orange Pi 板端 RX | 40Pin Pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042 RXD（`FROZEN`） |
| Orange Pi Controller | CAN3 / `822d0000.mttcan`，`mttcan-id=3`（`VERIFIED`） |
| Linux Interface | SocketCAN `can3`（`FROZEN`） |
| Nominal Bitrate | 500 kbit/s，80% Sample Point |
| STM32 CAN FD Data Timing | 2 Mbit/s，82.3529% Sample Point |
| Linux CAN FD Data Timing | 2 Mbit/s，显式 82.5% Sample Point |
| Physical Layer | TJA1042T(K)/3 Family（用户确认的系列，`VERIFIED`） |
| Bus Signal | CANH / CANL |
| Point-to-point Wiring | CANH 对 CANH、CANL 对 CANL、公共地（`VERIFIED`） |
| Termination | 物理总线两端各一个 120 ohm 终端（`VERIFIED`） |
| Production Transport Direction | CAN FD |

### 冻结 Protocol v1 Transport

| Phase | Prescaler | SEG1 | SEG2 | SJW | Result |
|---|---:|---:|---:|---:|---|
| Nominal | 17 | 15 | 4 | 4 | 500 kbit/s，80% Sample Point |
| Data | 5 | 13 | 3 | 3 | 2 Mbit/s，82.3529% Sample Point |

Protocol v1 使用 Standard 11-bit ID、CAN FD+BRS、显式 Little-endian Serialization、250 ms
Motion Command Timeout，并只采用 CAN FD Link Integrity、不增加 Application CRC。CAN ID、
Layout、Unit、Rate、Authority/Reconnection 与 ROS 实现要求已冻结在
[`interfaces/protocol_v1.md`](../interfaces/protocol_v1.md)。

CANH/CANL 连通、公共地以及双端 120 ohm Termination 已是确认接线事实。当前硬件版本的
Orange Pi CAN 分配冻结为 40Pin Pin 36、GPIO2_17/CAN_TX3 连接 TJA1042 TXD，以及 40Pin Pin 11、
GPIO2_18/CAN_RX3 连接 TJA1042 RXD。匹配的 Huawei 25.2.0 CAN3 Pinctrl 值为 TX
`<0x40 0x1>`、RX `<0x44 0x1>`。

Board-specific Signed Device Tree 已安装并通过启动验证。实时系统以 `mttcan-id=3` Probe
`822d0000.mttcan` / `mttcan@3`，并暴露为 SocketCAN `can3`；CAN3 实时 Pinmux 与 40Pin 物理映射
均已验证。旧 `822c0000.mttcan` / `can2` 调查只属于历史：CAN2 不驱动冻结接线，也不是生产
Fallback。

Linux `drv_mttcan` 默认 Sample Point 曾与 STM32 Timing 不匹配并产生大量 CAN Error。生产启动
因此显式固定 Nominal 500 kbit/s / 80%，Data 2 Mbit/s / 82.5%。该配置下真实链路保持
Error Active，TX Error=0、RX Error=0、Bus Error=0、Bus-off=0，Protocol 1.0 双向帧通过验收。

主要依据：[Huawei 25.2.0 CAN Device Tree / Pinctrl 调测流程](https://www.hiascend.com/document/detail/zh/Atlas%20200I%20A2/2520/RC/driverdevelopmentguide/atlasdg_11_0065.html)、
[Huawei 25.2.0 CAN Register / Pad Map](https://www.hiascend.com/document/detail/zh/Atlas%20200I%20A2/2520/RC/driverdevelopmentguide/atlasdg_11_0064.html)
与 [Huawei 40Pin 接口映射](https://www.hiascend.com/document/detail/zh/Atlas200IDKA2DeveloperKit/23.0.RC2/Hardware%20Interfaces/hiug/hiug_0024.html)。
TJA1042T(K)/3 Transceiver 系列已由用户确认，真实 Orange Pi <-> STM32 CAN FD+BRS 已通过验收；
精确 Module Suffix 继续作为硬件维护信息，不由软件猜测。

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

Drivetrain Wheel Track 是独立 Physical Measurement 参数，不从 Navigation Footprint 推导。

## 11. 底盘实测与标定参数

| 参数 | 状态 |
|---|---|
| Geometric Wheel Radius | 0.023 m（`MEASURED / COMMISSIONING`） |
| Wheel-track Distance | 0.125 m（`MEASURED / COMMISSIONING`） |
| Half Track | 0.0625 m（由实测 Track `DERIVED`） |
| Geometric Wheel Circumference | 0.1445132621 m（`DERIVED`） |
| 右轮每转 Decoded Counts | 1059.5（`MEASURED / COMMISSIONING`，Encoder 1） |
| 左轮每转 Decoded Counts | 1060.8（`MEASURED / COMMISSIONING`，Encoder 2） |
| 右轮 Meters per Count | 约 0.0001363976 m/count（`DERIVED`） |
| 左轮 Meters per Count | 约 0.0001362305 m/count（`DERIVED`） |
| 右轮 Radians per Count | 约 0.005930331 rad/count（`DERIVED`） |
| 左轮 Radians per Count | 约 0.005923063 rad/count（`DERIVED`） |
| Encoder PPR / CPR Definition | TBD；可直接实测时不得从产品规格推断 |
| Gear Ratio | TBD；直接测量输出轮计数比例后不再是换算必需项 |
| Motor A/B → Left/Right Mapping | Motor A → 右轮；Motor B → 左轮（`VERIFIED`） |
| Motor Forward Sign | Motor A `-1`；Motor B `-1`（`VERIFIED`） |
| Encoder 1/2 → Left/Right Mapping | Encoder 1 → 右轮；Encoder 2 → 左轮（`VERIFIED`） |
| Forward Encoder Sign | 右轮 Raw `+1`；左轮 Raw `-1`（`VERIFIED`） |

Wheel Radius 与 Track 由用户于 2026-08-28 物理测得，目前属于 Commissioning Geometry，不是最终
Calibrated Effective Value。升级为最终标定值前，必须用真实直线行驶距离和真实旋转运动验证。
2026-08-29，用户分别将两侧物理车轮正向完整旋转 10 圈：右轮
`10595 / 10 = 1059.5` decoded counts/rev，左轮 `abs(-10608) / 10 = 1060.8` decoded counts/rev。
两侧差约 0.123%，因此 Commissioning 阶段保留独立左右比例，不做平均。
派生值使用 `meters_per_count = (2 * pi * 0.023) / counts_per_wheel_rev` 和
`radians_per_count = (2 * pi) / counts_per_wheel_rev`。在直线距离和真实旋转运动验证完成前，
这些值仍属于 Commissioning Value。

固件 0.5.4 使用 0.30 m/s 车体线速度、1.50 rad/s 车体角速度、3000 count/s 轮速和 0.60
Normalized Output 的调试包络。这些不是硬件额定值。实际安装电机的负载/堵转电流、带载可达轮速
以及 TB6612 Carrier/Module 热裕量仍待物理测量，因此目前不能声明更高的安全持续速度上限。

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
