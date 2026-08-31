# STM32 M1-M5 生产固件

[English](firmware.md)

## 状态

| 里程碑 | 状态 | 证据边界 |
|---|---|---|
| M1 MCU 基础 | `VERIFIED` | 真实 STM32 执行、SWD、USART2、时钟与启动行为 |
| M2 外设基础 | `VERIFIED` | 真实 ADC、Encoder、ICM-42688-P、TIM1 PWM 行为 |
| M3 实时控制基础 | `VERIFIED` | FreeRTOS、安全、看门狗架构、TB6612 链路与真实电机转动 |
| M4 Protocol v1 + FDCAN 固件 | `FROZEN / PHYSICALLY INTEGRATED` | 共享合同、固件集成与 Orange Pi CAN3 真实链路均已验收 |
| M5 实机集成 | `VERIFIED / PASS` | BODY_COMMAND_READY、Motion Authority、0.30 m/s 闭环直行、LiDAR Stop、撤权与 STM32 Safe Stop 均已验收 |

M1-M5 已完成真实硬件验收。Protocol 1.0 保持冻结，生产传输保持 Orange Pi CAN3 / SocketCAN
`can3`。

## MCU 基线

| 项目 | 冻结配置 |
|---|---|
| Target | STM32G474RET6，LQFP64；DeveBox STM32G474R Ver:20 |
| Clock | 8 MHz HSE，PLL M=2/N=85/R=2，SYSCLK/HCLK/PCLK1/PCLK2 170 MHz |
| Debug / Time Base | 保留 SWD；TIM6 提供 HAL 1 ms；SysTick 提供 FreeRTOS 1 ms Tick |
| PWM | TIM1 CH1/CH2，10 kHz，PSC=0，ARR=16999，启动 CCR=0 |
| Encoder | TIM2/TIM3 Hardware Encoder Mode，均为 0..65535 的 16 位范围 |
| IMU | SPI1 Mode 0，ICM-42688-P 100 Hz 基线，EXTI 唤醒任务 |
| Battery | ADC1 IN6，执行 ADC 校准，软件触发长采样 |
| Diagnostics | 永久 USART2 115200 8N1 完整块串行 CLI；默认安静 |
| Watchdog | IWDG 名义约 4 s；仅 Supervisor 刷新；调试暂停时冻结 |
| FDCAN | Nominal 500 kbit/s，Data 2 Mbit/s，FD+BRS，Standard ID |

冻结引脚图和安全启动电平继续以 [hardware.zh-CN.md](hardware.zh-CN.md) 为准。

## 已验证硬件事实

首次集成板级验收已获得以下真实证据：

- STM32 执行与 USART2 实际字节；
- 真实编码器采集；
- ICM-42688-P 通信与真实惯性数据；
- Battery ADC 测量链路；
- TIM1 PWM 输出；
- TB6612 电机控制电气链路及真实电机转动。

已验证编码器符号为：

```text
左轮正转  -> raw CPS < 0
右轮正转  -> raw CPS > 0
逻辑正转  -> 两侧归一化速度均 > 0
```

符号修正位于 `app_drivetrain`，不修改 Timer Encoder Driver。Commissioning 已验证 Encoder 1
属于右轮且正向 Raw Sign 为 `+1`，Encoder 2 属于左轮且正向 Raw Sign 为 `-1`。Encoder 1/2 Raw
诊断保持不变；逻辑轮状态和 Protocol `0x181` 使用右轮=`+raw`、左轮=`-raw`。真实调试也已验证
Motor A 属于右轮、Motor B 属于左轮，两路电机的逻辑正向符号均为 `-1`；归一化仍位于 Motor
GPIO/PWM Driver 上层。

## 生产源码架构

CubeMX 管理的初始化位于 `firmware/Core`，项目自有逻辑位于 `firmware/App`。

| 模块 | 职责 |
|---|---|
| `app_config` | 固件身份、调度、安全和通信常量 |
| `app_state` / `app_safety` | BOOT/INIT/SAFE/READY/ACTIVE/FAULT 与集中安全收敛 |
| `app_supervisor` | 关键任务心跳和唯一 Watchdog Feed Owner |
| `app_command` | 共用命令、来源、时间戳与本地 Freshness |
| `app_drivetrain` | 轮/电机映射、已验证逻辑符号、标定 Guard、差速换算 |
| `app_control` | 左右轮独立 PI/PID-compatible 控制、限幅与 Anti-windup |
| `app_motor` | TB6612 Direction/PWM/STBY 与 Emergency-safe 输出 |
| `app_encoder` | 16 位 Wrap-safe Delta、累计计数、Raw/Filtered CPS |
| `app_imu` | ICM Register Driver、WHO_AM_I、数据采集与中断状态 |
| `app_battery` | ADC Calibration、分压模型、滤波估计与标定有效性 |
| `app_telemetry` | 一致的 Sensor/Control/Safety Snapshot |
| `app_protocol` | Protocol v1 显式 Little-endian 编解码；禁止 Packed Struct Cast |
| `app_can` | FDCAN Filter、ISR RX Ring、Session/Sequence/运行限值校验、CAN Health、TX Schedule |
| `app_diagnostics` | 轻量非阻塞完整块串行 UART 控制台、1 Hz Watch 与链路诊断 |
| `app_rtos` | 紧凑任务模型与确定性所有权 |

固件版本为 `0.5.4`，共享线协议版本保持 `1.0`。

## FreeRTOS 与中断架构

| Task | Priority | Stack | 基线行为 |
|---|---:|---:|---|
| SupervisorTask | Realtime | 2048 B | 20 ms Safety / Watchdog Cycle |
| MotorControl | High | 1024 B | 10 ms Encoder / Control / Output Path |
| Imu | AboveNormal | 1024 B | EXTI Wake，20 ms Fallback |
| Communication | Normal | 2048 B | Event Wake 或最大 5 ms Wait；UART + FDCAN Task-context 处理 |
| Telemetry | BelowNormal | 1024 B | 50 ms Service；只负责 Battery Acquisition 调度 |

FDCAN1_IT0 Priority 为 5，与 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 相容。ISR 将三项
Hardware FIFO 搬运到有界八槽项目 Ring，记录 Overflow/Error，并唤醒 CommunicationTask；
ISR 不解析 Protocol v1，也不改变电机目标。协议校验与 Telemetry Serialization 均位于任务上下文。

## 安全与 CAN Authority

生产状态机保持：

```text
BOOT -> INIT -> SAFE -> READY -> ACTIVE
                     \          /
                      -> FAULT <-
```

上电和所有 Fault Path 均强制 STBY LOW、Direction LOW、PWM 0。CAN 不能绕过既有 Supervisor、
内部 Command Model、Drivetrain Guard、Wheel Controller 或 Motor Layer。

M5 在 CAN `BODY_VELOCITY` 写入共享命令模型之前，先验证 Controller Configuration、Drivetrain
Readiness 和本地 Operating Limits。配置不完整或越界的 Active Command 会被拒绝，并通过既有
Safety Supervisor 安全收敛；绝不 Clamp 成另一种运动。

Protocol v1 的激活顺序是：

```text
Fresh Heartbeat
-> 显式 DISARMED Handshake
-> 显式 ARMED Authority
-> Fresh BODY_VELOCITY Command
-> 所有 Drivetrain / Controller / Safety Gate
-> ACTIVE
```

Motion Command Timeout 为 250 ms，Host Heartbeat 与 Motion Authority Timeout 均为 500 ms，
全部使用 STM32 本地接收时间。Timeout、Session Replacement、Bus Off 或 RX Loss 都会清除 CAN Authority，绝不会
复用旧速度。恢复必须重新执行 DISARMED/ARMED。

Disabled/Disarmed Command Snapshot 被明确视为未超时。只有已经建立的 Motion/Control Session
失去必要 Freshness（例如 Valid Motion Command 到期，或 Armed CAN Authority/Heartbeat 超时）
时，`COMMAND_TIMEOUT` 才生效；Boot/SAFE 状态不会仅因没有命令而产生该 Fault。

Encoder Acquisition Validity 与 Drivetrain/Controller Readiness 相互独立。Timer Start 失败、
Invalid Sample 或任一路 Sample Age 超过 50 ms 时置位 `ENCODER_VALIDITY`。只有两路 Encoder
Timer 均已初始化成功、两路 Sample 均恢复有效且 Age 不超过 50 ms 时，该位才会自动清除。
未知 Motor A/B Mapping/Polarity、未配置 Gain/Limit 和 `BODY_COMMAND_READY=false` 都不会置位它。

当通信丢失影响 CAN 所有的运动时，集中 Fault Word 设置 FDCAN Communication Bit。Warning
与 Error Passive 保持可观测，不建立第二套 Safety State Machine。固件每 1 s 尝试一次 Bus-off
恢复，但硬件恢复不会恢复运动授权。

USART2 以紧凑格式显示 System/Sensor/Fault，以及 RX/TX/Reject/Sequence/Overflow/Bus-off Count、
STM32 本地 Heartbeat/Authority/Command Age、Session ID、Authority 与 Motor Enable。启动块后没有
非请求周期遥测；Watch 只支持 Encoder、IMU、CAN，全部固定为 1 Hz。完整响应块串行输出，UART
Transmission 不阻塞 MotorControlTask。全部命令见
[STM32 USART2 生产版 CLI](uart_cli.zh-CN.md)。

## Protocol v1 Transport

| 项目 | 数值 |
|---|---|
| Nominal Timing | Prescaler 17，SEG1 15，SEG2 4，SJW 4；500 kbit/s，80% Sample Point |
| Data Timing | Prescaler 5，SEG1 13，SEG2 3，SJW 3；2 Mbit/s，82.3529% Sample Point |
| RX IDs | `0x080` Authority、`0x081` Motion、`0x082` Heartbeat |
| TX IDs | `0x180` System、`0x181` Wheel、`0x182` IMU、`0x183` Battery |
| Integrity | 仅 CAN FD Link CRC；无 Application CRC |
| RX Policy | 精确 Filter；校验 FD+BRS、Length、Version、Reserved、Session、Sequence |
| TX Policy | CommunicationTask 唯一负责确定性调度；其他模块只更新内部 Telemetry |

完整字段偏移、Golden Frame、ROS 换算和 Fault 含义见
[Protocol v1](../interfaces/protocol_v1.md)。

## Drivetrain 配置边界

集中固件默认值现已包含用户实测的 Commissioning Geometry：

| 数值 | Commissioning Setting | 状态 |
|---|---:|---|
| Wheel Radius | 0.023 m | `MEASURED / COMMISSIONING` |
| Wheel Track | 0.125 m | `MEASURED / COMMISSIONING` |
| 差速换算使用的 Half Track | 0.0625 m | `DERIVED` |
| Geometric Circumference | 0.1445132621 m | `DERIVED` |
| Encoder 1 归属 / 正向符号 | 右轮 / `+1` | `VERIFIED` |
| Encoder 2 归属 / 正向符号 | 左轮 / `-1` | `VERIFIED` |
| Motor A 归属 / 逻辑正向符号 | 右轮 / `-1` | `VERIFIED` |
| Motor B 归属 / 逻辑正向符号 | 左轮 / `-1` | `VERIFIED` |
| 右轮每转计数 | 1059.5 | `MEASURED / COMMISSIONING` |
| 左轮每转计数 | 1060.8 | `MEASURED / COMMISSIONING` |
| 右轮每 Count 米数 / 弧度 | 0.0001363976 m / 0.005930331 rad | `DERIVED` |
| 左轮每 Count 米数 / 弧度 | 0.0001362305 m / 0.005923063 rad | `DERIVED` |
| 最大车体线速度 | 0.30 m/s | `COMMISSIONING LIMIT` |
| 最大车体角速度 | 1.50 rad/s | `COMMISSIONING LIMIT` |
| 最大归一化轮速 | 3000 count/s（约 0.409 m/s） | `COMMISSIONING LIMIT` |
| 轮目标斜率 | 4400 count/s^2（约 0.60 m/s^2） | `COMMISSIONING LIMIT` |
| 电机 Effort / PWM 输出 | +/-0.60 normalized | `COMMISSIONING LIMIT` |

固件实现的 Differential-drive Relationship 因此为：

```text
v_left  = v - omega * 0.0625
v_right = v + omega * 0.0625
```

Encoder Scale 来自每个物理车轮正向完整旋转 10 圈：Encoder 1 累计 +10,595 counts，Encoder 2
累计 -10,608 counts。两侧约 0.123% 的差异会分别保留，固件不取平均。Radius、Track 和 Encoder
Scale 目前仍是 Commissioning Value，后续必须用真实直线行驶距离和真实旋转运动验证。

初始左右独立 PI 配置均为 `Kp=0.00020`、`Ki=0.00060`、`Kd=0`，Integrator Limit 为
700 count-seconds，Output Limit 为 0.60。这是有意相同的初始调试参数，不是已经实车调谐的 Gain。
控制器会拒绝非有限值、执行 Output Clamp 和 Conditional-integration Anti-windup，并只在有效
Active Command 下使用目标斜坡。任何 Safety Convergence 都会复位 Controller 并直接强制 Motor
Safe，Authority Withdrawal 不会被斜坡延迟。

初始化时，只有已验证 Mapping/Scale/Geometry、有限 Operating Envelope 和两个有效 Controller
Configuration 同时存在，BODY_COMMAND_READY 才为 true。Protocol `0x180` bit 15、`0x181` bit 12、
UART `status` 和 Safety Guard 都采用这一完整条件。越界 BODY_VELOCITY 会被拒绝，不会被 Clamp。

0.30 m/s 是固件调试上限，不是已验证的电机或整机最高速度。实际安装电机的负载/堵转电流、
带载可达轮速以及 TB6612 Carrier/Module 热裕量尚未实测，因此目前无法为更高持续速度提供安全
依据。0.60 Output Clamp 只限制 Duty Request；由于当前固件链路没有电流反馈，它不等于电流限制。
Encoder PPR/CPR 定义与 Gear Ratio 分解仍未知，但直接输出轮 Count Scale 已使它们不再是 Body
Conversion 的必要条件。禁止从 Nav2 Footprint 推导底盘几何。

## Telemetry 与 ROS 边界

固件输出归一化累计轮计数/速度、冻结的 Raw Encoder 1/2 Diagnostic Field、Controller
Target/Output、IMU SI 数据、Battery Estimate/Validity、Safety State、Fault Word、
Supervisor/Watchdog Health、STM32 Monotonic Time 与 Sequence。Raw 值也继续通过本地 UART 提供；
它们与 Protocol `0x181` 的逻辑字段并存且不会替代逻辑字段。固件不生成 ROS Odometry、ROS Time、
Orientation、Covariance 或 TF。

Orange Pi 拥有 `nav_msgs/msg/Odometry`，并在后续负责唯一 `odom -> base_link` TF Authority。
只有完成底盘尺度实测与真实运动一致性验收后，才能接受公制 Odometry。

## 构建与验收边界

2026-08-29 使用 Arm GNU Toolchain 完成 Debug 与 Release Clean Build，Compiler/Linker Warning
均为 0，并生成 ELF、HEX、BIN：

| Build | FLASH | RAM | Result |
|---|---:|---:|---|
| Debug | 105,544 B / 512 KiB (20.13%) | 42,848 B / 128 KiB (32.69%) | PASS |
| Release | 67,936 B / 512 KiB (12.96%) | 42,848 B / 128 KiB (32.69%) | PASS |

跨域物理链路已通过 Orange Pi CAN3 / SocketCAN `can3` 验收：在 Nominal 500 kbit/s、Data
2 Mbit/s 下真实观察到 FD+BRS `0x080`、`0x082` 和 `0x180..0x183`。Linux 显式使用 80% / 82.5%
Sample Point 后保持 Error Active，TX/RX Error 与 Bus-off 均为 0。Protocol 1.0、STM32 Timing
和机器人默认 DISARMED 行为均未改变。

最终 M5 实机 Demo 已使用固件 `0.5.4` 通过：BODY_COMMAND_READY 与 Motion Authority 均确认，
系统以当前 0.30 m/s Commissioning Limit 进行闭环直行；真实 RPLIDAR A1 `/scan` 在 30° 前方
扇区和 0.60 m 阈值内检出障碍物后，触发 Zero Velocity、Authority Withdrawal 与 STM32 Safe
Stop。障碍物移除后 STOPPED 仍锁存，必须由用户重新显式 START 才能恢复运动。该验收速度不代表
机器人物理最高速度。

从 Obstacle Detection 起，观测到 Zero Velocity Command 约 0.075 ms、Motion Authority
Withdrawal 约 1.276 ms、STM32 Stop Confirmation 约 30.8 ms。这些是成功 Demo 的观测值，不是
有保证的最坏情况安全上限。
