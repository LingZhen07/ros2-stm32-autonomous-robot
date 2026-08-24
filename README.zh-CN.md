# RobotProject

[English](README.md)

RobotProject 是一套融合 **ROS 2、Ascend NPU、STM32 实时控制、LiDAR、RGB-D 与 CAN/CAN FD** 的异构自主移动机器人系统。

系统覆盖从感知、建图、规划到实时运动控制、轮速反馈和物理闭环的完整链路。

## 项目概览

| 层级 | 平台 / 技术 | 核心职责 | 状态 |
|---|---|---|---|
| 高算力域 | Orange Pi AI Pro 8GB | ROS 2、感知、SLAM、Nav2、系统桥接 | 已完成 Navigation v1 |
| AI 加速 | Ascend 310B4 | YOLOv8n 推理 | Frozen |
| 实时控制域 | STM32G474RET6 + FreeRTOS | 安全、电机、编码器、IMU、ADC、CAN | 当前开发重点 |
| 环境感知 | RPLIDAR A1 + Astra RGB-D | LaserScan、RGB、Depth | Frozen |
| 运动执行 | TB6612 + 双路 DC Motor | 差速底盘驱动 | 待 Bring-up |
| 状态反馈 | 正交编码器 + ICM-42688-P | 轮状态与惯性测量 | 待 Bring-up |

## 系统架构

| 阶段 | 输入 | 处理域 / 所有者 | 输出 |
|---|---|---|---|
| 感知 | RGB-D、LiDAR | Orange Pi + Ascend 310B4 | 障碍物与传感器数据 |
| 建图 | `/scan`、TF | Cartographer | `/map`、SLAM 位姿 |
| 导航 | Goal、Map、Costmap | Nav2 | `geometry_msgs/msg/Twist` |
| 传输 | 机体速度命令 | Linux ↔ MCU Bridge | CAN / CAN FD 帧 |
| 实时控制 | 已校验的 `v`、`ω` | STM32G474RET6 | 左右轮目标速度 |
| 执行 | 轮速目标 | PWM + TB6612 | 电机输出与机器人运动 |
| 反馈 | Encoder、IMU、Battery ADC | STM32G474RET6 | Telemetry 与轮状态 |
| 里程计 | 轮状态 | Orange Pi Bridge | `nav_msgs/msg/Odometry`、`/odom` |
| 闭环 | `/odom`、LaserScan、Costmap | Nav2 Controller | 连续路径跟随 |

### 命令链路

| 步骤 | 接口 |
|---:|---|
| 1 | Navigation Goal |
| 2 | Nav2 Planner / Controller |
| 3 | `geometry_msgs/msg/Twist` |
| 4 | Orange Pi Transport Bridge |
| 5 | CAN / CAN FD |
| 6 | STM32 Command Validation |
| 7 | Differential-drive Wheel Targets |
| 8 | Closed-loop Wheel Control |
| 9 | 机器人实际运动 |

### 反馈链路

| 步骤 | 接口 |
|---:|---|
| 1 | Encoder Counts + IMU + System Telemetry |
| 2 | STM32 Wheel-state Estimation |
| 3 | CAN / CAN FD |
| 4 | Orange Pi Transport Bridge |
| 5 | `nav_msgs/msg/Odometry` |
| 6 | `/odom` 与 `odom → base_link` |
| 7 | Nav2 Controller Feedback |

## 当前进度

| 子系统 | 状态 |
|---|---|
| Orange Pi / Ubuntu / Vendor BSP | Complete |
| Ascend 310B4 Baseline | Complete / Frozen |
| ROS 2 Humble Foundation | Complete / Frozen |
| Astra RGB-D | Complete / Frozen |
| YOLOv8n Ascend Inference | Working / Frozen |
| RPLIDAR A1 | Complete / Frozen |
| TF System v1 | Complete / Frozen |
| rosbag2 / Diagnostics | Complete / Frozen |
| Cartographer LiDAR SLAM v1 | Complete / Frozen |
| Nav2 Costmaps / Global Planning | Complete / Frozen |
| STM32 Hardware Definition | Complete |
| STM32 Pin Map v1 | Frozen |
| STM32CubeMX Baseline | In Progress |
| Encoder / IMU / Motor Firmware | Planned |
| CAN / CAN FD Integration | Planned |
| Real Wheel Odometry | Planned |
| Nav2 Physical FollowPath | Planned |
| Closed-loop Autonomous Navigation | Target |

**当前工程阶段：** `STM32 REAL-TIME CONTROL DOMAIN`

## 硬件组成

### 高算力域

| 组件 | 角色 |
|---|---|
| Orange Pi AI Pro 8GB | ROS 2 主机与高层计算 |
| Ascend 310B4 | NPU 推理加速 |
| Astra RGB-D | RGB 与深度感知 |
| RPLIDAR A1 | 2D 激光雷达 |

### 实时控制域

| 组件 | 角色 |
|---|---|
| STM32G474RET6 | 实时控制器 |
| DeveBox STM32G474R Ver:20 | MCU 开发板 |
| ICM-42688-P | 6 轴 IMU |
| TB6612 双路 DC Motor Driver | 电机 H 桥 |
| TJA1042/TJA1043 系列模块 | CAN 物理层 |
| 正交编码器 | 轮速反馈 |
| 12 V Battery | 电机供电 |

详细信息见：[硬件基线与 STM32 Pin Map](docs/hardware.zh-CN.md)

## STM32 Pin Map v1

| 功能 | Pin | Peripheral |
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
| Motor Direction | PB12–PB15 | GPIO |

### 控制基线

| 参数 | 数值 |
|---|---:|
| HSE | 8 MHz |
| SYSCLK | 170 MHz |
| Motor PWM | 10 kHz |
| Debug UART | 115200 8N1 |
| CAN Nominal Bring-up Bitrate | 500 kbit/s |

## 仓库结构

```text
RobotProject/
├── firmware/          STM32 firmware
├── ros2_ws/           ROS 2 packages 与高算力软件
├── interfaces/        Linux ↔ STM32 通信合同
├── docs/              中英双语工程知识库
├── AGENTS.md          Codex 共享工程规则
├── CHANGELOG.md       跨域工程变更通信
├── README.md          英文项目主页
└── README.zh-CN.md    中文项目主页
```

两个 Codex CLI 并行协作：

| Agent | 主要工作域 | 运行环境 |
|---|---|---|
| Firmware Codex | `firmware/` | Windows 本地 STM32 开发 |
| ROS Codex | `ros2_ws/` | SSH 连接 Orange Pi |

共享信息通过 `interfaces/`、`docs/` 与 `CHANGELOG.md` 同步。

## 工程流程

| 阶段 | 验收原则 |
|---|---|
| Inspect | 读取真实硬件与实际运行环境 |
| Implement | 完成最小有效子系统 |
| Verify | 获取真实电气与软件证据 |
| Stabilize | 修复已观察到的问题 |
| Accept | 记录可测量的通过条件 |
| Freeze | 固化已工作的模块 |
| Integrate | 推进到下一个系统边界 |

### 硬件验收证据

| 子系统 | 验收证据 |
|---|---|
| Encoder | 实际转轮产生连续、方向一致的计数 |
| IMU | 有效 `WHO_AM_I` 与真实运动数据 |
| PWM | 实测 10 kHz 波形与目标 Duty |
| CAN | 真实 TX/RX Frame |
| Motor | 受控运动与可靠 Safe Stop |
| Odometry | 真实位移能够定量反映到 `/odom` |
| Navigation | 实机完成避障并到达目标位置 |

## Navigation 与 Odometry

Nav2 速度命令：

```text
linear.x  [m/s]
angular.z [rad/s]
```

STM32 负责命令校验和 Differential-drive Wheel Target 计算。

轮状态经 Linux Bridge 转换为：

```text
nav_msgs/msg/Odometry
```

Wheel Odometry 接管后，系统保持唯一的：

```text
odom → base_link
```

TF Authority。

## 开发路线

| 阶段 | 里程碑 |
|---:|---|
| 1 | STM32CubeMX Baseline |
| 2 | SWD + Safe GPIO Startup |
| 3 | USART2 Debug Bring-up |
| 4 | Battery ADC Verification |
| 5 | TIM2 / TIM3 Encoder Bring-up |
| 6 | ICM-42688-P SPI + Interrupt Bring-up |
| 7 | TIM1 10 kHz PWM Verification |
| 8 | FreeRTOS Minimal Runtime |
| 9 | Safety Supervisor + Watchdog |
| 10 | Controlled Motor Test |
| 11 | Closed-loop Wheel Velocity |
| 12 | FDCAN Physical Bring-up |
| 13 | Shared CAN / CAN FD Protocol v1 |
| 14 | Orange Pi ↔ STM32 Bridge |
| 15 | Wheel Odometry + Real `/odom` |
| 16 | Nav2 Controller / FollowPath |
| 17 | Closed-loop Autonomous Navigation |
| 18 | Obstacle Avoidance + Target-reaching Acceptance |

## 文档入口

| 文档 | 内容 |
|---|---|
| [Hardware Baseline / Pin Map](docs/hardware.md) | 英文硬件定义与 MCU 资源表 |
| [硬件基线 / Pin Map](docs/hardware.zh-CN.md) | 中文硬件知识库 |
| `interfaces/` | 跨域 Transport / Protocol 合同 |
| `CHANGELOG.md` | 需要另一个 Codex 同步知晓的跨域变更 |
| `AGENTS.md` | Codex 强制工程规则 |

## 项目能力覆盖

项目最终展示以下工程能力：

- STM32G4 / FreeRTOS
- Motor / Encoder Closed-loop Control
- IMU / ADC Integration
- CAN / CAN FD
- Embedded Safety / Watchdog
- Embedded Linux + MCU Heterogeneous System
- ROS 2 / Nav2
- LiDAR SLAM
- Ascend NPU Perception
- Real Wheel Odometry
- End-to-end Autonomous Navigation

## License

TBD.
