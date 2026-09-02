# ros2-stm32-autonomous-robot 项目文档

[English](README.md)

`docs/` 是 `ros2-stm32-autonomous-robot` 的长期工程知识库。

## 文档索引

| 文档 | 语言 | 内容 |
|---|---|---|
| [Hardware Baseline and STM32 Pin Map](hardware.md) | English | Hardware Model、Pin Map、Peripheral Ownership、Geometry |
| [硬件基线与 STM32 Pin Map](hardware.zh-CN.md) | 中文 | 硬件型号、Pin Map、外设归属、机器人几何 |
| [Orange Pi / STM32 ROS Bridge](ros_bridge.md) | English | SocketCAN、Protocol 1.0 Bridge、ROS Interface 与 Obstacle-stop Demo |
| [Orange Pi / STM32 ROS 桥接](ros_bridge.zh-CN.md) | 中文 | SocketCAN、Protocol 1.0 桥接、ROS 接口与遇障停车演示 |
| [STM32 Real-Time Control Firmware](firmware.md) | English | MCU Peripheral、Real-time Control、CAN FD、Safety 与 Drivetrain Commissioning |
| [STM32 实时控制固件](firmware.zh-CN.md) | 中文 | MCU 外设、实时控制、CAN FD、安全与底盘调试 |
| [STM32 USART2 Engineering Console](uart_cli.md) | English | Command、Output Rate、Commissioning 与 Safety Reference |
| [STM32 USART2 工程控制台](uart_cli.zh-CN.md) | 中文 | 命令、输出频率、标定与安全手册 |
| `README.md` | English | 文档索引与维护规则 |
| `README.zh-CN.md` | 中文 | 中文索引与维护规则 |

## 未决项标签

| 标签 | 仅在以下情况使用 |
|---|---|
| `IN PROGRESS` | 结果尚未确定的当前工作 |
| `PROPOSED` | 等待决策或测量的候选项 |
| `TBD` | 有意保留的未决项 |

既有配置和实测事实直接陈述，不附加生命周期标签。

## 知识库更新触发条件

以下信息获得真实结果后，同步更新对应中英文文档：

| 类别 | 典型内容 |
|---|---|
| Drivetrain | Wheel Radius、Wheel Track、Gear Ratio |
| Encoder | CPR/PPR、方向、左右轮归属 |
| Motor | A/B 归属、方向、Safe Stop |
| IMU | 配置、标定、Interrupt Behavior |
| Battery | ADC Calibration、Voltage Conversion |
| CAN / CAN FD | Bit Timing、ID、Payload、Timeout Semantics |
| Safety | State Machine、Watchdog、Fault Recovery |
| Odometry | Wheel Model、TF Ownership、Accuracy |
| Navigation | Real FollowPath、Obstacle Avoidance Evidence |
| Performance | Rate、Latency、Tracking Error、CPU Load |

## 文档格式

统一使用：

- 简洁、工程化的技术表达；
- 表格呈现稳定结构化事实；
- 树状图呈现目录与层级；
- 所有物理量明确单位；
- Pin、Peripheral、Topic、Frame、Register、Protocol Name 保持精确；
- 实测数据附带验收上下文。

未决假设保持 `PROPOSED` 或 `TBD`；既有事实直接陈述。
