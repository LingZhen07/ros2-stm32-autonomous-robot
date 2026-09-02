# ros2-stm32-autonomous-robot Documentation

[中文](README.zh-CN.md)

`docs/` is the persistent engineering knowledge base for `ros2-stm32-autonomous-robot`.

## Document Map

| Document | Language | Scope |
|---|---|---|
| [Hardware Baseline and STM32 Pin Map](hardware.md) | English | Hardware models, pin map, peripheral ownership, geometry |
| [硬件基线与 STM32 Pin Map](hardware.zh-CN.md) | 中文 | 硬件型号、Pin Map、外设归属、几何信息 |
| [Orange Pi / STM32 ROS Bridge](ros_bridge.md) | English | SocketCAN, Protocol 1.0 bridge, ROS interfaces, and obstacle-stop demo |
| [Orange Pi / STM32 ROS 桥接](ros_bridge.zh-CN.md) | 中文 | SocketCAN、Protocol 1.0 桥接、ROS 接口与遇障停车演示 |
| [STM32 Real-Time Control Firmware](firmware.md) | English | MCU peripherals, real-time control, CAN FD, safety, and drivetrain commissioning |
| [STM32 实时控制固件](firmware.zh-CN.md) | 中文 | MCU 外设、实时控制、CAN FD、安全与底盘调试 |
| [STM32 USART2 Engineering Console](uart_cli.md) | English | Commands, output rates, commissioning, and safety reference |
| [STM32 USART2 工程控制台](uart_cli.zh-CN.md) | 中文 | 命令、输出频率、标定与安全手册 |
| `README.md` | English | Documentation index and maintenance rules |
| `README.zh-CN.md` | 中文 | 文档索引与维护规则 |

## Unresolved-item labels

| Label | Use only when needed |
|---|---|
| `IN PROGRESS` | Active work whose outcome is not established |
| `PROPOSED` | Candidate awaiting a decision or measurement |
| `TBD` | Intentionally unresolved |

State established configuration and measured facts directly instead of attaching lifecycle labels.

## Knowledge Base Update Triggers

Update the relevant English and Chinese documents when any of these become known:

| Category | Examples |
|---|---|
| Drivetrain | Wheel radius, wheel track, gear ratio |
| Encoder | CPR/PPR definition, polarity, wheel ownership |
| Motor | A/B ownership, direction, safe-stop behavior |
| IMU | Configuration, calibration, interrupt behavior |
| Battery | ADC calibration and measured voltage conversion |
| CAN / CAN FD | Bit timing, IDs, payloads, timeout semantics |
| Safety | State machine, watchdog, fault recovery |
| Odometry | Wheel model, TF ownership, measured accuracy |
| Navigation | Real FollowPath and obstacle-avoidance evidence |
| Performance | Measured rates, latency, tracking error, CPU load |

## Documentation Style

Use:

- concise engineering language;
- tables for stable structured facts;
- trees for repository or hierarchy views;
- explicit units;
- exact pin, peripheral, topic, frame, register, and protocol names;
- real measured values with measurement context.

Keep unresolved assumptions marked as `PROPOSED` or `TBD`; state established facts directly.
