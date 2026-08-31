# ros2-stm32-autonomous-robot Documentation

[中文](README.zh-CN.md)

`docs/` is the persistent engineering knowledge base for `ros2-stm32-autonomous-robot`.

## Document Map

| Document | Language | Scope |
|---|---|---|
| [Hardware Baseline and STM32 Pin Map](hardware.md) | English | Hardware models, pin map, peripheral ownership, geometry |
| [硬件基线与 STM32 Pin Map](hardware.zh-CN.md) | 中文 | 硬件型号、Pin Map、外设归属、几何信息 |
| [Orange Pi / STM32 ROS Bridge](ros_bridge.md) | English | SocketCAN, Protocol 1.0 bridge, ROS interfaces, and M5 demo |
| [Orange Pi / STM32 ROS 桥接](ros_bridge.zh-CN.md) | 中文 | SocketCAN、Protocol 1.0 桥接、ROS 接口与 M5 演示 |
| [STM32 M1-M5 Production Firmware](firmware.md) | English | Accepted real-time foundation, Protocol v1/M5 firmware integration, safety and acceptance boundary |
| [STM32 M1-M5 生产固件](firmware.zh-CN.md) | 中文 | 已验收实时基础、Protocol v1/M5 固件集成、安全与验收边界 |
| [STM32 USART2 Production CLI](uart_cli.md) | English | Complete permanent command, output-rate, commissioning, and safety reference |
| [STM32 USART2 生产版 CLI](uart_cli.zh-CN.md) | 中文 | 永久命令、输出频率、标定与安全完整手册 |
| `README.md` | English | Documentation index and maintenance rules |
| `README.zh-CN.md` | 中文 | 文档索引与维护规则 |

## Status Labels

| Label | Use |
|---|---|
| `FROZEN` | Accepted configuration retained across later stages |
| `VERIFIED` | Supported by real system evidence |
| `IN PROGRESS` | Active integration work |
| `PROPOSED` | Candidate awaiting validation |
| `TBD` | Intentionally unresolved |

## Knowledge Base Update Triggers

Update the relevant English and Chinese documents when any of these become known:

| Category | Examples |
|---|---|
| Drivetrain | Wheel radius, wheel track, gear ratio |
| Encoder | CPR/PPR definition, polarity, wheel ownership |
| Motor | A/B ownership, direction, safe-stop behavior |
| IMU | Configuration, calibration, interrupt behavior |
| Battery | ADC calibration and validated voltage conversion |
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
- real measured values with acceptance context.

Keep assumptions marked as `PROPOSED` or `TBD` until real evidence supports promotion to `VERIFIED` or `FROZEN`.
