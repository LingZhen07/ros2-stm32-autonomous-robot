# RobotProject Documentation

[中文](README.zh-CN.md)

`docs/` is the persistent engineering knowledge base for RobotProject.

## Document Map

| Document | Language | Scope |
|---|---|---|
| [Hardware Baseline and STM32 Pin Map](hardware.md) | English | Hardware models, pin map, peripheral ownership, geometry |
| [硬件基线与 STM32 Pin Map](hardware.zh-CN.md) | 中文 | 硬件型号、Pin Map、外设归属、几何信息 |
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
