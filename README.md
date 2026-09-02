# ros2-stm32-autonomous-robot

[简体中文](README.zh-CN.md)

A heterogeneous autonomous mobile robot built around an Orange Pi AI Pro running ROS 2 Humble and
an STM32G474 real-time controller. The system combines LiDAR/RGB-D perception, SLAM and Nav2 with a
CAN FD drivetrain bridge, closed-loop wheel control, and layered motion safety.

## Stack

| Domain | Hardware / software | Role |
|---|---|---|
| High computing | Orange Pi AI Pro 8GB, Ubuntu 22.04, ROS 2 Humble | Perception, SLAM, Nav2, command supervision, logging |
| AI acceleration | Ascend 310B4 | YOLOv8n inference |
| Real-time control | STM32G474RET6, FreeRTOS, firmware `0.5.4` | Command validation, safety, wheel control, sensors, telemetry |
| Environment sensing | RPLIDAR A1, Astra RGB-D | Real `/scan`, RGB and depth input |
| Drivetrain | TB6612, dual DC motors, quadrature encoders | Differential-drive actuation and feedback |
| Transport | TJA1042T(K)/3 transceiver, SocketCAN `can3` | Protocol 1.0 CAN FD link |

## Architecture

```text
RPLIDAR / RGB-D ──> ROS 2 perception, SLAM and Nav2 ──> geometry_msgs/Twist
                                                             │
                                                             v
                               Orange Pi motion supervisor + CAN FD bridge
                                                             │
                                               Protocol 1.0 on can3
                                                             │
                                                             v
                       STM32 safety + differential drive + closed-loop wheels
                                                             │
                                                             v
                                      TB6612 + motors + encoder feedback
```

Production CAN FD is `can3`: nominal 500 kbit/s at sample point `0.800`, data 2 Mbit/s at data
sample point `0.825`. The current robot reports `BODY_COMMAND_READY = TRUE` when its configured
drivetrain and control gates are available.

## Demonstrated capabilities

- Orange Pi AI Pro ROS 2 Humble upper-computing stack with RPLIDAR A1 real `/scan` input.
- Real bidirectional CAN3 physical link and Protocol 1.0 Motion Authority handshake.
- STM32G474 firmware `0.5.4` closed-loop left/right wheel control and deterministic safe stop.
- Closed-loop straight motion at the current `0.30 m/s` commissioning limit.
- A 30° frontal LiDAR sector with a `0.60 m` stop threshold.
- Real-hardware obstacle stop with zero velocity, Motion Authority withdrawal, STM32 motor-safe
  stop, and latched STOPPED behavior.

`0.30 m/s` is the demonstrated commissioning limit under the present electrical and power
constraints; the robot's physical maximum has not been measured.

## Motion safety and obstacle-stop demonstration

Motion is disabled by default. Velocity commands do not grant authority to move; Motion Authority
is a separate gate.

```text
explicit START
→ Motion Authority granted
→ fresh motion commands
→ motion allowed
```

Stopping uses layered withdrawal of motion authority:

```text
obstacle / stale command / communication loss / fault / explicit STOP
→ zero motion command where available
→ Motion Authority withdrawn
→ STM32 motor-safe stop
```

Loss or withdrawal of Motion Authority stops motion. Command freshness, heartbeat/watchdog, and
fault handling remain independent lower-level protections. An obstacle stop withdraws authority and
enters latched STOPPED; clearing the obstacle does not restart motion. A new explicit START is
required before motion can resume.

Observed during the successful demo:

| Event interval | Observed time |
|---|---:|
| Obstacle detection → zero velocity command | approximately `0.075 ms` |
| Obstacle detection → Motion Authority withdrawal | approximately `1.276 ms` |
| Obstacle detection → STM32 stop confirmation | approximately `30.8 ms` |

These are measurements from one hardware demonstration, not guaranteed worst-case safety limits.

## Documentation and setup

| Resource | Purpose |
|---|---|
| [Documentation index](docs/README.md) | Bilingual engineering knowledge base |
| [ROS bridge and obstacle-stop demo](docs/ros_bridge.md) | Build, CAN3 startup, ROS interfaces, demo procedure and evidence |
| [STM32 production firmware](docs/firmware.md) | Firmware build, architecture, safety and commissioning scope |
| [Hardware baseline](docs/hardware.md) | Hardware definitions and STM32 pin map |
| [Protocol 1.0](interfaces/protocol_v1.md) | Linux ↔ STM32 wire contract |

Clone the canonical repository:

```bash
git clone git@github.com:LingZhen07/ros2-stm32-autonomous-robot.git
cd ros2-stm32-autonomous-robot
```

Internal ROS packages, firmware modules, executable names, and protocol identifiers retain their
established names; the GitHub repository rename does not change runtime contracts.

## License

Copyright (c) 2026 Ling Zhen. This project is licensed under the [MIT License](LICENSE).
