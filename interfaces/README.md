# ros2-stm32-autonomous-robot Shared Interfaces

This directory is the authoritative Linux <-> STM32 communication-contract layer.

## Protocol contract

| Contract | Human specification | Machine-readable definition |
|---|---|---|
| RobotProject Communication Protocol v1.0 | [protocol_v1.md](protocol_v1.md) | [protocol_v1.yaml](protocol_v1.yaml) |

Golden serialization examples are in [examples/protocol_v1_examples.md](examples/protocol_v1_examples.md).

Protocol v1 defines CAN FD transport timing, identifiers, payload layouts, units, scaling,
ordering, command freshness, motion authority, reconnection, and status/fault semantics. The
firmware and ROS domains must implement against these files; neither side may infer wire fields
from the peer implementation directory.

## Ownership

| Domain | Responsibility |
|---|---|
| STM32 firmware | Decode and validate commands, enforce local freshness and safety, run differential-drive and wheel control, serialize telemetry |
| Orange Pi / ROS | Convert `geometry_msgs/msg/Twist` to Protocol v1, supervise the session, decode telemetry, publish ROS state and diagnostics |
| `interfaces/` | Source of truth for every cross-domain binary and behavioral contract |

## Robot deployment

The current hardware revision uses Orange Pi CAN3 (`822d0000.mttcan`, `mttcan-id=3`) as SocketCAN
`can3`, with GPIO2_17/CAN_TX3 and GPIO2_18/CAN_RX3. Linux production timing is explicitly configured
as 500 kbit/s at 80.0% and 2 Mbit/s at 82.5%; BRS is set per Protocol 1.0 frame. This host setting
interoperates with the STM32 500 kbit/s / 2 Mbit/s timing without changing any wire field or
protocol semantic.

Real `0x080`, `0x082`, and `0x180..0x183` traffic was observed with Error Active, zero TX/RX errors,
and zero bus-off events. The robot remained DISARMED. CAN2 is historical and is not a production
fallback.
