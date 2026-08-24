# Interfaces

This directory is the authoritative **Linux ↔ STM32 communication-contract layer** for RobotProject.

## Scope

| Contract area | Content |
|---|---|
| Transport | CAN / CAN FD, UART fallback |
| Command | Body velocity, enable state, command mode |
| Freshness | Sequence number, heartbeat, timeout semantics |
| Telemetry | Encoder, wheel speed, IMU, battery, state, faults |
| Encoding | Byte order, scaling, signedness, units |
| Versioning | Protocol version and compatibility rules |
| Integrity | CRC policy when required |

## Current Status

| Item | Status |
|---|---|
| Production transport direction | CAN FD |
| STM32 controller | FDCAN1 |
| Nominal CAN bring-up bitrate | 500 kbit/s |
| CAN IDs | TBD |
| CAN FD data-phase bitrate | TBD |
| Payload layout | TBD |
| Scaling | TBD |
| Heartbeat format | TBD |
| Timeout semantics | TBD |
| CRC policy | TBD |
| Protocol version | TBD |

## Ownership Model

| Domain | Implementation responsibility |
|---|---|
| Firmware | Parse commands, validate freshness, publish telemetry |
| ROS / Linux | Encode commands, decode telemetry, expose ROS interfaces |
| `interfaces/` | Shared source of truth for semantics and binary contracts |

A protocol item moves from `TBD` to a frozen contract after deliberate cross-domain design and acceptance.

Once frozen, both implementation domains follow the same contract version.

## Recommended Future Layout

```text
interfaces/
├── README.md
├── protocol_v1.md
├── can_ids.md
├── messages/
│   ├── command.md
│   ├── telemetry.md
│   └── fault_status.md
└── examples/
    └── frame_examples.md
```

Create these files when the corresponding protocol decisions are ready for implementation.
