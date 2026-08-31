# Protocol v1 Golden Frames

Status: **FROZEN**. Hex bytes are shown in wire order. Every frame is a Standard-ID CAN FD data
frame with BRS enabled. These examples are normative serializer/decoder fixtures, not simulated
hardware evidence.

## Host heartbeat (`0x082`, 16 bytes)

Semantic values:

```text
version        = 1.0
sequence       = 0x1234
session_id     = 0xA1B2C3D4
host_uptime_ms = 123456
BRIDGE_READY   = 1
```

```text
01 00 34 12 D4 C3 B2 A1 40 E2 01 00 01 00 00 00
```

## Motion authority (`0x080`, 16 bytes)

Required safe handshake example:

```text
version         = 1.0
sequence        = 0x1235
session_id      = 0xA1B2C3D4
authority_state = DISARMED (0)
host_uptime_ms  = 123556
```

```text
01 00 35 12 D4 C3 B2 A1 00 00 00 00 A4 E2 01 00
```

An `ARMED` frame with otherwise identical data changes byte 8 from `00` to `01` and uses a new
sequence value.

## Body-velocity command (`0x081`, 16 bytes)

Semantic values:

```text
version              = 1.0
sequence             = 0x1236
session_id           = 0xA1B2C3D4
mode                 = BODY_VELOCITY (1)
linear_velocity      = +0.250 m/s -> wire +250
angular_velocity     = -0.500 rad/s -> wire -500
```

```text
01 00 36 12 D4 C3 B2 A1 01 00 FA 00 0C FE 00 00
```

### Sequence wrap and zero velocity

The following two zero-velocity BODY_VELOCITY frames are consecutive. `0x0000` is newer than
`0xFFFF` under the frozen modulo comparison.

```text
sequence 0xFFFF:
01 00 FF FF D4 C3 B2 A1 01 00 00 00 00 00 00 00

sequence 0x0000:
01 00 00 00 D4 C3 B2 A1 01 00 00 00 00 00 00 00
```

A `DISABLED` command uses mode byte `00` and requires both velocity fields to remain zero.

## System status (`0x180`, 32 bytes)

Semantic values:

```text
version                     = 1.0
sequence                    = 0x0102
timestamp_ms                = 1000
firmware                    = 0.4.0
system_state                = ACTIVE (4)
motion_flags                = 0xFFFF (all currently defined conditions true)
communication_flags         = 0x3F00 (session, bridge ready, FD+BRS, command/heartbeat sequences seen, authority fresh; no error latch)
fault_flags                 = 0
reset_reason                = PIN (bit 0)
active_session_id           = 0xA1B2C3D4
last_command_sequence       = 0x1236
last_heartbeat_sequence     = 0x1234
```

```text
01 00 02 01 E8 03 00 00 00 04 00 04 FF FF 00 3F
00 00 00 00 01 00 00 00 D4 C3 B2 A1 36 12 34 12
```

Fault example: setting `COMMAND_TIMEOUT` and `FDCAN_COMMUNICATION` produces
`fault_flags=0x00000802`, encoded at offsets 16..19 as `02 08 00 00`.

## Wheel state (`0x181`, 64 bytes)

This example demonstrates normalized forward-positive rates while Encoder 1 raw CPS is negative.

```text
sequence                 = 0x2001
timestamp_ms             = 1000
left_position_counts     = +12345
right_position_counts    = +12345
left_measured_cps        = +1200 logical
right_measured_cps       = +1180 logical
left/right_target_cps    = +1200
left_controller_output   = 0.2500 -> wire 2500
right_controller_output  = 0.2400 -> wire 2400
encoder_1_raw_counter    = 54321
encoder_2_raw_counter    = 12345
encoder_1_raw_cps        = -1200
encoder_2_raw_cps        = +1180
flags                    = 0x3F3F
encoder ages             = 5 ms, 5 ms
sample_period_ms         = 10
```

```text
01 00 01 20 E8 03 00 00 39 30 00 00 00 00 00 00
39 30 00 00 00 00 00 00 B0 04 00 00 9C 04 00 00
B0 04 00 00 B0 04 00 00 C4 09 60 09 31 D4 39 30
50 FB FF FF 9C 04 00 00 3F 3F 05 00 05 00 0A 00
```

When Encoder 1/2 wheel ownership is unset, both logical cumulative fields encode `INT64_MIN`
(`00 00 00 00 00 00 00 80`), logical measured fields encode `INT32_MIN`
(`00 00 00 80`), and flag 0 is clear. Raw fields remain meaningful diagnostics.

## IMU data (`0x182`, 48 bytes)

Semantic values:

```text
sequence            = 0x3001
sample_timestamp_ms = 1000
flags               = 0x006F (valid, ready, WHO_AM_I, INT1, accel, gyro)
WHO_AM_I            = 0x47
state               = READY (2)
acceleration        = [0, 0, 9.8067] m/s^2
angular_velocity    = [0.01000, -0.02000, 0] rad/s
sample_age_ms       = 5
INT1 / INT2 counts  = 42 / 0
```

```text
01 00 01 30 E8 03 00 00 6F 00 47 02 00 00 00 00
00 00 00 00 13 7F 01 00 E8 03 00 00 30 F8 FF FF
00 00 00 00 05 00 00 00 2A 00 00 00 00 00 00 00
```

## Battery state (`0x183`, 16 bytes)

Semantic values:

```text
sequence            = 0x4001
sample_timestamp_ms = 900
battery_voltage     = 12.345 V -> 12345 mV
raw_adc             = 2500
flags               = 0x0007 (valid, ADC calibrated, divider calibrated)
sample_age_ms       = 100
```

```text
01 00 01 40 84 03 00 00 39 30 C4 09 07 00 64 00
```
