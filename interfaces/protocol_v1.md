# RobotProject Communication Protocol v1.0

Wire name: `RobotProject Protocol v1`
Authority: this file and `protocol_v1.yaml` are the shared Linux <-> STM32 contract.

## 1. Scope and responsibility

Protocol v1 transports motion authority, body-velocity commands, and deterministic STM32
telemetry. It does not move wheel control to Linux and does not define ROS serialization.

```text
geometry_msgs/msg/Twist
  -> Orange Pi bridge
  -> Protocol v1 CAN FD
  -> STM32 validation / freshness / differential drive / wheel control / safety

STM32 encoder + IMU + battery + state
  -> Protocol v1 CAN FD
  -> Orange Pi bridge
  -> ROS messages and diagnostics
```

The STM32 monotonic reception time is authoritative for motion-command and heartbeat safety.
No synchronized Linux/STM32 wall clock is required.

## 2. Transport

| Property | Value |
|---|---:|
| CAN identifier format | 11-bit Standard ID |
| Frame format | CAN FD, data frame, Bit Rate Switching enabled |
| Remote frames | Rejected |
| FDCAN kernel clock | 170 MHz, PCLK1 source, divider 1 |
| Nominal bitrate | 500,000 bit/s |
| Nominal prescaler | 17 |
| Nominal SEG1 | 15 tq |
| Nominal SEG2 | 4 tq |
| Nominal SJW | 4 tq |
| Nominal total | 20 tq/bit |
| Nominal sample point | 80.000% |
| Data bitrate | 2,000,000 bit/s |
| Data prescaler | 5 |
| Data SEG1 | 13 tq |
| Data SEG2 | 3 tq |
| Data SJW | 3 tq |
| Data total | 17 tq/bit |
| Data sample point | 82.3529% |
| Payload byte order | Little-endian |
| Application CRC | None; CAN FD link-layer CRC/integrity only |

Timing equations are:

```text
nominal = 170 MHz / (17 * (1 + 15 + 4)) = 500 kbit/s
data    = 170 MHz / (5  * (1 + 13 + 3)) = 2 Mbit/s
```

The robot uses a `TJA1042T(K)/3` transceiver. Its 5 Mbit/s CAN FD capability covers the configured
2 Mbit/s data rate. Logic-level wiring, mode pins, termination, and physical-link quality are
separate hardware integration properties.

Every Protocol v1 frame is FD+BRS. A Classic CAN frame, FD frame without BRS, Extended-ID
frame, or remote frame using a Protocol v1 ID is invalid.

### Integrity decision

Protocol v1 deliberately uses no application-level CRC. CAN FD already detects link corruption,
and this is a direct single-bus controller link rather than an untrusted multi-hop payload path.
Explicit length, version, reserved-bit, mode, session, sequence, and range validation provide the
application checks. Adding an application CRC later requires a protocol layout/version change.

## 3. Identifier allocation and arbitration

Lower identifiers win arbitration. Host control frames therefore precede periodic telemetry.

| CAN ID | Direction | Name | Length | Baseline rate / trigger |
|---:|---|---|---:|---|
| `0x080` | Orange Pi -> STM32 | `MOTION_AUTHORITY` | 16 B | 10 Hz and on state change |
| `0x081` | Orange Pi -> STM32 | `MOTION_COMMAND` | 16 B | 50 Hz while controlling; disabled command as needed |
| `0x082` | Orange Pi -> STM32 | `HOST_HEARTBEAT` | 16 B | 10 Hz |
| `0x180` | STM32 -> Orange Pi | `SYSTEM_STATUS` | 32 B | 10 Hz |
| `0x181` | STM32 -> Orange Pi | `WHEEL_STATE` | 64 B | 50 Hz |
| `0x182` | STM32 -> Orange Pi | `IMU_DATA` | 48 B | 100 Hz |
| `0x183` | STM32 -> Orange Pi | `BATTERY_STATE` | 16 B | 2 Hz |

Reserved allocation:

| Range | Reservation |
|---|---|
| `0x000..0x07F` | Future emergency/safety traffic |
| `0x080..0x0FF` | Host-to-STM32 control/supervision |
| `0x100..0x17F` | Future high-priority STM32 status |
| `0x180..0x1FF` | STM32 telemetry |
| `0x200..0x5FF` | Future robot functions |
| `0x600..0x6FF` | Future commissioning/diagnostics |
| `0x700..0x7FF` | Future discovery/version services |

Unassigned IDs are not Protocol v1 messages. The STM32 acceptance filter rejects them and they
cannot modify command or safety state.

## 4. Common encoding rules

- Integer fields are fixed width and little-endian.
- Signed integers use two's complement.
- Payloads are serialized byte-by-byte; compiler struct layout is never the wire format.
- All reserved bytes and bits must be transmitted as zero. A nonzero reserved command field is
  rejected.
- All frames begin with `protocol_major` at byte 0 and `protocol_minor` at byte 1.
- `protocol_major=1`, `protocol_minor=0` is the only accepted version in this baseline.
- Telemetry `sequence` counters are independent per CAN ID.
- `timestamp_ms` is the STM32 monotonic millisecond tick and wraps modulo 2^32 after
  49.7102696 days. It starts again from zero after MCU reset.

### Version policy

Major changes are required for incompatible layouts or behavior. A minor increment is allowed
only for a deliberately specified backward-compatible extension. The v1.0 implementations do
not guess future-minor compatibility: any version other than exactly 1.0 is rejected for control,
and the received status version lets Linux report the mismatch.

Changes requiring a version update include field meaning/layout, ID reassignment, unit/scale,
invalid representation, timing/freshness, or motion-safety semantic changes.

## 5. Command session, ordering, and safety

### 5.1 Session identifier

The Orange Pi bridge selects a nonzero 32-bit `session_id` each time the bridge process starts or
intentionally establishes a new control session. The same ID is used in heartbeat, authority, and
motion frames. Zero is invalid.

Only a valid `HOST_HEARTBEAT` can establish/change the active session. A new session clears all
previous authority and sequence history. If a previous CAN session owned motion, the session
change first forces a safe stop and communication fault.

### 5.2 Sequence comparison

Each host message type has an independent unsigned 16-bit sequence stream.

```text
delta = uint16(new_sequence - last_accepted_sequence)
delta == 0                  -> duplicate
0 < delta < 0x8000          -> newer, accept
delta >= 0x8000             -> old/out-of-order, reject
```

The first frame of a new session may use any sequence. Increment wraps naturally from `0xFFFF`
to `0x0000`; that wrap is newer, not a fault. Duplicate and out-of-order heartbeat, authority,
and command frames do not refresh local freshness timers and do not reapply motion.

### 5.3 Required authority transaction

After boot, reset, new session, timeout, bus-off, or RX overflow, Linux must perform:

```text
1. HOST_HEARTBEAT(session_id, bridge_ready=1)
2. MOTION_AUTHORITY(session_id, DISARMED)
3. MOTION_AUTHORITY(session_id, ARMED)
4. fresh MOTION_COMMAND(session_id, BODY_VELOCITY, v, omega)
```

`DISARMED` is a real handshake, not merely a heartbeat. It proves that the new controller has
observed a safe state before `ARMED` can be accepted. An `ARMED` frame received before this
handshake is rejected. `ARMED` grants authority but does not move the robot; a valid fresh body
command and every STM32 configuration/safety gate must also pass.

`MOTION_COMMAND/DISABLED` sets the shared command to safe zero and disables the current motor
execution. It does not create authority. If the CAN session otherwise remains healthy and armed,
a later fresh body command may resume through the normal safety path. A loss/reconnect event,
unlike an intentional disabled command, clears authority and requires the full transaction above.

### 5.4 Freshness

| Item | Timeout / behavior |
|---|---|
| Motion command | 250 ms from STM32 local reception of the last accepted new command |
| Host heartbeat | 500 ms from STM32 local reception of the last accepted new heartbeat |
| Motion authority | 500 ms from STM32 local reception; baseline repeat is 10 Hz |

Motion-command or armed-authority timeout while CAN owns motion sets target/output safe, forces STBY LOW/PWM zero,
sets `COMMAND_TIMEOUT`, enters `FAULT`, clears authority, and requires a new DISARM/ARM handshake.
Heartbeat timeout invalidates the session and performs the same safe convergence for CAN-owned
motion. Communication returning never restores the prior velocity or `ACTIVE` state.

### 5.5 Invalid input policy

| Input | STM32 behavior |
|---|---|
| Unknown ID | Hardware filter reject; no state change |
| Wrong frame kind or length | Reject; no freshness refresh |
| Unsupported version | Reject and latch version-mismatch status |
| Nonzero reserved field | Reject |
| Invalid/unsupported mode | Reject |
| Invalid sentinel or session zero/mismatch | Reject |
| Duplicate or out-of-order sequence | Ignore, count, no freshness refresh |
| Velocity outside local calibrated operational limits | Reject through drivetrain/safety guards; never clamp into motion |
| Critical fault | Immediate motor-safe state; authority is ineffective |
| Explicit DISARMED | Immediate motor-safe state; clears recoverable command/CAN faults only when CAN controller is no longer bus-off |

Malformed CAN traffic cannot arm the robot. A malformed frame only forces a system fault when CAN
already owns or claims motion; otherwise it remains rejected and observable without disabling the
independent UART commissioning path.

## 6. Host-to-STM32 frames

### 6.1 `HOST_HEARTBEAT` — `0x082`, 16 bytes, 10 Hz

| Offset | Width | Type | Field | Unit/scale | Valid values |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | 1 |
| 1 | 1 | `u8` | `protocol_minor` | none | 0 |
| 2 | 2 | `u16` | `sequence` | modulo 2^16 | any |
| 4 | 4 | `u32` | `session_id` | none | `1..0xFFFFFFFF` |
| 8 | 4 | `u32` | `host_uptime_ms` | ms, modulo 2^32 | diagnostic only |
| 12 | 2 | `u16` | `flags` | bit field | bit 0 `BRIDGE_READY`; bits 1..15 zero |
| 14 | 2 | `u16` | reserved | none | zero |

`BRIDGE_READY=0` explicitly makes CAN motion safe and prevents authority acceptance while keeping
communication observable. Heartbeat traffic alone never arms motion.

### 6.2 `MOTION_AUTHORITY` — `0x080`, 16 bytes, 10 Hz

| Offset | Width | Type | Field | Unit/scale | Valid values |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | 1 |
| 1 | 1 | `u8` | `protocol_minor` | none | 0 |
| 2 | 2 | `u16` | `sequence` | modulo 2^16 | any |
| 4 | 4 | `u32` | `session_id` | none | active nonzero session |
| 8 | 1 | `u8` | `authority_state` | enum | 0 `DISARMED`, 1 `ARMED` |
| 9 | 1 | `u8` | reserved | none | zero |
| 10 | 2 | `u16` | reserved | none | zero |
| 12 | 4 | `u32` | `host_uptime_ms` | ms, modulo 2^32 | diagnostic only |

`communication alive`, `valid command`, `motion authorized`, and `motor electrically enabled`
are four separate conditions. The state/flag telemetry exposes them separately.

### 6.3 `MOTION_COMMAND` — `0x081`, 16 bytes, 50 Hz while controlling

| Offset | Width | Type | Field | Unit/scale | Valid values / invalid |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | 1 |
| 1 | 1 | `u8` | `protocol_minor` | none | 0 |
| 2 | 2 | `u16` | `sequence` | modulo 2^16 | any |
| 4 | 4 | `u32` | `session_id` | none | active nonzero session |
| 8 | 1 | `u8` | `command_mode` | enum | 0 `DISABLED`, 1 `BODY_VELOCITY` |
| 9 | 1 | `u8` | reserved | none | zero |
| 10 | 2 | `i16` | `linear_velocity` | integer * 0.001 m/s | -32.767..+32.767 m/s; `-32768` invalid |
| 12 | 2 | `i16` | `angular_velocity` | integer * 0.001 rad/s | -32.767..+32.767 rad/s; `-32768` invalid |
| 14 | 2 | `u16` | reserved | none | zero |

For example, wire `250` means `+0.250 m/s`, and wire `-500` means `-0.500 rad/s`.
`DISABLED` requires both velocity fields to be exactly zero. The numeric ranges are serialization
ranges, not calibrated robot capability. STM32 body control remains unavailable until wheel
radius, track, encoder scale, gear ratio, wheel/motor mapping, controller gains, and operational
speed limits are valid.

## 7. STM32-to-host telemetry

### 7.1 `SYSTEM_STATUS` — `0x180`, 32 bytes, 10 Hz

| Offset | Width | Type | Field | Unit/scale | Invalid |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | none |
| 1 | 1 | `u8` | `protocol_minor` | none | none |
| 2 | 2 | `u16` | `sequence` | per-ID modulo counter | none |
| 4 | 4 | `u32` | `timestamp_ms` | STM32 monotonic ms | wraps |
| 8 | 1 | `u8` | `firmware_major` | none | none |
| 9 | 1 | `u8` | `firmware_minor` | none | none |
| 10 | 1 | `u8` | `firmware_patch` | none | none |
| 11 | 1 | `u8` | `system_state` | enum below | other values invalid |
| 12 | 2 | `u16` | `motion_flags` | bit field below | reserved bits none |
| 14 | 2 | `u16` | `communication_flags` | bit field below | bits 14..15 reserved zero |
| 16 | 4 | `u32` | `fault_flags` | bit field below | none |
| 20 | 4 | `u32` | `reset_reason` | firmware reset bit field | none |
| 24 | 4 | `u32` | `active_session_id` | none | 0 means no session |
| 28 | 2 | `u16` | `last_command_sequence` | none | meaningful when communication flag 11 is set |
| 30 | 2 | `u16` | `last_heartbeat_sequence` | none | meaningful when communication flag 12 is set |

System/safety states: 0 `BOOT`, 1 `INIT`, 2 `SAFE`, 3 `READY`, 4 `ACTIVE`, 5 `FAULT`.

`motion_flags`:

| Bit | Meaning |
|---:|---|
| 0 | FDCAN application initialized |
| 1 | controller currently Error Active (not a physical-link acceptance claim) |
| 2 | host heartbeat fresh |
| 3 | required DISARMED handshake observed for current session |
| 4 | CAN motion authority armed |
| 5 | internal command valid |
| 6 | internal command fresh |
| 7 | motor layer authorized |
| 8 | TB6612 STBY asserted / electrical motor domain enabled |
| 9 | critical RTOS task heartbeats healthy |
| 10 | supervisor currently permits IWDG refresh |
| 11 | IMU valid |
| 12 | Encoder 1 valid |
| 13 | Encoder 2 valid |
| 14 | battery measurement valid |
| 15 | complete drivetrain BODY_VELOCITY configuration valid |

`communication_flags` (bits 3..7 are latched since MCU reset):

| Bit | Meaning |
|---:|---|
| 0 | FDCAN warning level active |
| 1 | FDCAN Error Passive |
| 2 | FDCAN Bus Off |
| 3 | software RX ring overflow observed |
| 4 | hardware RX FIFO message loss observed |
| 5 | TX enqueue/RAM failure observed |
| 6 | protocol rejection observed |
| 7 | protocol-version mismatch observed |
| 8 | nonzero host session active |
| 9 | last accepted heartbeat has `BRIDGE_READY=1` |
| 10 | firmware transport configured for CAN FD+BRS |
| 11 | at least one motion-command sequence accepted in the current session |
| 12 | at least one heartbeat sequence accepted in the current session |
| 13 | last accepted motion-authority frame fresh within 500 ms |
| 14..15 | reserved zero |

### 7.2 `WHEEL_STATE` — `0x181`, 64 bytes, 50 Hz

All logical wheel fields use robot convention: forward is positive, reverse is negative.

| Offset | Width | Type | Field | Unit/scale | Invalid |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | none |
| 1 | 1 | `u8` | `protocol_minor` | none | none |
| 2 | 2 | `u16` | `sequence` | modulo counter | none |
| 4 | 4 | `u32` | `timestamp_ms` | STM32 monotonic ms | wraps |
| 8 | 8 | `i64` | `left_position_counts` | normalized cumulative decoded counts since reset | `INT64_MIN` |
| 16 | 8 | `i64` | `right_position_counts` | normalized cumulative decoded counts since reset | `INT64_MIN` |
| 24 | 4 | `i32` | `left_measured_cps` | normalized counts/s | `INT32_MIN` |
| 28 | 4 | `i32` | `right_measured_cps` | normalized counts/s | `INT32_MIN` |
| 32 | 4 | `i32` | `left_target_cps` | normalized counts/s | `INT32_MIN` |
| 36 | 4 | `i32` | `right_target_cps` | normalized counts/s | `INT32_MIN` |
| 40 | 2 | `i16` | `left_controller_output` | integer * 0.0001 effort | `INT16_MIN` |
| 42 | 2 | `i16` | `right_controller_output` | integer * 0.0001 effort | `INT16_MIN` |
| 44 | 2 | `u16` | `encoder_1_raw_counter` | raw TIM count | none |
| 46 | 2 | `u16` | `encoder_2_raw_counter` | raw TIM count | none |
| 48 | 4 | `i32` | `encoder_1_raw_cps` | hardware-sign counts/s | `INT32_MIN` |
| 52 | 4 | `i32` | `encoder_2_raw_cps` | hardware-sign counts/s | `INT32_MIN` |
| 56 | 2 | `u16` | `flags` | bit field below | bits 14..15 reserved |
| 58 | 2 | `u16` | `encoder_1_age_ms` | ms, saturated | `0xFFFF` means >=65535 ms |
| 60 | 2 | `u16` | `encoder_2_age_ms` | ms, saturated | `0xFFFF` means >=65535 ms |
| 62 | 2 | `u16` | `sample_period_ms` | ms | baseline 10 |

Wheel flags: bit 0 logical mapping valid; 1 Encoder 1 valid; 2 Encoder 2 valid; 3 left
controller configured; 4 right controller configured; 5 both controllers enabled; 6 left
saturated; 7 right saturated; 8 motor layer authorized; 9 STBY enabled; 10 left target valid;
11 right target valid; 12 body-command drivetrain ready; 13 configured left/right encoder sign
calibration present; 14..15 reserved.

Physical testing established `left wheel forward -> raw CPS < 0` and `right wheel forward ->
raw CPS > 0`. The drivetrain abstraction uses left sign `-1` and right sign `+1`.
Encoder 1/2 wheel ownership remains separately configurable. Until that ownership is set,
logical position/rate fields are invalid while the raw Encoder 1/2 fields remain available.

The Orange Pi may integrate the normalized cumulative count delta for odometry only when wheel
flag 0 is set and both encoder-valid flags are set. Conversion to metres requires the separately
measured decoded-counts-per-wheel-revolution and effective wheel radius; Protocol v1 does not
invent them.

### 7.3 `IMU_DATA` — `0x182`, 48 bytes, 100 Hz

| Offset | Width | Type | Field | Unit/scale | Invalid |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | none |
| 1 | 1 | `u8` | `protocol_minor` | none | none |
| 2 | 2 | `u16` | `sequence` | modulo counter | none |
| 4 | 4 | `u32` | `sample_timestamp_ms` | STM32 monotonic ms | wraps |
| 8 | 2 | `u16` | `flags` | bit field below | bits 7..15 reserved |
| 10 | 1 | `u8` | `who_am_i` | raw register | 0 when unavailable |
| 11 | 1 | `u8` | `imu_state` | 0 not initialized, 1 initializing, 2 ready, 3 fault | other invalid |
| 12 | 4 | `i32` | `accel_x` | integer * 0.0001 m/s^2 | `INT32_MIN` |
| 16 | 4 | `i32` | `accel_y` | integer * 0.0001 m/s^2 | `INT32_MIN` |
| 20 | 4 | `i32` | `accel_z` | integer * 0.0001 m/s^2 | `INT32_MIN` |
| 24 | 4 | `i32` | `gyro_x` | integer * 0.00001 rad/s | `INT32_MIN` |
| 28 | 4 | `i32` | `gyro_y` | integer * 0.00001 rad/s | `INT32_MIN` |
| 32 | 4 | `i32` | `gyro_z` | integer * 0.00001 rad/s | `INT32_MIN` |
| 36 | 2 | `u16` | `sample_age_ms` | ms, saturated | `0xFFFF` means >=65535 ms |
| 38 | 2 | `u16` | reserved | none | zero |
| 40 | 4 | `u32` | `int1_count` | events since reset | wraps |
| 44 | 4 | `u32` | `int2_count` | events since reset | wraps |

IMU flags: bit 0 sensor valid; 1 data-ready state; 2 WHO_AM_I is `0x47`; 3 INT1 observed;
4 INT2 observed; 5 acceleration valid; 6 gyroscope valid; 7..15 reserved. No orientation
estimate is transmitted.

### 7.4 `BATTERY_STATE` — `0x183`, 16 bytes, 2 Hz

| Offset | Width | Type | Field | Unit/scale | Invalid |
|---:|---:|---|---|---|---|
| 0 | 1 | `u8` | `protocol_major` | none | none |
| 1 | 1 | `u8` | `protocol_minor` | none | none |
| 2 | 2 | `u16` | `sequence` | modulo counter | none |
| 4 | 4 | `u32` | `sample_timestamp_ms` | STM32 monotonic ms | wraps |
| 8 | 2 | `u16` | `battery_voltage_mv` | 1 mV | `0xFFFF` |
| 10 | 2 | `u16` | `raw_adc` | ADC counts | none |
| 12 | 2 | `u16` | `flags` | bit field | bits 3..15 reserved |
| 14 | 2 | `u16` | `sample_age_ms` | ms, saturated | `0xFFFF` means >=65535 ms |

Battery flags: bit 0 measurement/encoded voltage valid; bit 1 ADC calibration completed; bit 2
divider ratio physically calibrated. A valid estimate with bit 2 clear is observable commissioning
data, not an accepted voltage calibration. Protocol v1 defines no undervoltage threshold.

## 8. Fault wire representation

`SYSTEM_STATUS.fault_flags` is the firmware's centralized 32-bit fault word.

| Bit | Name | Category | Motor effect | Clear condition |
|---:|---|---|---|---|
| 0 | `IMU_INITIALIZATION` | degraded sensor | no independent full shutdown | successful IMU initialization/retry |
| 1 | `COMMAND_TIMEOUT` | critical motion | safe state / FAULT | explicit DISARMED recovery handshake or UART recover command |
| 2 | `INVALID_MOTOR_COMMAND` | critical motion | safe state / FAULT | explicit recover action while inputs are safe |
| 3 | `SUPERVISOR` | critical runtime | safe state / FAULT; watchdog feed inhibited | reset after root cause correction |
| 4 | `ENCODER_VALIDITY` | sensor/control | wheel control cannot continue; paired abnormal control is critical | valid encoder reinitialization/sample where supported |
| 5 | `BATTERY_MEASUREMENT` | degraded sensor | no invented battery shutdown threshold | successful valid measurement |
| 6 | `CONTROL_SATURATION` | degraded control | reported; configured controller still clamps output | explicit recover/reset after command is safe |
| 7 | `CONTROL_ABNORMAL` | critical control | safe state / FAULT | reset after configuration/root cause correction |
| 8 | `INTERNAL_CONFIGURATION` | critical configuration | safe state / FAULT | reset after valid firmware/configuration |
| 9 | `RTOS_STACK_OVERFLOW` | critical runtime | immediate safe state | reset after firmware correction |
| 10 | `RTOS_MALLOC_FAILURE` | critical runtime | immediate safe state | reset after firmware correction |
| 11 | `FDCAN_COMMUNICATION` | critical when CAN-owned motion is affected | safe state / FAULT | controller recovered plus explicit DISARMED handshake |
| 12..31 | reserved | none | none | transmit zero |

The wire fault word reports firmware semantics directly; ROS must not rename bits into a second,
incompatible safety model.

## 9. CAN controller errors and reconnection

- Error Warning and Error Passive are reported. They do not by themselves extend freshness or
  authorize motion.
- Bus Off immediately revokes CAN motion authority. When CAN owned motion, the STM32 sets the
  communication fault and enforces motor safe output.
- Hardware FIFO loss or bounded software-ring overflow is latched and revokes CAN-owned motion.
- TX failures are counted/latching diagnostics; they never fabricate successful delivery.
- The firmware initiates bounded bus-off recovery attempts at 1 s intervals. Hardware recovery
  does not clear the safety handshake.
- After recovery, Linux sends a new valid heartbeat, explicit DISARMED, then ARMED, then a fresh
  body command. The previous target is never reused.

Physical CAN integration requires observed bidirectional frames, error state, termination, and
bitrate behavior on the Orange Pi/STM32 hardware.

## 10. Baseline bandwidth

At the configured rates there are 232 application frames/s: 70 host-to-STM32 and 162
STM32-to-host. Payload alone is 75,776 bit/s. A conservative engineering estimate including
arbitration/control/CRC/inter-frame overhead and 25% bit-stuffing allowance is approximately 8%
and bounded below 10% bus time at 500 kbit/s arbitration and 2 Mbit/s data phase. This is a design
estimate, not a measured bus load, and leaves at least 90% margin for retransmission, diagnostics,
and future messages.

## 11. ROS / ORANGE PI IMPLEMENTATION CONTRACT

The ROS Codex can implement the bridge using only this section plus the frame definitions above.

### Linux CAN assumptions

- Inspect the actual Orange Pi CAN controller and transceiver before configuration.
- Use SocketCAN CAN FD mode with 11-bit IDs and payload lengths up to 64 bytes.
- The RobotProject deployment uses Orange Pi CAN3 / SocketCAN `can3`; there is no CAN2
  production fallback.
- Configure nominal 500000 bit/s and data 2000000 bit/s with FD enabled.
- On the Linux `drv_mttcan` controller, explicitly set nominal sample point 0.800 and data
  sample point 0.825. The STM32 data sample point is 0.823529; this is an endpoint
  timing configuration detail and does not change Protocol 1.0 wire semantics.
- Enable bus-error reporting where supported and observe error counters/state.
- Confirm physical integration by observing real bidirectional FD+BRS frames.

### Frames sent by Linux

Linux transmits only `0x080`, `0x081`, and `0x082` for Protocol v1. It generates one nonzero
session ID per bridge process/control session and independent `u16` sequences per ID.

- Publish `HOST_HEARTBEAT` at 10 Hz whether armed or disarmed.
- Publish the current `MOTION_AUTHORITY` at 10 Hz and immediately on arm/disarm changes.
- Publish `MOTION_COMMAND/BODY_VELOCITY` at 50 Hz while controlling.
- Convert `Twist.linear.x` using `round(linear.x / 0.001 m/s)` and
  `Twist.angular.z` using `round(angular.z / 0.001 rad/s)` after checking finite and representable
  range. Never encode `-32768`.
- Other Twist components are unsupported by this differential-drive contract and should be
  rejected or ignored with explicit diagnostics; they are not placed on the wire.
- Do not clamp an invalid/unrepresentable command into a different motion. Send DISABLED/disarm
  and report the rejection.
- Complete heartbeat -> DISARMED -> ARMED before sending motion after every start/reconnect.
- A bridge shutdown must send DISABLED and DISARMED when the bus is available, but STM32 local
  timeouts remain the authoritative protection if those frames are lost.

The bridge should treat the 250 ms command timeout as a hard upper bound. The 50 Hz publication
period gives approximately 12 missed command periods before expiry. It must not intentionally
pause command publication near the timeout boundary.

### Frames received by Linux

Linux receives `0x180..0x183`, validates exact version, FD+BRS envelope, ID, and length, then uses
explicit little-endian decoding.

- `SYSTEM_STATUS`: ROS diagnostics, firmware/protocol mismatch, safety/fault/motion authority,
  command acknowledgement sequences, and session acknowledgement.
- `WHEEL_STATE`: normalized cumulative count delta and measured CPS for odometry/control
  diagnostics. Do not generate metric odometry until logical mapping and measured drivetrain
  scale are valid.
- `IMU_DATA`: convert acceleration and angular velocity directly to SI for
  `sensor_msgs/msg/Imu`; orientation is unavailable and must not be fabricated.
- `BATTERY_STATE`: voltage and calibration validity for diagnostics; do not invent safety
  thresholds.

ROS owns `nav_msgs/msg/Odometry`, ROS timestamps, covariance policy, and the eventual single
`odom -> base_link` TF authority. STM32 provides monotonic measurement timestamps and physical
measurements only. The bridge must handle STM32 timestamp wrap/reset and may correlate monotonic
time to ROS time without feeding that correlation back into command-safety timeout logic.

### Bridge health requirements

- Reject telemetry with unexpected version/length/reserved bits.
- Detect missing system/wheel/IMU streams from their baseline rates and report diagnostics.
- Display every defined fault bit and the distinction among heartbeat fresh, authority armed,
  internal command valid/fresh, motor authorized, and STBY enabled.
- Compare acknowledged session/last sequences in `SYSTEM_STATUS` with transmitted values.
- On Linux CAN error, bridge restart, status protocol mismatch, or stale telemetry: publish safe
  ROS diagnostics, stop body commands, begin a new session, and repeat DISARMED/ARMED only after
  the operator/autonomy layer explicitly authorizes motion.

## 12. Primary references

- STMicroelectronics, [RM0440 STM32G4 reference manual](https://www.st.com/resource/en/reference_manual/dm00355726.pdf), FDCAN bit timing and controller behavior.
- NXP, [TJA1042T(K)/3 high-speed CAN transceiver](https://www.nxp.com/products/TJA1042), CAN FD fast-phase capability.
