# STM32 USART2 Engineering Console

[中文](uart_cli.zh-CN.md)

## Purpose and safety boundary

USART2 is the permanent low-rate commissioning and diagnostic console in the production
firmware. It is not a test application. Motor commands enter the shared command model, Safety
Supervisor, and production motor-output layer; the console cannot bypass safety or take control
while CAN motion authority is armed.

The console never arms at boot, never restores an armed state after reset, and is quiet after its
single boot message. Protocol 1.0 is unchanged.

## Serial settings

| Item | Setting |
|---|---|
| Pins | PA2 USART2_TX, PA3 USART2_RX |
| Format | 115200 bit/s, 8 data bits, no parity, 1 stop bit (`115200 8N1`) |
| Flow control | None |
| Commands | Printable ASCII |
| Accepted line ending | CR, LF, or CRLF |
| Response line ending | CRLF |
| Maximum command line | 63 characters, excluding the line ending |
| Prompt | `> ` |

Expected boot block for firmware 0.5.4:

```text
RobotProject STM32 boot
FW 0.5.4 / Protocol 1.0
Quiet console; type 'help'

>
```

No periodic output follows unless the operator explicitly enables `watch`.

## Command summary

| Command | Purpose | Motion effect |
|---|---|---|
| `help` | List the console commands | None |
| `status` | Compact system and safety summary | None |
| `encoder` | Logical right/left encoder totals and rates | None |
| `imu` | Current IMU identity, validity, acceleration, and gyro | None |
| `battery` | Current ADC and battery estimate | None |
| `can` | FDCAN state, counters, session, and freshness | None |
| `fault` | Active centralized fault word and names | None |
| `clear` | Clear only recovered, recoverable faults | Disarms and forces safe output first |
| `arm` | Request local UART commissioning authority | Does not move a motor by itself |
| `disarm` | Invalidate command and force safe motor output | Immediate safe output |
| `stop` | Alias of `disarm` | Immediate safe output |
| `motor <a> <b> <duration_ms>` | Timed direct Motor A/B effort | May cause real motion |
| `watch encoder` | Show `encoder` at 1 Hz | None |
| `watch imu` | Show `imu` at 1 Hz | None |
| `watch can` | Show `can` at 1 Hz | None |
| `watch off` | Stop periodic console output | None |

The former `version`, status subcommands, CAN subcommands, encoder subcommands, wheel/body target,
controller/PID, and drivetrain-configuration commands are not part of this lightweight console.
Production control modules and Protocol 1.0 remain intact; only the local human command surface
was reduced.

## Response and scheduling policy

Each command produces one bounded response block beginning with the submitted line and ending in
the prompt. For example:

```text
> fault

0x00000000 NONE

>
```

Responses and watch reports are queued as complete blocks and transmitted with UART interrupts.
They do not interleave individual lines. Queue pressure drops a whole diagnostic block instead of
blocking Safety, MotorControl, Supervisor, or FDCAN execution. A command response takes priority
over a watch report in the same service cycle.

Watch is always off after reset and has a fixed 1 Hz rate. There is no high-rate UART mode.

## Read-only commands

### `help`

- Syntax: `help`
- Arguments/units: none.
- Valid states: all states after console initialization.
- Effect: lists exactly the supported command surface and motor bounds.
- Safety: read-only.
- Example: `help`
- Rejection: any argument returns a syntax error.

### `status`

- Syntax: `status`
- Arguments/units: none.
- Valid states: all runtime states.
- Effect: reports firmware, system state, fault word, motor/STBY state, raw encoder-acquisition
  health, IMU health, FDCAN state, current authority owner, and BODY_VELOCITY readiness.
- Safety: read-only. `Body Cmd : READY` means the verified drivetrain configuration, operating
  envelope, and both wheel controllers are configured; runtime authority and health gates still
  apply before motion.
- Example response:

```text
FW        : 0.5.4
State     : READY
Fault     : NONE
Motor     : DISABLED
STBY      : LOW
Encoder   : OK
IMU       : OK
CAN       : ERROR_ACTIVE
Authority : NONE
Body Cmd  : READY
```

- Rejection: any argument.

### `encoder`

- Syntax: `encoder`
- Arguments/units: none; totals are decoded counts and rates are decoded counts/s.
- Valid states: all runtime states.
- Effect: shows logical, forward-positive right/left cumulative totals, filtered rates, validity,
  and sample age.
- Safety: read-only. The low-level raw counters are not modified.
- Mapping: Encoder 1 is the right wheel with raw forward sign `+1`; Encoder 2 is the left wheel
  with raw forward sign `-1`. Therefore right logical values equal Encoder 1 raw values and left
  logical values are the negation of Encoder 2 raw values.
- Scale: right `1059.5` and left `1060.8` decoded counts/wheel revolution remain independent
  commissioning values.
- 64-bit formatting: cumulative totals use a dedicated signed decimal conversion, including
  negative values and `INT64_MIN`; the console does not depend on reduced-library `%lld` support.
- Example response:

```text
Right : total=10595 cps=0.0 valid=yes age=2 ms
Left  : total=10608 cps=0.0 valid=yes age=2 ms
```

- Rejection: any argument.

### `imu`

- Syntax: `imu`
- Arguments/units: none; acceleration is m/s², angular velocity is rad/s, age is ms.
- Valid states: all runtime states.
- Effect: reports driver state, WHO_AM_I, validity/age, and the current three-axis sample.
- Safety: read-only; a missing IMU cannot authorize a motor.
- Example: `imu`
- Rejection: any argument.

### `battery`

- Syntax: `battery`
- Arguments/units: none; ADC in counts/V, battery estimate in V, age in ms.
- Valid states: all runtime states.
- Effect: reports measurement validity, raw ADC, ADC voltage, filtered battery estimate, and
  whether the divider calibration is verified or still commissioning-only.
- Safety: read-only; this command does not invent an undervoltage threshold.
- Example: `battery`
- Rejection: any argument.

### `can`

- Syntax: `can`
- Arguments/units: none; frame/error counters are cumulative and ages are ms.
- Valid states: all runtime states.
- Effect: reports FDCAN state, RX/TX, rejected frames, duplicate/out-of-order sequence errors, RX
  overflow, TX failures, bus-off count, session ID, heartbeat age, authority age/state, and command
  age.
- State values: `NOT_INITIALIZED`, `ERROR_ACTIVE`, `ERROR_WARNING`, `ERROR_PASSIVE`, `BUS_OFF`.
- Safety: read-only. It adds no Protocol 1.0 wire fields.
- Example response:

```text
State   : ERROR_PASSIVE
RX/TX   : 0 / 3
Reject  : 0  SeqErr=0
Overflow: 0  TXFail=0  BusOff=0
Session : NONE
HB      : NONE
Auth    : NONE
Cmd     : NONE
```

- Rejection: any argument.

### `fault`

- Syntax: `fault`
- Arguments/units: none; hexadecimal 32-bit centralized fault word.
- Valid states: all runtime states.
- Effect: prints only active fault names after the mask.
- Safety: read-only; it does not clear or mask a fault.
- Examples: `0x00000000 NONE`, `0x00000010 ENCODER_VALIDITY`.
- Rejection: any argument.

## Fault recovery

### `clear`

- Syntax: `clear`
- Arguments/units: none.
- Valid states: all runtime states.
- Effect: first disarms, resets control, and forces motor-safe output. It may then clear
  `COMMAND_TIMEOUT`, `INVALID_MOTOR_COMMAND`, and `CONTROL_SATURATION`. It clears
  `ENCODER_VALIDITY` only if both encoder timers initialized, both latest samples are valid, and
  both sample ages are at most 50 ms.
- Safety: an actively failing condition is never cleared. Other faults remain owned by their
  originating modules/reset policy.
- Success: `OK`.
- Example rejection: `REJECTED: ENCODER_VALIDITY still active`.
- Rejection: any argument or any fault whose condition remains active.

Encoder validity is deliberately independent of Motor A/B mapping, motor polarity, PID gains,
speed limits, and BODY command readiness. A single delayed sample may assert the bit, but normal
valid sampling clears it automatically; timer-start failure or stale/invalid sampling keeps it
active.

## Authority and safe stop

### `arm`

- Syntax: `arm`
- Arguments/units: none.
- Valid states: no critical fault and no armed CAN authority.
- Effect: records a local arm request. It does not create a motion command or assert STBY.
- Safety: motion still requires a subsequent fresh valid `motor` command and all centralized
  safety checks. Boot/reset never arms automatically.
- Success: `OK`.
- Rejection: CAN authority active, critical fault active, or any argument.

### `disarm`, `stop`

- Syntax: `disarm` or `stop`.
- Arguments/units: none.
- Valid states: all runtime states.
- Effect: invalidates the current command, removes local arm, resets control, sets PWM/directions
  safe, and forces STBY LOW.
- Safety: immediate safe convergence through the existing production modules.
- Success: `OK`.
- Rejection: any argument.

A disabled/disarmed command is intentionally not considered timed out. `COMMAND_TIMEOUT` protects
an established motion/control session when required command or CAN authority/heartbeat freshness
is lost; it is not generated merely because Mode is DISABLED, Arm is No, and Motor is disabled.

## Timed motor commissioning

### `motor <a> <b> <duration_ms>`

- Syntax: `motor <motor_a_effort> <motor_b_effort> <duration_ms>`.
- Arguments: Motor A/B normalized effort, finite `-1.0..+1.0`; integer duration `10..1000 ms`.
- Valid states: local `arm` already requested, no critical fault, and no armed CAN authority.
- Effect: submits direct Motor A/B effort through the shared command, Safety Supervisor, and
  motor-output path. The duration is the command freshness deadline.
- Safety: it does not imply left/right wheel mapping, does not bypass STBY policy, and never sends
  an invalid numeric value to PWM. Expiry produces the existing deterministic command-timeout
  safe stop; use `clear` after correcting/ending that condition before another armed pulse.
- Example: `motor 0.05 0.00 250`.
- Success: `OK: motor command accepted (250 ms)`.
- Rejection: syntax/nonfinite value, effort out of range, duration out of range, missing arm,
  active CAN authority, critical fault, or shared command rejection.

This command can move real hardware. Use only under the project’s physical motor-acceptance
conditions with wheels clear, a reachable power disconnect, and `stop` ready.

## Watch mode

- Syntax: `watch encoder`, `watch imu`, `watch can`, or `watch off`.
- Arguments/units: one source; rate is fixed at 1 report/s and is not configurable.
- Valid states: all runtime states.
- Effect: periodically emits the same concise source output. `watch off` restores the quiet
  console.
- Safety: watch reports are low-priority whole blocks and are dropped under queue pressure rather
  than delaying real-time work.
- Success: `OK: watch encoder at 1 Hz` or `OK` for off.
- Rejection: missing/unknown source or extra argument.

## Error forms

```text
ERROR: unknown command 'xyz'
Hint : type 'help'
```

```text
ERROR: syntax
Usage: motor <a> <b> <duration_ms>
```

```text
REJECTED: arm required
```

Input overflow or an overlength line discards the current line and returns one bounded error.
