# CHANGELOG.md

This file is the **cross-domain engineering communication log** for RobotProject.

It is intentionally **English-only**.

Record only changes that the other Codex domain must know about, shared-interface changes, safety/architecture changes, or project milestones. Do not record trivial local implementation details.

## Entry format

```text
## YYYY-MM-DDTHH:MM:SS±HH:MM — Title

Domain: Firmware | ROS | Interface | System
Impact: <what the other domain must know>

Changed:
- ...

Action required:
- None
```

---

## 2026-08-31T09:02:05+08:00 — Orange Pi CAN3 physical link accepted and productionized

Domain: System
Impact: Protocol 1.0 is unchanged. The ROS production transport is now frozen on Orange Pi CAN3 / SocketCAN `can3`; CAN2 is historical and must not be used as a fallback.

Changed:
- The installed and boot-verified path is `822d0000.mttcan`, `mttcan-id=3`, GPIO2_17/CAN_TX3, and GPIO2_18/CAN_RX3.
- Real bidirectional CAN FD+BRS Protocol 1.0 traffic passed: host `0x080`/`0x082` and STM32 `0x180`/`0x181`/`0x182`/`0x183`, with the robot DISARMED.
- Linux production timing is explicitly fixed at 500 kbit/s with 80.0% sample point and 2 Mbit/s with 82.5% sample point. This corrected the earlier Linux default sample-point mismatch.
- Accepted health evidence is Error Active, TX error 0, RX error 0, bus-errors 0, and bus-off 0.
- `robot_stm32_bridge` now defaults to `can3`. Version-controlled systemd units configure CAN3 idempotently before starting the DISARMED bridge; unarmed startup does not emit `0x081`.

Action required:
- Deploy and enable the version-controlled `robot-can3.service` and `robot-stm32-bridge.service` on the Orange Pi after building the updated package.
- The next safety-gated milestone is the controlled straight-drive and latched LiDAR obstacle-stop demo.

## 2026-08-29T17:53:32+08:00 — M5 BODY_VELOCITY drivetrain handoff ready in firmware 0.5.4

Domain: Firmware
Impact: Protocol 1.0 is unchanged. The ROS bridge may now treat STM32 BODY_COMMAND_READY as true when the existing session, communication, runtime-health, and safety gates are also valid; physical straight-drive acceptance is still required before the `/scan` demo.

Changed:
- Verified production mapping is Motor A -> right wheel and Motor B -> left wheel, with logical-forward motor sign -1 on both channels. Encoder 1/right/+1 raw and Encoder 2/left/-1 raw normalization is unchanged.
- Firmware now initializes the independent left/right wheel PI controllers and complete drivetrain configuration. BODY_COMMAND_READY requires valid mapping, geometry/scales, finite limits, and both configured controllers.
- The commissioning envelope is body linear 0.30 m/s, body angular 1.50 rad/s, wheel rate 3000 count/s, wheel-target slew 4400 count/s^2, and normalized motor output +/-0.60. Initial per-side gains are Kp 0.00020, Ki 0.00060, Kd 0, with a 700 count-second integrator limit.
- Authority withdrawal, disarm, timeout, bus-off, invalid command, fault, and reset continue to bypass the command ramp and converge through the existing motor-safe path. A new session/authority and fresh command are required after stop.
- The 0.30 m/s value is a commissioning limit, not a verified hardware maximum. Installed-motor loaded/stall current, attainable loaded wheel speed, and TB6612 carrier/module thermal margin remain unmeasured.

Action required:
- ROS domain: no serializer or Protocol 1.0 change. Keep the first straight-drive acceptance at 0.05 m/s, require BODY_COMMAND_READY plus all existing gates, and do not raise speed until real closed-loop tracking and authority-withdrawal stopping are accepted.
- Cross-domain acceptance: after the controlled straight-drive run passes, connect that accepted path to the existing explicit-START, latched-STOPPED `/scan` obstacle-stop flow.

## 2026-08-29T11:42:02+08:00 — Encoder validity semantics corrected in firmware 0.5.3

Domain: Firmware
Impact: Protocol 1.0 is unchanged. System-status fault bit 4 now reports encoder acquisition validity only, so the ROS domain must not interpret incomplete drivetrain/controller commissioning as an encoder failure.

Changed:
- `ENCODER_VALIDITY` now clears automatically only after both encoder timers initialized successfully, both latest samples are valid, and both local sample ages are at most 50 ms; real initialization/stale-sample failures still assert it.
- Unknown Motor A/B mapping or polarity, unset controller gains/speed limits, and `BODY_COMMAND_READY=false` do not assert encoder fault bit 4.
- A DISABLED, unarmed command snapshot remains intentionally non-timed-out. Existing timeout protection for an established motion/control session is unchanged.
- The local USART2 console was reduced to a quiet essential commissioning command set with optional fixed 1 Hz watch output; this does not alter CAN IDs, layouts, rates, scaling, session, sequence, authority, or timeout contracts.

Action required:
- ROS domain: no serializer or bridge change; consume Protocol 1.0 as before and treat bit 4 strictly as STM32 encoder acquisition health.
- Hardware acceptance: after flashing firmware 0.5.3, confirm `encoder` reports both sides valid and `fault` keeps bit 4 clear.

## 2026-08-29T00:39:38+08:00 — Physical wheel encoder mapping and independent scales verified

Domain: System
Impact: Firmware 0.5.2 now normalizes verified physical wheel encoders and uses independent directly measured output-wheel scales. Protocol 1.0 wire layouts and behavior are unchanged; ROS odometry must preserve the independent commissioning scales.

Changed:
- Ten complete forward wheel revolutions measured Encoder 1 at +10,595 counts and Encoder 2 at -10,608 counts.
- Encoder 1 is the right wheel with forward raw sign +1 and 1059.5 decoded counts/wheel-rev; Encoder 2 is the left wheel with forward raw sign -1 and 1060.8 decoded counts/wheel-rev.
- Logical wheel convention remains forward-positive: right logical count/rate is `+right raw`, left logical count/rate is `-left raw`. Raw Encoder 1/2 UART diagnostics and frozen Protocol `0x181` raw diagnostic fields remain unchanged.
- With commissioning radius 0.023 m, the derived right scale is approximately 0.0001363976 m/count and 0.005930331 rad/count; the left scale is approximately 0.0001362305 m/count and 0.005923063 rad/count. The approximately 0.123% difference is not averaged.
- BODY_COMMAND_READY remains false pending measured Motor A/B wheel ownership/polarity, accepted operating limits, and commissioned left/right controller gains.

Action required:
- ROS domain: interpret Protocol `0x181` left/right wheel fields as logical forward-positive values and retain independent right/left commissioning scales for odometry; do not average them or treat them as final effective calibration.
- Firmware commissioning: identify Motor A/B ownership and forward polarity with separately gated low-effort tests before selecting operating limits or tuning controllers.

## 2026-08-28T16:12:51+08:00 — Commissioning wheel geometry measured

Domain: System
Impact: Firmware 0.5.1 now uses measured commissioning wheel geometry for future BODY_VELOCITY conversion. The values are not final effective calibration and Protocol 1.0 is unchanged.

Changed:
- User-measured wheel radius is 0.023 m and wheel track is 0.125 m; derived half track is 0.0625 m and geometric circumference is approximately 0.1445 m.
- Differential-drive commissioning conversion is `v_left = v - omega * 0.0625` and `v_right = v + omega * 0.0625`.
- The verified encoder convention remains left-forward raw CPS negative, right-forward raw CPS positive, and logical forward positive on both wheels.
- BODY_COMMAND_READY remains false because decoded encoder scale, mappings, motor polarity, operating limits, and controller gains are not yet verified/configured.

Action required:
- Firmware commissioning: directly measure decoded encoder counts over multiple complete wheel revolutions and divide by the revolution count. Do not infer encoder scale from product specifications.
- ROS domain: treat radius/track as `MEASURED / COMMISSIONING`, not final odometry calibration.

## 2026-08-28T15:50:39+08:00 — Physical CAN wiring confirmed

Domain: System
Impact: The Orange Pi/STM32 CAN physical wiring topology is now confirmed, but real CAN FD traffic and controller error state are not yet accepted.

Changed:
- CANH is connected to CANH, CANL to CANL, and both nodes share a common ground.
- One 120 ohm termination is installed at each physical end of the bus.
- Protocol 1.0 and all CAN nominal/data timing, IDs, layouts, rates, and safety semantics remain unchanged.

Action required:
- Cross-domain integration: observe real bidirectional FD+BRS frames and Error Active state before declaring the physical CAN link PASS.
- Firmware commissioning: retain BODY_VELOCITY gating until measured drivetrain mapping, geometry, encoder scale, operating limits, and controller gains are configured.

## 2026-08-24T20:45:25+08:00 — STM32 M5 firmware side ready for cross-domain acceptance

Domain: Firmware
Impact: The production STM32 firmware is ready for physical Orange Pi/STM32 CAN FD integration against unchanged Protocol 1.0. Closed-loop BODY_VELOCITY execution remains deliberately gated by real drivetrain measurements and controller commissioning.

Changed:
- Firmware identity is 0.5.0; Protocol 1.0 CAN IDs, layouts, rates, scaling, session, sequence, authority, and timeout semantics are unchanged.
- CAN BODY_VELOCITY commands are validated against controller configuration, drivetrain readiness, and local operating limits before entering the shared command model; invalid active commands converge through the existing Safety Supervisor.
- Permanent USART2 diagnostics now expose FDCAN state, counters, session/authority/freshness ages, safety/fault state, and bounded commissioning controls. The console is quiet by default and does not change the shared CAN interface.
- Debug and Release clean builds pass with zero compiler/linker warnings. Physical CAN traffic and the straight-drive obstacle-stop demo have not yet been accepted.

Action required:
- ROS domain: proceed with physical Protocol 1.0 integration. Do not expect STM32 ACTIVE/BODY_VELOCITY until wheel/motor mapping, motor polarity, wheel radius, track, encoder scale, gear ratio, operating limits, and wheel-controller gains are measured/configured.

## 2026-08-24T13:38:04+08:00 — Communication Protocol v1 frozen and STM32 M4 complete

Domain: Interface
Impact: STM32 M1-M3 physical acceptance is PASS, and the stable CAN FD boundary is ready for the Orange Pi ROS bridge implementation. The ROS domain can implement entirely from `interfaces/protocol_v1.md`, `interfaces/protocol_v1.yaml`, and the golden frames.

Changed:
- Real STM32 execution, USART2, encoder acquisition, ICM-42688-P, battery ADC, TIM1 PWM, TB6612 control, and motor rotation are accepted hardware facts.
- Verified encoder convention is left-forward raw CPS negative and right-forward raw CPS positive; normalized logical forward is positive on both wheels. Encoder 1/2 and Motor A/B wheel ownership remain TBD.
- Protocol version 1.0 is frozen: Standard 11-bit CAN IDs, CAN FD+BRS, little-endian explicit serialization, and CAN FD link-layer CRC only.
- Transport timing is nominal 500 kbit/s (prescaler 17, SEG1 15, SEG2 4, SJW 4, 80% sample point) and data 2 Mbit/s (prescaler 5, SEG1 13, SEG2 3, SJW 3, 82.3529% sample point).
- Host IDs are `0x080` motion authority at 10 Hz, `0x081` body motion command at 50 Hz, and `0x082` heartbeat at 10 Hz. STM32 IDs are `0x180` system at 10 Hz, `0x181` wheel at 50 Hz, `0x182` IMU at 100 Hz, and `0x183` battery at 2 Hz.
- Body command scaling is signed 1 mm/s and 1 mrad/s. Wheel rate is signed counts/s, IMU scaling is 0.0001 m/s^2 and 0.00001 rad/s, battery is 1 mV, and STM32 timestamps are unsigned monotonic milliseconds.
- STM32 enforces a 250 ms command timeout and 500 ms host-heartbeat/authority timeouts using local reception time. Startup/reconnect requires heartbeat, explicit DISARMED, ARMED, then a fresh body command; old motion is never restored.
- STM32 firmware now includes bounded interrupt-driven FDCAN RX, task-context protocol validation, centralized command/safety integration, CAN health/error handling, and deterministic telemetry serialization.

Action required:
- ROS domain: inspect the real Orange Pi CAN capability, configure SocketCAN FD for 500 kbit/s / 2 Mbit/s, implement the bridge exactly against Protocol v1, and preserve the required authority/session/freshness behavior.
- Cross-domain integration: verify the exact transceiver module, termination, logic levels, and real bidirectional CAN FD frames before claiming physical CAN acceptance.

## 2026-08-24T11:11:42+08:00 — STM32 M1-M3 firmware ready for hardware acceptance

Domain: Firmware
Impact: The STM32 production firmware foundation now builds locally and is ready for first physical acceptance. The ROS domain must not assume hardware acceptance or a finalized CAN application protocol.

Changed:
- Integrated MCU/peripheral, FreeRTOS, safety, watchdog, motor, sensor, wheel-control, diagnostics, command, and telemetry foundations are present in one firmware project.
- TIM2 and TIM3 are both configured and handled as 16-bit encoder counters.
- Debug and Release firmware images build with zero warnings; no MCU flashing or motor actuation was performed.
- Final CAN IDs, payloads, heartbeat semantics, and ROS bridge contracts remain intentionally undefined.

Action required:
- None for the ROS domain until STM32 hardware acceptance succeeds and the shared transport protocol stage begins.

## 2026-08-23T20:25:00+08:00 — STM32 real-time control phase established

Domain: System
Impact: The high-computing perception/SLAM/Navigation v1 stack is frozen. Active development moves to the STM32 real-time control domain.

Changed:
- STM32G474RET6 / DeveBox STM32G474R Ver:20 selected as the real-time controller.
- Pin Allocation v1 is frozen.
- HSE baseline is 8 MHz and SYSCLK target is 170 MHz.
- TIM1 motor PWM target is 10 kHz.
- TIM2 and TIM3 are reserved for hardware quadrature encoders.
- SPI1 is reserved for ICM-42688-P.
- ADC1_IN6 on PC0 is reserved for battery measurement.
- USART2 is reserved for debug.
- FDCAN1 on PA11/PA12 is the CAN controller with 500 kbit/s nominal bring-up bitrate.
- Safe startup requires MOTOR_STBY LOW, direction GPIO LOW, and IMU_CS HIGH.

Action required:
- Firmware domain: complete CubeMX baseline and safe MCU bring-up.
- ROS domain: remain on the frozen high-computing baseline until interface/bridge integration begins.
