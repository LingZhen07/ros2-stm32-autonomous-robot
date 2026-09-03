# AGENTS.md — RobotProject Engineering Contract

> Mandatory operating contract for every Codex CLI session working on RobotProject.
>
> **Read this file before every stage. Follow it at all times. Modify it only when the user explicitly requests a repository-policy update.**
>
> This file is English-only so Firmware Codex and ROS Codex share one unambiguous contract.

---

## 1. Mission and Production-Execution Policy

RobotProject is a real-hardware differential-drive autonomous robot using:

- Orange Pi AI Pro 8GB, Ubuntu 22.04.5, ROS 2 Humble, Ascend 310B4;
- STM32G474RET6 real-time control;
- TB6612 motors, hardware quadrature encoders and ICM-42688-P IMU;
- RPLIDAR A1;
- production CAN FD between Orange Pi `can3` and STM32 FDCAN1;
- ROS 2 mapping, localization and Nav2.

Current objective:

```text
0x181 wheel telemetry
→ ROS wheel odometry
→ odom → base_link
→ LiDAR mapping
→ saved map
→ localization
→ Nav2
→ explicit goal
→ autonomous physical motion
→ target reached
```

The accepted LiDAR emergency-stop path remains an independent safety layer.

### Execution rule

Only perform work that directly advances the current real project or real-hardware demo.

```text
INSPECT ONLY WHAT IS NEEDED
→ MAKE THE SMALLEST VALID CHANGE
→ BUILD WHEN NEEDED
→ RUN ONLY NECESSARY SIMPLE VALIDATION
→ FIX ACTUAL FAILURES
→ ACCEPT
→ DOCUMENT
→ MOVE FORWARD
```

Do not perform broad regression testing, repeated acceptance testing, speculative benchmarks, exhaustive diagnostics, infrastructure work, refactoring, or re-validation of already accepted behavior unless a concrete current change or observed failure requires it.

Prefer no test over an irrelevant test. Prefer one bounded validation over a broad test campaign.

Do not bypass safety protections for testing.

---

## 2. Repository Ownership and Isolation

```text
RobotProject/
├── firmware/       # STM32 domain
├── ros2_ws/        # ROS 2 / Orange Pi domain
├── interfaces/     # shared protocol contract
├── docs/           # persistent technical knowledge
├── AGENTS.md       # shared policy — user-managed
├── CHANGELOG.md    # cross-domain communication
├── README.md
└── README.zh-CN.md
```

### Firmware Codex

Owns `firmware/`, firmware-related documentation, and `interfaces/` only when interface work is required.

May read shared documentation and relevant `CHANGELOG.md` entries.

**Must never inspect, analyze, edit, build, or modify `ros2_ws/`.**

Firmware responsibilities include STM32CubeMX/`.ioc`, FreeRTOS/HAL, FDCAN, motors, encoders, wheel control, IMU, battery acquisition, telemetry, watchdogs, Motion Authority, faults, Safety Supervisor and Protocol 1.0 firmware implementation.

Firmware Codex must not implement ROS nodes, TF, host odometry, SLAM, localization, Nav2, ROS costmaps or ROS sensor fusion.

### ROS Codex

Owns `ros2_ws/`, ROS configuration and ROS documentation.

May read `interfaces/`, shared technical documentation and relevant `CHANGELOG.md` entries.

**Must never inspect, analyze, edit, build, or modify firmware source code.**

Consume firmware only through documented shared interfaces.

ROS responsibilities include CAN bridge, Protocol 1.0 decoding, host odometry, TF, RPLIDAR, mapping, map handling, localization, Nav2, launch/parameters, diagnostics and navigation-demo integration.

Do not duplicate STM32 safety logic in ROS.

### Cross-domain coordination

Communicate through:

```text
interfaces/
CHANGELOG.md
docs/
accepted milestone reports
```

Do not use the peer implementation directory as an informal communication path.

Do not run both Codex roles in parallel merely because both exist. Run both only when the work is genuinely independent and the shared contract is already stable. If one domain produces an input required by the other, finish that dependency first.

---

## 3. Sources of Truth and Inspection Discipline

Priority:

```text
1. accepted real measured behavior
2. production source / generated configuration
3. frozen interfaces/
4. current docs and CHANGELOG.md
5. official manufacturer/upstream documentation
6. secondary references only when necessary
```

Never turn an assumption, commissioning value or intended design into a verified hardware fact.

Before changing code, read only what is relevant:

1. `AGENTS.md`;
2. applicable technical documentation;
3. applicable interface documentation;
4. recent relevant `CHANGELOG.md` entries;
5. the implementation being changed.

Do not rediscover or re-test frozen facts unless a current contradiction or blocker exists.

Frozen hardware resources must not be remapped or reconfigured for stylistic reasons. If CubeMX owns a required peripheral/clock/NVIC/RTOS configuration, keep `.ioc` synchronized with production firmware.

Use hardware encoder peripherals; do not replace TIM2/TIM3 quadrature decoding with high-rate GPIO polling.

Unsafe assumptions about power wiring, voltage compatibility, CAN termination/transceiver behavior, unknown electrical behavior or other physical hardware require a decision rather than invention in software or documentation.

---

## 4. Current Accepted Production State

### Firmware and drivetrain

Accepted firmware:

```text
0.5.4
```

Accepted production behavior includes:

- FreeRTOS and FDCAN1;
- CAN FD + BRS;
- hardware quadrature encoders;
- closed-loop left/right wheel velocity control;
- IMU and battery telemetry;
- TB6612 motor control;
- watchdog and command freshness supervision;
- Motion Authority;
- fault handling and motor-safe startup;
- Safety Supervisor;
- Protocol 1.0 telemetry and command handling.

Commissioning limits:

```text
forward velocity:  0.30 m/s
angular velocity:  ±1.50 rad/s
wheel target:      approximately ±3000 count/s
motor/PWM command: ±0.60
```

These are commissioning limits, not verified physical maxima.

Accepted wheel mapping:

```text
Motor A   = right wheel
Motor B   = left wheel
Encoder 1 = right wheel
Encoder 2 = left wheel
```

Raw forward signs:

```text
right motor command:        -1
left motor command:         -1
right encoder raw forward:  positive
left encoder raw forward:   negative
```

Protocol logical wheel position/rate fields normalize forward as positive for both wheels.

Commissioning geometry/scales:

```text
left:        0.0001362305 m/count
right:       0.0001363976 m/count
wheel radius: 0.023 m
track width:  0.125 m
half track:   0.0625 m
```

```text
v_left  = v - omega * 0.0625
v_right = v + omega * 0.0625
```

Do not average left/right wheel scales. Do not derive drivetrain geometry from the Nav2 footprint.

### Production CAN FD

```text
interface:            can3
can2:                 deprecated
nominal bitrate:      500000
nominal sample point: 0.800
data bitrate:         2000000
data sample point:    0.825
BRS:                  enabled
bus error reporting:  enabled
```

Accepted CAN3 real-hardware state: ERROR-ACTIVE, zero TX/RX error counters, zero bus errors, zero bus-off and stable bidirectional traffic.

Do not repeat CAN bring-up/acceptance testing unless a current CAN change or failure requires it.

### Frozen Protocol 1.0

Authoritative sources:

```text
interfaces/protocol_v1.md
interfaces/protocol_v1.yaml
protocol examples under interfaces/
```

```text
0x080 Motion Authority
0x081 Body Motion Command
0x082 Host Heartbeat
0x180 System / Safety Status
0x181 Wheel / Encoder State
0x182 IMU
0x183 Battery
```

Transport: 11-bit standard CAN ID, CAN FD, BRS, little-endian.

```text
command timeout:   250 ms
heartbeat timeout: 500 ms
authority timeout: 500 ms
motion modes:      DISABLED, BODY_VELOCITY
```

Protocol 1.0 is frozen. Do not redesign, extend or reinterpret it unless a demonstrated blocker proves it insufficient.

### Accepted `0x181` odometry readiness

```text
VERDICT: 0x181 SUFFICIENT
FIRMWARE CHANGE: NONE
```

Use normalized cumulative logical wheel positions for pose:

```text
left_position_counts   signed i64, forward-positive cumulative counts
right_position_counts  signed i64, forward-positive cumulative counts
```

Do not integrate pose from raw 16-bit timer counters or filtered wheel-rate fields.

Required host semantics:

```text
protocol version:      1.0
sequence:              modulo 2^16
timestamp_ms:          STM32 monotonic ms, modulo 2^32
0x181 period:          20 ms / 50 Hz
encoder sample period: 10 ms / 100 Hz
```

Integrate only when `flags` bits 0, 1 and 2 are all valid. Invalid logical positions use `INT64_MIN`; invalid rates use `INT32_MIN`; flags remain authoritative.

Firmware already unwraps raw encoder rollover into signed i64 cumulative positions.

ROS must detect MCU reset/timestamp discontinuity and re-baseline without integrating the reset jump. Sequence gaps do not lose traveled distance because positions are cumulative.

This firmware-readiness question is closed. Do not re-audit `0x181` unless ROS finds concrete contradictory interface evidence.

### Orange Pi / ROS baseline

```text
Orange Pi AI Pro 8GB
Ubuntu 22.04.5 LTS
aarch64
Vendor Linux 5.10 BSP
ROS 2 Humble
Ascend 310B4
workspace: /data/ros2_ws
project data: /data/projects
expected ROS Python: /usr/bin/python3
```

ROS environment:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
```

Do not replace the vendor kernel/BSP, perform broad system upgrades, reinstall a working CANN environment, or merge Conda Python into the ROS runtime Python environment.

Existing RGB-D/perception work is frozen and outside the current navigation critical path. Do not reopen it without a demonstrated navigation blocker.

### TF and navigation constraints

```text
base_link origin = drive-wheel axle midpoint
+X forward
+Y left
+Z up
```

```text
base_link → camera_link: x +0.130, y 0, z +0.110, roll 0, pitch 0, yaw 0
base_link → laser_frame: x +0.043, y 0, z +0.165, roll 0, pitch 0, yaw π
```

Frozen footprint:

```yaml
footprint: "[[0.140, 0.080], [0.140, -0.080], [-0.070, -0.080], [-0.070, 0.080]]"
footprint_padding: 0.01
```

Exactly one TF authority may publish `odom → base_link`. Never create duplicate TF publishers.

Preserve accepted Navigation v1 settings when used:

```text
lifecycle_manager.bond_timeout = 0.0
scan max_obstacle_height = 2.0
```

Known standalone Humble costmap shutdown-only exit `-11` remains deferred. Do not investigate it unless it becomes a real blocker.

---

## 5. Frozen Safety Architecture

STM32 Safety Supervisor remains authoritative for actuator-level safety.

Never weaken or bypass:

- Motion Authority;
- command, heartbeat and CAN freshness;
- watchdogs;
- fault handling;
- motor-safe startup;
- Safety Supervisor;
- latched emergency-stop behavior.

The Orange Pi determines where the robot should go. The STM32 determines whether requested motion can safely be executed at the actuator level.

Do not move LiDAR obstacle detection into STM32. Do not duplicate the STM32 safety state machine in ROS.

Accepted real-hardware safety path:

```text
explicit START
→ straight motion at 0.30 m/s
→ real /scan
→ frontal obstacle detection
→ zero velocity
→ Motion Authority withdrawal
→ STM32 safe stop
→ STOPPED latch
→ obstacle removal does not restart
→ new explicit START required
```

Accepted LiDAR configuration:

```text
RPLIDAR A1
frontal sector: 30° centered forward
stop distance: 0.60 m
```

Accepted one-run observed timing:

```text
detection → zero velocity:              ~0.075 ms
detection → Authority withdrawal:       ~1.276 ms
detection → STM32 stop confirmation:    ~30.8 ms
```

These are observed values, not guaranteed worst-case limits.

Do not repeat this accepted safety demo during unrelated stages. Revalidate navigation plus the safety-stop path only at the integrated navigation+safety stage or when a change directly affects it.

---

## 6. Active Navigation Execution Order

Proceed one dependency at a time:

```text
1. ROS differential-drive odometry from 0x181
2. single-authority odom → base_link TF
3. minimum real-hardware odometry validation
4. real LiDAR mapping
5. save/reload map
6. localization
7. Nav2 + real drivetrain
8. point-to-point autonomous navigation
9. navigation + independent safety-stop validation
```

Do not begin with depth-camera integration, perception expansion, EKF/sensor fusion without demonstrated need, Nav2 tuning before odometry/TF, speculative firmware changes, or unrelated refactoring.

### Current immediate stage

**ROS Codex only.** Firmware Codex is not needed unless a concrete Protocol 1.0 blocker appears.

```text
0x181 cumulative wheel positions
→ differential-drive integration
→ nav_msgs/msg/Odometry
→ odom → base_link
→ minimal real-hardware validation
```

For consecutive valid samples:

```text
d_left  = delta_left_counts  * 0.0001362305
d_right = delta_right_counts * 0.0001363976

d_center = (d_left + d_right) / 2
d_theta  = (d_right - d_left) / 0.125
```

Use standard planar differential-drive integration consistent with ROS frame conventions.

Implementation must:

- preserve independent left/right scales;
- ignore invalid samples rather than integrate them;
- re-baseline on MCU restart/timestamp discontinuity;
- tolerate sequence gaps through cumulative counts;
- maintain one `odom → base_link` authority;
- add only what the current navigation architecture needs.

Do not add firmware telemetry, new CAN messages, sensor fusion or extra infrastructure for basic wheel odometry.

### Minimum odometry validation

Required evidence only:

```text
physical wheel motion
→ valid 0x181 cumulative telemetry
→ ROS /odom changes
→ forward motion produces positive forward displacement
→ turning produces correct yaw sign
→ odom → base_link is stable and single-authority
```

Use one or a small number of controlled motions sufficient to establish those facts.

Do not perform broad calibration, long-duration drift characterization, exhaustive angle/distance testing, stress testing, repeated runs or full regression suites unless this simple validation exposes an actual problem.

Refine wheel/track calibration later only if real navigation shows that calibration error materially affects the demo.

---

## 7. Minimal-Change Firmware Policy

Accepted firmware behavior remains unchanged unless a verified blocker requires a narrow change.

Default firmware decision:

```text
NO CHANGE
```

Do not add without a demonstrated current requirement:

- FreeRTOS tasks;
- DMA;
- GPIO interrupts;
- queues/mutexes/timers;
- higher-rate sensors;
- additional telemetry or CAN messages;
- broad abstractions/refactors.

If a required change touches CubeMX-managed configuration, keep `.ioc` synchronized with production firmware.

---

## 8. Codex Autonomy and Action Gates

### Pre-approved read-only work

Within the assigned domain and known project resources, relevant read-only inspection is pre-approved. Do not ask permission to inspect files/source, Git state, build configuration/output, logs, ROS graph/state, device/configuration state, documentation or the known Orange Pi runtime.

If one safe read-only method fails, try another bounded method only when useful.

Domain isolation still applies:

```text
Firmware Codex: no ros2_ws/
ROS Codex:      no firmware/
```

### Routine reversible implementation

Within its assigned domain, Codex may directly perform reversible work required by the active task: edit project-owned source/configuration, build when useful, run minimal non-destructive validation, inspect diffs, use temporary diagnostics, and update relevant documentation/CHANGELOG.

Do not automatically run formatting, static analysis, unit tests, integration suites or full builds merely because they exist. Run only what provides meaningful confidence for the current change.

Always inspect existing files before overwriting them. Never destroy unrelated user work.

### ROS runtime host

ROS Codex may use the normal non-root account on:

```text
HwHiAiUser@robot-core.local
/data/ros2_ws
/data/projects
```

Do not scan the LAN or substitute another host.

The actual Orange Pi runtime is the source of truth for ROS/hardware runtime acceptance. A local build alone does not prove real robot behavior.

### Stop before destructive/privileged/system-wide work

Examples:

```text
sudo or root login
system package installation/removal
OS upgrade
kernel/BSP replacement
system-wide configuration/security/permission changes
destructive filesystem operations
git push --force
git reset --hard
git clean -fd
```

Do not use `chmod 777`. Do not install speculative dependencies.

Firmware flashing requires a decision if the new image may alter motor enable, safety semantics, option bytes/readout protection, boot configuration, irreversible flash state or unexpected actuation.

### Decision gates

Stop and report only for a real unresolved decision involving:

- cross-domain architecture/responsibility split;
- frozen Protocol 1.0 evolution;
- public ROS contract or TF ownership architecture;
- safety-critical behavior;
- materially new/unsafe actuation;
- Motion Authority/watchdog/timeout/fault/emergency-stop semantics;
- unsafe physical/electrical uncertainty;
- destructive/privileged/system-wide action;
- sensor replacement or major localization/navigation architecture change.

Do not ask questions repository inspection can answer.

Use:

```text
DECISION REQUIRED

Observed:
...
Current stage:
...
Why execution stopped:
...
Recommended action:
...
Alternatives:
... only if useful
Impact:
...
Rollback:
... if applicable
Decision requested:
...
```

---

## 9. Documentation, CHANGELOG and Git Discipline

Technical documentation is part of implementation.

When a meaningful stage is accepted:

1. update relevant technical documentation immediately;
2. update `CHANGELOG.md` if the result changes a cross-domain contract, assumption, milestone or next integration boundary;
3. record limitations and measured values accurately;
4. keep documentation aligned before unrelated work starts.

Do not postpone documentation until the end of a long development sequence.

Persistent docs should distinguish `VERIFIED`, `FROZEN`, `IN PROGRESS`, `PROPOSED` and `TBD`. Maintain English/Chinese counterparts where the repository already requires them.

README files are showcase documents, not development logs. Update them only for meaningful externally visible milestones such as accepted real `/odom`, mapping/localization, physical Nav2 execution or completed autonomous navigation.

### CHANGELOG

`CHANGELOG.md` is for cross-domain communication, not trivial local history.

Record peer-relevant changes to CAN/interface behavior, telemetry/command/safety semantics, timing, encoder semantics, wheel signs/scales/geometry, startup, Motion Authority/watchdogs, deployment configuration, production CAN interface, or milestones that change the peer's next action.

Do not record formatting-only changes, temporary diagnostics, trivial refactors or isolated implementation details with no peer impact.

Every **new** entry must use exactly:

```text
YYYY-MM-DDTHH:MM
```

Example:

```text
2026-09-03T14:25
```

No seconds, timezone suffix, space between date/time or alternative format. Do not rewrite old entries solely to normalize formatting.

A cross-domain entry must state what changed, why, what interface/behavior is affected, and whether the peer domain must act.

### Git

Before editing, inspect the relevant file and relevant Git status/diff.

After editing, run only checks required by the change and review the resulting diff.

Do not revert unrelated dirty-worktree changes. Do not perform destructive Git operations.

Keep local details in their owning domain and communicate peer-impacting changes through `CHANGELOG.md`.

Finish a coherent accepted stage before starting unrelated work.

---

## 10. Stage Completion and Current Target

A stage is complete only when the behavior required by that stage has sufficient real-target or minimum appropriate evidence. A successful build alone does not prove hardware behavior, but unrelated subsystem revalidation is not required.

Use milestone reports:

```text
MILESTONE REPORT

Stage:
...
Status:
PASS / PARTIAL / FAIL
Files changed:
...
Build / validation performed:
...
Verified evidence:
...
Measured values:
...
Safety state:
...
Shared-interface impact:
...
CHANGELOG entry:
added / not required
Documentation updated:
...
Open issues:
...
Recommended next stage:
...
```

Report only measurements actually observed or already accepted by the project.

Current accepted chain:

```text
STM32 drivetrain + closed-loop wheel control          ACCEPTED
CAN FD production transport on can3                   ACCEPTED
Protocol 1.0                                          FROZEN
Motion Authority / watchdog / Safety Supervisor       ACCEPTED
real RPLIDAR latched safety-stop demo                  ACCEPTED
0x181 sufficiency for host wheel odometry              ACCEPTED
```

Active unfinished chain:

```text
ROS wheel odometry
→ odom → base_link
→ real LiDAR mapping
→ saved-map reload
→ localization
→ Nav2 real drivetrain integration
→ explicit navigation goal
→ physical autonomous motion with odometry feedback
→ target reached
→ integrated navigation + independent safety-stop validation
```

Depth-camera/perception expansion is outside this critical path.

Final real-hardware target:

```text
system startup
→ localization in saved map
→ explicit user navigation goal
→ autonomous path planning
→ autonomous differential-drive motion
→ accepted safety protections preserved
→ requested goal reached
```

Continue one dependency at a time. Prefer real execution progress over extra analysis, testing, infrastructure or documentation volume that does not advance this chain.
