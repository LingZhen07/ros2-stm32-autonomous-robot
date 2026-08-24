# AGENTS.md — RobotProject Engineering Rules

> **Repository policy:** This file is the mandatory operating contract for every Codex CLI session working on RobotProject.
>
> **Codex MUST read this file before starting a stage. Codex MUST NOT modify this file.**
>
> This document is intentionally written in English so both Codex instances use the same unambiguous engineering contract.

---

## 1. Project Mission

RobotProject is a long-term heterogeneous mobile-robot engineering project built around:

- an Orange Pi AI Pro high-computing domain running Ubuntu 22.04 and ROS 2 Humble;
- Ascend 310B4 acceleration for perception;
- an STM32G474RET6 real-time control domain;
- differential-drive motors with quadrature encoders;
- an ICM-42688-P IMU;
- a TB6612-based dual DC motor driver;
- CAN / CAN FD as the production control-transport direction;
- UART as a bring-up and fallback diagnostic path;
- ROS 2 Nav2 for high-level navigation.

The target is a maintainable embedded robotics system rather than a single-board demo:

```text
Embedded Linux
+ ROS 2
+ NPU perception
+ SLAM / Navigation
+ MCU real-time control
+ sensors
+ CAN
+ safety
+ wheel odometry
+ closed-loop autonomous navigation
```

Engineering strategy:

```text
REAL SYSTEM
→ INSPECT
→ DEFINE INTERFACE
→ MINIMAL VALID IMPLEMENTATION
→ BUILD
→ RUN
→ VERIFY REAL BEHAVIOR
→ FIX ACTUAL FAILURES
→ ACCEPT
→ FREEZE
→ MOVE FORWARD
```

Do not over-engineer a subsystem before the next subsystem can be integrated.

---

## 2. Repository Layout and Ownership

```text
RobotProject/
├── firmware/       # STM32G474 firmware domain
├── ros2_ws/        # ROS 2 / Orange Pi software domain
├── interfaces/     # Cross-domain CAN / UART protocol contracts
├── docs/           # Project knowledge base, English + Chinese
├── AGENTS.md       # Mandatory shared agent policy — READ ONLY
├── CHANGELOG.md    # Cross-domain engineering communication log — English
├── README.md       # Primary GitHub-facing README — English
└── README.zh-CN.md # Chinese README
```

There are two independent Codex roles.

### 2.1 Firmware Codex

Primary write scope:

```text
RobotProject/firmware/
```

Shared read scope:

```text
RobotProject/AGENTS.md
RobotProject/CHANGELOG.md
RobotProject/interfaces/
RobotProject/docs/
RobotProject/README.md
RobotProject/README.zh-CN.md
```

Shared write scope, only when relevant:

```text
RobotProject/CHANGELOG.md
RobotProject/docs/
RobotProject/README.md
RobotProject/README.zh-CN.md
RobotProject/interfaces/
```

The Firmware Codex **must not inspect or modify `ros2_ws/`** unless the user explicitly authorizes a cross-domain investigation.

### 2.2 ROS Codex

Primary write scope:

```text
RobotProject/ros2_ws/
```

Runtime environment:

```text
HwHiAiUser@robot-core.local
/data/ros2_ws
/data/projects
```

Shared read scope:

```text
RobotProject/AGENTS.md
RobotProject/CHANGELOG.md
RobotProject/interfaces/
RobotProject/docs/
RobotProject/README.md
RobotProject/README.zh-CN.md
```

Shared write scope, only when relevant:

```text
RobotProject/CHANGELOG.md
RobotProject/docs/
RobotProject/README.md
RobotProject/README.zh-CN.md
RobotProject/interfaces/
```

The ROS Codex **must not inspect or modify `firmware/`** unless the user explicitly authorizes a cross-domain investigation.

### 2.3 Cross-domain rule

The two Codex instances communicate through:

```text
interfaces/
CHANGELOG.md
docs/
milestone reports
```

Do not use the peer implementation directory as an informal communication mechanism.

The goal is to minimize unnecessary code reading, duplicated analysis, context consumption, and cross-domain coupling.

---

## 3. Shared Files

### 3.1 `AGENTS.md`

Rules:

```text
READ before every project stage.
FOLLOW at all times.
DO NOT MODIFY.
```

### 3.2 `interfaces/`

This directory is the authoritative software contract between the Linux high-computing domain and the STM32 real-time domain.

It will eventually define:

- CAN / CAN FD message IDs;
- command semantics;
- telemetry semantics;
- binary packing;
- byte order;
- integer scaling;
- units;
- sequence numbers;
- timestamps;
- heartbeat semantics;
- timeout semantics;
- protocol versioning;
- compatibility rules.

Do **not** freeze protocol details before they are deliberately designed.

A protocol change that changes meaning, packing, timing semantics, IDs, compatibility, or safety behavior is a **cross-domain architecture decision** and requires a decision report unless already explicitly approved by the current task.

Once a protocol contract is frozen, both domains implement against `interfaces/`; neither domain invents private incompatible variants.

### 3.3 `CHANGELOG.md`

`CHANGELOG.md` is not a commit diary.

It is a low-overhead communication channel for changes that:

1. affect both domains; or
2. change a shared contract; or
3. create a new constraint the peer Codex must know; or
4. complete a project milestone that changes the next integration boundary.

The file must remain **English-only**.

Every entry must include an ISO-8601 timestamp with timezone.

Preferred format:

```text
## 2026-08-23T20:25:00+08:00 — Short title

Domain: Firmware | ROS | Interface | System
Impact: <what the other agent must know>
Changed:
- ...

Action required:
- None
```

Do not record trivial local refactors, formatting changes, temporary debugging, or isolated implementation details that cannot affect the other domain.

### 3.4 `docs/`

`docs/` is the project knowledge base.

For persistent project knowledge, maintain both:

```text
English
Chinese
```

Documentation should track:

- hardware;
- pin maps;
- architecture;
- interfaces;
- bring-up procedures;
- acceptance evidence;
- safety behavior;
- calibration results;
- measured drivetrain parameters;
- protocol definitions;
- odometry architecture;
- milestone records.

Documentation must distinguish:

```text
VERIFIED
FROZEN
IN PROGRESS
PROPOSED
TBD
```

Never convert an assumption into a verified fact.

### 3.5 README

`README.md` is the primary GitHub-facing document and is English-first.

`README.zh-CN.md` is the Chinese counterpart.

Update README only when a milestone has strong project-showcase value, for example:

- a subsystem becomes genuinely operational;
- a major architecture boundary is completed;
- real measured performance becomes available;
- Orange Pi ↔ STM32 communication is demonstrated;
- closed-loop wheel control works;
- real `/odom` works;
- Nav2 physically drives the robot;
- autonomous navigation / obstacle avoidance is accepted.

Do not turn README into a development log.

---

## 4. Current Verified Project State

### 4.1 High-computing domain

```text
Orange Pi AI Pro 8GB
Ubuntu 22.04.5 LTS
aarch64
Vendor Linux 5.10 BSP
ROS 2 Humble
Ascend 310B4
```

Primary runtime workspace:

```text
/data/ros2_ws
```

Project data:

```text
/data/projects
```

ROS environment:

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
```

Expected ROS Python:

```text
/usr/bin/python3
```

Do not:

- replace the vendor kernel;
- replace the BSP;
- perform broad system upgrades;
- reinstall a working CANN environment;
- merge Conda Python into the ROS runtime Python environment.

### 4.2 Frozen high-computing milestones

```text
Ascend 310B4 baseline                    COMPLETE / FROZEN
ROS 2 foundation                         COMPLETE / FROZEN
Astra RGB / Depth                        COMPLETE / FROZEN
RGB-D registration                       COMPLETE / FROZEN
YOLOv8n Ascend 2D detection              WORKING / FROZEN
RPLIDAR A1                               COMPLETE / FROZEN
Robot TF System v1                       COMPLETE / FROZEN
rosbag2 / diagnostics                    COMPLETE / FROZEN
Cartographer LiDAR SLAM v1               COMPLETE / FROZEN
Navigation v1 Upper Computing Domain     COMPLETE / FROZEN
```

Perception is frozen for the current integration phase.

Do not reopen perception, SLAM, costmap tuning, or navigation planning unless real STM32/downstream integration exposes a concrete blocker.

### 4.3 Navigation boundary

Nav2 command semantics:

```text
geometry_msgs/msg/Twist
linear.x  [m/s]
angular.z [rad/s]
```

Real wheel odometry is not yet available.

Current physical execution boundary:

```text
Nav2 planning works
Real motor execution does not exist yet
Real /odom does not exist yet
```

Current Cartographer ownership of:

```text
odom → base_link
```

is temporary.

When real wheel odometry is introduced, exactly one TF authority may own:

```text
odom → base_link
```

Never create duplicate TF publishers.

### 4.4 Frozen geometry

```text
base_link origin:
midpoint of left/right drive-wheel axle

+X forward
+Y left
+Z up
```

```text
base_link → camera_link
x = +0.130 m
y =  0.000 m
z = +0.110 m
roll = 0
pitch = 0
yaw = 0
```

```text
base_link → laser_frame
x = +0.043 m
y =  0.000 m
z = +0.165 m
roll = 0
pitch = 0
yaw = π
```

Frozen footprint:

```yaml
footprint: "[[0.140, 0.080], [0.140, -0.080], [-0.070, -0.080], [-0.070, 0.080]]"
footprint_padding: 0.01
```

### 4.5 Frozen Navigation v1 runtime constraints

Preserve:

```text
lifecycle_manager:
bond_timeout: 0.0
```

Preserve scan-source obstacle-height configuration:

```text
max_obstacle_height: 2.0
```

Known standalone Humble costmap shutdown-only process-destruction exit `-11` is deferred and does not invalidate runtime acceptance.

Do not investigate it without a real downstream blocker.

---

## 5. STM32 Real-Time Domain

The active project phase is:

```text
STM32 REAL-TIME CONTROL DOMAIN
```

MCU:

```text
STM32G474RET6
LQFP64
```

Board:

```text
DeveBox STM32G474R
Ver: 20
```

Main references:

```text
ST DS12288 — STM32G474xB/xC/xE datasheet
ST RM0440 — STM32G4 reference manual
```

### 5.1 Clock baseline

```text
HSE      = 8 MHz
SYSCLK   = 170 MHz
HCLK     = 170 MHz target
```

### 5.2 Frozen Pin Map v1

| Function | MCU Pin | Peripheral / Mode |
|---|---|---|
| Encoder 1 A | PA0 | TIM2_CH1 / AF1 |
| Encoder 1 B | PA1 | TIM2_CH2 / AF1 |
| Debug UART TX | PA2 | USART2_TX / AF7 |
| Debug UART RX | PA3 | USART2_RX / AF7 |
| IMU CS | PA4 | GPIO output |
| IMU SCLK | PA5 | SPI1_SCK / AF5 |
| IMU MISO | PA6 | SPI1_MISO / AF5 |
| IMU MOSI | PA7 | SPI1_MOSI / AF5 |
| Motor A PWM | PA8 | TIM1_CH1 / AF6 |
| Motor B PWM | PA9 | TIM1_CH2 / AF6 |
| CAN RX | PA11 | FDCAN1_RX / AF9 |
| CAN TX | PA12 | FDCAN1_TX / AF9 |
| SWDIO | PA13 | SWD reserved |
| SWCLK | PA14 | SWD reserved |
| Battery ADC | PC0 | ADC1_IN6 |
| IMU INT1 | PC4 | EXTI4 |
| IMU INT2 | PC5 | EXTI5 / EXTI9_5 IRQ group |
| Encoder 2 A | PC6 | TIM3_CH1 / AF2 |
| Encoder 2 B | PC7 | TIM3_CH2 / AF2 |
| TB6612 STBY | PC8 | GPIO output |
| TB6612 AIN1 | PB12 | GPIO output |
| TB6612 AIN2 | PB13 | GPIO output |
| TB6612 BIN1 | PB14 | GPIO output |
| TB6612 BIN2 | PB15 | GPIO output |

Do not remap frozen pins for stylistic reasons.

Change a frozen resource only when a real conflict or hardware failure proves the current allocation unusable.

### 5.3 Current CubeMX baseline

```text
TIM1
PWM CH1 + CH2
10 kHz
initial duty = 0

TIM2
Encoder Interface

TIM3
Encoder Interface

SPI1
ICM-42688-P

ADC1
Battery voltage

USART2
Debug
115200 8N1 baseline

FDCAN1
500 kbit/s nominal bring-up bitrate

EXTI
PC4 → EXTI4
PC5 → EXTI9_5

GPIO startup
MOTOR_STBY = LOW
AIN1/AIN2/BIN1/BIN2 = LOW
IMU_CS = HIGH
```

CAN FD production data-phase timing and message protocol are not frozen.

### 5.4 Peripheral ownership

```text
TIM1 → synchronized motor PWM
TIM2 → encoder 1 hardware quadrature decoding
TIM3 → encoder 2 hardware quadrature decoding
SPI1 → ICM-42688-P
ADC1 → battery measurement
USART2 → debug / bring-up / fallback
FDCAN1 → production transport direction
SWD → debug / flash / bring-up
```

Use hardware peripherals instead of high-rate software polling.

Do not decode quadrature encoders in GPIO polling tasks.

### 5.5 Motor driver

Motor driver:

```text
TB6612-based dual DC motor driver
```

Motor supply:

```text
12 V battery
```

Battery ADC divider:

```text
Vadc ≈ Vbattery / 11
```

The ratio must be calibrated against a multimeter before battery voltage is used for safety thresholds.

Motor A/B ownership and motor polarity must be determined experimentally.

Do not assume wheel mapping or sign.

### 5.6 IMU

```text
ICM-42688-P
SPI interface
```

Current MCU wiring:

```text
PA4 CS
PA5 SCLK
PA6 MISO
PA7 MOSI
PC4 INT1
PC5 INT2
```

The breakout exposes both `VCC` and `3.3V`; their exact board-level meaning must be physically understood before unsafe power wiring is attempted.

### 5.7 CAN transceiver

Current module:

```text
TJA1042 / TJA1043 family transceiver module
```

MCU controller:

```text
FDCAN1
```

The exact module subvariant, termination arrangement, standby behavior, and logic-level implementation remain hardware verification items.

Do not invent those facts in software or documentation.

---

## 6. Real-Time Responsibility Split

### Orange Pi

Owns:

```text
ROS 2
Perception
SLAM
Nav2
high-level goals
body velocity command
ROS bridge
odometry publication
system logging
```

### STM32

Owns:

```text
command validation
command freshness
heartbeat handling
motor enable / disable
PWM
motor direction
encoder acquisition
wheel-speed estimation
wheel velocity control
IMU acquisition
battery ADC
watchdog
fault detection
safety state machine
CAN / CAN FD
UART diagnostics
telemetry
```

Deterministic wheel control belongs on the STM32 unless a future architecture decision explicitly changes this split.

---

## 7. Differential-Drive Contract

Nav2 produces:

```text
linear.x  = body linear velocity v [m/s]
angular.z = body angular velocity ω [rad/s]
```

The intended architecture is:

```text
Orange Pi sends body command (v, ω)
→ STM32 validates command
→ STM32 performs differential-drive conversion
→ left/right wheel velocity targets
→ wheel control
```

Conceptually:

```text
v_left  = v - ω * track_width / 2
v_right = v + ω * track_width / 2
```

The following drivetrain measurements are still required before real velocity control / odometry is accepted:

```text
effective wheel radius
wheel-track distance
encoder CPR/PPR definition
gear ratio
motor A/B → wheel mapping
encoder 1/2 → wheel mapping
forward-count sign
```

Do not derive drivetrain geometry from the Nav2 collision footprint.

---

## 8. Safety Architecture

Autonomous motor motion is not allowed before a real safety layer exists.

Minimum state model:

```text
BOOT
INIT
SAFE
READY
ACTIVE
FAULT
```

Required behavior:

```text
power-up
→ motors disabled

invalid command
→ reject

stale command
→ deterministic safe stop

communication loss
→ safe stop

watchdog failure
→ safe motor state

control fault
→ safe motor state
```

`MOTOR_STBY` is a hardware safety mechanism and must remain LOW through initialization until the safety supervisor authorizes motion.

Communication software must not be the only runaway-motor protection.

---

## 9. FreeRTOS Direction

FreeRTOS is part of the STM32 architecture.

Prefer:

```text
hardware timers
encoder mode
DMA when justified
interrupts / EXTI
FDCAN hardware
SPI hardware
```

over high-frequency polling tasks.

Do not create one task per peripheral without a measured scheduling need.

Initial conceptual execution domains may later include:

```text
Safety / Supervisor
Motor Control
IMU acquisition
Communication
Telemetry
```

Task rates, priorities, stack sizes, queues, and DMA usage must be validated against real requirements.

Do not present guessed rates as final design.

---

## 10. Development Roadmap

Current ordered path:

```text
Pin Allocation v1                         FROZEN
        ↓
STM32CubeMX configuration                 CURRENT
        ↓
Clock / SWD / safe GPIO baseline
        ↓
USART2 debug bring-up
        ↓
ADC battery measurement
        ↓
TIM2 / TIM3 encoder bring-up
        ↓
SPI1 + ICM WHO_AM_I
        ↓
IMU sample / interrupt bring-up
        ↓
TIM1 10 kHz PWM verification
(motors still disabled)
        ↓
FreeRTOS minimal bring-up
        ↓
Safety supervisor
        ↓
Watchdog
        ↓
Controlled motor test
        ↓
Encoder-based wheel-speed estimation
        ↓
Closed-loop wheel velocity control
        ↓
FDCAN physical bring-up
        ↓
Shared transport protocol v1
        ↓
Orange Pi ↔ STM32 bridge
        ↓
Body velocity command execution
        ↓
Wheel odometry
        ↓
ROS nav_msgs/msg/Odometry
        ↓
Real /odom
        ↓
Deliberate odom→base_link TF authority transition
        ↓
Nav2 Controller / FollowPath
        ↓
Closed-loop physical navigation
        ↓
Obstacle avoidance
        ↓
Autonomous navigation to target
```

Do not move Localization/EKF ahead of the real-control critical path without a demonstrated requirement.

---

## 11. Autonomy Policy

The user explicitly prefers high agent autonomy.

### 11.1 Read-only operations

Within the agent's permitted code domain, shared files, known project machine, and known project resources:

```text
ALL ORDINARY READ-ONLY OPERATIONS ARE PRE-APPROVED.
```

Do not ask permission to:

- inspect files;
- search source;
- inspect Git state;
- inspect build files;
- inspect process state;
- inspect logs;
- inspect device nodes;
- inspect ROS graph state;
- inspect tool versions;
- inspect compiler output;
- inspect hardware enumeration;
- inspect `/proc` or `/sys`;
- inspect configuration;
- inspect documentation;
- query the known Orange Pi host;
- run bounded read-only diagnostics.

Examples:

```text
ls
find
rg
grep
cat
head
tail
sed -n
stat
git status
git diff
git log
git show
ros2 node list
ros2 topic list
ros2 topic echo --once
ros2 topic hz
ros2 param get
ros2 interface show
systemctl status
journalctl
lsusb
ip addr
ip route
```

If a read-only command fails:

```text
inspect error
→ try another safe read-only method
→ continue
```

Do not stop simply because one diagnostic path failed.

**This read-only authorization does not override the peer-domain isolation rule.**

Firmware Codex does not inspect `ros2_ws/`.
ROS Codex does not inspect `firmware/`.

### 11.2 Routine project-local work

Within the assigned implementation domain, Codex is authorized to perform routine, reversible engineering work needed by the active task without asking for every step.

This includes:

- create/edit project-owned source;
- create/edit build configuration;
- generate code from an already approved configuration;
- format code;
- run static analysis;
- compile/build;
- run unit tests;
- run non-actuating integration tests;
- inspect generated diffs;
- create temporary diagnostics;
- remove temporary files created by the same task;
- update relevant documentation;
- add a cross-domain `CHANGELOG.md` entry when required.

Always inspect existing files before overwriting them.

Never destroy unrelated user work.

### 11.3 ROS SSH authorization

ROS Codex may connect without conversational approval to:

```text
HwHiAiUser@robot-core.local
```

Use a normal non-root account.

Do not scan the LAN or substitute another host.

On the known Orange Pi, read-only inspection is pre-approved.

Routine project-owned work under:

```text
/data/ros2_ws
/data/projects
```

is allowed when directly required by the active task and reversible.

The actual Orange Pi runtime is the source of truth for runtime acceptance.

Do not claim a local build proves remote hardware behavior.

### 11.4 Firmware build / flash behavior

Firmware Codex may:

- edit firmware source;
- regenerate CubeMX-owned code from the current approved `.ioc`;
- build the firmware;
- inspect map/size output;
- run static checks;
- inspect SWD/debug configuration.

Firmware flashing is allowed when it is a normal step of an already approved non-actuating bring-up stage **and** the motor-safe startup contract remains intact.

Stop before flashing if the new firmware may:

- assert motor enable;
- change safety semantics;
- change option bytes / readout protection;
- alter boot configuration;
- perform irreversible flash operations;
- actuate external hardware unexpectedly.

---

## 12. Actions That Require a Decision Report

Do not interrupt the user for routine engineering.

Stop only when one of the following is reached.

### 12.1 Architecture decisions

Examples:

- MCU ↔ Linux responsibility split;
- communication transport architecture;
- CAN message model;
- public ROS topic/action contract;
- TF ownership architecture;
- package/repository boundary changes;
- drivetrain control architecture;
- localization architecture;
- sensor replacement.

### 12.2 Safety-critical behavior

Examples:

- first motor actuation of a new stage;
- autonomous motor enable;
- changing `STBY` safety policy;
- watchdog semantics;
- command-timeout policy;
- fault-recovery semantics;
- battery safety thresholds;
- emergency-stop behavior.

### 12.3 Physical electrical uncertainty

Stop when progress requires unsafe assumptions about:

- power wiring;
- module voltage compatibility;
- transceiver termination;
- motor polarity;
- encoder ownership;
- unknown pin electrical behavior.

### 12.4 Destructive / privileged / system-wide actions

Stop before:

```text
sudo
root login
system package installation/removal
OS upgrade
kernel/BSP replacement
system-wide configuration changes
permission/security changes
destructive filesystem operations
git push --force
git reset --hard
git clean -fd
```

Do not use `chmod 777`.

Do not install speculative dependencies.

### 12.5 Cross-domain contract changes

Stop before inventing or changing a frozen interface that requires coordinated firmware and ROS changes unless the current task explicitly approved that protocol evolution.

---

## 13. Decision Report Format

When a real decision gate is reached, report:

```text
DECISION REQUIRED

Observed:
<verified evidence>

Current stage:
<stage / milestone>

Why execution stopped:
<exact architecture / safety / physical / destructive decision>

Recommended action:
<one recommended option>

Alternatives:
<only if materially useful>

Impact:
<firmware / ROS / interface / safety / docs impact>

Rollback:
<how to reverse if applicable>

Decision requested:
<precise question>
```

Do not ask vague questions that can be answered by inspection.

---

## 14. Stage Completion Behavior

At the end of a stage, stop and produce a complete milestone report before advancing into a new stage that requires new direction.

Report:

```text
MILESTONE REPORT

Stage:
...

Status:
PASS / PARTIAL / FAIL

Files changed:
...

Build / test commands:
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

A module is not `COMPLETE` merely because code builds.

---

## 15. Evidence-First Acceptance

Never claim hardware completion because:

```text
CubeMX generated code
build succeeded
HAL API returned
topic exists
node exists
```

Acceptance requires real evidence.

### UART

```text
real transmitted / received bytes
```

### ADC

```text
raw ADC
+ calculated voltage
+ multimeter comparison
```

### Encoder

```text
real counter changes from physical wheel rotation
direction verified
```

### IMU

```text
WHO_AM_I
real accelerometer / gyroscope samples
interrupt behavior when used
```

### PWM

```text
scope / logic analyzer / equivalent electrical measurement
frequency verified
duty verified
motor remains disabled during non-actuating test
```

### CAN

```text
real transmitted and received frames
error state inspected
termination / bitrate physically valid
```

### Motor

```text
controlled low-risk motion
direction verified
safe stop verified
```

### Wheel control

```text
measured wheel velocity follows target
```

### Odometry

```text
physical motion produces quantitatively consistent /odom
```

### Autonomous navigation

```text
real goal
→ real path
→ real cmd_vel
→ STM32 execution
→ real odometry feedback
→ obstacle avoidance
→ target reached
```

---

## 16. Documentation Source Policy

Priority for technical truth:

```text
1. real measured system behavior
2. actual source / generated configuration
3. official manufacturer documentation
4. official upstream project documentation
5. secondary references only when necessary
```

Primary references include:

```text
STMicroelectronics:
DS12288
RM0440

TDK InvenSense:
ICM-42688-P datasheet DS-000347

Toshiba:
TB6612FNG datasheet

NXP:
TJA1042 / TJA1043 datasheets

ROS:
ROS 2 Humble / Nav2 official documentation and installed behavior
```

Do not replace observed hardware behavior with a tutorial assumption.

---

## 17. Git and Change Discipline

Before editing:

```text
inspect file
inspect relevant Git diff/status
```

After editing:

```text
build/test
review diff
```

Do not revert unrelated dirty worktree changes.

Do not perform destructive Git operations.

A local implementation detail should stay in its owning domain.

A change that affects the peer domain should be summarized in `CHANGELOG.md`.

---

## 18. Current Immediate Task

The current project boundary is:

```text
High Computing Domain:
COMPLETE THROUGH NON-ACTUATING NAVIGATION

STM32:
PIN MAP FROZEN
CUBEMX / FIRMWARE BASELINE IN PROGRESS
```

Immediate firmware path:

```text
CubeMX configuration
→ generate project
→ build
→ inspect generated initialization
→ safe SWD bring-up
→ USART2
```

Immediate ROS path:

```text
remain frozen unless needed for STM32 integration
→ prepare only when shared protocol / bridge work reaches its stage
```

Do not reopen perception work now.

---

## 19. Final System Acceptance Target

The project reaches its principal closed-loop milestone when:

```text
Navigation goal
→ Nav2 planning
→ Nav2 controller
→ geometry_msgs/msg/Twist
→ Linux-to-MCU bridge
→ CAN / CAN FD
→ STM32 command validation
→ differential-drive wheel targets
→ closed-loop motor control
→ physical robot motion
→ encoder feedback
→ wheel odometry
→ nav_msgs/msg/Odometry
→ Nav2 feedback
→ obstacle avoidance
→ target reached
```

That final behavior must be demonstrated on the real robot.

Until then, continue breadth-first integration and freeze completed subsystems.
