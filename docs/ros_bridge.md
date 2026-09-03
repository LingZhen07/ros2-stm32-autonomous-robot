# Orange Pi / STM32 ROS Bridge

Chinese counterpart: [ros_bridge.zh-CN.md](ros_bridge.zh-CN.md)

## Scope

`robot_stm32_bridge` is the ROS 2 Humble boundary between Linux SocketCAN and
RobotProject Communication Protocol 1.0. It owns transport, host sessions, heartbeat, authority,
body-command serialization, telemetry decoding, and ROS diagnostics. Wheel control, motor safety,
sensor acquisition, watchdog behavior, and fault enforcement remain on STM32.

The package does not publish `/odom` and does not estimate drivetrain geometry.

```text
/cmd_vel -> robot_stm32_bridge -> SocketCAN can3 -> RK3588 CAN3 -> STM32
STM32 -> RK3588 CAN3 -> SocketCAN can3 -> robot_stm32_bridge -> ROS telemetry

/scan -> straight_obstacle_stop_demo -> /cmd_vel
```

## CAN FD configuration

| Item | Configuration / observation |
|---|---|
| Wire contract | Protocol 1.0 |
| Orange Pi TX | 40-pin pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042T(K)/3 TXD |
| Orange Pi RX | 40-pin pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042T(K)/3 RXD |
| CAN3 pinctrl | TX `<0x40 0x1>` / GPIO2_17; RX `<0x44 0x1>` / GPIO2_18 |
| CAN3 controller | `mttcan@3`, `mttcan-id=3`, `822d0000.mttcan` |
| SocketCAN interface | `can3` |
| CAN transceiver | TJA1042T(K)/3 |
| Bus topology | CANH/CANL, common ground, one 120-ohm terminator at each end |
| CAN FD | 500 kbit/s nominal, 2 Mbit/s data, BRS; Linux sample points 80% / 82.5% |
| Observed traffic | Bidirectional `0x080`, `0x082`, and `0x180..0x183`; robot DISARMED |

The production wiring is:

```text
40-pin pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042T(K)/3 TXD
40-pin pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042T(K)/3 RXD
-> mttcan@3 / mttcan-id 3 / controller base 0x822d0000
-> SocketCAN can3
```

The installed board-specific Device Tree preserves vendor board ID `0x280B` and exposes CAN3. The
old `822c0000.mttcan` / `can2` path is historical and cannot drive the connected CAN3 pins; there is
no CAN2 fallback.

Linux driver-default sample points previously produced sustained CAN errors against the STM32
timing. Startup therefore sets 500 kbit/s at 80.0% and
2 Mbit/s at 82.5%. With this profile the real bus is Error Active with TX error 0, RX error 0,
bus-errors 0, and bus-off 0.

## Package contents

| Component | Purpose |
|---|---|
| `robot_stm32_bridge_node` | Production SocketCAN and Protocol 1.0 bridge |
| `straight_obstacle_stop_demo` | Explicit-start, straight-drive, LiDAR stop commissioning node |
| `BridgeStatus.msg` | Structured bridge, session, protocol, CAN, and STM32 safety state |
| `WheelState.msg` | Logical and raw wheel/encoder telemetry with explicit validity |
| `DemoStatus.msg` | Latched demo state and stop-latency timestamps |
| `bridge.launch.py` | DISARMED bridge-only bring-up |
| `straight_obstacle_stop_demo.launch.py` | Bridge plus commissioning demo |
| `drivetrain_commissioning.yaml` | Conservative, overrideable defaults used by the drivetrain and obstacle-stop demo |
| `configure_can3.sh` | Idempotent CAN3/controller/timing verification and setup |
| `robot-can3.service` | Boot-time CAN3 availability gate |
| `robot-stm32-bridge.service` | Starts the DISARMED bridge only after CAN3 is ready |

## ROS interfaces

### Inputs and outputs

| Name | Type | Direction | Meaning |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | bridge input | `linear.x` m/s and `angular.z` rad/s only |
| `/stm32/status` | `robot_stm32_bridge/msg/BridgeStatus` | bridge output | Connection, session, safety, fault, counters, and freshness |
| `/stm32/wheel_state` | `robot_stm32_bridge/msg/WheelState` | bridge output | Counts/s, targets, effort, raw timers, ages, validity flags |
| `/stm32/imu_raw` | `sensor_msgs/msg/Imu` | bridge output | Acceleration and angular velocity supplied by STM32 |
| `/stm32/battery` | `sensor_msgs/msg/BatteryState` | bridge output | Voltage only when valid; no invented percentage/threshold |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | bridge output | Low-rate protocol/CAN/STM32 health |
| `/scan` | `sensor_msgs/msg/LaserScan` | demo input | Real RPLIDAR obstacle source |
| `/straight_obstacle_stop_demo/status` | `robot_stm32_bridge/msg/DemoStatus` | demo output | State, front range, stop timestamps, software latencies |

IMU orientation is unavailable. The bridge sets `orientation_covariance[0] = -1` and never
fabricates orientation. Unavailable acceleration or angular-velocity groups use the corresponding
covariance first element `-1`. `BatteryState.percentage` and unavailable electrical fields remain
NaN. STM32 monotonic timestamps are retained in project messages; ROS headers use receive time
adjusted by the supplied sample age where available.

### Services

| Service | Type | Behavior |
|---|---|---|
| `/robot_stm32_bridge/arm` | `std_srvs/srv/Trigger` | Requests authority only after all health/readiness gates pass |
| `/robot_stm32_bridge/disarm` | `std_srvs/srv/Trigger` | Withdraws authority; sends DISABLED first only if motion had been explicitly requested |
| `/straight_obstacle_stop_demo/start` | `std_srvs/srv/Trigger` | Explicitly requests a RUNNING episode after validating scan and bridge; authority acknowledgement is required |
| `/straight_obstacle_stop_demo/stop` | `std_srvs/srv/Trigger` | Publishes zero, disarms, and latches STOPPED |

Bridge startup, process restart, transport reconnection, STM32 restart/session replacement,
protocol violation, status timeout, invalid Twist, and command timeout all leave authority
DISARMED. `/cmd_vel` traffic alone never arms the robot.

## Parameters

### Bridge

| Parameter | Default | Meaning |
|---|---:|---|
| `can_interface` | `can3` | SocketCAN interface; no CAN2 fallback |
| `cmd_vel_topic` | `/cmd_vel` | Production Nav2-compatible command topic |
| `imu_frame_id` | `imu_link` | Frame label only; no transform is created |
| `command_timeout_ms` | `100` | Host timeout, deliberately inside STM32's 250 ms deadline |
| `status_timeout_ms` | `350` | Maximum accepted SYSTEM_STATUS age |
| `reconnect_period_ms` | `1000` | Socket reopen interval after transport failure |

### Demo commissioning defaults

| Parameter | Default | Meaning |
|---|---:|---|
| `forward_speed_mps` | `0.30` | Highest currently justified commissioning speed; cannot exceed 0.30 m/s |
| `stop_distance_m` | `0.60` | Conservative commissioning obstacle threshold at 0.30 m/s |
| `front_sector_deg` | `30.0` | Full angular width of evaluated sector |
| `front_center_deg` | `180.0` | Robot-forward in the yaw-pi `laser_frame` |
| `scan_timeout_ms` | `400` | Maximum steady-clock receipt age and ROS source-timestamp age |
| `bridge_timeout_ms` | `300` | Stale bridge-status fail-safe deadline |

These values are commissioning defaults, not robot safety specifications. Override them through a
launch parameter file or `--ros-args -p name:=value` only after checking the real test setup.
The firmware accepts at most 0.30 m/s body-linear speed in this commissioning build. The 0.60 m
threshold is deliberately conservative but is not a measured robot safety distance: physical
stopping distance at 0.30 m/s must be measured before reducing it or defining a safety limit.

## Protocol behavior

The bridge uses Standard-ID CAN FD frames with BRS and explicit little-endian serialization.

| Direction | CAN IDs |
|---|---|
| Host to STM32 | `0x080` authority at 10 Hz, `0x081` body command at 50 Hz while controlling, `0x082` heartbeat at 10 Hz |
| STM32 to host | `0x180` status, `0x181` wheel, `0x182` IMU, `0x183` battery |

Every process/control session uses a new nonzero 32-bit session identifier and independent 16-bit
sequence streams. Arming requires this transaction:

```text
HOST_HEARTBEAT(BRIDGE_READY)
-> MOTION_AUTHORITY(DISARMED)
-> STM32 session + DISARMED handshake acknowledgement
-> explicit ROS arm request
-> MOTION_AUTHORITY(ARMED)
-> fresh MOTION_COMMAND(BODY_VELOCITY)
```

Wrong length, non-FD/BRS envelope, extended/remote frame, reserved-field violation, or a version
other than exactly 1.0 is rejected and blocks motion. Sequence gaps, duplicates, and old frames are
counted. Recognized CAN errors, bus-off, socket failure, stale status, critical STM32 faults, and
invalid commands converge to the safe path.

## Motion safety and obstacle-stop state

The continuing motion-safety contract is:

```text
explicit START
-> Motion Authority granted
-> fresh motion commands
-> motion allowed
```

Motion is disabled by default, and Motion Authority is independent from velocity commands. Stopping
uses layered withdrawal of motion authority:

```text
obstacle / stale command / communication loss / fault / explicit STOP
-> zero motion command where available
-> Motion Authority withdrawn
-> STM32 motor-safe stop
```

Loss or withdrawal of authority stops motion. Command freshness, heartbeat/watchdog, and fault
handling remain independent lower-level protections. The obstacle-stop node implements the
following latched operator-visible states:

```text
STOPPED --explicit START + authority confirmed--> RUNNING
   ^                                                |
   +--obstacle / stale / fault / explicit STOP------+
```

Only `STOPPED` and `RUNNING` are semantic states. While the asynchronous authority request is
pending, the node remains STOPPED and publishes zero. A valid finite front-sector sample at or below
the configured distance causes zero Twist publication and bridge disarm in the same 20 ms control
cycle. STOPPED remains latched after obstacle removal. Only another explicit `~/start` may begin a
new run; stale commands, old authority, and obstacle removal cannot restart motion.

The scan motion gate requires both a usable scan received within `scan_timeout_ms` on the steady
clock and a nonzero source `header.stamp` no older than the same timeout on the node's active ROS
clock. A fixed 50 ms future tolerance permits minor scheduling/clock skew; timestamps farther in the
future, zero timestamps, and timestamps that cannot be meaningfully compared with the active ROS
clock are invalid. Receipt timeout, stale source time, and invalid source time all use the existing
zero Twist, authority-withdrawal, and latched STOPPED path. Valid scans returning never auto-resume.

On 2026-09-02, with the real RPLIDAR A1 at 10 Hz,
`ros2 topic delay /scan` measured approximately 0.133-0.134 s average source age over a 10-sample
window (approximately 0.123-0.136 s observed range). The production predicate reported
`scan_fresh: true` while the node remained STOPPED and unarmed; no START request was issued.

`DemoStatus` records obstacle detection, zero-Twist publication, disarm request, bridge authority
withdrawal transmission, and safe STM32 status confirmation timestamps. These measure software and
command-path timing only; they are not physical stopping-distance evidence.

## Production startup

The version-controlled `robot-can3.service` runs the idempotent `configure_can3.sh` before the
bridge service. If `can3` already has the configured controller mapping, MTU, timing, FD state, and
Error Active state, the script makes no link change. Otherwise it performs:

```bash
ip link set dev can3 down
ip link set dev can3 type can \
  bitrate 500000 sample-point 0.800 \
  dbitrate 2000000 dsample-point 0.825 \
  fd on berr-reporting on
ip link set dev can3 up
```

CAN FD BRS is set on each Protocol 1.0 frame by the bridge. The setup script refuses any mapping
other than `can3 -> 822d0000.mttcan`, verifies MTU 72 and Error Active, and never falls back to
CAN2. `robot-stm32-bridge.service` requires successful CAN setup and then starts the bridge as
`HwHiAiUser`.

After building the package, install and enable the version-controlled units once:

```bash
sudo install -m 0644 \
  /data/ros2_ws/install/robot_stm32_bridge/share/robot_stm32_bridge/systemd/robot-can3.service \
  /etc/systemd/system/robot-can3.service
sudo install -m 0644 \
  /data/ros2_ws/install/robot_stm32_bridge/share/robot_stm32_bridge/systemd/robot-stm32-bridge.service \
  /etc/systemd/system/robot-stm32-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable --now robot-can3.service robot-stm32-bridge.service
```

The bridge starts DISARMED, sends no `0x081` unless motion was explicitly requested, and inhibits
automatic socket reconnect after a runtime transport write/receive failure to prevent a persistent
ENOBUFS loop. Repair `can3`, then restart the bridge service to establish a new safe session.

## Build and non-actuating checks

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
cd /data/ros2_ws
colcon build --packages-select robot_stm32_bridge --symlink-install
source install/setup.bash
colcon test --packages-select robot_stm32_bridge
colcon test-result --verbose
ros2 launch robot_stm32_bridge bridge.launch.py
```

Bridge-only launch remains DISARMED. The bridge exchanged the required host and STM32 frames with
Protocol 1.0 valid and no abnormal CAN errors.

Future drivetrain commissioning runs must confirm BODY_COMMAND_READY and deliberately prepare the
commissioning area, obstacle, and emergency
intervention before calling the demo `~/start` service.

## Obstacle-stop physical demonstration

The systemd service already owns the single production bridge. The demo launch therefore starts
only `straight_obstacle_stop_demo`:

```bash
source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
ros2 launch robot_stm32_bridge straight_obstacle_stop_demo.launch.py
```

After confirming the real `/scan` is active and the commissioning area is prepared, START and STOP
are explicit:

```bash
ros2 service call /straight_obstacle_stop_demo/start std_srvs/srv/Trigger '{}'
ros2 service call /straight_obstacle_stop_demo/stop std_srvs/srv/Trigger '{}'
ros2 topic echo /straight_obstacle_stop_demo/status --once
```

The demonstrated sequence was explicit START, continuous 0.30 m/s closed-loop straight
motion, a real RPLIDAR A1 `/scan` obstacle inside the 30° frontal sector at 0.60 m, zero velocity and
Motion Authority withdrawal, STM32 motor-safe stop, obstacle removal with STOPPED still latched, then a
new explicit START. The STOP service is the operator's immediate ROS-side withdrawal command; STM32
safety remains authoritative.

Observed from obstacle detection in the successful demo:

| Confirmation | Observed interval |
|---|---:|
| Zero velocity command | approximately `0.075 ms` |
| Motion Authority withdrawal | approximately `1.276 ms` |
| STM32 stop confirmation | approximately `30.8 ms` |

These are observed measurements from the demonstration, not guaranteed worst-case safety limits.
The 0.30 m/s value is the current commissioning limit, not the robot's
physical maximum.
