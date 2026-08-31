# Orange Pi / STM32 ROS Bridge

Status: **VERIFIED — physical CAN FD accepted; production startup finalized**

Chinese counterpart: [ros_bridge.zh-CN.md](ros_bridge.zh-CN.md)

## Scope

`robot_stm32_bridge` is the permanent ROS 2 Humble boundary between Linux SocketCAN and frozen
RobotProject Communication Protocol 1.0. It owns transport, host sessions, heartbeat, authority,
body-command serialization, telemetry decoding, and ROS diagnostics. Wheel control, motor safety,
sensor acquisition, watchdog behavior, and fault enforcement remain on STM32.

The package does not publish `/odom` and does not estimate drivetrain geometry.

```text
/cmd_vel -> robot_stm32_bridge -> SocketCAN can3 -> RK3588 CAN3 -> STM32
STM32 -> RK3588 CAN3 -> SocketCAN can3 -> robot_stm32_bridge -> ROS telemetry

/scan -> straight_obstacle_stop_demo -> /cmd_vel
```

## Status boundary

| Item | Status |
|---|---|
| Protocol 1.0 wire contract | `FROZEN` |
| 40-pin pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042 TXD | `FROZEN` |
| 40-pin pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042 RXD | `FROZEN` |
| CAN3 pinctrl | TX `<0x40 0x1>` / GPIO2_17 and RX `<0x44 0x1>` / GPIO2_18 (`VERIFIED`) |
| CAN3 controller | `mttcan@3`, `mttcan-id=3`, `822d0000.mttcan` (`VERIFIED`) |
| CAN3 SocketCAN interface | `can3` (`FROZEN` production default) |
| External CAN transceiver family | TJA1042T(K)/3 (`VERIFIED` user-confirmed family) |
| CANH/CANL/common-ground/two-end 120-ohm topology | `VERIFIED` by physical confirmation |
| 500 kbit/s nominal + 2 Mbit/s data + CAN FD+BRS | `VERIFIED`; Linux sample points fixed at 80% / 82.5% |
| Bidirectional Protocol 1.0 traffic | `PASS`; real `0x080`, `0x082`, and `0x180..0x183`, robot DISARMED |
| Straight obstacle-stop physical demo | `TBD` |

The production wiring is:

```text
40-pin pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042 TXD
40-pin pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042 RXD
-> mttcan@3 / mttcan-id 3 / controller base 0x822d0000
-> SocketCAN can3
```

The installed board-specific Device Tree preserves vendor board ID `0x280B` and exposes the frozen
CAN3 path. The old `822c0000.mttcan` / `can2` path is historical and deprecated because it cannot
drive the connected CAN3 pins; production has no CAN2 fallback.

Linux driver-default sample points previously produced sustained CAN errors against the accepted
STM32 timing. The verified production profile is therefore explicit: 500 kbit/s at 80.0% and
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
| `m5_commissioning.yaml` | Conservative, overrideable commissioning defaults |
| `configure_can3.sh` | Idempotent CAN3/controller/timing verification and setup |
| `robot-can3.service` | Boot-time CAN3 setup gate |
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
| `/straight_obstacle_stop_demo/start` | `std_srvs/srv/Trigger` | Explicitly starts/restarts ARMING after validating scan and bridge |
| `/straight_obstacle_stop_demo/stop` | `std_srvs/srv/Trigger` | Publishes zero, disarms, and latches STOPPED |

Bridge startup, process restart, transport reconnection, STM32 restart/session replacement,
protocol violation, status timeout, invalid Twist, and command timeout all leave authority
DISARMED. `/cmd_vel` traffic alone never arms the robot.

## Parameters

### Bridge

| Parameter | Default | Meaning |
|---|---:|---|
| `can_interface` | `can3` | Frozen production SocketCAN interface; no CAN2 fallback |
| `cmd_vel_topic` | `/cmd_vel` | Production Nav2-compatible command topic |
| `imu_frame_id` | `imu_link` | Frame label only; no transform is created |
| `command_timeout_ms` | `100` | Host timeout, deliberately inside STM32's frozen 250 ms deadline |
| `status_timeout_ms` | `350` | Maximum accepted SYSTEM_STATUS age |
| `reconnect_period_ms` | `1000` | Socket reopen interval after transport failure |

### Demo commissioning defaults

| Parameter | Default | Meaning |
|---|---:|---|
| `forward_speed_mps` | `0.05` | Low commissioning forward speed |
| `stop_distance_m` | `0.35` | Front obstacle trigger distance |
| `front_sector_deg` | `30.0` | Full angular width of evaluated sector |
| `front_center_deg` | `180.0` | Robot-forward in the frozen yaw-pi `laser_frame` |
| `scan_timeout_ms` | `400` | Stale-scan fail-safe deadline |
| `bridge_timeout_ms` | `300` | Stale bridge-status fail-safe deadline |

These values are commissioning defaults, not frozen safety specifications. Override them through a
launch parameter file or `--ros-args -p name:=value` only after checking the real test setup.
The firmware accepts at most 0.30 m/s body-linear speed in this commissioning build; the existing
0.05 m/s demo default remains the required first straight-drive acceptance speed and may only be
raised after physical closed-loop evidence.

## Protocol behavior

The bridge uses Standard-ID CAN FD frames with BRS and explicit little-endian serialization.

| Direction | CAN IDs |
|---|---|
| Host to STM32 | `0x080` authority at 10 Hz, `0x081` body command at 50 Hz while controlling, `0x082` heartbeat at 10 Hz |
| STM32 to host | `0x180` status, `0x181` wheel, `0x182` IMU, `0x183` battery |

Every process/control session uses a new nonzero 32-bit session identifier and independent 16-bit
sequence streams. Arming requires the frozen transaction:

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

## Demo state machine

```text
IDLE --explicit start--> ARMING --authority confirmed--> DRIVING
  ^                         |                              |
  |                         +--invalid/stale------------> FAULT
  |                                                        |
  +--new explicit start <--- STOPPED <---obstacle----------+
```

IDLE, ARMING, STOPPED, and FAULT continuously publish zero Twist. An obstacle at or below the
configured distance causes an immediate zero publication followed by bridge disarm. STOPPED is
latched when the obstacle disappears. Only another explicit `~/start` may begin a new run.

`DemoStatus` records obstacle detection, zero-Twist publication, disarm request, bridge authority
withdrawal transmission, and safe STM32 status confirmation timestamps. These measure software and
command-path timing only; they are not physical stopping-distance evidence.

## Production startup

The version-controlled `robot-can3.service` runs the idempotent `configure_can3.sh` before the
bridge service. If `can3` already has the verified controller mapping, MTU, timing, FD state, and
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

## Build and non-actuating acceptance

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

Bridge-only launch remains DISARMED. Physical-link acceptance is complete: the production bridge
exchanged the required host and STM32 frames with Protocol 1.0 valid and no abnormal CAN errors.

First motion remains a separate safety decision gate. Do not call the demo `~/start` service until
STM32 reports BODY_COMMAND_READY and the physical commissioning area, supports, obstacle, and
emergency intervention are deliberately prepared.
