# Orange Pi / STM32 ROS 桥接

状态：**VERIFIED — 物理 CAN FD 已验收，生产启动配置已完成**

English: [ros_bridge.md](ros_bridge.md)

## 范围

`robot_stm32_bridge` 是 Linux SocketCAN 与已冻结的 RobotProject Communication Protocol 1.0
之间的长期 ROS 2 Humble 边界。它负责传输、主机会话、心跳、运动授权、车体命令序列化、
遥测解码和 ROS 诊断。轮速控制、电机安全、传感器采集、看门狗和故障执行仍由 STM32 负责。

该包不发布 `/odom`，也不推测底盘几何参数。

```text
/cmd_vel -> robot_stm32_bridge -> SocketCAN can3 -> RK3588 CAN3 -> STM32
STM32 -> RK3588 CAN3 -> SocketCAN can3 -> robot_stm32_bridge -> ROS telemetry

/scan -> straight_obstacle_stop_demo -> /cmd_vel
```

## 当前边界

| 项目 | 状态 |
|---|---|
| Protocol 1.0 线协议 | `FROZEN` |
| 40Pin Pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042 TXD | `FROZEN` |
| 40Pin Pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042 RXD | `FROZEN` |
| CAN3 Pinctrl | TX `<0x40 0x1>` / GPIO2_17，RX `<0x44 0x1>` / GPIO2_18（`VERIFIED`） |
| CAN3 Controller | `mttcan@3`、`mttcan-id=3`、`822d0000.mttcan`（`VERIFIED`） |
| CAN3 SocketCAN Interface | `can3`（`FROZEN` Production Default） |
| 外部 CAN Transceiver 系列 | TJA1042T(K)/3（用户确认的系列，`VERIFIED`） |
| CANH/CANL、共地、总线两端各 120 Ω | 已物理确认，`VERIFIED` |
| Nominal 500 kbit/s + Data 2 Mbit/s + CAN FD+BRS | `VERIFIED`；Linux Sample Point 固定 80% / 82.5% |
| 双向 Protocol 1.0 通信 | `PASS`；真实 `0x080`、`0x082`、`0x180..0x183`，机器人 DISARMED |
| 直行遇障停车实车演示 | `TBD` |

生产接线为：

```text
40Pin Pin 36 -> GPIO2_17 / CAN_TX3 -> TJA1042 TXD
40Pin Pin 11 -> GPIO2_18 / CAN_RX3 <- TJA1042 RXD
-> mttcan@3 / mttcan-id 3 / Controller Base 0x822d0000
-> SocketCAN can3
```

已安装的 Board-specific Device Tree 保留 Vendor Board ID `0x280B` 并暴露冻结 CAN3 路径。
旧 `822c0000.mttcan` / `can2` 仅为历史且已弃用，因为它不能驱动已接线的 CAN3 Pin；生产环境
不存在 CAN2 Fallback。

Linux Driver 默认 Sample Point 曾与 STM32 Timing 不匹配并产生持续 CAN Error。最终验证的
Production Profile 显式固定 500 kbit/s @ 80.0% 与 2 Mbit/s @ 82.5%。真实总线保持 Error Active，
TX Error=0、RX Error=0、Bus Error=0、Bus-off=0。

## 包结构

| 组件 | 用途 |
|---|---|
| `robot_stm32_bridge_node` | 生产 SocketCAN / Protocol 1.0 桥接 |
| `straight_obstacle_stop_demo` | 显式启动、直行、LiDAR 停车调试节点 |
| `BridgeStatus.msg` | 桥接、会话、协议、CAN 和 STM32 安全状态 |
| `WheelState.msg` | 带显式有效性的逻辑/原始车轮编码器遥测 |
| `DemoStatus.msg` | 锁存的演示状态和停车时延时间戳 |
| `bridge.launch.py` | 仅启动 DISARMED 桥接 |
| `straight_obstacle_stop_demo.launch.py` | 桥接和调试演示 |
| `m5_commissioning.yaml` | 保守且可覆盖的调试默认值 |
| `configure_can3.sh` | 幂等验证并配置 CAN3 Controller/Timing |
| `robot-can3.service` | 开机 CAN3 Ready Gate |
| `robot-stm32-bridge.service` | 仅在 CAN3 Ready 后启动 DISARMED Bridge |

## ROS 接口

### Topic

| 名称 | 类型 | 方向 | 含义 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 桥接输入 | 仅支持 `linear.x` m/s 和 `angular.z` rad/s |
| `/stm32/status` | `robot_stm32_bridge/msg/BridgeStatus` | 桥接输出 | 连接、会话、安全、故障、计数器和新鲜度 |
| `/stm32/wheel_state` | `robot_stm32_bridge/msg/WheelState` | 桥接输出 | 计数率、目标、控制输出、原始计数器、年龄和有效性 |
| `/stm32/imu_raw` | `sensor_msgs/msg/Imu` | 桥接输出 | STM32 提供的加速度和角速度 |
| `/stm32/battery` | `sensor_msgs/msg/BatteryState` | 桥接输出 | 仅在有效时提供电压，不虚构百分比或阈值 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 桥接输出 | 低频协议/CAN/STM32 健康状态 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 演示输入 | 真实 RPLIDAR 障碍物来源 |
| `/straight_obstacle_stop_demo/status` | `robot_stm32_bridge/msg/DemoStatus` | 演示输出 | 状态、前方距离、停车时间戳和软件时延 |

STM32 不提供姿态，因此桥接设置 `orientation_covariance[0] = -1`，不会虚构 Orientation。
加速度或角速度不可用时，对应 covariance 的第一项设为 `-1`。`BatteryState.percentage`
等不可用量保持 NaN。项目消息保留 STM32 单调时间戳；如果帧提供 Sample Age，ROS Header
使用接收时间减去该 Age。

### Service

| Service | 类型 | 行为 |
|---|---|---|
| `/robot_stm32_bridge/arm` | `std_srvs/srv/Trigger` | 仅在所有健康/就绪门通过后请求运动授权 |
| `/robot_stm32_bridge/disarm` | `std_srvs/srv/Trigger` | 撤销 Authority；仅在曾显式请求 Motion 时先发 DISABLED |
| `/straight_obstacle_stop_demo/start` | `std_srvs/srv/Trigger` | 检查 Scan/Bridge 后显式启动或重新启动 ARMING |
| `/straight_obstacle_stop_demo/stop` | `std_srvs/srv/Trigger` | 发布零速度、Disarm 并锁存 STOPPED |

桥接启动、进程重启、传输重连、STM32 重启或会话替换、协议违规、状态超时、无效 Twist
和命令超时都会收敛到 DISARMED。仅存在 `/cmd_vel` 永远不会自动 Arm。

## 参数

### 桥接

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `can_interface` | `can3` | 冻结 Production SocketCAN Interface；无 CAN2 Fallback |
| `cmd_vel_topic` | `/cmd_vel` | 与未来 Nav2 兼容的生产命令 Topic |
| `imu_frame_id` | `imu_link` | 仅为 Frame 标签，不创建 TF |
| `command_timeout_ms` | `100` | 主机超时，显著小于 STM32 已冻结的 250 ms 截止时间 |
| `status_timeout_ms` | `350` | SYSTEM_STATUS 允许的最大年龄 |
| `reconnect_period_ms` | `1000` | 传输故障后的 Socket 重连周期 |

### 演示调试默认值

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `forward_speed_mps` | `0.05` | 低速调试前进速度 |
| `stop_distance_m` | `0.35` | 前方障碍物触发距离 |
| `front_sector_deg` | `30.0` | 前方检测扇区总角宽 |
| `front_center_deg` | `180.0` | 已冻结 yaw=π 的 `laser_frame` 中，机器人前方对应的角度 |
| `scan_timeout_ms` | `400` | Scan 过期的故障安全截止时间 |
| `bridge_timeout_ms` | `300` | Bridge Status 过期的故障安全截止时间 |

这些是调试默认值，不是已冻结的机器人安全规格。只有在检查真实测试环境后，才能通过
Launch 参数文件或 `--ros-args -p name:=value` 覆盖。
该调试固件最多接受 0.30 m/s Body Linear Speed；现有 0.05 m/s Demo 默认值仍是首次直行验收
要求的速度，只有取得真实闭环证据后才能提高。

## 协议行为

桥接使用 Standard-ID CAN FD+BRS 和显式小端序列化。

| 方向 | CAN ID |
|---|---|
| 主机到 STM32 | `0x080` Authority 10 Hz；控制期间 `0x081` Body Command 50 Hz；`0x082` Heartbeat 10 Hz |
| STM32 到主机 | `0x180` Status；`0x181` Wheel；`0x182` IMU；`0x183` Battery |

每个进程/控制会话使用新的非零 32 位 Session ID 和独立的 16 位 Sequence Stream。Arm
必须完成已冻结的事务：

```text
HOST_HEARTBEAT(BRIDGE_READY)
-> MOTION_AUTHORITY(DISARMED)
-> STM32 Session + DISARMED Handshake ACK
-> 显式 ROS Arm 请求
-> MOTION_AUTHORITY(ARMED)
-> Fresh MOTION_COMMAND(BODY_VELOCITY)
```

错误长度、非 FD+BRS 包络、扩展帧或远程帧、保留字段违规、任何非 1.0 版本都会被拒绝并
阻断运动。Sequence Gap、Duplicate、Old Frame 会计数。CAN Error、Bus-off、Socket Failure、
Stale Status、STM32 Critical Fault 和 Invalid Command 都进入安全路径。

## 演示状态机

```text
IDLE --显式 Start--> ARMING --Authority 确认--> DRIVING
  ^                       |                         |
  |                       +--Invalid/Stale-------> FAULT
  |                                                 |
  +--新的显式 Start <----- STOPPED <---Obstacle----+
```

IDLE、ARMING、STOPPED 和 FAULT 持续发布 Zero Twist。前方障碍物距离小于等于设定值时，
立即发布 Zero，然后调用 Bridge Disarm。障碍物移除后，STOPPED 仍保持锁存；只有新的显式
`~/start` 才能开始下一次运行。

`DemoStatus` 记录障碍物检测、Zero Twist 发布、Disarm 请求、Bridge Authority Withdrawal
发送以及 STM32 Safe Status 确认时间戳。这些仅为软件/命令路径时序，不是物理停车距离证据。

## Production Startup

版本控制内的 `robot-can3.service` 在 Bridge 前运行幂等 `configure_can3.sh`。若 Controller 映射、
MTU、Timing、FD 和 Error Active 均正确，脚本不改变 Link；否则执行：

```bash
ip link set dev can3 down
ip link set dev can3 type can \
  bitrate 500000 sample-point 0.800 \
  dbitrate 2000000 dsample-point 0.825 \
  fd on berr-reporting on
ip link set dev can3 up
```

BRS 由 Bridge 对每个 Protocol 1.0 CAN FD Frame 设置。脚本拒绝任何非
`can3 -> 822d0000.mttcan` 映射，验证 MTU 72 / Error Active，且绝不回退 CAN2。

Build 后一次性安装并启用：

```bash
sudo install -m 0644 /data/ros2_ws/install/robot_stm32_bridge/share/robot_stm32_bridge/systemd/robot-can3.service /etc/systemd/system/robot-can3.service
sudo install -m 0644 /data/ros2_ws/install/robot_stm32_bridge/share/robot_stm32_bridge/systemd/robot-stm32-bridge.service /etc/systemd/system/robot-stm32-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable --now robot-can3.service robot-stm32-bridge.service
```

Bridge 启动保持 DISARMED；未显式请求运动时不发送 `0x081`。运行时 Transport Write/Receive
故障后会禁止自动 Socket Reconnect，避免持续 ENOBUFS；修复 `can3` 后重启 Bridge Service，
以新安全 Session 恢复。

## 构建和非执行器验收

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

Bridge-only Launch 始终保持 DISARMED。物理链路已通过 Production Bridge 验收：Host/STM32
必需帧双向交换，Protocol 1.0 有效且 CAN Error 无异常。

第一次运动仍是独立的安全决策门。在 STM32 报告 BODY_COMMAND_READY，并完成物理测试区域、
支撑、障碍物和紧急人工干预准备之前，不得调用演示 `~/start`。
