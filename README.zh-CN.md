# ros2-stm32-autonomous-robot

[English](README.md)

这是一套异构自主移动机器人系统：Orange Pi AI Pro 运行 ROS 2 Humble，STM32G474 负责实时
控制。系统将 LiDAR/RGB-D 感知、SLAM 与 Nav2，同 CAN FD 底盘桥接、轮速闭环控制和分层运动
安全机制集成在一起。

## 技术栈

| 域 | 硬件 / 软件 | 职责 |
|---|---|---|
| 高算力域 | Orange Pi AI Pro 8GB、Ubuntu 22.04、ROS 2 Humble | 感知、SLAM、Nav2、命令监督与日志 |
| AI 加速 | Ascend 310B4 | YOLOv8n 推理 |
| 实时控制域 | STM32G474RET6、FreeRTOS、固件 `0.5.4` | 命令校验、安全、轮速控制、传感器与遥测 |
| 环境感知 | RPLIDAR A1、Astra RGB-D | 真实 `/scan`、RGB 与深度输入 |
| 底盘 | TB6612、双路直流电机、正交编码器 | 差速驱动与反馈 |
| 传输 | TJA1042/TJA1043 系列收发器、SocketCAN `can3` | 已冻结 Protocol 1.0 CAN FD 链路 |

## 系统架构

```text
RPLIDAR / RGB-D ──> ROS 2 感知、SLAM 与 Nav2 ──> geometry_msgs/Twist
                                                       │
                                                       v
                              Orange Pi 安全/Demo 监督 + CAN FD Bridge
                                                       │
                                           can3 上的 Protocol 1.0
                                                       │
                                                       v
                              STM32 安全 + 差速转换 + 左右轮闭环控制
                                                       │
                                                       v
                                      TB6612 + 电机 + 编码器反馈
```

生产 CAN FD 接口为 `can3`：Nominal 500 kbit/s、Sample Point `0.800`；Data 2 Mbit/s、Data
Sample Point `0.825`。Protocol 1.0 保持冻结，真实硬件已验证 `BODY_COMMAND_READY = TRUE`。

## 已演示能力

- Orange Pi AI Pro ROS 2 Humble 高算力栈与 RPLIDAR A1 真实 `/scan` 输入。
- 真实 CAN3 双向物理链路与 Protocol 1.0 Motion Authority 握手。
- STM32G474 固件 `0.5.4` 左右轮闭环控制与确定性 Safe Stop。
- 当前 `0.30 m/s` Commissioning Limit 下的直行运动验收。
- 30° 前方 LiDAR 扇区与 `0.60 m` 停车阈值。
- 实机 Obstacle STOP、Zero Velocity、Motion Authority Withdrawal、STM32 Stop Confirmation 和
  STOPPED 锁存行为全部通过。

`0.30 m/s` 是当前电气与供电约束下已验收的最高 Commissioning Speed，不代表机器人物理
最高速度。

## M5 实机验收

**状态：PASS**

```text
显式 START
→ 0.30 m/s 直行闭环运动
→ RPLIDAR 前方障碍物检测
→ Zero Velocity + Motion Authority Withdrawal
→ STM32 Safe Stop
→ STOPPED 锁存
→ 必须显式 START 才能再次运动
```

障碍物移除后 STOP 仍保持锁存；只有用户重新发出显式 START，系统才能恢复运动。

成功 Demo 中的观测值：

| 事件区间 | 观测时间 |
|---|---:|
| Obstacle Detection → Zero Velocity Command | 约 `0.075 ms` |
| Obstacle Detection → Motion Authority Withdrawal | 约 `1.276 ms` |
| Obstacle Detection → STM32 Stop Confirmation | 约 `30.8 ms` |

以上数据是本次成功实机验收的观测测量值，不是有保证的最坏情况安全上限。

## 文档与设置

| 资源 | 内容 |
|---|---|
| [文档索引](docs/README.zh-CN.md) | 中英双语工程知识库 |
| [ROS Bridge 与 M5 Demo](docs/ros_bridge.zh-CN.md) | 构建、生产 CAN3 启动、ROS 接口、Demo 流程与证据 |
| [STM32 生产固件](docs/firmware.zh-CN.md) | 固件构建、架构、安全与验收边界 |
| [硬件基线](docs/hardware.zh-CN.md) | 硬件定义与冻结的 STM32 Pin Map |
| [Protocol 1.0](interfaces/protocol_v1.md) | 冻结的 Linux ↔ STM32 线协议 |

克隆正式仓库：

```bash
git clone git@github.com:LingZhen07/ros2-stm32-autonomous-robot.git
cd ros2-stm32-autonomous-robot
```

内部 ROS Package、Firmware Module、Executable 与冻结的 Protocol Identifier 保留原名；GitHub
仓库更名不会改变运行时合同。

## License

Copyright (c) 2026 Ling Zhen。本项目采用 [MIT License](LICENSE)。
