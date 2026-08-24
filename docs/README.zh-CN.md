# RobotProject 项目文档

[English](README.md)

`docs/` 是 RobotProject 的长期工程知识库。

## 文档索引

| 文档 | 语言 | 内容 |
|---|---|---|
| [Hardware Baseline and STM32 Pin Map](hardware.md) | English | Hardware Model、Pin Map、Peripheral Ownership、Geometry |
| [硬件基线与 STM32 Pin Map](hardware.zh-CN.md) | 中文 | 硬件型号、Pin Map、外设归属、机器人几何 |
| `README.md` | English | 文档索引与维护规则 |
| `README.zh-CN.md` | 中文 | 中文索引与维护规则 |

## 状态定义

| 状态 | 使用场景 |
|---|---|
| `FROZEN` | 已验收并在后续阶段保持稳定 |
| `VERIFIED` | 已有真实系统证据支持 |
| `IN PROGRESS` | 当前集成项 |
| `PROPOSED` | 等待验证的候选方案 |
| `TBD` | 有意保留的未决项 |

## 知识库更新触发条件

以下信息获得真实结果后，同步更新对应中英文文档：

| 类别 | 典型内容 |
|---|---|
| Drivetrain | Wheel Radius、Wheel Track、Gear Ratio |
| Encoder | CPR/PPR、方向、左右轮归属 |
| Motor | A/B 归属、方向、Safe Stop |
| IMU | 配置、标定、Interrupt Behavior |
| Battery | ADC Calibration、Voltage Conversion |
| CAN / CAN FD | Bit Timing、ID、Payload、Timeout Semantics |
| Safety | State Machine、Watchdog、Fault Recovery |
| Odometry | Wheel Model、TF Ownership、Accuracy |
| Navigation | Real FollowPath、Obstacle Avoidance Evidence |
| Performance | Rate、Latency、Tracking Error、CPU Load |

## 文档格式

统一使用：

- 简洁、工程化的技术表达；
- 表格呈现稳定结构化事实；
- 树状图呈现目录与层级；
- 所有物理量明确单位；
- Pin、Peripheral、Topic、Frame、Register、Protocol Name 保持精确；
- 实测数据附带验收上下文。

假设项保持 `PROPOSED` 或 `TBD`，获得真实证据后再升级为 `VERIFIED` 或 `FROZEN`。
