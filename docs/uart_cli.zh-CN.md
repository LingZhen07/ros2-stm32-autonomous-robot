# STM32 USART2 工程控制台

[English](uart_cli.md)

## 用途与安全边界

USART2 是生产固件中的低速标定与诊断控制台，不是测试应用。电机命令会进入共享
Command Model、Safety Supervisor 和生产 Motor Output Layer；控制台不能绕过安全逻辑，也不能在
CAN Motion Authority 已 Armed 时夺取控制权。

控制台上电不 Arm，复位后不恢复 Arm 状态；除一次启动信息外默认保持安静。Protocol 1.0 不变。

## 串口设置

| 项目 | 设置 |
|---|---|
| 引脚 | PA2 USART2_TX、PA3 USART2_RX |
| 格式 | 115200 bit/s、8 数据位、无校验、1 停止位（`115200 8N1`） |
| 流控 | 无 |
| 命令 | 可打印 ASCII |
| 接受的行结束符 | CR、LF 或 CRLF |
| 响应行结束符 | CRLF |
| 最大命令行 | 63 字符，不含行结束符 |
| 提示符 | `> ` |

固件 0.5.4 的启动块应为：

```text
RobotProject STM32 boot
FW 0.5.4 / Protocol 1.0
Quiet console; type 'help'

>
```

除非操作者显式启用 `watch`，否则之后没有周期输出。

## 命令速查

| 命令 | 用途 | 对运动的影响 |
|---|---|---|
| `help` | 列出控制台命令 | 无 |
| `status` | 紧凑的系统与安全摘要 | 无 |
| `encoder` | 逻辑左右轮编码器累计值与速率 | 无 |
| `imu` | 当前 IMU 身份、有效性、加速度和角速度 | 无 |
| `battery` | 当前 ADC 与电池电压估计 | 无 |
| `can` | FDCAN 状态、计数、Session 与 Freshness | 无 |
| `fault` | 集中 Fault Word 与 Active Fault 名称 | 无 |
| `clear` | 只清除已恢复的可恢复 Fault | 先 Disarm 并强制安全输出 |
| `arm` | 请求本地 UART 标定 Authority | 本身不会驱动电机 |
| `disarm` | 使命令无效并强制电机安全 | 立即安全输出 |
| `stop` | `disarm` 的别名 | 立即安全输出 |
| `motor <a> <b> <duration_ms>` | 定时直控 Motor A/B Effort | 可能产生真实运动 |
| `watch encoder` | 以 1 Hz 显示 `encoder` | 无 |
| `watch imu` | 以 1 Hz 显示 `imu` | 无 |
| `watch can` | 以 1 Hz 显示 `can` | 无 |
| `watch off` | 停止周期控制台输出 | 无 |

旧的 `version`、各类 status/CAN/encoder 子命令、wheel/body target、controller/PID 和
drivetrain configuration 命令已不属于这个轻量控制台。生产控制模块和 Protocol 1.0 保持完整；
只缩减了本地人机命令面。

## 响应与调度策略

每条命令生成一个有边界的响应块：开头显示已提交命令，结尾显示提示符。例如：

```text
> fault

0x00000000 NONE

>
```

命令响应与 Watch Report 都作为完整块排队，并由 UART 中断发送，不会逐行交错。队列压力只会
丢弃整个诊断块，不会阻塞 Safety、MotorControl、Supervisor 或 FDCAN。若同一服务周期同时有
命令和 Watch 到期，命令响应优先。

每次复位后 Watch 都是 Off，且速率固定为 1 Hz；不存在高速 UART 模式。

## 只读命令

### `help`

- 语法：`help`
- 参数/单位：无。
- 有效状态：控制台初始化后的全部状态。
- 作用：列出准确的命令集合和 Motor 参数边界。
- 安全：只读。
- 示例：`help`
- 拒绝：任何参数都会返回 Syntax Error。

### `status`

- 语法：`status`
- 参数/单位：无。
- 有效状态：全部运行状态。
- 作用：显示 Firmware、System State、Fault Word、Motor/STBY、Raw Encoder Acquisition
  Health、IMU Health、FDCAN State、当前 Authority Owner 和 BODY_VELOCITY Readiness。
- 安全：只读。`Body Cmd : READY` 表示 Drivetrain Configuration、Operating Envelope
  和两路 Wheel Controller 均已配置；运动仍必须通过运行时 Authority 与 Health Gate。
- 响应示例：

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

- 拒绝：任何参数。

### `encoder`

- 语法：`encoder`
- 参数/单位：无；Total 为 Decoded Count，Rate 为 Decoded Count/s。
- 有效状态：全部运行状态。
- 作用：显示正向归一化后的逻辑右/左轮累计值、Filtered Rate、Validity 和 Sample Age。
- 安全：只读；底层 Raw Counter 不会被修改。
- 映射：Encoder 1=右轮，Raw Forward Sign `+1`；Encoder 2=左轮，Raw Forward Sign `-1`。
  因此右轮逻辑值等于 Encoder 1 Raw 值，左轮逻辑值等于 Encoder 2 Raw 值的相反数。
- 比例：右轮 `1059.5`、左轮 `1060.8` Decoded Counts/Wheel Revolution，继续作为独立的
  Commissioning Value，不取平均。
- 64-bit 格式：累计值使用独立的 Signed Decimal 转换，支持负值和 `INT64_MIN`，不依赖精简
  C Library 的 `%lld` 支持。
- 响应示例：

```text
Right : total=10595 cps=0.0 valid=yes age=2 ms
Left  : total=10608 cps=0.0 valid=yes age=2 ms
```

- 拒绝：任何参数。

### `imu`

- 语法：`imu`
- 参数/单位：无；Acceleration 为 m/s²，Angular Velocity 为 rad/s，Age 为 ms。
- 有效状态：全部运行状态。
- 作用：显示 Driver State、WHO_AM_I、Validity/Age 和当前三轴样本。
- 安全：只读；IMU 缺失不能授权电机。
- 示例：`imu`
- 拒绝：任何参数。

### `battery`

- 语法：`battery`
- 参数/单位：无；ADC 使用 Counts/V，Battery Estimate 使用 V，Age 使用 ms。
- 有效状态：全部运行状态。
- 作用：显示 Measurement Validity、Raw ADC、ADC Voltage、Filtered Battery Estimate，并说明
  Divider 是否已标定，或仍仅用于 Commissioning。
- 安全：只读；该命令不会凭假设创建欠压阈值。
- 示例：`battery`
- 拒绝：任何参数。

### `can`

- 语法：`can`
- 参数/单位：无；Frame/Error Counter 为累计值，Age 为 ms。
- 有效状态：全部运行状态。
- 作用：显示 FDCAN State、RX/TX、Rejected Frame、Duplicate/Out-of-order Sequence Error、
  RX Overflow、TX Failure、Bus-off Count、Session ID、Heartbeat Age、Authority State/Age 和
  Command Age。
- 状态值：`NOT_INITIALIZED`、`ERROR_ACTIVE`、`ERROR_WARNING`、`ERROR_PASSIVE`、`BUS_OFF`。
- 安全：只读；不增加 Protocol 1.0 Wire Field。
- 响应示例：

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

- 拒绝：任何参数。

### `fault`

- 语法：`fault`
- 参数/单位：无；显示 32-bit 集中 Fault Word 的十六进制值。
- 有效状态：全部运行状态。
- 作用：在 Mask 后只列出 Active Fault 名称。
- 安全：只读；不清除或屏蔽 Fault。
- 示例：`0x00000000 NONE`；`0x00000010 ENCODER_VALIDITY`。
- 拒绝：任何参数。

## Fault 恢复

### `clear`

- 语法：`clear`
- 参数/单位：无。
- 有效状态：全部运行状态。
- 作用：首先 Disarm、复位 Control 并强制 Motor-safe Output；之后可清除
  `COMMAND_TIMEOUT`、`INVALID_MOTOR_COMMAND` 和 `CONTROL_SATURATION`。只有两路 Encoder
  Timer 都初始化成功、两路最新 Sample 都有效且 Age 都不超过 50 ms 时，才清除
  `ENCODER_VALIDITY`。
- 安全：仍然 Active 的故障条件绝不会被清除；其他 Fault 继续由来源模块或 Reset Policy 管理。
- 成功：`OK`。
- 拒绝示例：`REJECTED: ENCODER_VALIDITY still active`。
- 拒绝：任何参数，或任一故障条件仍 Active。

Encoder Validity 明确独立于 Motor A/B Mapping、Motor Polarity、PID Gain、Speed Limit 和 BODY
Command Readiness。一次延迟采样可以置位 Fault，但正常有效采样恢复后会自动清除；Timer Start
失败或持续 Stale/Invalid Sampling 会保持 Active。

## Authority 与安全停止

### `arm`

- 语法：`arm`
- 参数/单位：无。
- 有效状态：没有 Critical Fault，且 CAN Authority 未 Armed。
- 作用：记录本地 Arm Request；不会创建 Motion Command，也不会 Assert STBY。
- 安全：运动仍需要后续新鲜有效的 `motor` 命令和全部集中安全检查。Boot/Reset 永不自动 Arm。
- 成功：`OK`。
- 拒绝：CAN Authority Active、Critical Fault Active 或任何参数。

### `disarm`、`stop`

- 语法：`disarm` 或 `stop`。
- 参数/单位：无。
- 有效状态：全部运行状态。
- 作用：使当前命令无效，撤销本地 Arm，复位 Control，把 PWM/Direction 置于安全状态并强制
  STBY LOW。
- 安全：通过已有生产模块立即收敛到 Safe。
- 成功：`OK`。
- 拒绝：任何参数。

Disabled/Disarmed Command 被明确视为有意的安全状态，而不是 Timeout。`COMMAND_TIMEOUT` 保护
已经建立的 Motion/Control Session 在所需 Command 或 CAN Authority/Heartbeat Freshness 丢失时的
安全收敛；仅仅因为 Mode=DISABLED、Arm=No、Motor Disabled 不会产生它。

## 定时 Motor 标定

### `motor <a> <b> <duration_ms>`

- 语法：`motor <motor_a_effort> <motor_b_effort> <duration_ms>`。
- 参数：Motor A/B Normalized Effort，必须为有限值且在 `-1.0..+1.0`；Duration 为整数
  `10..1000 ms`。
- 有效状态：已先执行本地 `arm`、没有 Critical Fault、没有 Armed CAN Authority。
- 作用：通过共享 Command、Safety Supervisor 和 Motor Output Path 提交 Motor A/B 直接 Effort；
  Duration 是 Command Freshness Deadline。
- 安全：它不代表左右轮映射，不绕过 STBY Policy，也不会把无效数值送入 PWM。Duration 到期后
  进入已有的 Deterministic Command-timeout Safe Stop；修正/结束条件后，下一次 Arm Pulse 前使用
  `clear`。
- 示例：`motor 0.05 0.00 250`。
- 成功：`OK: motor command accepted (250 ms)`。
- 拒绝：语法/非有限值、Effort 越界、Duration 越界、未 Arm、CAN Authority Active、Critical
  Fault 或 Shared Command Reject。

该命令可使真实硬件运动。只能在项目规定的实机电机测试安全条件下使用：车轮悬空或
运动区域安全、电源可立即切断，并随时准备发送 `stop`。

## Watch 模式

- 语法：`watch encoder`、`watch imu`、`watch can` 或 `watch off`。
- 参数/单位：一个 Source；速率固定为 1 Report/s，不可配置。
- 有效状态：全部运行状态。
- 作用：周期发送相同的简洁 Source 输出；`watch off` 恢复安静控制台。
- 安全：Watch 使用低优先级完整块；Queue Pressure 下丢弃 Watch，而不是延迟实时任务。
- 成功：`OK: watch encoder at 1 Hz`；关闭时为 `OK`。
- 拒绝：缺少/未知 Source 或多余参数。

## 错误格式

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

RX Overflow 或超长输入会丢弃当前行，并返回一个有界错误块。
