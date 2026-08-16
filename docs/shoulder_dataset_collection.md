# 肩部 quaternion 数据采集

## 任务边界

本程序只采集右肩的三种实验动作：

- `forward`：右臂从自然下垂位置向身体前方抬起；
- `lateral`：右臂从自然下垂位置向身体右外侧抬起；
- `backward`：右臂从自然下垂位置向身体后方抬起。

当前监督学习定义固定为：

```text
X = [sensor_1, sensor_2, sensor_3]       X.shape = [N, 3]
Y = [shoulder_qw, shoulder_qx,
     shoulder_qy, shoulder_qz]           Y.shape = [N, 4]
```

`protocol_direction`、`derived_direction`、`amplitude_deg` 以及两个连续角度均为分析或未来控制所需的派生信息，不替代 quaternion Ground Truth。两个 IMU 只用于生成训练 Ground Truth，不是模型输入。

采集入口是纯 headless CLI；它不导入 Qt/VTK，也不创建或连接机器人对象。

## 配置三个柔性通道

配置文件为 `configs/shoulder_dataset.yaml`。`flex.channels` 使用原始 sleeve frame 的零基索引，必须恰好包含三个互不相同且合法的通道：

```yaml
flex:
  channels: [2, 4, 7]
  interpolation: linear  # 也可设为 nearest
```

顺序会原样保留：上例对应 `sensor_1 -> channel_2`、`sensor_2 -> channel_4`、`sensor_3 -> channel_7`。映射也会写入 `metadata.json`。`raw_flex.csv` 始终保存设备提供的全部通道；只有 `aligned.csv` 和 `training.csv` 提取这三个通道。

这些默认索引是实验配置，不是代码常量。正式采集前应根据实际袖套布置确认它们。

## 两个 IMU 与 quaternion convention

`imu_roles` 把现有 `sensors.yaml` 中的 endpoint 分配给实验角色：

```yaml
imu_roles:
  torso_imu: imu2
  upper_arm_imu: imu1
```

- `torso_imu` 稳定固定在胸口或躯干；
- `upper_arm_imu` 稳定固定在右侧大臂。

程序复用仓库已有的 `create_imu_source`、`Imu770SerialSource` 和 `Imu770Parser`，没有另建 serial worker 或 IMU transport。当前解析器把 `0x41` 的四个数按 `WXYZ` 读取。HiPNUC 官方资料把 quaternion 定义为 `WXYZ`（`q0` 为标量），并定义 `Q_b2n` 为 body/sensor frame 到 navigation/world frame 的旋转：[HiPNUC Command and Programming Manual](https://download.hipnuc.com/en/products/imu/cum.html)。

程序内部 canonical convention 为：

```text
order: WXYZ
direction: Sensor -> World
multiplication: Hamilton active rotation
left ⊗ right: 先应用 right，再应用 left
```

当前设备使用的旧帧头协议、两个设备固件版本、实际安装朝向和 body anatomical axes 仍须在真实硬件上做已知方向转动验证；程序不会把安装位置自动猜成已经验证。

## 时间戳与对齐

每个现有 source worker 在数据 `read()` 返回后立即调用 `time.monotonic_ns()`，并把结果写入该 sample 的 `host_timestamp_ns`。三个 source 独立打时间戳。IMU 的设备时间 `device_timestamp_us` 和 TID/sequence 也会保留；sleeve 协议没有设备时间，因此其 `source_timestamp` 留空。

`time.time_ns()` 和 wall-clock datetime 只用于 session 创建/结束时间与 metadata，不参与核心对齐。

采集器从现有 source 的 `drain()` 接口取得自上次读取以来的所有 frame，并维护三个独立有界 timestamp buffer：

```text
flex_buffer
torso_imu_buffer
upper_arm_imu_buffer
```

统一时间轴由 `collection.output_hz` 生成，默认 60 Hz。对齐先等待 `alignment_delay_ms`，使目标时刻右侧的 sample 有机会到达：

- flex 使用配置的 linear interpolation 或 nearest；
- IMU quaternion 使用 shortest-path SLERP，并先处理 `q/-q` 符号；
- `flex_skew_ms`、`torso_skew_ms`、`arm_skew_ms` 是目标时刻与最近参与样本的绝对偏差；
- 插值还要求左右 bracket 的间隔不超过 `2 * max_skew_ms`，禁止跨长 dropout 插值；
- 缺少 bracket、quaternion 或最近偏差大于 `max_skew_ms` 时，对应 source invalid；
- `frame_valid = flex_valid and torso_valid and arm_valid`，旧值可以作为 invalid 诊断值落盘，但不会进入 `training.csv`。

## Neutral calibration 与肩部 quaternion

启动后保持以下 neutral pose：身体自然直立、面朝正前方、右臂自然下垂、肘部自然伸直。CLI 会显示 3-2-1 倒计时，然后在 `neutral_duration_s` 内采集多个已对齐的 torso/arm quaternion pair。

在 Sensor→World convention 下：

```text
q_relative = inverse(q_torso) ⊗ q_upper_arm
q_zero = MarkleyAverage(q_relative_neutral_samples)
```

Markley averaging 不会逐 component 普通平均，并处理等价的 `q/-q`。当前姿态满足：

```text
q_relative_current = q_shoulder ⊗ q_zero
q_shoulder = q_relative_current ⊗ inverse(q_zero)
```

零偏必须从右侧消去；synthetic test 使用非对易安装偏置锁定这个顺序。每次 quaternion 运算都单位化；对连续的 torso、arm 和 shoulder quaternion，如果与前一 sample 点积小于 0，则整体变号。Neutral 时 `q_shoulder` 应接近 `[1, 0, 0, 0]`。

## 连续控制量与分析方向

默认 torso anatomical frame 为 `X=forward`、`Y=right lateral`、`Z=up`，neutral 大臂长轴 `u0=[0,0,-1]`。程序用 `q_shoulder` 主动旋转 `u0` 得到 `u_current`，而不是比较 quaternion component：

```text
forward_projection = dot(u_current, forward_axis)
lateral_projection = dot(u_current, lateral_axis)
down_projection = dot(u_current, u0)

signed_forward_backward_deg = deg(atan2(forward_projection, down_projection))
lateral_deg = max(0, deg(atan2(lateral_projection, down_projection)))
amplitude_deg = deg(acos(clamp(dot(u0, u_current), -1, 1)))
```

所以 forward 为正、backward 为负、neutral 为 0；右侧 lateral elevation 为非负。未来机器人控制应使用 `signed_forward_backward_deg` 和 `lateral_deg` 这样的连续量，不应先离散成 direction，以免 neutral 附近发生 forward/backward 抖动。

`derived_direction` 只用于分析和数据清洗。幅度小于 `min_amplitude_deg` 时输出 `neutral` 且 `direction_valid=false`。其余样本比较水平运动方向与 forward/lateral/backward 三个 canonical direction；角误差超过 `max_direction_error_deg` 时同样 invalid，但 quaternion 和连续角度仍保留。

`protocol_direction` 是 CLI 已知的本轮实验方向，例如 `--motion forward`。它与 IMU 算出的 `derived_direction` 不同。两者不一致可能来自受试者动作偏离、安装轴错误、neutral 错误或计算配置错误。

## Session 文件

每次运行产生独立目录：

```text
data/shoulder/session_YYYYMMDD_HHMMSS_forward/
├── metadata.json
├── raw_flex.csv
├── raw_torso_imu.csv
├── raw_upper_arm_imu.csv
├── aligned.csv
└── training.csv
```

- `raw_flex.csv`：host timestamp、可选 source timestamp、sequence 和所有原始通道；
- 两个 raw IMU 文件：host/device timestamp、sequence、canonical WXYZ quaternion、accel、gyro；
- `aligned.csv`：60 Hz 的三个模型输入、两个已对齐 IMU quaternion、shoulder quaternion、连续角度、两类 direction、repetition、skew 和 validity；
- `training.csv`：只写 `frame_valid=true` 的行，保留 `X=[sensor_1..3]`、`Y=[shoulder_qw..qz]` 和必要派生/分组字段；
- `metadata.json`：配置、channel mapping、convention、neutral `q_zero`、IMU roles、git 状态和统计。

CSV 在采集期间按 `flush_interval_s` 周期 flush。正常退出、`Q` 和 `Ctrl+C` 都会停止 source、最终 drain、flush/close CSV、完成 metadata 并打印统计。

## 采集命令与 repetition

从仓库根目录执行：

```bash
python tools/collect_shoulder_dataset.py --config configs/shoulder_dataset.yaml --motion forward --output data/shoulder
python tools/collect_shoulder_dataset.py --config configs/shoulder_dataset.yaml --motion lateral --output data/shoulder
python tools/collect_shoulder_dataset.py --config configs/shoulder_dataset.yaml --motion backward --output data/shoulder
```

Neutral calibration 完成后：

```text
SPACE  开始/停止当前 repetition
N      结束当前 repetition 并切换到下一个编号
Q      安全结束 session
Ctrl+C 安全中断 session
```

编号以 `001`、`002` 等形式写入 `repetition_id`。后续训练/验证/测试应按 session 或 repetition 分组拆分，不能随机拆 frame。

无硬件验证整个落盘链可运行：

```bash
python tools/collect_shoulder_dataset.py --config configs/shoulder_dataset.yaml --motion forward --output data/shoulder_fake --fake --duration 2 --no-countdown
```

Fake 模式分别生成 30 Hz flex、60 Hz torso IMU 和 100 Hz upper-arm IMU；它只验证软件与文件格式，不代表真实硬件、安装轴、串口吞吐或时间同步已经验证。
