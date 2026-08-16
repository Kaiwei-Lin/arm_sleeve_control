# sleeve_arm_control

面向真实机械臂的安全控制层，以及与机械臂完全隔离的传感器采集基础。

正式的三方向右肩 quaternion 机器学习数据采集入口与实验说明见 [docs/shoulder_dataset_collection.md](docs/shoulder_dataset_collection.md)。该 headless CLI 只采集 sleeve 与双 IMU，不连接机器人。

```text
Python tools -> SafetyController -> DyMotorArm (ctypes)
             -> libdymotor_bridge.so -> 厂家 libMotorDrive.so -> mechanical arm
```

## 项目当前阶段

Phase 1 原有三个机械臂关节的位置控制与 PVCT 读取现已扩展加入大臂旋转电机。Phase 2 新增袖套、IMU、时间同步和数据记录。Phase 3/4 已接入 CH2 肘部规则、可切换的双 IMU/三柔性肩部方向与幅度估计，以及可选的 IMU3/4 大臂旋转；四个语义关节经同一个 Mapper、SafetyController 和 Robot command owner 下发。

当前 DyMotor 连接严格复用厂家 `pose_control_get_pvct.c` 的启动 pipeline，因此任何连接都会执行状态机切换、位置模式、45 次 `big_pose` 预填充和 Servo On。`--execute` 只控制是否在启动完成后继续发送工具请求的目标；即使不带 `--execute`，也必须按真机使能操作对待。

## Phase 2 — Sensor Foundation

```text
Sleeve ──────────────┐
                     │
Optional IMU1 ───────┼→ SensorSynchronizer → SensorSample → SensorRecorder
                     │
Optional IMU2 ───────┘
```

这条管线不导入 Robot backend，也不存在 `SensorSample → Robot` 路径。未来 Predictor 将消费同一个 `SensorSample`；当前没有创建模型实现，也没有在 Source 层做窗口、特征提取、归一化或通道到关节的映射。

支持三种模式：Sleeve only、Sleeve + IMU1、Sleeve + IMU1 + IMU2。启用的 IMU 若在同步阈值内没有匹配帧，对应字段为 `None`，Sleeve 样本仍可记录，不会用陈旧 IMU 数据填充。

### 已确认的传感器协议

- Sleeve：ASCII 串口，115200/8N1，以 `;` 结束一条记录；每条为 11 个逗号分隔的有限数值。`SleeveFrame.channels` 完整保留全部 11 个字段；肘部读取 CH2，柔性肩部推理读取 CH3/CH4/CH5。
- IMU770：二进制串口，460800/8N1；帧头为 `59 53`，使用 TLV 数据段和双字节校验。已支持加速度、角速度及可选四元数；主机收到完整有效帧时使用 `time.monotonic()`。

协议来自旧项目 `arm_data_collector` 的已验证采集实现。串口名称因机器而异，必须在 `configs/sensors.yaml` 中填写；默认不会猜测 `/dev/ttyUSB*`。IMU 默认为 disabled，disabled 时不会打开串口。

### 时间同步

Sleeve 是主时间轴。同步器在每个已启用 IMU 的 `deque` 有界缓冲中选择与 Sleeve 时间差绝对值最小的帧。软件初始默认值为：

```yaml
max_time_delta_ms: 20
buffer_duration_ms: 500
```

这些是待实验验证的软件默认值，不是传感器或人体运动的最终参数。内部同步统一使用 monotonic timestamp；metadata 另存人类可读的 wall clock 开始时间。

### 读取与记录

填写 Sleeve 端口后，只读 Sleeve：

```bash
python tools/read_sleeve.py
```

读取真实 IMU（必须先 enabled 并配置端口）：

```bash
python tools/read_imu.py --imu imu1
python tools/read_imu.py --imu imu3
```

无硬件验证 IMU 和同步：

```bash
python tools/read_imu.py --imu imu1 --fake
python tools/test_sensor_sync.py --fake
```

### IMU770 大臂轴向旋转对照验证 Demo

`tools/demo_upper_arm_twist_imu770.py` 是一个独立、只读的双 IMU770 实机测试工具，不依赖项目内部的 `sleeve_arm` Source，也不会向传感器发送命令或修改设备配置。它按已确认的协议将 `0x41` 解释为 Sensor → World 的 `[w, x, y, z]` 四元数，并同时计算：

- `world`：大臂 IMU 相对标定零位绕自身 `+X` 轴的旋转角。
- `relative`：原方案中大臂相对小臂绕 `+X` 轴的旋转角。

安装时，大臂 IMU 的本地 `+X` 轴应由肩部指向肘部；小臂 IMU 的 `+X` 轴应由肘部指向手腕。手臂伸直时尽量对齐两个 IMU 的坐标轴，剩余安装偏差由零位标定消除。

```powershell
python tools/demo_upper_arm_twist_imu770.py `
  --upper-port COM5 `
  --forearm-port COM6 `
  --csv upper_arm_twist_test.csv
```

默认串口参数是 460800/8N1，双流最大时间差为 20 ms，EMA 系数为 0.35。连接成功后，保持约定的手臂零位并按 Enter；程序默认采集 2 秒同步数据完成多帧标定，然后实时显示两个角度、差值、同步间隔、帧率和解析错误数。可用 `--calibration-seconds`、`--max-sync-ms`、`--ema-alpha` 和 `--print-hz` 调整现场参数，按 `Ctrl+C` 安全结束。

建议按以下顺序验证：

1. 保持零位不动，观察两种结果的漂移。
2. 固定肘关节角度，沿大臂 `+X` 轴做正向和反向旋转。
3. 不主动旋转大臂，只反复屈肘和伸肘。
4. 保持近似相同的大臂轴向角，在不同屈肘角下重复测量。

CSV 保存两个 IMU 的主机/设备时间戳、TID、原始四元数、两种算法的 raw/unwrapped/filtered 角度、角度差和同步间隔。若 `world` 在屈肘时明显比 `relative` 稳定，说明原相对方案存在屈肘串扰或共同旋转抵消。两个 IMU 本身不提供可追溯的角度真值；如需给出绝对精度，应增加机械角度尺、编码器或光学跟踪。若还需要消除身体整体运动，则应再增加躯干 IMU 作为参考。

### WT901PWIFI 独立实时读取 Demo

`tools/read_wt901pwifi.py` 是独立工具，不接入上述 IMU770 Source，也不会连接 WiFi、修改 WiFi 账号/密码、配置设备 IP 或写传感器寄存器。请先手动完成电脑联网和传感器端参数设置，再将对应地址和端口传给脚本。

已按厂家规格和协议实现固定 54 字节 `WT55...0D0A` 数据帧，可输出设备 ID、片上时间、三轴加速度/角速度/磁场、Roll/Pitch/Yaw、温度、电池电压、RSSI 和版本号。串口默认参数为 9600/8N1；传感器出厂网络模式为 AP + UDP，默认远端计算机地址/端口为 `192.168.4.2:1399`。

```bash
# Type-C 串口；Windows 示例
python tools/read_wt901pwifi.py serial --port COM5

# UDP：在电脑本地监听传感器发送的数据
python tools/read_wt901pwifi.py udp --host 0.0.0.0 --port 1399

# TCP 服务端：等待手动配置为 TCP 客户端的传感器连接电脑
python tools/read_wt901pwifi.py tcp-server --host 0.0.0.0 --port 1399

# TCP 客户端：仅用于手动配置成监听端点的传感器
python tools/read_wt901pwifi.py tcp-client --host 192.168.4.1 --port 9250

# 每帧输出一个 JSON 对象，便于管道处理
python tools/read_wt901pwifi.py --json udp --host 0.0.0.0 --port 1399
```

TCP/UDP 的 `--host`、`--port` 都可按手动配置修改。Windows 首次监听入站 UDP/TCP 时，可能需要在防火墙提示中允许当前 Python 解释器访问对应网络。按 `Ctrl+C` 可安全关闭串口或 socket。

记录真实已启用传感器，或短时 fake 数据：

```bash
python tools/record_sensors.py
python tools/record_sensors.py --fake --duration 2
```

每次会在 `recordings/YYYYMMDD_HHMMSS_xxxxxx/` 生成：

- `samples.csv`：固定 schema，包括 monotonic timestamp、全部 `sleeve_ch_*`、IMU1 和 IMU2 字段；缺失 IMU 留空。
- `metadata.yaml`：Git commit、采集开始时间、backend/串口配置、IMU enabled 状态和同步参数，不记录个人隐私。

`Ctrl+C` 会停止循环、flush/close Recorder，并关闭所有 Source。采集线程不打印每帧，工具只低频显示统计。

> Source 与 Robot 仍完全解耦。CH2 和 Flex 模型的关节语义在 Predictor 层产生；双 IMU 的 rotation 在独立 estimator 中产生，随后统一进入 MotionIntent、Mapper 和 SafetyController，任何传感器都不会直接控制电机。

## 机械臂关节

| Semantic joint | Motor ID | CAN |
| --- | ---: | ---: |
| `shoulder_flexion` | 22 | 2 |
| `shoulder_abduction` | 23 | 2 |
| `elbow_flexion` | 25 | 2 |
| `upper_arm_rotation` | 24 | 2 |

业务层只使用以上语义名称。motor/CAN 映射、方向、零位和安全参数统一位于 `configs/robot.yaml`。

> **ID24 硬件确认状态：** 仓库默认配置仍可能保留待确认值；真机必须使用机器端已经标定的 `direction/zero_position/min_position/max_position`。当前 SDK 路径不提供速度命令，本项目也没有添加速度限制。第一次双 IMU 联动仍只允许小角度、有人监护的方向验证。

## SDK 位置与已确认行为

厂家 SDK 原样保存在 `third_party/dymotor_sdk/`，本项目不修改或复制其中内容。主要参考为 `example_C/pose_control_get_pvct.c`、`get_motor_list.c`、公开头文件和厂家 `CMakeLists.txt`。

已确认：

- `robot_create` + `robot_config_net` 创建并连接主板上下文。
- bridge 的 `arm_open` 在 motor object 建立后严格执行厂家示例的主板状态机 `0x80 → 1`、heartbeat、位置模式、预填充和 Servo On 顺序。
- bridge 与厂家示例一样直接用配置的 ID/CAN 创建 motor object；启动后的逐电机 PVCT 校验负责确认 22/CAN2、23/CAN2、25/CAN2、24/CAN2 均可用。
- `robot_motor_get_PVCTFast` 提供 position、velocity、estimated torque、state 和 error 等缓存反馈。
- `robot_motor_set_control_mode(..., MOTOR_CTRL_MODE_POSITION)` 切换位置模式，`CTRL_SERVO_ON/OFF` 控制使能。
- `robot_motor_set_position` 暂存每台电机目标，`robot_motor_set_big_pose` 一次发送已注册电机的大包；三关节控制复用此 batch 路径。
- position 单位为 rad，velocity 为 rad/s，estimated torque 为 N·m。

Fast PVCT API **没有 current 输出**，所以 Python 中 `JointState.current` 为 `None`，`read_pvct.py` 显示 `unavailable`。若配置 `max_current`，SafetyController 会因无法证明电流安全而拒绝控制；不会伪造电流值。厂家文档也没有完整给出 state/error、温度和母线电压的枚举/缩放说明。

厂家位置示例包含无限运动循环，且其 Servo Off/清理代码不可达。本项目没有照搬示例的目标值、无限重试、heartbeat=0 或退出方式。

## 环境与构建

厂家只提供 Linux x86-64 ELF 的 `libMotorDrive.so`；因此 native bridge 必须在 Linux x86-64 上构建。Windows 本机不能链接该 `.so`，可使用装有 Linux 的机械臂控制机或 WSL2（且 WSL2 必须能够访问机械臂网卡）。

需要：CMake 3.15+、C 编译器、Python 3.10+、PyYAML；真实串口 Source 还需要 pyserial。建议先创建虚拟环境：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-dev.txt
```

在项目根目录构建：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

产物为：

```text
build/native/dymotor_bridge/libdymotor_bridge.so
```

CMake 为 bridge 设置相对 RPATH，使其从项目原位置 `third_party/dymotor_sdk/lib` 加载 `libMotorDrive.so`，无需复制厂家库。检查动态库解析：

```bash
ldd build/native/dymotor_bridge/libdymotor_bridge.so
```

若构建树布局被改变，可显式指定 bridge；若 loader 仍无法解析厂家依赖，再临时设置 `LD_LIBRARY_PATH`：

```bash
export DYMOTOR_BRIDGE_LIBRARY="$PWD/build/native/dymotor_bridge/libdymotor_bridge.so"
export LD_LIBRARY_PATH="$PWD/third_party/dymotor_sdk/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

## 配置检查（连接前必做）

打开 `configs/robot.yaml`，确认主板与控制机网络参数。随包两个厂家示例使用了不同的主板/端口组合；当前配置采用 `pose_control_get_pvct.c` 中的 `device_id=0xFE` 和网络值，只是待验证起点，已标记 `NEEDS_HARDWARE_VALIDATION`。

`max_position_step` 当前设为 1°（rad 表示），它只是保守的调试期单命令步长，不是经过验证的机械极限。测试工具默认以 `--command-dt 0.05` 作为 `max_velocity * dt` 的控制周期；改变真实发送周期时必须同步修改该参数。真实硬件调试完成后，必须填入并复核：

- 每个关节的 `direction` 和 `zero_position`
- `min_position` / `max_position`
- `max_velocity` / `max_current`
- `max_tracking_error`
- 主板 ID、双方 IP、端口、fast-mode 参数
- 厂家 state/error 代码含义和反馈超时特性

## Step 1：按厂家 pipeline 启动后读取 PVCT

确保机械臂周围无人、可立即断电/急停，然后运行：

```bash
python tools/read_pvct.py
```

它会对配置中的四台电机执行厂家启动 pipeline，然后持续读取 PVCT。启动包含 `StateMachine(0x80 → 1)`、关闭 heartbeat、位置模式、`set_pos(0,0,0)`、45 次 `big_pose`、Servo On 和等待 1 秒；不会在此后调用 `robot_motor_set_position`。输出中的 position/velocity/torque 已按配置的方向与零位转换为语义坐标。按 `Ctrl+C` 后执行 Servo Off 和 close。

首先核对：四个 motor/CAN 地址正确、反馈有限且稳定、error 均为 0；不要在存在错误或映射不符时继续。

### PVCT bridge 原始诊断

修改 native bridge 后必须先重新构建，并确认不存在或未加载旧副本：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
find . -name 'libdymotor_bridge.so' -ls
```

然后用显式路径运行诊断：

```bash
python tools/debug_bridge.py \
    --joint all \
    --samples 3 \
    --library "$PWD/build/native/dymotor_bridge/libdymotor_bridge.so"
```

该工具对配置中的 CAN2/motor ID22/23/25/24 运行与厂家示例相同的启动 pipeline，再重复读取 PVCT 并关闭。它同时打印 C 侧原始诊断、Python 收到的 `wrapper_status`、state/bus/error 十进制和十六进制，以及实际加载的 `.so` 绝对路径。`bus == 0` 仍会被判定为无效反馈并返回 `wrapper_status=-4`，不会把全零输出当作有效反馈。

bridge 的启动顺序现与厂家示例一致：`robot_create` → `robot_config_net` → `robot_set_fast_mode` → `robot_create_motor` → `StateMachine(0x80)` → `StateMachine(1)` → `setHeartbeat(0)` → position mode → `set_pos(0,0,0)` → 45 次 `big_pose` → Servo On → 等待 1 秒 → 首次 PVCT。项目不会复制示例循环中的固定 `-0.45 rad` 运动目标。

与厂家示例对照时，使用相同 motor/CAN，并分别记录：

```text
vendor FastErrorCode
bridge motor_error
bridge wrapper_status
loaded_so
```

厂家 FastErrorCode 为 0 时，bridge 的 `motor_error` 也必须为 0。任何真实非零 motor error 仍会被 SafetyController 拒绝；没有错误码白名单。

## Phase 3 — Sleeve-to-Elbow Control

当前唯一的传感器控制链为：

```text
Sleeve CH2 → SensorSample → RuleBasedPredictor → MotionIntent.elbow_flexion
           → ArmMapper → SafeArmController → elbow_flexion (ID25/CAN2)
```

串口协议的字段按顺序命名 CH1、CH2……，因此配置中的通道号都是用户可读的 1-based 编号。Phase 3 的 `run_sleeve_elbow.py` 仍只读取 Sleeve，并保持其他关节启动位置；`run_model_control.py` 可用双 IMU 或 CH3/CH4/CH5 估计肩部，`upper_arm_rotation.enabled` 独立控制是否由 IMU3/4 生成第四个 twist 自由度。

Phase 3 配置位于 `configs/phase3.yaml`。`input_min`/`input_max` 是两个人体标定姿态的 CH2 实测端点，`angle_range.min_deg/max_deg` 是这两个姿态对应的人体绝对肘角；任一缺失都会阻止控制启动。临时规则先将 CH2 clamp/归一化，再线性换算成绝对人体肘角 rad。Mapper 不叠加启动位置；DyMotor backend 以 `SDK target = zero_position + direction × semantic target` 转换，最终仍必须经过 Phase 1 的位置、单步、速度、跟踪误差和反馈错误检查。

Watchdog 使用 monotonic timestamp：超过 `sensor_timeout_ms` 不产生新目标并保持最后安全目标；超过 `hard_timeout_ms` 抛出故障并进入 Servo Off/close。无效、缺失或非有限 CH2 不会产生命令。

严格按以下顺序验收；连接 DyMotor 的第 4、5 步都会执行厂家 Servo On pipeline，只有第 5 步会继续发送袖套生成的位置目标。

### 1. 标定 CH2（不连接机械臂）

```bash
python tools/calibrate_sleeve_elbow.py
```

工具分别采集肘伸直和屈曲姿态，打印 median/min/max/std 及建议的 `input_min`、`input_max`、`invert_input`。写入配置前必须人工确认姿态和方向。

### 2. FakeSleeve + FakeRobot

```bash
python tools/run_sleeve_elbow.py --sleeve fake --robot fake
```

### 3. 真实 Sleeve + FakeRobot

```bash
python tools/run_sleeve_elbow.py --sleeve real --robot fake
```

先观察 CH2 raw、normalized、filtered、target delta、sensor age 和 stale/invalid 统计，不连接真实机械臂。

### 4. 真实 Sleeve + DyMotor dry-run

```bash
python tools/run_sleeve_elbow.py --sleeve real --robot dymotor
```

此模式会执行厂家 Servo On pipeline、读取 PVCT 并预览 Safety 结果，但不发送袖套生成的位置目标。

### 5. 真实 Sleeve + DyMotor execute

```bash
python tools/run_sleeve_elbow.py --sleeve real --robot dymotor --execute
```

必须在急停可用、现场监护、配置完成且前四步结果正确后执行。顺序为三路稳定反馈 → 保存启动位置 → Sleeve fresh/CH2 有效 → Servo On → 首条启动位置命令 → 仅 ID25 在启动位置附近小范围跟随。`Ctrl+C`、source/robot 异常或 hard timeout 都停止新目标并执行 Servo Off、robot close、source close。

以上是 Phase 3 的边界；Phase 4 将可选择的肩部估计结果转换为同一个 `MotionIntent`，不修改 Mapper、SafetyController 或 Robot 层。

## Phase 4 — 可选择的肩部推理器

`tools/run_model_control.py` 同时保留两条肩部推理链。命令行 `--shoulder-predictor` 优先于 `configs/phase4.yaml` 中的 `predictor.backend`：

```powershell
# 默认方案：IMU1/IMU2
python tools/run_model_control.py --shoulder-predictor dual_imu --sleeve real --imus real --robot fake

# 备选方案：Sleeve CH3/CH4/CH5
python tools/run_model_control.py --shoulder-predictor flexarm_estimator --sleeve real --imus real --robot fake
```

默认配置保留两套参数，但 `backend: dual_imu` 只选择其中一套运行：

```yaml
predictor:
  backend: dual_imu
  flexarm_estimator:
    model_dir: ../../arm_data_collector/flexarm_estimator/models
    sleeve_channels: [3, 4, 5]
    calibration_file: ../calibrations/flexarm_live_calibration.json
    calibration_seconds: 3.0
    angle: {min_deg: 0, max_deg: 180}
```

两种推理器都输出肩部方向、幅度和置信度，并使用相同的绝对人体语义映射：`Forward` → 正肩前屈，`Backward` → 负肩前屈，`Lateral` → 肩外展。CH2 肘部规则、IMU3/4 大臂旋转、Mapper、安全检查和 Robot 生命周期在两种模式下完全共用。

### 方案一：双 IMU 肩部估计

该方案复用 `sleeve_arm.predictor.DualImuShoulderPredictor`，底层 API 为：

```python
from flexarm import DualImuArmEstimator

estimator = DualImuArmEstimator(
    down_axis=(-1, 0, 0),
    forward_axis=(0, 0, 1),
    lateral_axis=(0, 1, 0),
    rest_threshold_deg=5,
    dominance_ratio=1.1,
)
estimator.calibrate(chest_rest_samples, arm_rest_samples)
estimator.calibrate_forward(chest_forward_samples, arm_forward_samples)
result = estimator.update(chest_q, arm_q)
```

`shoulder_imu.arm_imu=imu1` 安装在右大臂，`shoulder_imu.chest_imu=imu2` 安装在胸部。两路输入均按 `[w, x, y, z]` 解释为 Sensor→同一 World 坐标系的旋转；程序消费 `result.direction`、`result.magnitude_deg` 和 `result.confidence`。每次启动或重新佩戴后，程序先提示自然下垂并采集同步样本完成零位标定，再提示保持正前方抬起 45–60° 并采集第二组同步样本学习前抬方向；两步完成后才进入实时推理。`--calibration-seconds` 对这两次采集分别生效，双 IMU 模式不复用旧穿戴标定。

```text
IMU2 chest_q --+
                 +-> DualImuArmEstimator -> direction/magnitude/confidence
IMU1 arm_q ------+
```

### 方案二：三个柔性传感器肩部估计

该方案复用 `sleeve_arm.predictor.FlexModelPredictor` 和 `FlexArmEstimator`。输入固定为配置中的 1-based `CH3/CH4/CH5`，内部依次传为 `flex1/flex2/flex3`，不会把 CH2 肘部通道送入肩部模型：

```python
estimator = FlexArmEstimator.from_pretrained(model_dir)
result = estimator.update(
    flex1=ch3,
    flex2=ch4,
    flex3=ch5,
    timestamp_ns=timestamp_ns,
)
```

默认会提示手臂自然下垂，采集配置时长的三通道样本，调用模型标定并保存结果。只有明确传入 `--reuse-calibration` 才复用配置的标定文件；`--calibration-output PATH` 可覆盖输出位置：

```powershell
# 现场重新标定
python tools/run_model_control.py --shoulder-predictor flexarm_estimator --sleeve real --imus real --robot fake

# 明确复用已有标定
python tools/run_model_control.py --shoulder-predictor flexarm_estimator --reuse-calibration --sleeve real --imus real --robot fake
```

单独标定、在线测试和离线回放仍不创建 Robot：

```powershell
python tools/calibrate_flex_model.py
python tools/test_flex_model.py
python tools/test_flex_model.py --input recordings/.../samples.csv --reuse-calibration
```

### 独立的 IMU3/IMU4 大臂旋转

`upper_arm_rotation.enabled` 与肩部推理器选择相互独立。启用时，无论肩部使用哪种方案，程序都会另外创建 IMU3/4、执行 twist 零位标定并生成第四个自由度；柔性肩部模式不会创建或读取 IMU1/2。关闭该配置时不会创建 IMU3/4。

```yaml
upper_arm_rotation:
  enabled: true
  upper_imu: imu3
  reference_imu: imu4
  twist_axis: x
  ema_alpha: 0.35
  max_sync_ms: null
  calibration_seconds: 2.0
  startup_timeout_s: 10.0
```

现有 IMU770 parser 按 `[w, x, y, z]` 保存四元数；代码按用户提供 demo 的 Sensor→World 约定解释它。该坐标系方向仍需实物转动验证。默认测量轴为 IMU local `+X`，其他安装轴可通过 `twist_axis` 选择。算法保持为：

```text
world_delta    = inverse(upper_zero) * upper_now
relative_now   = inverse(reference_now) * upper_now
relative_delta = inverse(relative_zero) * relative_now
upper_arm_rotation_deg = world.filtered_deg - relative.filtered_deg
```

两路 twist 均保留 ±180° unwrap 和配置化 EMA。肩部 IMU1/2 和旋转 IMU3/4 使用独立同步器；超过同步阈值、stale 帧或非法四元数不会产生新目标。

不创建 Robot 的 IMU 诊断命令：

```powershell
python tools/test_upper_arm_rotation.py --config configs/sensors.yaml
python tools/test_imu_motion.py --config configs/sensors.yaml --duration 60
```

### 集成验证与真机执行

先用 fake source 验证选择分支；两种模式分别需要对应的 `flexarm` API，柔性模式还需要配置的模型文件：

```powershell
python tools/run_model_control.py --shoulder-predictor dual_imu --sleeve fake --imus fake --robot fake --duration 5
python tools/run_model_control.py --shoulder-predictor flexarm_estimator --sleeve fake --imus fake --robot fake --duration 5
```

真实传感器测试后，再依次进行 DyMotor 预览和显式执行：

```powershell
python tools/run_model_control.py --shoulder-predictor dual_imu --sleeve real --imus real --robot fake
python tools/run_model_control.py --shoulder-predictor dual_imu --sleeve real --imus real --robot dymotor
python tools/run_model_control.py --shoulder-predictor dual_imu --sleeve real --imus real --robot dymotor --execute
```

选择柔性方案时只需把上述三条命令中的 `dual_imu` 改为 `flexarm_estimator`。DyMotor 连接仍执行厂家 Servo On pipeline；`--execute` 只决定启动后是否发送模型目标。真机前必须确认急停、现场监护、端口、四枚 IMU 的安装与方向，以及所有受控电机的 zero、direction、min/max。

### Offline mapper diagnostic

不打开 Sleeve、串口、bridge 或 Robot 即可检查手工提供的模型结果：

```powershell
python tools/debug_model_mapping.py --action Backward --shoulder-angle-deg 30 --elbow-angle-deg 90
python tools/run_manual_model_control.py --action Forward --shoulder-angle-deg 5 --elbow-angle-deg 30 --upper-arm-rotation-deg 10
```

## Step 2：测试肘关节

先 dry-run：

```bash
python tools/test_joint.py \
    --joint elbow_flexion \
    --delta-deg 1
```

程序在厂家启动 pipeline 后从实测位置计算相对目标；不带 `--execute` 时不发送这个增量目标，但连接过程已经 Servo On。确认打印值后才执行：

```bash
python tools/test_joint.py \
    --joint elbow_flexion \
    --delta-deg 1 \
    --execute
```

`--execute` 仍经过 SafetyController：厂家启动并取得稳定反馈 → 小步长目标 → 一次 batch flush → 持续监控 → Servo Off → close。

## Step 3：依次验证单关节

严格按顺序逐个确认正负方向和小增量行为：

1. `elbow_flexion`
2. `shoulder_abduction`
3. `shoulder_flexion`

每个关节先 dry-run，再用 **不超过 1°** 的单次 `--execute`。若方向与语义不一致，停止并只修改配置中的 `direction`；不要修改业务代码。验证后才把 `direction_status` 改为 `VALIDATED` 并记录验证过程。

新增 ID24 大臂旋转电机也复用同一个安全单关节工具。先运行厂家启动 pipeline 并预览目标：

```bash
python tools/test_joint.py \
    --joint upper_arm_rotation \
    --delta-deg 1
```

确认 Motor ID 24、CAN 2、当前反馈和目标方向均正确，机械结构周围无干涉且急停可用后，才允许显式执行：

```bash
python tools/test_joint.py \
    --joint upper_arm_rotation \
    --delta-deg 1 \
    --execute
```

两条命令连接时都会执行厂家 Servo On pipeline；只有第二条会发送额外的 1° 位置目标。测试不会寻找零位或限位；若方向错误立即停止。

## Step 4：三个关节联合小幅运动

先 dry-run：

```bash
python tools/test_three_joints.py \
    --shoulder-flexion-delta-deg 1 \
    --shoulder-abduction-delta-deg 1 \
    --elbow-delta-deg 1
```

三个单关节都验证完成后才运行：

```bash
python tools/test_three_joints.py \
    --shoulder-flexion-delta-deg 1 \
    --shoulder-abduction-delta-deg 1 \
    --elbow-delta-deg 1 \
    --execute
```

bridge 先分别更新三个目标缓存，再调用一次 `robot_motor_set_big_pose`，尽可能同步地下发。

## 安全设计与停止方法

- DyMotor 启动严格执行厂家 pipeline，包括 `set_pos(0,0,0)`、45 次 `big_pose` 和 Servo On；任何连接都必须清空工作区、准备急停并按可能运动处理。
- `SafeArmController` 是强制路径；tools 不直接调用 backend 的位置发送 API。
- 厂家 pipeline 完成并等待 1 秒后，Python 才读取和验证四台电机的 PVCT；全零、非有限值或非零 error 仍会失败并 Servo Off/close。
- 已配置的 min/max 会 clamp；`max_position_step` 与 `max_velocity * dt` 同时存在时取更严格者；配置 `max_velocity` 后也检查实测反馈速度。
- 每次发送前 Python 和 C bridge 都复查四路反馈与 error；反馈/SDK 失败会停止发送并 Servo Off。
- `max_tracking_error` 配置后会监控指令和反馈偏差。
- `Ctrl+C`、普通异常和工具退出都通过 `finally` 执行 Servo Off + close。

正常停止：按一次 `Ctrl+C`，等待 `Servo Off`/close 完成。异常运动：**立即使用现场急停或切断驱动电源**，不要依赖 Python 进程作为唯一安全装置。软件安全层不能替代物理急停、机械限位和现场监护。

禁止在当前阶段自动扫描限位、寻找零位、执行连续轨迹或大角度动作。

## 无机械臂测试

测试只使用 `FakeRobotArm`、FakeSleeve 和 FakeIMU，不会加载 native bridge、串口或连接硬件：

```bash
pytest
```

覆盖 Phase 1 安全控制回归，以及领域 Frame、完整 Sleeve parser、IMU770 parser、Optional IMU、最近时间同步、buffer 清理、Fake Source 生命周期和 Recorder schema/metadata。

## 项目结构

```text
configs/robot.yaml                 motor/CAN、网络与待验证安全参数
configs/sensors.yaml               Sleeve、Optional IMU、同步与记录配置
native/dymotor_bridge/             厂家 C SDK 的薄封装与 CMake
sleeve_arm/domain/                 Joint 与 Sensor Frame/Sample
sleeve_arm/robot/                  RobotArm、DyMotorArm、FakeRobotArm
sleeve_arm/control/                SafetyController 与纯安全函数
sleeve_arm/sources/                真实与 Fake Sleeve/IMU Source
sleeve_arm/sync/                   Sleeve 主时间轴同步器
sleeve_arm/recording/              SensorSample CSV Recorder
tools/read_pvct.py                 厂家启动 pipeline + PVCT 验证
tools/read_sleeve.py               完整 Sleeve 只读工具
tools/read_imu.py                  单 IMU 只读/Fake 工具
tools/test_sensor_sync.py          Fake 时间同步验证
tools/record_sensors.py            真实/Fake 统一记录工具
tools/test_joint.py                默认 dry-run 的单关节小增量测试
tools/test_three_joints.py         默认 dry-run 的三关节 batch 测试
tests/                             纯离线测试
third_party/dymotor_sdk/           只读厂家 SDK 与示例
```

## 当前未验证内容

传感器端口名、真实采样率、丢包率、20 ms 同步阈值和 500 ms 缓冲时长仍需现场验证。当前肩部可选择 `DualImuArmEstimator`（IMU1/2）或 `FlexArmEstimator`（Sleeve CH3/CH4/CH5），CH2 仍独立负责肘部；rotation estimator 独立使用 IMU3/4。真实 IMU 安装轴、Sensor→World 约定、两种肩部模型的方向/幅度、真实柔性标定及真实机械臂小范围联调仍未验证。
