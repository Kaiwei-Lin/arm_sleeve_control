# sleeve_arm_control

面向真实三自由度机械臂的安全控制层，以及与机械臂完全隔离的传感器采集基础。

```text
Python tools -> SafetyController -> DyMotorArm (ctypes)
             -> libdymotor_bridge.so -> 厂家 libMotorDrive.so -> mechanical arm
```

## 项目当前阶段

Phase 1 支持三个机械臂关节的位置控制与 PVCT 读取。Phase 2 新增袖套、Optional 双 IMU、时间同步和数据记录。Phase 3 新增临时规则式 CH2 → 肘关节小范围验证链路；尚未接入训练模型、肩部袖套控制、IMU 融合、GUI 或网络远控。

默认行为不会产生运动：`test_joint.py` 和 `test_three_joints.py` 只有显式加入 `--execute` 才会 Servo On 和发送目标。

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

- Sleeve：ASCII 串口，115200/8N1，以 `;` 结束一条记录；每条为 11 个逗号分隔的有限数值。`SleeveFrame.channels` 完整保留全部 11 个字段，不筛选 CH2/CH3/CH4。
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
```

无硬件验证 IMU 和同步：

```bash
python tools/read_imu.py --imu imu1 --fake
python tools/test_sensor_sync.py --fake
```

记录真实已启用传感器，或短时 fake 数据：

```bash
python tools/record_sensors.py
python tools/record_sensors.py --fake --duration 2
```

每次会在 `recordings/YYYYMMDD_HHMMSS_xxxxxx/` 生成：

- `samples.csv`：固定 schema，包括 monotonic timestamp、全部 `sleeve_ch_*`、IMU1 和 IMU2 字段；缺失 IMU 留空。
- `metadata.yaml`：Git commit、采集开始时间、backend/串口配置、IMU enabled 状态和同步参数，不记录个人隐私。

`Ctrl+C` 会停止循环、flush/close Recorder，并关闭所有 Source。采集线程不打印每帧，工具只低频显示统计。

> CH2 与肘部、CH3/CH4 与肩部的关系仅是未来 Predictor 集成信息，当前没有实现任何机械臂 mapping。IMU 将来用于辅助模型判断人体手臂的运动方向和状态，而不是直接控制电机。

## 三个关节

| Semantic joint | Motor ID | CAN |
| --- | ---: | ---: |
| `shoulder_flexion` | 22 | 2 |
| `shoulder_abduction` | 23 | 2 |
| `elbow_flexion` | 25 | 2 |

业务层只使用以上语义名称。motor/CAN 映射、方向、零位和安全参数统一位于 `configs/robot.yaml`。

> **硬件确认状态：** 三个 `direction` 只是待验证的调试假设，均标记为 `NEEDS_HARDWARE_VALIDATION`。真实零位、机械限位、最大速度、最大电流和最大跟踪误差目前都是 `null`；禁止把它们视为已知事实。

## SDK 位置与已确认行为

厂家 SDK 原样保存在 `third_party/dymotor_sdk/`，本项目不修改或复制其中内容。主要参考为 `example_C/pose_control_get_pvct.c`、`get_motor_list.c`、公开头文件和厂家 `CMakeLists.txt`。

已确认：

- `robot_create` + `robot_config_net` 创建并连接主板上下文。
- 厂家位置示例在 motor object 建立后执行主板状态机 `0x80 → 1`，但实机已确认该调用不适合放在只读连接路径；bridge 的 `arm_open` 不执行状态机、Servo 或位置命令。
- `get_robot_motorlist` + `robot_create_motorObjectList` 枚举电机；bridge 要求 22/CAN2、23/CAN2、25/CAN2 各精确出现一次。
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

## Step 1：只读取 PVCT

确保机械臂周围无人、可立即断电/急停，然后运行：

```bash
python tools/read_pvct.py
```

它只连接、验证三台电机并持续读取；不会执行状态机切换、Servo On/Off 或位置命令。输出中的 position/velocity/torque 已按配置的方向与零位转换为语义坐标。按 `Ctrl+C` 后只关闭未使能的 SDK 会话。

首先核对：三个 motor/CAN 地址正确、反馈有限且稳定、error 均为 0；不要在存在错误或映射不符时继续。

### PVCT bridge 只读诊断

修改 native bridge 后必须先重新构建，并确认不存在或未加载旧副本：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
find . -name 'libdymotor_bridge.so' -ls
```

然后用显式路径运行只读诊断：

```bash
python tools/debug_bridge.py \
    --joint all \
    --samples 3 \
    --library "$PWD/build/native/dymotor_bridge/libdymotor_bridge.so"
```

该工具只连接、确认配置中的 CAN2/motor ID22/23/25、重复读取 PVCT 并关闭；不会执行状态机切换、Servo On/Off、控制模式切换或位置发送。它同时打印 C 侧原始诊断、Python 收到的 `wrapper_status`、state/bus/error 十进制和十六进制，以及实际加载的 `.so` 绝对路径。`bus == 0` 会被判定为遥测缓存未就绪并返回 `wrapper_status=-4`，不会把全零输出当作有效反馈。

厂家示例不是只读程序：读取 PVCT 前已经执行状态机切换、位置模式、Servo On 和位置发送。不要直接运行它来做静止诊断，也不要为了得到非零 PVCT 而把这些调用加回 `debug_bridge.py`。如果严格只读连接仍无法获得有效缓存，需要厂家确认“Servo Off 状态下启用 fast telemetry”的安全初始化顺序。

与厂家示例对照时，先将示例中的 motor 改为相同的 ID/CAN，并删除或注释其 Servo On、位置模式和位置发送部分，仅保留初始化与 `robot_motor_get_PVCTFast` 读取。分别记录：

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

串口协议的字段按顺序命名 CH1、CH2……，因此配置中的 `sleeve_channel: 2` 是用户可读的 1-based 编号，内部只在 Predictor 中转换为 `SleeveFrame.channels[1]`。ID22、ID23 不由袖套预测；Mapper 始终给它们启动反馈位置。IMU 即使启用也暂时旁路，RuleBasedPredictor 只读取 Sleeve。

Phase 3 配置位于 `configs/phase3.yaml`。`input_min`/`input_max` 必须来自实测标定，默认 `null` 会阻止真实 Sleeve 控制启动。临时规则将 CH2 线性归一化并 clamp 到 `[0, 1]`，可用 `invert_input` 翻转方向；中心 deadzone 后可选 EMA。Mapper 将 `[0, 1]` 映射到启动肘位置附近 `-3°..+3°`，这只是首次验证窗口，不是机械限位，最终仍必须经过 Phase 1 的位置、单步、速度、跟踪误差和反馈错误检查。

Watchdog 使用 monotonic timestamp：超过 `sensor_timeout_ms` 不产生新目标并保持最后安全目标；超过 `hard_timeout_ms` 抛出故障并进入 Servo Off/close。无效、缺失或非有限 CH2 不会产生命令。

严格按以下顺序验收；只有最后一步可能 Servo On 并产生真实机械臂运动。

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

此模式连接并读取三路 PVCT、计算和预览 Safety 结果，但不 Servo On、不发送位置。

### 5. 真实 Sleeve + DyMotor execute

```bash
python tools/run_sleeve_elbow.py --sleeve real --robot dymotor --execute
```

必须在急停可用、现场监护、配置完成且前四步结果正确后执行。顺序为三路稳定反馈 → 保存启动位置 → Sleeve fresh/CH2 有效 → Servo On → 首条启动位置命令 → 仅 ID25 在启动位置附近小范围跟随。`Ctrl+C`、source/robot 异常或 hard timeout 都停止新目标并执行 Servo Off、robot close、source close。

以上是 Phase 3 的边界；Phase 3 本身不包含 CH3/CH4 肩部控制、训练模型、IMU 融合或三自由度袖套控制。Phase 4 在下面通过同一 `MotionPredictor.predict(SensorSample) -> MotionIntent` contract 接入外部模型，不修改 SafetyController 或 Robot 层。

## Phase 4 — FlexPredictor Model Integration

Phase 4 保留 Phase 3 的 CH2 肘部规则，并用 pip 安装的 `flex_model_0003.FlexPredictor` 生成肩部人体语义：

```text
CH2 ─→ RuleBasedPredictor ─→ normalized elbow ───────┐
CH2/CH3/CH4 ─→ FlexModelPredictor ─→ action+angle ──┼→ MotionIntent
                                                     ↓
                         startup-relative ArmMapper → SafeArmController
                                                     ↓
                                     one three-joint batch Robot command
```

通道严格按 `[CH2, CH3, CH4]` 传给模型，即 `SleeveFrame.channels[1:4]`。模型模块只在 `sleeve_arm/predictor/flex_model.py` 动态导入，`FlexPredictor()` 在 predictor 构造时初始化一次并在每帧复用；项目不复制或修改模型文件。若 pip 包未安装，会明确报告预期模块名，不会回退到假模型或 RuleBasedPredictor。

`configs/phase4.yaml` 中必须填写三组各 3 个有限值：

```yaml
calibration:
  baseline: [b1, b2, b3]
  scale: [s1, s2, s3]
  trial_rest: [r1, r2, r3]

angle:
  min_deg: 0.0
  max_deg: <模型训练标签的真实上限>
```

调用保持厂家 API 不变：

```python
result = model.predict_raw(
    flex=[ch2, ch3, ch4],
    calibration_baseline=baseline,
    calibration_scale=scale,
    trial_rest=trial_rest,
)
```

action 固定映射为 `0=Forward`、`1=Lateral`、`2=Backward`。`angle_deg` 先转换为人体语义：Forward → shoulder flexion `+A`，Backward → flexion `-A`，Lateral → abduction `+A`，非当前肩部轴为 neutral。它不会直接发送给电机；Mapper 将人体语义夹到启动反馈附近的肩部 ±5°验证窗口，SafetyController 再执行 Phase 1 的位置、单步、速度、跟踪误差和 PVCT 错误检查。

模型概率必须至少包含三个 `[0,1]` 有限值；当前 action 对应概率作为 confidence telemetry。`min_action_confidence` 按 Phase 4 约束暂不参与过滤，避免低置信度造成突然回零。新 action 必须连续满足 `required_consecutive_frames` 才切换；候选未稳定时保持上一条已接受肩部 intent。无效 action/概率/角度或模型异常时保持最后安全目标，连续达到配置阈值则 FAULT 并安全退出。IMU 当前不传给模型，保持 Optional，可全部关闭。

严格按以下顺序验证；只有最后一步可能 Servo On：

### 1. 模型单独测试（不连接 Robot）

```bash
python tools/test_flex_model.py
```

历史数据离线回放同样不连接 Sleeve 或 Robot：

```bash
python tools/test_flex_model.py --input recordings/.../samples.csv
```

### 2. 真实 Sleeve + Model + FakeRobot

```bash
python tools/run_model_control.py --sleeve real --robot fake
```

### 3. 真实 Sleeve + Model + DyMotor dry-run

```bash
python tools/run_model_control.py --sleeve real --robot dymotor
```

该步骤只读取真实 PVCT、推理、映射并预览 Safety 结果；不 Servo On、不发送位置。

### 4. 最终真机 execute

```bash
python tools/run_model_control.py --sleeve real --robot dymotor --execute
```

现场急停和监护必须就绪，并在前三步确认方向、角度和目标正确后，分别缓慢测试 Forward、Lateral、Backward。首次测试仍只允许启动位置附近的小范围，不代表人体绝对角度复现。

如果需要回归 Phase 3，将 `predictor.backend` 设为 `rule_based`，并继续使用 `tools/run_sleeve_elbow.py`。尚未实现 IMU-assisted inference、新模型训练和更细粒度方向模型。

## Step 2：测试肘关节

先 dry-run：

```bash
python tools/test_joint.py \
    --joint elbow_flexion \
    --delta-deg 1
```

程序从当前实测位置计算相对目标，不会跳到绝对零位，也不会 Servo On。确认打印的当前值、请求目标、安全限制和方向状态后，现场人员就位、急停可用时才执行：

```bash
python tools/test_joint.py \
    --joint elbow_flexion \
    --delta-deg 1 \
    --execute
```

`--execute` 仍经过 SafetyController：稳定反馈 → 当前位预装 → Servo On → 再读三路反馈 → 小步长目标 → 一次 batch flush → 持续监控 → Servo Off → close。

## Step 3：依次验证单关节

严格按顺序逐个确认正负方向和小增量行为：

1. `elbow_flexion`
2. `shoulder_abduction`
3. `shoulder_flexion`

每个关节先 dry-run，再用 **不超过 1°** 的单次 `--execute`。若方向与语义不一致，停止并只修改配置中的 `direction`；不要修改业务代码。验证后才把 `direction_status` 改为 `VALIDATED` 并记录验证过程。

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

- 启动不会自动运动；`--execute` 是唯一的运动许可开关。
- `SafeArmController` 是强制路径；tools 不直接调用 backend 的位置发送 API。
- Servo On 前必须精确发现三台目标电机，并连续取得多次无错误、有限反馈；相邻启动样本的位置变化不得超过各关节 `max_position_step`。
- C bridge 在 Servo On 前把实测当前位置预装 45 次，再使能，避免陈旧目标造成跳变。
- 已配置的 min/max 会 clamp；`max_position_step` 与 `max_velocity * dt` 同时存在时取更严格者；配置 `max_velocity` 后也检查实测反馈速度。
- 每次发送前 Python 和 C bridge 都复查三路反馈与 error；反馈/SDK 失败会停止发送并 Servo Off。
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
tools/read_pvct.py                 只读硬件验证
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

Phase 2 尚未在本项目中连接真实 Sleeve 或 IMU770 串口；端口名、真实采样率、丢包率、20 ms 同步阈值和 500 ms 缓冲时长仍需现场验证。真实模型、Motion prediction、Sleeve → Robot、IMU → model fusion 均未实现。
