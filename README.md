# sleeve_arm_control

面向真实机械臂的安全控制层，以及与机械臂完全隔离的传感器采集基础。

```text
Python tools -> SafetyController -> DyMotorArm (ctypes)
             -> libdymotor_bridge.so -> 厂家 libMotorDrive.so -> mechanical arm
```

## 项目当前阶段

Phase 1 原有三个机械臂关节的位置控制与 PVCT 读取现已扩展加入大臂旋转电机。Phase 2 新增袖套、Optional 双 IMU、时间同步和数据记录。Phase 3 新增临时规则式 CH2 → 肘关节小范围验证链路；大臂旋转目前仅支持独立小角度调试，尚未接入 IMU 或模型控制。

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

> CH2 与肘部、CH3/CH4 与肩部的关系仅是未来 Predictor 集成信息，当前没有实现任何机械臂 mapping。IMU 将来用于辅助模型判断人体手臂的运动方向和状态，而不是直接控制电机。

## 机械臂关节

| Semantic joint | Motor ID | CAN |
| --- | ---: | ---: |
| `shoulder_flexion` | 22 | 2 |
| `shoulder_abduction` | 23 | 2 |
| `elbow_flexion` | 25 | 2 |
| `upper_arm_rotation` | 24 | 2 |

业务层只使用以上语义名称。motor/CAN 映射、方向、零位和安全参数统一位于 `configs/robot.yaml`。

> **ID24 硬件确认状态：** `upper_arm_rotation.direction: 1` 只是配置格式要求的临时符号，仍标记为 `NEEDS_HARDWARE_VALIDATION`；其零位和机械限位均为 `null`。只允许从当前反馈位置做小角度、有人监护的方向验证。当前 SDK 路径不提供速度命令，本项目也没有添加速度限制；当前没有 IMU → ID24 控制。

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

串口协议的字段按顺序命名 CH1、CH2……，因此配置中的 `sleeve_channel: 2` 是用户可读的 1-based 编号，内部只在 Predictor 中转换为 `SleeveFrame.channels[1]`。ID22、ID23 不由袖套预测；Mapper 始终给它们启动反馈位置。IMU 即使启用也暂时旁路，RuleBasedPredictor 只读取 Sleeve。

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

以上是 Phase 3 的边界；Phase 3 本身不包含 CH3/CH4 肩部控制、训练模型、IMU 融合或三自由度袖套控制。Phase 4 在下面通过同一 `MotionPredictor.predict(SensorSample) -> MotionIntent` contract 接入外部模型，不修改 SafetyController 或 Robot 层。

## Phase 4 — FlexArmEstimator Model Integration（当前）

当前肩部模型已经迁移到 `flexarm-estimator 0.1.0`。本仓库不复制模型包源码、
wheel 或训练权重；控制代码只调用已安装包的公开入口，并从外置目录加载模型产物。

安装 wheel（远程控制环境已安装时无需重复执行）：

```powershell
python -m pip install "E:\PythonProject\electronic_skin_project\袖套控制\arm_data_collector\flexarm_estimator\dist\flexarm_estimator-0.1.0-py3-none-any.whl"
```

`configs/phase4.yaml` 中的 `model_dir` 必须指向包含 `metadata.json`、分类器和三个
角度回归器的 `models` 子目录。当前配置为：

```yaml
predictor:
  backend: flexarm_estimator
  model_dir: E:/PythonProject/electronic_skin_project/袖套控制/arm_data_collector/flexarm_estimator/models
  sleeve_channels: [3, 4, 5]
  calibration_file: ../calibrations/flexarm_live_calibration.json
  calibration_seconds: 3.0
```

若远程系统目录不同，只修改 `model_dir`；不需要把 `flexarm_estimator/src` 加入本项目。

当前数据流：

```text
CH2 -> RuleBasedPredictor ---------------------------> elbow_flexion
CH3/CH4/CH5 -> FlexArmEstimator.update(timestamp_ns) -> shoulder action/angle
                         -> ArmMotionPredictor -> ArmMapper
                         -> SafeArmController -> Robot
```

每次重新穿戴袖套后，默认执行约 3 秒自然下垂标定。单独标定且不连接机器人：

```powershell
python tools/calibrate_flex_model.py
```

模型单独测试默认也会现场标定：

```powershell
python tools/test_flex_model.py
```

仅在确认保存的标定与当前穿戴一致时显式复用：

```powershell
python tools/test_flex_model.py --reuse-calibration
python tools/run_model_control.py --sleeve real --robot fake --reuse-calibration
```

模型产生 `Forward`、`Backward`、`Lateral` 时分别映射为肩前屈、负向肩前屈和
肩外展绝对语义角。`Rest` 与 `Unknown` 是正常状态：肩部保持最近一次有效目标；
尚无有效活动预测时保持机器人启动位置；CH2 肘部预测仍继续更新。日志同时输出
`action_conf`、`angle_conf`、`moving`、模型角度与推理时间。

严格按以下顺序进行硬件验证；DyMotor dry-run 也会执行厂家 Servo On pipeline，只有最后一步会发送模型生成的位置目标：

```powershell
# 1. 真实 Sleeve + FakeRobot（默认现场标定）
python tools/run_model_control.py --sleeve real --robot fake

# 2. 真实 Sleeve + DyMotor dry-run；厂家 pipeline 会 Servo On，但不发模型目标
python tools/run_model_control.py --sleeve real --robot dymotor

# 3. 现场急停、监护、零位和限位均确认后才执行
python tools/run_model_control.py --sleeve real --robot dymotor --execute
```

模型包导入、权重加载、标定、传感器有效性和模型预热均在机器人连接/使能前完成。
模型位置目标仍必须经过 `SafeArmController`，且必须显式提供 `--execute`；但连接阶段的厂家 Servo On 不受该开关控制。

## Phase 4 — Legacy FlexPredictor Notes（已废弃，请勿执行）

### Offline mapper diagnostic

To inspect a manually supplied model result without opening any Sleeve, serial port, bridge library, or robot:

```bash
python tools/debug_model_mapping.py \
    --action Backward \
    --shoulder-angle-deg 30 \
    --elbow-angle-deg 90
```

The output separates the absolute semantic mapper request, the SafetyController-equivalent result for one control period, and the final raw SDK request after `zero_position + direction × semantic_position`. Optional `--current-*-deg` arguments simulate the current semantic joint feedback used by step/velocity limiting.

To pass the same manual output through the production robot lifecycle, first preview with FakeRobot, then use read-only DyMotor preview, and only then explicitly execute:

```bash
python tools/run_manual_model_control.py --action Forward --shoulder-angle-deg 5 --elbow-angle-deg 30 --upper-arm-rotation-deg 10
python tools/run_manual_model_control.py --action Forward --shoulder-angle-deg 5 --elbow-angle-deg 30 --upper-arm-rotation-deg 10 --robot dymotor
python tools/run_manual_model_control.py --action Forward --shoulder-angle-deg 5 --elbow-angle-deg 30 --upper-arm-rotation-deg 10 --robot dymotor --execute
```

`--upper-arm-rotation-deg` is an absolute semantic angle. It is clamped by ID24's calibrated `min_position/max_position`, then converted using `SDK target = zero_position + direction × semantic target`. Omitting it keeps ID24 at its measured startup position. Real execution requires calibrated zero/min/max for all four joints. The first enabled command holds all measured startup positions, then all four targets pass through `SafeArmController` and one SDK batch. Ctrl+C, feedback faults, and timeouts all enter Servo Off and close.

Phase 4 保留 Phase 3 的 CH2 肘部规则，并用 pip 安装的 `flex_model_0003.FlexPredictor` 生成肩部人体语义：

```text
CH2 ─→ RuleBasedPredictor ─→ absolute elbow rad ─────┐
CH2/CH3/CH4 ─→ FlexModelPredictor ─→ action+angle ──┼→ MotionIntent
                                                     ↓
                         absolute shoulder ArmMapper → SafeArmController
                                                     ↓
                                     one three-joint batch Robot command
```

通道严格按 `[CH2, CH3, CH4]` 传给模型，即 `SleeveFrame.channels[1:4]`。模型模块只在 `sleeve_arm/predictor/flex_model.py` 动态导入，`FlexPredictor()` 在 predictor 构造时初始化一次并在每帧复用；项目不复制或修改模型文件。若 pip 包未安装，会明确报告预期模块名，不会回退到假模型或 RuleBasedPredictor。

每次穿戴袖套后，先运行纯 Flex 快速标定；该工具不会创建或连接 Robot：

```bash
python tools/calibrate_flex_model.py
```

它复用 `flex_model_0003` 提供的 `collect_flex_samples()`、`quick_calibrate_flex()` 和 `FlexCalibration.save_json()`，依次采集 3 秒自然下垂基线、12 秒前/侧/后三个完整动作、2 秒 trial rest。输入严格为 `[CH2, CH3, CH4]`。结果默认写入被 `.gitignore` 排除的：

```text
calibrations/flex_calibration.json
```

`configs/phase4.yaml` 只引用该文件：

```yaml
calibration_file: calibrations/flex_calibration.json
```

生成的 JSON 必须包含三组各 3 个有限值：

```yaml
calibration_baseline: [b1, b2, b3]
calibration_scale: [s1, s2, s3]
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

action 接受模型的 `Forward`、`Lateral`、`Backward` 标签，也兼容整数 `0`、`1`、`2`，并固定映射为 `0=Forward`、`1=Lateral`、`2=Backward`。概率既可按该顺序返回序列，也可返回使用这三个标签作为键的映射。`angle_deg` 是相对人体标定零位的绝对关节角：Forward → shoulder flexion `+A`，Backward → flexion `-A`，Lateral → abduction `+A`，非当前肩部轴为绝对语义零位。Mapper 直接输出该绝对语义 rad；DyMotor backend 再以 `SDK target = zero_position + direction × semantic target` 转换。真实执行要求肩部 `zero_position/min_position/max_position` 已标定，SafetyController 继续执行位置、单步、速度、跟踪误差和 PVCT 错误检查。

模型概率必须至少包含三个 `[0,1]` 有限值；当前 action 对应概率作为 confidence telemetry。`min_action_confidence` 按 Phase 4 约束暂不参与过滤，避免低置信度造成突然回零。新 action 必须连续满足 `required_consecutive_frames` 才切换；候选未稳定时保持上一条已接受肩部 intent。无效 action/概率/角度或模型异常时保持最后安全目标，连续达到配置阈值则 FAULT 并安全退出。IMU 当前不传给模型，保持 Optional，可全部关闭。

严格按以下顺序验证；DyMotor dry-run 也会执行厂家 Servo On pipeline，只有最后一步会发送模型目标：

### 0. 每次穿戴后的快速标定（不连接 Robot）

```bash
python tools/calibrate_flex_model.py
```

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

该步骤执行厂家 Servo On pipeline，再读取真实 PVCT、推理、映射并预览 Safety 结果；不发送模型生成的位置目标。

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

Phase 2 尚未在本项目中连接真实 Sleeve 或 IMU770 串口；端口名、真实采样率、丢包率、20 ms 同步阈值和 500 ms 缓冲时长仍需现场验证。真实模型、Motion prediction、Sleeve → Robot、IMU → model fusion 均未实现。
