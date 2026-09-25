# Fourier Aurora 0.1.8 控制与现场确认

当前 backend 固定使用 **`fourier_aurora_client==0.1.8`**，不调用 1.0.1 的 configure/start/lease API。已实现左右臂具名关节、方向角度、有限摆臂、默认 preview、fake 和模型控制入口。本次只做离线验收；**没有启动真实 DDS，没有运行真机 connect/execute，也没有确认现场控制可用。** 未验证模板会阻止真实 execute。

## 安装与版本依据

在 WSL/Linux 终端、项目根目录执行：

```bash
conda activate skin
python -m pip install -r requirements.txt
python -m pip install -r requirements-aurora.txt
python -m pip check
python tools/aurora_sdk_doctor.py
```

本机是 Ubuntu 22.04 / WSL2 / x86_64，解释器 `/home/lkw/miniconda3/envs/skin/bin/python`，Python 3.10.20。0.1.8 wheel 内含 Linux ELF 原生库；未验证原生 Windows/ARM。SDK 是可选依赖，缺失时 fake、DyMotor、传感器与 `--help` 仍可使用；不会自动安装、升级或修改 SDK、DDS 环境变量或 LD_LIBRARY_PATH。

实际安装/API 审计见 [aurora_sdk_environment.md](aurora_sdk_environment.md)。本轮起始项目 HEAD 为 `edfedf272c4bf9e11c67a0631b162ba3a9d42de4`，保留已有修改，未切分支、reset、stash、提交或 push。当前官方 main 为另一套 1.0.1 API；本轮依据是 **PyPI 0.1.8 的实际安装源码及方法签名**，并核对官方历史 revision `b434869719ae256d02d8285e88ea2b22c1515602` 的 [API 文档](https://github.com/FFTAI/fourier_aurora_sdk/blob/b434869719ae256d02d8285e88ea2b22c1515602/python/docs/API_document_EN.md)、[GR3 关节示例](https://github.com/FFTAI/fourier_aurora_sdk/blob/b434869719ae256d02d8285e88ea2b22c1515602/python/example/gr3/demo_joint_command.py)。没有运行这些示例。

| 实际 SDK 调用 | 参数、返回和限制 |
|---|---|
| `AuroraClient.get_instance(domain_id, participant_qos=None, robot_name=None, namespace=None, is_ros_compatible=None)` | domain 必填。配置支持 domain、robot_name、namespace、ROS 命名；不猜网络字段。初始化失败有时记录日志并返回 None，adapter 明确拒绝 |
| `client.get_fsm_state()` | 返回整数 FSM；可能 KeyError；每次命令前重新检查，不切换 FSM |
| `client.get_group_state(group_name, key='position')` | 返回缓存的 **list[float]**；key 为 position/velocity/effort。不存在组/key 时 KeyError。没有 timestamp 或 SDK OperationResult |
| `client.set_group_cmd(position_cmd, velocity_cmd=None, torque_cmd=None)` | `dict[str, list[float]]`：组名映射完整向量。本项目只提供 position_cmd，不发送速度、力矩、FSM 或增益命令 |
| `client.close()` | 返回 None，关闭本应用客户端，不是 Servo Off/急停。SDK 单例没有公开 reset 接口 |

业务和安全层单位是 rad，速度为 rad/s。SDK 的 position 数组不做 degree 转换；历史 [GR2 步行示例](https://github.com/FFTAI/fourier_aurora_sdk/blob/b434869719ae256d02d8285e88ea2b22c1515602/python/example/gr2/demo_walk.py) 将传给关节位置命令的 policy 输出标注为 rad。现场仍需确认具体组/关节语义，不能将示例布局和限位迁移到未知机器人。degree 只在高层动作 API/CLI 边界转换。

**0.1.8 的发送不是可靠成功回执。** `set_group_cmd` 返回 None，内部组消息构造和 `Publisher.publish` 的部分错误仅写日志，底层 write 返回码也未上抛。adapter 捕获抛出的异常和 SDK ERROR 日志，锁存共享故障；对无法观测的丢包只能依靠新反馈、跟踪误差和有界到位超时。结果字段 `submitted=true` 只表示调用返回；`delivery_confirmed=false`，不能解释成机器人收到命令。`arrived=true` 要求提交后观察到新的有效反馈且目标在容差内。

## 架构与保持兼容的部分

```text
SensorSample → Predictor/Estimator → MotionIntent → AuroraIntentMapper
                                                     ↓
CLI / AuroraMotionService → SafeArmController → AuroraRobotArm
                                                     ↓
                                  AuroraSession (0.1.8 SDK adapter)
                                                     ↓
                                    fourier_aurora_client==0.1.8
```

`aurora_profile.py` 只描述部署事实与安全参数；`aurora_session.py` 负责 SDK、缓存更新观测、共享生命周期、FSM、命令 owner 和异常转换；`aurora.py` 做一次坐标转换和完整组命令；`aurora_motion.py` 复用 `upper_limb.py` 的有界轨迹，最后通过共同安全控制器提交。

`create_robot()` 工厂支持 fake/dymotor/aurora/aurora-fake。模型入口与手动模型入口均复用工厂；手动模型入口本轮仍只提供 fake/dymotor，Aurora 手动动作使用独立 CLI。没有修改训练、传感器协议、标定算法或 DyMotor native ABI。`JOINT_NAMES`/`DYMOTOR_JOINT_NAMES` 仍是原四关节，ctypes 数组长度仍为 4；Aurora 受控关节取自 profile，不伪造 motor_id、can_id 或 DyMotor 网络字段。

## 只读能力探测

离线 API doctor 不建立 DDS：

```bash
python tools/aurora_sdk_doctor.py
# 等价入口，复用同一个 doctor：
python tools/aurora_control.py doctor
```

现场明确授权连接时，可手动执行以下命令。**会启动 DDS，仅创建状态订阅，不调用 AuroraClient.get_instance、不创建命令 publisher、不发送动作；本轮未执行。** Domain ID 必须先与现场配置核对，123 仅是官方示例值：

```bash
python tools/aurora_sdk_doctor.py --connect --domain-id 123
# 或：
python tools/aurora_control.py doctor --connect --domain-id 123
```

namespace/ROS 命名可用原 doctor 的 `--namespace`、`--ros-compatible` / `--no-ros-compatible`；不覆盖环境变量。输出真实收到的组名、位置/速度/力矩向量、维度、FSM、端点匹配、消息接收新鲜度。0.1.8 状态消息不提供 robot_type/hardware_type/end_effector_type，输出为 unavailable，不能据此猜型号。组名和维度也不能证明 joint ordering。

与 doctor 不同，**backend 的 `connect()` 按要求调用完整 `AuroraClient.get_instance(...)`**。该 SDK 自身会创建所有订阅、publisher 和 policy service client，等待匹配；此过程没有动作调用，但也不是最小订阅连接。匹配失败明确终止，不修改 SDK 私有初始化、不做 velocity publisher 补丁、不吞异常。现场旧版服务若缺少 SDK 要求的端点，需要厂家确认匹配版本；本项目不猜兼容性。

## 配置：未知值必须保持未知

复制独立模板，不能使用 DyMotor 配置代替：

```bash
cp configs/robot_aurora.yaml configs/aurora.site.yaml
```

`configs/aurora.unverified.yaml` 保留为同内容兼容路径。模板所有硬件字段均为 null/false，FSM 为空，**不能 execute**。结构使用 `groups` 列表，每个条目明确 side/part；字段对应关系如下：

| 字段 | 意义 / 必须确认的内容 |
|---|---|
| `api_family`, `sdk_version` | 必须为 `aurora-python-dds-0.1.8`、`"0.1.8"` |
| `connection.domain_id` | 实际 Domain ID，模板 null，无默认真实值 |
| `connection.robot_name` | SDK 可选参数；如果服务部署/示例需要，填写核实名称，不是型号反馈 |
| `connection.namespace`, `is_ros_compatible` | 实际端点命名；null 的 ROS 选项保留 SDK 读取现有环境的行为 |
| `robot_type`, `verification_note` | 现场核实的型号、依据/日期等。SDK 本身无法自动验证型号，需要设备记录补证 |
| `verified`, `authority_verified` | 显式确认硬件参数及本应用可独占所选组；不是通过软件猜测出来的能力 |
| `allowed_fsm` | 现场允许控制的 FSM 整数集合，空列表阻止 execute；不默认 10 或 13 |
| `control_hz` | 已确认的应用控制频率 1–500 Hz，模板 null |
| `feedback_timeout_s`, `endpoint_timeout_s` | 缓存更新时效/等待新样本上限；模板提供诊断默认值，整份 profile 的 verified 意味着现场复核这些值 |
| `arrival_tolerance_rad`, `arrival_timeout_s` | 到位容差和有界等待时间 |
| `stop_policy` | 仅支持明确接受的 `stop_publishing`，模板 null；没有 release lease/FSM 行为 |
| `groups[].name`, `side`, `part`, `count` | 实际组名、left/right、arm/hand、**完整 expected_dof**；不写死 manipulator 名称或七关节 |
| `groups[].sdk_position_limits` | 长度等于 count 的 `[SDK下限, SDK上限]` 列表，必须覆盖每一槽，包括未具名槽位；不能填猜测值 |
| `groups[].max_tracking_error` | 完整组的最大跟踪差 rad，也检查未映射槽位 |
| `groups[].capabilities`, `verified` | 已确认的 joint_position 等能力、该侧实际组的验证状态 |
| `joints[].name`, `index` | 具名语义关节及组内索引；不按示例猜顺序 |
| `joints[].sign`, `zero` | 分别是 direction（±1）与 zero_position（SDK rad） |
| `joints[].limits` | 语义 rad 的 min/max_position、max_position_step、max_tracking_error，以及 rad/s 的 max_velocity；缺失则拒绝 execute |
| `joints[].verified`, `kind` | 关节校准已验证；kind 为实际 arm/wrist/finger，不虚构 palm 关节 |
| `simulated` | 合成测试 profile 标志；true 永远不能用于真实 execute |

只要求本次选定侧/部位的硬件映射齐全，未选择的另一侧可继续保留未验证。每个已选择组都必须具备完整 SDK 槽位限位，即使此次只控制一个肩关节。未提供的电流、逐电机错误、bus/state 使用 None；如果要求相应安全策略，直接拒绝，不能填零绕过。velocity 缺失时 max_velocity 反馈策略也拒绝控制；effort 缺失保留 None。

0.1.8 没有 lease API，但这不等于已经获得控制权。官方 GR3 示例先手动切 FSM 并设置增益，**本项目不复制这些写操作**，也没有从示例证明现场的隐式 authority。现场必须独立确认 FSM、伺服/控制模式/增益、其他写入方和组控制权条件，再标记 authority_verified。一个应用 owner 不能防止其他进程或外部控制器抢写同组。

## 角度与方向语义

所有方向均以机器人自身为参照，不是观察者视角：

| direction | 语义目标 |
|---|---|
| forward | shoulder_flexion 正方向（肩前屈） |
| backward | shoulder_flexion 负方向（肩后伸） |
| outward | shoulder_abduction 正方向（远离身体中线） |
| inward | shoulder_abduction 负方向（靠近身体中线） |

转换只在 backend 做一次：`q_sdk = zero + sign * q_semantic`；反馈为 `q_semantic = sign * (q_sdk - zero)`。左右侧分别配置 sign/zero，不能统一假定右侧取反。Mapper 只传语义弧度。

`reference=neutral` 是相对校准中立位的绝对目标，如 forward 20° → 语义 +20°。`reference=current` 在动作开始读取新反馈后**只计算一次**增量目标，如当前 +20° 加 5° → 固定目标 +25°，后续轨迹不再累加。`move_joint` 支持有符号角度。第一版没有末端 Cartesian、IK 或世界坐标移动。

## preview 与 fake

以下默认 **NO MOTION，也不连接 DDS**：

```bash
python tools/aurora_control.py move --side left --direction forward --angle-deg 10
python tools/aurora_control.py joint --side right --joint elbow_flexion --angle-deg 10
```

输出 side/group/joint/index、SDK/语义当前值、请求值、限幅值、SDK 目标、完整组前后向量、FSM 和阻止执行的原因。未取得真实反馈时 current/before/after/FSM 为 null；未校准转换时 SDK target 为 null，不伪造当前零位。限幅值只是诊断，实际 execute 遇到超限会拒绝原请求。

无需 SDK 的合成反馈 preview 与有界 fake 轨迹：

```bash
python tools/aurora_control.py --backend fake move --side left --direction forward --angle-deg 10
python tools/aurora_control.py --backend fake joint --side right --joint upper_arm_rotation --angle-deg 3 --duration 2 --simulate
python tools/aurora_control.py --backend fake swing --side left --joint shoulder_flexion --amplitude-deg 3 --period 4 --cycles 2 --simulate
```

`--simulate` 只能用于 fake，不接受 `--execute` 或真实连接；fake profile 的 7 槽、6 个具名关节、不同符号/零位、FSM 42 均是**测试夹具，不是 GR3 参数**。它刻意含一个未具名的非零槽位，用于验证不会清零其余位置。

已有组名、DOF 和坐标映射后，可显式只读连接生成真实 preview：

```bash
python tools/aurora_control.py --profile configs/aurora.site.yaml move --side left --direction forward --angle-deg 2 --reference current --connect
```

这仍复用 SDK doctor 的只读订阅，不运行 backend 的完整控制客户端。未收到新鲜反馈或模板不齐全，输出阻止原因，不执行动作。

## Python API 与轨迹

无硬件即可运行的 Python 示例：

```python
from sleeve_arm.robot.aurora_fake import fake_session
from sleeve_arm.robot.factory import create_robot
from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.control.aurora_motion import AuroraMotionService

session = fake_session()  # FakeClock，不需要真实等待
robot = create_robot("aurora-fake", session=session, side="left")
controller = SafeArmController(robot, robot.config, clock=session.clock)
try:
    controller.connect()
    controller.enable()
    motion = AuroraMotionService(controller)
    motion.move_arm(side="left", direction="forward", angle_deg=3,
                    reference="current", duration_s=2)
    motion.move_joint(side="left", joint="elbow_flexion", angle_deg=5,
                      reference="neutral", duration_s=2)
    motion.swing_arm(side="left", joint="shoulder_flexion", amplitude_deg=3,
                     period_s=4, cycles=2, center="current")
finally:
    controller.shutdown()
```

`AuroraRobotArm` 支持 connect/enable/disable/read_joint_state/read_joint_states/set_joint_position/set_joint_positions/close；`read_group_positions(side)` 返回完整 SDK 位置向量的副本。直接 backend setter 也检查 FSM、范围、单步变化、速度与组 owner；应用动作推荐通过 SafeArmController 和 motion service。

双臂 fake 可将上例创建替换为：

```python
robot = create_robot("aurora-fake", session=session, sides=("left", "right"))
# controller.connect()/enable() 后：
motion.move_joints({"left.elbow_flexion": 0.03, "right.elbow_flexion": 0.04}, duration_s=2)
```

同组多目标先合并；双组先全部验证，再一次 `set_group_cmd(position_cmd={...})` 提交。**不保证硬件原子同步**。同一进程的真实 factory/AuroraSession.real 复用同配置客户端，拒绝冲突配置、重叠组 owner、第二个命令线程。关闭单侧不会关闭仍被另一侧引用的会话，最后引用才 client.close。SDK 关闭后保留其私有单例，本项目不修改 `_instance`；后续真实连接必须重启进程，不能自动复用关闭的 client。只读检查已有 `_instance` 可拒绝接管其他库建立的客户端，不写这个字段。

轨迹为 cubic smoothstep；swing 为有界 raised-cosine 往复，显式平滑进入下端点并最后回到中心，不回机器人全局零位。duration、period、cycles、幅度等拒绝 NaN/Inf/非法范围；周期至少有 20 个控制点，cycles 1–1000，摆动主体不超过 3600 秒。entry/exit 各使用 `settle_duration_s`。时钟为 monotonic，每步经过安全层，调度过晚失败退出，不按延迟放大步长。被安全层限幅的手动轨迹直接报告未完成，不冒称完整完成原请求。

## 完整组更新和反馈时效

SDK setter 为每个 dict 键构造完整 ControlGroupCmd.position，没有 sparse index 参数。例如只改变 index 0 时：

```text
before = [0.12, -0.21, 0.50, -0.70, 0.03, 0.11, 0.00]
after  = [0.30, -0.21, 0.50, -0.70, 0.03, 0.11, 0.00]
```

初始完整目标来自新鲜实测 vector，之后未更新槽位保留本 owner 的上次验证目标。一次只控制左臂时，命令 dict 只含左臂组；不附带右臂、手指、腰腿头。整个批次在 SDK 调用前完成维度、有限性、SDK/语义范围和变化率检查。

每个组每轮只读取一次 position list，再为全部语义关节提取位置。velocity/effort 各用一次对应 getter；0.1.8 无跨 key/跨组原子快照，不声称它们是同一物理采样时刻。

0.1.8 getter 直接返回缓存 list；已审计的回调在每次消息到达时创建替换 list。本项目保留上次 list 引用，只有身份发生变化才认定缓存样本更新。相同位置值但新 list 仍是新样本；重复读旧 list 不刷新新鲜度。第一次观察无法知道缓存年龄，必须再观察到一次替换。观测时间使用前一次读取的 monotonic 时间作为保守下界，**不是机器人采样时间或 SDK 接收 timestamp**。兼容 JointState 的 `received_at` 在此 backend 承载这个下界。

在人机确认/标定造成长间隔后、尚未 enable 时，会有界等待重新建立新鲜起点；运动中反馈过期立即故障，不自动恢复。到位还要求最终调用返回后再观察到新的位置样本。FSM getter 同样是缓存，但其公共 API 没有接收时间信息；本版能逐命令检查 FSM 值，不能独立证明 FSM topic 的接收年龄。这个限制需要现场通信/控制安全条件补证，不能用组数据新鲜推断所有状态新鲜。

## 真机 execute 与首次小角度验收

只有同时满足：真实 backend、`--execute`、SDK 正好 0.1.8、已确认的 domain/group/完整 DOF/index/方向/零位/限位/安全参数/FSM、现场独占控制条件、新鲜有效反馈，且交互输入**恰好 `YES`**，才允许动作。`yes`、`YES `、空输入都取消；EOF/Ctrl+C 退出。没有自动 FSM 切换、强制切换、抢占或 Servo On。`enable()` 仅打开本应用发送门控。

建议先只读确认身份/网络/反馈，再由现场独立流程确认允许 FSM 和控制模式、排除其他写入方，核实整组映射与限位。现场须有人监护、运动空间清空、独立硬件急停可用。**joint limit 检查不等于 self-collision avoidance，也不等于 environment collision avoidance。**

完成配置后，先看 preview；然后首次只试一侧、2° 的 current-relative 增量：

```bash
python tools/aurora_control.py --backend aurora --profile configs/aurora.site.yaml move --side left --direction forward --angle-deg 2 --reference current --duration 2 --connect
# 现场检查 preview 后，独立执行以下命令，并交互输入 YES：
python tools/aurora_control.py --backend aurora --profile configs/aurora.site.yaml move --side left --direction forward --angle-deg 2 --reference current --duration 2 --execute
```

每次只验证一个关节/方向，不将左侧结果推及右侧。上面命令仅作现场操作说明，本轮没有执行。未验证模板无论是否输入 YES 都不能 execute。

## 袖套/IMU/模型入口

```bash
# 不连接真实机器人：先用原已配置好的传感器/模型文件进行 fake 链路验收
python tools/run_model_control.py --robot aurora-fake --arm-side right --source-side right --sleeve fake --imus fake --duration 5
# 真机配置的离线预检，不打开传感器或 DDS
python tools/run_model_control.py --robot aurora --arm-side right --source-side right --aurora-profile configs/aurora.site.yaml
# 现场参数与标定全部完成后才使用；仍需交互 YES
python tools/run_model_control.py --robot aurora --arm-side right --source-side right --aurora-profile configs/aurora.site.yaml --sleeve real --imus real --duration 5 --execute
```

第一条仍要求原配置中的模型目录/标定存在、启用的 IMU 配置正确；fake 机器人不虚构训练权重。`--side` 保留为 `--arm-side` 的兼容别名。当前 shoulder predictors/标定提示是**右侧**合同，要求 source-side=right 且 arm-side=right；没有左右镜像，不会把一个袖套复制到双臂。独立手动 CLI 支持左侧，不代表右肩模型已适配左肩。

保持原单一控制线程、sensor watchdog、预测错误/IMU 错误处理和 finally 退出；传感器不调用 SDK。Aurora 传感器超时/预测错误立即阻止写入并关闭本应用。DyMotor 原 connect 仍会执行厂家 Servo On 等启动步骤，不能把所有 backend 的“无 execute”统称为无动作。

## 手部、停止与故障

预留 `set_hand_joint(side, joint, position)` / `set_hand_joints(side, targets)`，复用相同 session、owner 和组安全检查。没有 hand profile 时必定拒绝；`hand-joints` CLI 当前只在 preview 显示 unsupported，execute 拒绝。没有假定五指、六电机或任何固定手指顺序，也没有整手闭合度标定。具名物理腕关节只有 profile 真有映射/能力才可用；不提供虚构 palm 或 IK。

局部数据/限位/跟踪故障锁存当前 view/组，不给另一侧发送命令；FSM、SDK 异常和 SDK ERROR 日志故障锁存整个共享会话。SDK 日志处理只置线程安全事件，不在回调里操作客户端。故障组不能自动重新加入存活会话，必须排查后重启并重新确认。

`disable()` 立即禁止本应用新目标；`close()` 幂等关闭自有引用/最终客户端。默认不回零、不恢复初始姿态、不发送全零或“安全保持”、不切全局 FSM。**关闭 AuroraClient ≠ 硬件急停；停止发布 ≠ 机器人已物理停止。** 0.1.8 没有 lease API，也没有在退出时虚构 lease release。硬件急停和机器人端超时/停止行为必须由现场独立安全手段保障。

## NEEDS_HARDWARE_VALIDATION

- domain_id、namespace、ROS 命名与真实 SDK/Server 消息版本兼容性；完整客户端要求的端点是否存在。
- robot_type / hardware_type / end-effector 类型，及设备身份的独立依据。
- **left group、right group、完整 DOF、每个 joint index/ordering**。
- **每侧每个 joint direction、zero、semantic limits、完整 SDK slot limits**。
- **allowed FSM**、伺服/控制模式/增益、独占 authority、停止发布后的机器人行为。
- control_hz、max_position_step、max_velocity、各关节及完整组 max_tracking_error、反馈超时与到位阈值。
- **hand group、hand DOF、hand ordering** 及其方向/零位/限位、实际腕/末端能力。

以上没有自动从 GR3 示例填写。只读 doctor 能确认组名/维度/数值，不足以批准运动。

## 离线验收

```bash
python -m pytest -q tests/test_aurora*
python -m pytest -q
```

本轮将原 1.0.1 的 OperationResult/lease mock 测试迁移为实际 0.1.8 的 list/None/异常/日志合同测试，保留并扩展方向、ABI、完整组、陈旧反馈、跟踪/限位、单 owner、共享关闭、轨迹、到位、Ctrl+C 和模型 watchdog 覆盖；没有改动 DyMotor/FakeRobot/传感器已有测试，尤其没有改动已知 IMU770 CSV 失败测试。

本轮在 Conda `skin`（Python 3.10.20、`fourier_aurora_client==0.1.8`）执行：

| 检查 | 实际结果 |
| --- | --- |
| `python -m pytest -q tests/test_aurora* --tb=short` | 165 passed，1 warning |
| `python -m pytest -q --tb=short` | 360 passed，1 failed，2 warnings |
| `python -m pip check` | No broken requirements found |
| `python tools/aurora_sdk_doctor.py` | 0.1.8，`dds_started: false` |
| 默认 preview、无 SDK 的 `--help` | 通过；没有启动 DDS |
| fake joint / finite swing `--simulate` | 提交与反馈到位检查通过；只使用 fake client/clock |

唯一失败是原有 `tests/test_imu770_upper_arm_twist_demo.py::test_run_calibrates_records_live_pairs_and_closes_both_ports`：测试流在标定期间耗尽，随后读取不存在的 `live.csv`。该失败已在未修改的 HEAD 单独复现，本轮没有改动此测试。warnings 为已有 `.pytest_cache` 写权限问题。

**Aurora changes introduced no new regression.** 所有真机连接与动作仍未验证；测试没有启动真实 DDS、调用真实 `set_group_cmd` 或切换 FSM。
