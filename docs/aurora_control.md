# Fourier Aurora 控制

当前实现支持按现场 profile 配置的左右臂具名关节、机器人自身方向的角度运动、有界摆臂，以及能力驱动的腕部/手指接口。所有交付测试均离线；没有连接真机、DDS 或切换真实 FSM。随附真机模板未验证，**不能执行动作**。

## SDK 与安装

正式支持 API 家族 `aurora-configure-lease-v1`，客户端包 `fourier-aurora-client==1.0.1`，核实源码提交 `3d4e02e170364dff83e7f35e29b55c86cf728b92`。完整版本/API/改动记录见 [aurora_audit.md](aurora_audit.md)。未验证的客户端版本、旧 get_group_state/set_group_cmd 家族会明确失败，不会试探控制或自动回退。

官方 Python wheel 针对 Linux x86_64、标准 GIL CPython 3.10–3.13。现场需由维护人员取得匹配平台的官方 wheel，先安装官方 REVISIONS.yaml 对应的 DDS/message runtimes，再在选定 Python 环境安装：

```bash
python -m pip install /path/to/fourier_aurora_client-1.0.1-PLATFORM.whl
```

路径是现场 wheel 的占位符。DDS `fourier_dds 1.2.0-1`、`fourier-dds-msgs 0.1.0-1` 的确切 revision 见审计。客户端包版本不等于 Aurora 服务/固件版本；本项目不安装或升级机器人服务、固件。SDK 仅在真实 session.connect 时导入；所有 help、fake、传感器和 DyMotor 不要求此包。缺包会输出安装指引，没有运行时自动安装。

普通离线环境：

```bash
python -m pip install -r requirements-dev.txt
python tools/aurora_control.py --help
python tools/aurora_control.py doctor
python tools/aurora_control.py move --side left --direction forward --angle-deg 5 --duration 2
python tools/aurora_control.py joint --side right --joint upper_arm_rotation --angle-deg -3 --duration 2
python tools/aurora_control.py swing --side right --joint elbow_flexion --amplitude-deg 3 --period 2 --cycles 2
python tools/aurora_control.py hand-joints --side left --angle index_flexion 3 --angle thumb_opposition 2 --duration 2
```

默认 fake 使用 FakeAuroraClient 和虚拟时钟，摆臂无需实际等待。返回 `published` 和 `arrived` 分别表示已发布与反馈到位；`simulation=true` 不能解释为硬件验收。合成手为三个关节，仅用于证明维度由 profile 决定。

## DDS 与只读诊断

复制模板后填写现场 DDS domain、namespace、prefix、mode；不依赖程序覆盖全局环境变量。连接参数互相冲突时，同一进程的第二个配置会失败，不能偷偷重配正在运行的单例。

```bash
cp configs/aurora.unverified.yaml configs/aurora.site.yaml
# 编辑 aurora.site.yaml 中实际 DDS 参数。以下是只读连接命令：
python tools/aurora_control.py --backend aurora --profile configs/aurora.site.yaml doctor
```

doctor 只创建 AuroraState 和 ControlGroupState 两个 subscriber，不注册 lease、不创建命令 publisher、不切 FSM。输出型号信息、FSM id/name、每个组位置/速度/力矩维度及收到样本的年龄。超时、无数据或端点不匹配会失败，不能凭组名或维度将配置标为 verified。

真实动作子命令未加 `--execute` 时也只进行同样的只读预检，不运行轨迹。它报告请求未执行，不声称模拟或到位。模板可用于原始 doctor；模型链路即使只读也需要完整的语义索引/坐标/安全配置。

## Profile 字段与现场确认

`configs/aurora.unverified.yaml` 是明确未验证的空硬件模板，未复制 GR3 示例的 FSM/布局/姿态。所有数值单位都是 SI：位置 rad、速度 rad/s、时间 s。DDS service timeout 单位为 ms。

| 字段 | 含义与要求 |
|---|---|
| `api_family`, `sdk_version` | 必须精确匹配已核实 API/版本 |
| `verified`, `simulated`, `verification_note` | 现场签认后的 true、真实设备为 false、记录型号/日期/验证人和证据；缺签认禁止执行 |
| `robot_type` | 必填期望型号，执行时与新鲜 AuroraState 比较 |
| `hardware_type`, `end_effector_type` | 适用时必须填写；填写后严格比对。手部要求 end_effector_type |
| `connection` | domain_id、source_name（1–64 个 ASCII 字母数字/点/下划线/短横线）、topic_prefix、dds_mode、use_namespace 必填；可加 namespace_name、is_ros_compatible、use_discovery_server、service_timeout_ms |
| `allowed_fsm` | 经现场确认可接受 JOINT 命令的 FSM id 列表；空列表禁止执行。程序只检查，不切换 |
| `feedback_timeout_s`, `endpoint_timeout_s` | 样本失效预算、启动发现预算，有限且正 |
| `control_period_s` | 调度周期与命令变化时间上界；延迟不能放大每步速度预算 |
| `arrival_tolerance_rad`, `arrival_timeout_s` | 新反馈到位容差和有界超时 |
| `stop_policy` | 仅支持现场明确接受的 `stop_publishing_release`；必须确认停止发布/释放后的机器人端行为 |
| `groups[].name/side/part/resource` | 实际控制组、left/right、arm/hand、SDK lease resource 的明确对应；本版仅接入 manipulator/end_effector，不允许腿/腰/头 |
| `count`, `motor_mode`, `verified` | 完整维度、经验证的 PD 或 POSITION 模式、组验证状态；POSITION/PD 不代表程序会切换硬件模式 |
| `capabilities` | `joint_position` 必须存在；可选 `wrist_orientation`、`hand_joints`、`hand_closure`，必须有实际硬件/标定依据 |
| `joints` | 每一个组内槽位都要映射，不能只列欲控制的关节；全部 index 为 0..count-1、不重复 |
| 每个 joint 的 `name/index/kind` | 唯一具名语义、真实逻辑关节索引、arm/wrist/finger；本版使用 JOINT 索引，不能混入电机索引 |
| `sign/zero/verified` | 每关节独立 ±1、已标定中立位在 SDK 坐标下的位置、验证状态；右侧不做统一反号假设 |
| `limits` | `min_position/max_position/max_velocity/max_position_step/max_tracking_error` 必填且有限；范围为语义坐标。max_current 必须 null，require_motor_error 必须 false，因为当前反馈合同不提供这些量 |
| `open_position/closed_position` | 可选语义弧度；声明 hand_closure 时每个手关节都需标定且在限位内 |

关节项格式示意（**数值必须现场填写，不是可执行标定**）：

```yaml
# 在现场组的 joints 内，为每一个实际槽位逐一填写：
- name: elbow_flexion
  index: 0       # 这里只展示整数格式；不得据此认定肘索引为 0
  kind: arm
  sign: null
  zero: null
  verified: false
  limits:
    min_position: null
    max_position: null
    max_velocity: null
    max_current: null
    max_position_step: null
    max_tracking_error: null
    require_motor_error: false
```

最初所有组的目标来自新鲜完整反馈；其后未更新的槽位保留本 owner 的已验证目标。完整批次的全部向量校验后才发布。只向本次实际更新的组发布，不夹带另一侧、手或身体命令。执行前整个配置均须完整验证；没有手部依据时应从 profile 去掉手部组/能力，不要填写猜测数值。

## 方向、角度和轨迹

方向以机器人自身为参照：forward/backward 分别为肩部前屈/后伸，outward/inward 分别远离/靠近身体中线。direction 命令使用非负 angle_deg，具名关节可以使用有符号 angle_deg。没有末端 Cartesian 位移或 IK。

- `reference="neutral"`：相对已标定中立位的绝对语义角度，不是相对当前位置。
- `reference="current"`：动作开始时从新鲜反馈计算一次增量目标，不逐周期累计。
- `q_sdk = zero + sign * q_semantic`，反馈使用逆变换；业务、安全和模型链路均是语义弧度。CLI/动作服务角度只在边界转一次。
- move 用 cubic smoothstep；预检完整目标范围和峰值速度/单步变化。duration 在一个周期至 3600s 之间。
- swing 用有限次 raised-cosine 往复：先用 settle_duration_s 平滑移到 `center-amplitude`，每周期访问两个极值，最后平滑返回 center。总计划时间约 `2*settle_duration_s + cycles*period_s`，另有有限到位等待。center 默认为 current；数值 center 是语义 rad。
- cycles 必须整数 1..1000，摆动部分最多 3600s，每周期至少 20 个调度采样。拒绝 NaN/Inf/非法范围；不会无限摆动。
- monotonic 调度，超出 1.5 个周期的调度间隔中止；后台限制不会因传入巨大 dt 或晚调度而放大。
- 手动轨迹若被安全限幅则失败，不冒称原目标完成。模型连续控制可渐进限速，并输出实际安全目标；不承诺有限轨迹到位。
- SDK publish 成功只意味着本地接受发布。动作结果还要求更新的反馈在容差内，否则有界超时报错。

## Python API 与双臂

单臂（离线且可直接复制）：

```python
from sleeve_arm.robot.factory import create_robot
from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.control.upper_limb import UpperLimbService
from sleeve_arm.robot.aurora_fake import fake_session

session = fake_session()  # 虚拟时钟，不创建 DDS
robot = create_robot("aurora-fake", session=session, side="left")
controller = SafeArmController(robot, robot.config, clock=session.clock)
try:
    controller.connect()
    controller.enable()
    motion = UpperLimbService(controller)
    print(motion.move_arm(side="left", direction="forward", angle_deg=5,
                          reference="current", duration_s=2))
    print(motion.move_joint(side="left", joint="elbow_flexion", angle_deg=3, duration_s=2))
    print(motion.swing_arm(side="left", joint="elbow_flexion", amplitude_deg=2,
                           period_s=2, cycles=2, center="current"))
finally:
    controller.shutdown()
```

双臂合并批次（离线）：

```bash
python - <<'PY'
import math
from sleeve_arm.robot.factory import create_robot
from sleeve_arm.robot.aurora_fake import fake_session
from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.control.upper_limb import UpperLimbService
s = fake_session()
r = create_robot("aurora-fake", session=s, sides=("left", "right"))
c = SafeArmController(r, r.config, clock=s.clock)
try:
    c.connect(); c.enable()
    print(UpperLimbService(c).move_joints(
        {"left.elbow_flexion": math.radians(3), "right.elbow_flexion": math.radians(3)},
        duration_s=2, reference="current"))
finally:
    c.shutdown()
PY
```

真实 Python 路径使用 `create_robot("aurora", profile="configs/aurora.site.yaml", side="left", execute=True, operator_confirmed=True)`；确认参数必须来自现场操作员已完成的明确确认，不应在无人值守代码中默认打开。单臂 key 为 `elbow_flexion`；双臂 key 为 `left.elbow_flexion/right.elbow_flexion`。双臂一次多组发布不保证硬件原子同步。

同一进程使用 `AuroraSession.real(...)` 或 factory 自动复用相同配置的会话。多个 side/hand view 必须由同一 owner 线程调度，禁止多个 view 重叠占用同一个组。enable 第二个 view 时 replace_lease 扩展完整资源集合；关闭某 view 缩减资源，最后一个关闭才 stop SDK。只读和可执行会话不能热切换，需关闭全部旧 view 后显式重建。可以用单一多侧 view 避免多余租约变更。

手部 API：`hand_joints(side=..., angles_deg={真实名称: 角度}, duration_s=...)`；`hand_closure(side=..., closure=0..1, duration_s=...)` 需要完整开合标定。`wrist_joints` 只接收 profile 中 kind=wrist 且有 wrist_orientation 能力的物理关节。没有虚构 palm 关节，没有五指到六电机固定映射，没有掌部 Cartesian 姿态控制。需创建 `parts=("hand",)` 或明确组合的 view；手和臂仍共享安全命令通道/会话。

## 真机单臂与传感器入口

以下仅供现场完成 profile、支撑和安全条件后的操作员使用；本次交付未运行：

```bash
python tools/aurora_control.py --backend aurora --profile configs/aurora.site.yaml \
  --execute --confirm EXECUTE_AURORA \
  move --side left --direction forward --angle-deg 1 --reference current --duration 3

python tools/aurora_control.py --backend aurora --profile configs/aurora.site.yaml \
  --execute --confirm EXECUTE_AURORA \
  joint --side right --joint elbow_flexion --angle-deg 1 --reference current --duration 3
```

真实运动同时要求真实 backend、execute、完整 verified profile、操作员确认、新鲜反馈/身份/FSM/端点和可用 lease。`enable` 只授权本应用控制选定资源，不做全机器人 Servo On。FSM 切换功能本版不提供；如现场需要进入控制 FSM，须由独立且明确授权的现场流程完成。本版无 force_switch 和 emergency priority 路径。

模型控制链保持：SensorSample → Predictor/Estimator → MotionIntent → AuroraIntentMapper → SafeArmController → AuroraRobotArm。源线程从不调用 SDK。当前肩部估计器和交互标定均为右臂合同，必须显式 `--source-side right --side right`；左源或跨侧均拒绝，不能把右肩模型宣称为左肩模型。手动左臂接口不受这个模型限制。

```bash
# 沿用项目已配置的模型/标定依赖（包括所选 flexarm estimator），机器人、传感器均 fake：
python tools/run_model_control.py --robot aurora-fake --source-side right --side right \
  --sleeve fake --imus fake --duration 5

# 现场配置完整后的只读模型预览；不会取得 Aurora 控制权：
python tools/run_model_control.py --robot aurora --aurora-profile configs/aurora.site.yaml \
  --source-side right --side right --sleeve real --imus real --duration 10

# 现场确认后的有限时长模型控制：
python tools/run_model_control.py --robot aurora --aurora-profile configs/aurora.site.yaml \
  --source-side right --side right --sleeve real --imus real --duration 10 \
  --execute --confirm EXECUTE_AURORA
```

Aurora 路径遇到袖套超时、IMU/模型预测错误立即退出控制，不自动恢复。原 fake/dymotor 用法、DyMotor 四关节顺序、native ctypes ABI、原 mapper 肘阈值规则保留。**DyMotor connect 仍执行厂家的使能启动流程，即使没有 --execute 也不是只读无动作。**

## 故障、停止和现场验收

连接/发现、身份/FSM、异步 SDK 错误或 lease 故障锁存到共享会话，阻止全部相关写入；单侧数据/跟踪/范围故障锁存当前 view，不能误发另一侧命令。回调只设置线程安全事件。恢复必须关闭旧控制器/相关会话，检查原因，并以新的明确授权重建；不会自动 clear fault/re-enable。

反馈新鲜度使用 SDK header.received_time：首次换算到 monotonic 接收时刻，重复缓存不更新时间；还检查 wall-clock 年龄、倒退/未来时间。全组读取在一个 ControlGroupState snapshot 内完成，状态/FSM是独立 SDK 消息，不声称两类消息硬件同步。无电流、bus、逐电机状态/错误时返回 None；不把 Aurora 异步错误伪装成逐电机反馈。命令端需要 ErrorCodes 订阅匹配且至少已有有效消息；若现场服务不发布该消息，执行会拒绝，需先核对服务合同。

Ctrl+C、异常、disable、close 会阻止应用新目标，执行已验证 stop policy 并释放自己的 lease；不回零、不恢复初始姿态、不切全局 FSM、不整机断使能。失去 lease 或反馈失效时不继续发送所谓保持命令。**停止发布/释放 lease 不等于已确认物理停止；硬件急停必须由现场独立安全手段负责。**

建议现场验收顺序：

1. 查型号/硬件/末端、服务和 DDS 版本，确认物理支撑、工作区、独立急停和监护人。先运行只读 doctor，记录真实数据。
2. 使用厂家文档及受控的现场标定确认每个逻辑索引、符号、零位、上下限、模式、FSM、反馈频率及停止发布后的行为。不能以“维度能读到”替代标定。由负责人签认 profile。
3. 仅选一个侧别和一个关节，以小角度（如经过现场限位检查的 1°）和长时长测试；核对请求、SDK 变换、实际反馈与物理方向。先在离线环境验证故障处理，再按现场安全方案检验取消/失联行为。
4. 分别验证其余关节和另一侧；确认跟踪误差/速度预算后才增加有限摆动。双臂和手部另行验收。
5. 最后接入已验证的同侧袖套/IMU/模型链，短时运行并记录传感器超时、预测错误及停止后的实际行为。

关节限位和跟踪检查不等于完整自碰撞、环境避障或机器人功能安全认证。未实现 IK、末端姿态求解、整机平衡、硬件急停控制、自动 FSM 切换、实时线程保证或 Legacy API 兼容。

## 离线验证

```bash
python -m pytest -q tests/test_aurora.py
python -m pytest -q
```

FakeAuroraClient 使用新 SDK 的 None lifecycle、OperationResult.success、(result, state)、LeaseResult.usable 和 command 类型合同，运行不创建 DDS。虚拟时钟覆盖轨迹/摆动/缓存过期/到位超时；子进程 help 用 import blocker 阻止 SDK 导入。模型入口用 mock predictor/fake sensors 验证正常、预测失败、watchdog 清理。现有原测试未删除或放宽。实际测试结果与环境限制另见交付报告。
