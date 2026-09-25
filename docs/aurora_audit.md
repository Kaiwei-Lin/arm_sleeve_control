# Aurora 接入审计（2026-09-25）

> 本文是此前 1.0.1 后端开发阶段的历史记录。后续已实际安装并审计 PyPI 0.1.8；当前环境、HEAD、依赖清单和 API 结论以 [aurora_sdk_environment.md](aurora_sdk_environment.md) 为准。两套 API 不兼容；本文末尾 requirements-aurora 的旧说明已被新清单替代。后续动作 backend 也已迁移到 0.1.8，当前实现见 [aurora_control.md](aurora_control.md)，本文仅保留历史审计依据。

## 工作区与来源

- 实际工作区 HEAD：`5f9168fb3cf92e65a221ee19657e9fbdce63015f`，与给定 arm_sleeve_control master 参考一致；未切换分支、回退、提交或 push。
- 初始已有 34 个 tracked 文件修改，包括 README、robot/sensor/phase3 配置、config、MotionIntent、模型/标定、run_model_control、传感器工具/测试和 CSV。初始 `git diff --ignore-space-at-eol --stat` 为空：这些修改相对 HEAD 表现为换行差异。仍按用户修改保留；只对必要文件增量编辑，未改训练、传感协议、标定算法或原测试。
- 未发现工作区 `.codegraph/` 或文件系统 AGENTS.md；遵循用户提供的 AGENTS.md，未尝试索引。
- 官方 SDK main 实际 clone 到工作区外 `/tmp/sleeve-aurora-sdk-audit`：`3d4e02e170364dff83e7f35e29b55c86cf728b92`，与给定参考一致，`VERSION=1.0.1`。
- 已阅读该提交 README、REVISIONS.yaml、docs/python_api.md、docs/examples_and_clients.md、examples/gr3/python/get_info.py、swing_arms.py、Python wrapper/binding、C++ command/lifecycle/transport 对应实现。未运行官方示例。
- 旧 backend 的给定 GitHub 链接及对应 raw URL 返回 404；本地也未安装 SDK。没有旧版部署依据，未实现 Legacy adapter，未加入私有 velocity_cmd 补丁或失败回退。

## 正式支持的 API 家族

应用家族标识为 `aurora-configure-lease-v1`，仅允许 `fourier-aurora-client==1.0.1`。这是本次按源码核实的客户端 API 合同，并非已完成真机验证，也不是机器人服务/固件版本。其他版本、旧 get_group_state/set_group_cmd 家族均拒绝。

[官方 Python API（固定提交）](https://github.com/FFTAI/fourier_aurora_sdk/blob/3d4e02e170364dff83e7f35e29b55c86cf728b92/docs/python_api.md) 是 SDK 方法/返回值依据；[binding](https://github.com/FFTAI/fourier_aurora_sdk/blob/3d4e02e170364dff83e7f35e29b55c86cf728b92/python/bindings/bind_client.cpp) 与实现交叉核对。

| 实际调用 | 返回与处理 |
|---|---|
| `AuroraClient.get_instance()`、`state()` | 无 kwargs 获取单例；已被其他调用方启动则拒绝接管 |
| `ConnectionOptions()`、`configure()` | 显式 source_name、连接字段、最小 enabled_endpoints；None/异常合同 |
| `start()`、`stop()`、`on_lease_changed()` | None/异常合同；回调只置 Event，正常流处理故障和清理 |
| `wait_for_endpoints()`、`endpoint_match_status()` | 显式 bool 和逐必需端点 enabled/matched 检查 |
| `get_aurora_state()`、`get_control_group_state()`、`get_error_codes()` | `(OperationResult, value)`，调用 `success()`；异步错误独立于逐电机状态 |
| `JointCommand`、`ControlGroupCommand`、`publish_control_group_command()` | JOINT 索引；mode 由 profile 指定；完整组向量；显式 `success()` |
| `register_lease()`、`replace_lease()`、`get_lease()` | 检查 operation.success、usable、完整资源集合；NORMAL 默认优先级 |
| `release_lease()` | 检查 operation.success；释放后不可要求 usable |

不调用 request_fsm_state、servo/motor configuration、motion/velocity、IK 或私有 API。不改 DDS 全局环境。

[REVISIONS.yaml](https://github.com/FFTAI/fourier_aurora_sdk/blob/3d4e02e170364dff83e7f35e29b55c86cf728b92/REVISIONS.yaml) 记录 SDK 1.0.1 匹配 `fourier_dds 1.2.0-1`（`1aec6d3307c3f64c1210b4d26d113fe0979c29c3`）、`fourier-dds-msgs 0.1.0-1`（`18a24585984e04be401cf1acc0151d5b5e95b2cd`）；其中 Aurora 服务参考 commit `64cbd264608a1dfd09d82c8cfa4aee99947af5f7` 仅是依赖记录，不能据此认定现场版本。未安装或升级机器人端任何组件。

## 已确认与未确认

- 已确认 SDK groupResource 对 `left/right_manipulator`、`left/right_end_effector` 有对应资源枚举；不代表现场一定存在这些组。
- GR3 swing 示例使用 FSM 13、肩 roll 索引 1、肘索引 3；sub_controller 示例说明 GR3 七关节排列。它们只证明示例的合同，未用于本项目默认执行配置。
- 未观测现场 robot_type/hardware_type/end_effector_type、DDS、FSM、组维度、关节语义索引、正负方向、零位、限位、PD/POSITION 适配、反馈频率、允许速度/跟踪误差、停止策略或手部标定。
- `configs/aurora.unverified.yaml` 因上述缺失禁止 execute。fake 的六关节臂、三关节手、FSM 999、符号和限位完全为合成测试数据，带 simulated 标志，不能用于真机 execute。

## 修改范围

新增 profile / session / backend / factory / offline fake、独立于寻址的安全参数、上肢动作服务、Aurora CLI、审计/使用文档和离线测试。增量调整 RobotArm 的批量读/已检查发送接口、JointState 的可缺失反馈、SafeArmController 的动态关节/可注入时钟/故障锁存、FakeRobot 的动态关节、Aurora 专用意图 mapper、模型入口及 README。DyMotor ctypes 布局和四关节顺序未变，configs/robot.yaml 未改。

## 实际文件与职责

| 文件 | 变更 |
|---|---|
| `sleeve_arm/robot/aurora_profile.py`、`configs/aurora.unverified.yaml` | profile/schema 校验、每侧每关节坐标/限位/能力、未验证执行阻断 |
| `sleeve_arm/robot/aurora_session.py` | SDK 1.0.1 adapter、最小端点、结果检查、缓存新鲜度、单 owner、共享引用生命周期与 lease |
| `sleeve_arm/robot/aurora.py` | RobotArm 实现、单次整组快照、完整向量保留/验证、局部故障 |
| `sleeve_arm/robot/aurora_fake.py` | SDK 返回合同的离线 client、合成 profile、虚拟时钟 |
| `sleeve_arm/robot/factory.py` | fake/dymotor/aurora/aurora-fake 创建边界与真实会话复用 |
| `sleeve_arm/control/parameters.py`、`upper_limb.py` | 不含 DyMotor 寻址的安全参数、有限轨迹/方向/摆臂/腕手接口 |
| `sleeve_arm/robot/base.py`、`fake_robot.py`、`sleeve_arm/domain/joint.py` | 可复用批量接口、动态 fake 关节、Optional 反馈、固定 DyMotor 常量兼容别名 |
| `sleeve_arm/control/controller.py`、`safety.py`、`mapper.py` | 动态关节安全层、单快照校验发送、时钟注入、严格动作结果、单侧 Aurora 意图路由 |
| `tools/aurora_control.py`、`tools/run_model_control.py` | 独立 CLI、真机门禁、原控制循环接入 Aurora，保留原 fake/dymotor 行为 |
| `tests/test_aurora.py` | 90 项离线 API/安全/生命周期/轨迹/模型入口测试 |
| `README.md`、`docs/aurora_control.md`、本文件 | 入口、使用/现场验收说明和审计结果 |

## 离线验收结果

本机默认 Python 及已有 skin 环境没有 pytest；创建工作区外 `/tmp/sleeve-aurora-tests`（Python 3.13）安装测试依赖，未安装 Aurora SDK。新建测试环境包含 pytest 9.1.1、PyYAML 6.0.3、pyserial 3.5、numpy 2.5.3、joblib 1.6.0、scikit-learn 1.7.2。Aurora 接入初次验收未修改 requirements-dev.txt；后续依赖打包调整见下节。

```bash
/tmp/sleeve-aurora-tests/bin/python -m pytest -q \
  -o cache_dir=/tmp/sleeve-aurora-pytest-cache tests/test_aurora.py
# 90 passed

/tmp/sleeve-aurora-tests/bin/python -m pytest -q \
  -o cache_dir=/tmp/sleeve-aurora-pytest-cache
# 285 passed, 1 failed（下面说明原有失败）

/tmp/sleeve-aurora-tests/bin/python -m compileall -q sleeve_arm tools/aurora_control.py
# exit 0

git -c core.whitespace=cr-at-eol diff --check -- \
  sleeve_arm/control sleeve_arm/domain/joint.py sleeve_arm/robot tools/run_model_control.py README.md
# exit 0；保留工作区原有 CRLF，未将其误作新引入空白问题
```

唯一失败：`tests/test_imu770_upper_arm_twist_demo.py::test_run_calibrates_records_live_pairs_and_closes_both_ports`。该测试模拟串口在短暂标定结束/CSV 创建前耗尽，报 `reader failed: test stream ended`，随后因 live.csv 不存在失败。在 `/tmp/sleeve-aurora-baseline` 独立副本中恢复本次改动前的源码后，使用同一测试环境单独运行亦失败；已确认加载的是副本中的模块。原测试和 IMU770 实现均未修改，没有删测、改阈值或掩盖为通过。

已实跑无 SDK 的 fake doctor、单关节、有限摆臂、具名手指和双臂批次示例；全部成功并关闭会话。模型入口测试使用 fake sensors 与 mock predictor，覆盖正常右侧路由、预测错误、传感器 watchdog 退出与释放。现有模型映射回归通过，但没有在此次验证中运行真实模型权重、现场数据和完整外部模型部署。

未执行任何 DDS/真机连接、真实租约、FSM 或运动；未安装官方 native SDK/runtime，因此 pybind 二进制加载、DDS 发现、机器人身份/方向/零位/限位、物理停止效果及腕手能力仍待现场验收。上述离线通过只证明源码合同和注入模拟路径，不代表硬件通过。

与本次改动直接相关的 Aurora、DyMotor ctypes、FakeRobot、安全、Phase 3/4 回归组合：177 passed。


## 后续依赖打包

按用户要求加入仓库根目录的 `flexarm_estimator-0.3.0-py3-none-any.whl`。新增 `requirements.txt` 作为唯一运行依赖清单，含本地 wheel、PyYAML、pyserial、numpy、joblib 和固定的 scikit-learn 1.7.2；`requienment.txt` 为用户指定文件名的转引入口；`requirements-dev.txt` 复用运行依赖并增加 pytest。README 同步安装说明。Aurora native SDK/DDS 和 DyMotor bridge 仍按硬件文档单独部署。

在 `/tmp/sleeve-aurora-tests` 中实际运行 `python -m pip install -r requienment.txt -r requirements-dev.txt` 成功；`pip check` 无依赖冲突。真实 wheel 的 DualImuArmEstimator 完成合成单位四元数静态标定与 update，返回 Rest / 0°；FlexArmEstimator 导入成功。该检查未使用真实传感器、机器人或训练模型权重。

## Aurora SDK 安装清单补充

补充 `requirements-aurora.txt`，包含基础运行依赖与精确的 `fourier-aurora-client==1.0.1`；README、通用清单注释和安装文档增加该入口。SDK 继续作为真机专用可选依赖，安装需通过 `--find-links` 提供官方平台 wheel，并先按 REVISIONS.yaml 安装本机原生 DDS/message 运行库。

实际执行 `pip index versions fourier-aurora-client --index-url https://pypi.org/simple` 仅列出 0.1.8/0.1.1。`pip install --dry-run -r requirements-aurora.txt --index-url https://pypi.org/simple` 因缺少 1.0.1 返回失败，证实不会解析到未经验证的版本；未声称真实 SDK 安装通过。基础 `requienment.txt` 的 dry-run 和离线 fake doctor 均成功。

发现官方审计提交的 docs/overview.md 表格写消息包 0.2.0-1，而 REVISIONS.yaml compatibility 条目写 0.1.0-1；安装文档已披露，以精确 revision 清单为核对依据，并要求现场向厂家确认。当前工作区没有 Aurora wheel 或 DDS deb，未实际安装这些二进制包，也未启动真实 DDS。
