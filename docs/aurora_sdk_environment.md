# Aurora SDK 环境与只读 API 审计

> 本文记录 SDK 安装阶段的环境与真实 API。后续 backend 已迁移到同一 0.1.8 API，当前控制接口/测试与现场边界见 [aurora_control.md](aurora_control.md)。下方 315 passed/1 failed 是安装阶段基线。

审计日期：2026-09-25。范围仅限 Python 环境、SDK 安装、import/API introspection、只读 doctor 实现和离线测试。**本轮没有创建真实 DDS participant、连接机器人、申请 lease、切换 FSM 或发送动作。**

## 已安装在哪里，以及怎么使用

本机已安装成功，不需要再找 Aurora wheel。请在 WSL/Linux 终端进入项目根目录，然后执行：

```bash
conda activate skin
which python
python --version
python -m pip show fourier_aurora_client
python -m pip check
python tools/aurora_sdk_doctor.py
```

应看到 `/home/lkw/miniconda3/envs/skin/bin/python`、Python 3.10.20、SDK 0.1.8，doctor 输出 `"mode": "offline"`、`"dds_started": false`。若未激活环境，当前 shell 的 `python` 仍是 **base / 3.13.13**；base 未安装 Aurora，这不是 skin 安装失败。也可直接使用：

```bash
/home/lkw/miniconda3/envs/skin/bin/python tools/aurora_sdk_doctor.py
```

重建相同项目环境的命令（项目根目录）：

```bash
conda activate skin
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -r requirements-aurora.txt
python -m pip install -r requirements-dev.txt
python -m pip check
```

`requirements.txt` 包含项目自带的 `flexarm_estimator-0.3.0-py3-none-any.whl`，它是模型估计器，与 Aurora SDK 是两个包。`requienment.txt` 是原拼写的兼容入口。`requirements-aurora.txt` 仅一行 `fourier_aurora_client==0.1.8`，没有加入基础或开发清单。其他没有现成环境的机器可先 `conda create -n aurora python=3.10 pip -y`，再 `conda activate aurora`。本次复用 skin，没有新建第二套环境。

默认源找不到固定版本时才改用：

```bash
python -m pip install --index-url https://pypi.org/simple "fourier_aurora_client==0.1.8"
```

## 实际环境与工作区

| 项目 | 实际记录 |
|---|---|
| OS / architecture | Ubuntu 22.04.5 LTS / WSL2 / Linux x86_64 / glibc 2.35 |
| Kernel | 6.18.33.2-microsoft-standard-WSL2-x86_64 |
| 初始 shell 环境 | Conda base，`/home/lkw/miniconda3/bin/python`，Python 3.13.13，pip 26.0.1 |
| 本次安装和测试环境 | Conda skin，`/home/lkw/miniconda3/envs/skin/bin/python`，Python 3.10.20 |
| skin pip | 26.1.2 → 26.2.1（按请求升级） |
| 安装前 Aurora | base 与 skin 均未安装 |
| 安装后 Aurora | skin 的 `fourier_aurora_client==0.1.8`；base 未改 |
| SDK 路径 | `/home/lkw/miniconda3/envs/skin/lib/python3.10/site-packages/fourier_aurora_client/__init__.py` |
| 当前项目 HEAD | `edfedf272c4bf9e11c67a0631b162ba3a9d42de4`，master |
| 初始参考 HEAD | `5f9168fb3cf92e65a221ee19657e9fbdce63015f`；本地比该参考新增两次提交，未回退 |
| 其他 Conda 环境 | base、cannkit_py310、cannkit_torch_alg、skin、xpu_bench |

安装前执行并输出了 `git status`、`git rev-parse HEAD`、`which python`、Python/pip 版本、`pip list`、`conda info --envs`；原有 34 个 tracked 文件改动按用户内容保留。工作区没有 `.codegraph/`，未创建索引。当前阶段未 reset、clean、stash、切分支、提交或 push。

本次同时补齐 skin 的项目运行/测试依赖：flexarm-estimator 0.3.0、scikit-learn 1.7.2、joblib 1.6.0、pytest 9.1.1 等；原有 NumPy 2.2.6、PyYAML 6.0.3、pyserial 3.5 保留。未修改系统 Python、DDS 环境变量或 LD_LIBRARY_PATH，未升级机器人端服务/固件。

## 官方仓库与包版本交叉核对

| 项目 | 实际结果 |
|---|---|
| SDK repo revision（当前 Gitee/GitHub main） | 两者均为 `3d4e02e170364dff83e7f35e29b55c86cf728b92`；仓库 `VERSION=1.0.1` |
| 0.1.8 文档参考 revision | `b434869719ae256d02d8285e88ea2b22c1515602`；README 标题 v1.3.0，明确安装 client 0.1.8 |
| Python package | PyPI `fourier_aurora_client`（pip 将连字符/下划线规范化为同一个包） |
| Package version | 实际安装并 introspect：**0.1.8** |
| Python requirement | 0.1.8 METADATA：`>=3.9`；本项目 flexarm wheel 要求 `>=3.10`；实测 3.10.20 |
| Supported platform | 本次 import 验证 Linux x86_64 / WSL2；Windows 原生、ARM、其他 Python 版本未验证 |
| AuroraClient init API | `get_instance(domain_id, participant_qos=None, robot_name=None, namespace=None, is_ros_compatible=None)`；初始化即建 DDS 端点 |
| State API | `get_fsm_state()` 等旧 getters；没有 `get_aurora_state()` |
| Group state API | `get_group_state(group_name, key='position')`；没有 `get_group_position()` |
| Group command API | `set_group_cmd(position_cmd, velocity_cmd=None, torque_cmd=None)`；仅检查签名，未调用 |
| Lease required | **0.1.8 无 lease API**；只读订阅无需 lease。这不证明现场运动授权规则 |
| Domain ID default | client 的 `domain_id` **必填**；公开 DDSInterface 默认 0；官方服务器说明/示例使用 123，doctor 要求显式填写 |

来源与阅读范围：

- 当前 [Gitee main](https://gitee.com/FourierIntelligence/fourieraurorasdk/tree/main) 已实际 git clone；[GitHub 固定 main revision](https://github.com/FFTAI/fourier_aurora_sdk/tree/3d4e02e170364dff83e7f35e29b55c86cf728b92) 交叉核对。读取 README/README_CN、pyproject.toml、REVISIONS.yaml、docs/python_api.md 和 GR3 get_info 示例。当前 main 已没有 `python/docs/`、`python/example/`，属于另一套 configure/start/lease API。
- 历史 [README](https://github.com/FFTAI/fourier_aurora_sdk/blob/b434869719ae256d02d8285e88ea2b22c1515602/README.md)、[README_CN](https://github.com/FFTAI/fourier_aurora_sdk/blob/b434869719ae256d02d8285e88ea2b22c1515602/README_CN.md) 明确固定 0.1.8；对应 [Python API](https://github.com/FFTAI/fourier_aurora_sdk/blob/b434869719ae256d02d8285e88ea2b22c1515602/python/docs/API_document_EN.md) 和 `python/example/{gr3,gr2,gr1p,fouriern1}` 状态示例已读取，均未运行。该历史树没有 Python requirements/setup 包源码；实际包依赖和实现以 wheel 为准。
- [PyPI 0.1.8](https://pypi.org/project/fourier-aurora-client/0.1.8/) 和 [JSON 元数据](https://pypi.org/pypi/fourier-aurora-client/0.1.8/json)：2026-02-24 发布，依赖 `click>=8.0.0`、`pyyaml>=6.0`。wheel SHA256：`9ffc971080fd10488048e1ebd4d56ac9125ba2c1cbb170290a72bf97c1b41a05`。下载检查的 `client.py` 与正常 pip 安装的文件逐字节相同。

**没有把旧文档 revision 认定为 wheel 的构建 commit**：包元数据未提供可证明的源码 revision；API 实现依据是上述固定 hash 的 wheel 及实际安装文件。仓库标题 v1.3.0、包版本 0.1.8、新仓库 1.0.1 不构成统一版本序列，也不等于现场 Aurora Server/固件版本。

发现文档偏差：历史 GR1P `demo_get_state.py` 使用 `serial_number` 参数，而实际 0.1.8 签名不接受；历史 API 文档写 `is_ros_compatible=False`，包实际默认 `None`（此时读取 `FOURIERDDS_ROS_COMPATIBLE`）。均未通过真实控制试错，未照搬过时参数。

0.1.8 wheel 标记为 `py3-none-any`，但内部包含 Linux x86_64 ELF 的 `_fastdds_python.so`、消息 wrapper 和 Fast DDS/Fast CDR/tinyxml2 动态库，未附 Windows DLL，不能把 wheel 标签视作跨平台证明。实际 `ldd` 全部解析成功：Fast DDS 等从 site-packages 加载，glibc、OpenSSL 3、libstdc++ 等来自现有系统。本机 import 不需要另装 FourierDDS runtime。新 1.0.1 的 REVISIONS.yaml 指定的 DDS/message deb **不适用于此项安装结论**，本次没有安装它们；DDS 通信兼容性尚未验证。

## 不启动 DDS 的实际 API introspection

对 `aurora.AuroraClient` 使用 `dir`、`inspect.signature`；没有调用 `get_instance` 或其他 client 方法。所查方法全部为可读取签名的 Python 方法；doctor 也为无法 introspect 的 native binding 提供 `__doc__` 回退。

| 方法 | 实际存在性 | 签名 |
|---|---|---|
| `get_instance` | exists | `(domain_id: int, participant_qos=None, robot_name: Optional[str] = None, namespace: Optional[str] = None, is_ros_compatible: Optional[bool] = None) -> 'AuroraClient'` |
| `config` | absent | — |
| `configure` | absent | — |
| `start` | absent | — |
| `stop` | absent | — |
| `wait_for_endpoints` | absent | — |
| `close` | exists | `(self)` |
| `get_fsm_state` | exists | `(self) -> int` |
| `get_aurora_state` | absent | — |
| `get_group_state` | exists | `(self, group_name: str, key: str = 'position') -> list[float]` |
| `get_group_position` | absent | — |
| `get_control_group_state` | absent | — |
| `set_group_cmd` | exists | `(self, position_cmd: Dict[str, list[float]], velocity_cmd: Optional[Dict[str, list[float]]] = None, torque_cmd: Optional[Dict[str, list[float]]] = None)` |
| `publish_control_group_command` | absent | — |
| `register_lease` | absent | — |
| `get_lease` | absent | — |
| `release_lease` | absent | — |

其他实际存在的 public 方法：`get_base_data`、`get_cartesian_state`、`get_contact_data`、`get_fsm_name`、`get_group_motion_state`、`get_group_motor_cfg`、`get_stand_pose`、`get_true_data`、`get_upper_fsm_name`、`get_upper_fsm_state`、`get_velocity_source`、`get_velocity_source_name`、`set_fsm_state`、`set_joint_positions`、`set_motor_cfg`、`set_motor_cfg_pd`、`set_motor_cfg_pos`、`set_move_command`、`set_policy_id`、`set_stand_pose`、`set_upper_fsm_state`、`set_velocity`、`set_velocity_source`、`wait_groups_motion_complete`。doctor 会输出所有这些方法的签名；列出写方法仅用于 API 审计，代码不调用它们。

0.1.8 无 `ConnectionOptions` / `enabled_endpoints` / `OperationResult`。不能套用新版返回值判断，也不能将 `None` 或对象 truthiness 当成功。

## 只读 doctor 的连接实现

默认 `python tools/aurora_sdk_doctor.py` 只读包元数据、import 和类方法签名；`--help` 仅依赖标准库，SDK 未安装也可使用。SDK 缺失时给出当前解释器及固定依赖清单安装命令。未知版本可离线审计，但连接只接受明确核实的 0.1.8/旧 API，拒绝混合 API 和自动回退。

**以下命令会启动 DDS，本轮未执行；应在现场确认网络和 Domain ID 后手动运行：**

```bash
conda activate skin
python tools/aurora_sdk_doctor.py --connect --domain-id 123
```

可选连接参数（使用现场值）：`--namespace NAME`、`--ros-compatible` / `--no-ros-compatible`、`--timeout 5`、`--max-age 1`。不设置 ROS override 时保留 SDK 读取现有环境的行为，工具不覆盖环境变量。Domain ID 不会静默默认成 123；超时/最大样本年龄必须为有限的 `(0, 60]` 秒。

实现依据为安装包的公开 `DDSInterface`，没有修改任何 SDK 私有初始化方法：

```text
DDSInterface(domain_id=..., namespace=..., is_ros_compatible=...)
create_subscription(pub_sub_type, topic_name, callback, qos_profile=...)
SubscriberQosProfile.best_effort()
Subscriber.is_matched
DDSInterface.close()
```

只创建 `aurora_state` 和 `robot_control_group_state` 两个订阅；消息类型分别是 `fourier_msgs.msg.AuroraState.AuroraStatePubSubType` 与 `fourier_msgs.msg.MotionControlState.RobotControlGroupStatePubSubType`，字段与 0.1.8 `client.py` 的状态回调及生成 wrapper 一致。**不调用 AuroraClient.get_instance**，因为它还会创建 FSM、velocity、关节命令 publishers 和 policy service client，并等待全部端点匹配。doctor 没有 publisher、service client、command owner 或 lease，不存在写接口调用路径。

此 API 返回 Subscriber 或抛异常，doctor 检查对象和匹配状态，再要求两个订阅均收到有效新鲜消息；它不返回新版 OperationResult。超时、DDS 异常、空/重复组名、空 position、NaN/Inf、向量长度不一致均返回失败，不以默认零向量冒充反馈。velocity/effort 空向量保留为空，不冒充零反馈。组名和向量长度从实际消息读取，没有预设左右臂或手指布局。

回调复制完整 group-state 消息，使用锁保存本机 monotonic 接收时间；重复读取快照不刷新时间，解析失败覆盖上次有效反馈。两种 topic 各有接收时间，**不声称跨 topic 是原子同步快照**。这里的“有效”仅指数据结构、有限值、端点匹配和接收新鲜度，不代表关节限位、测量时刻、运动安全或控制准备就绪。

0.1.8 AuroraState 只有 FSM、velocity source、upper-body override 等字段，**没有 robot_type / hardware_type / end_effector_type**。doctor 对三者输出 `null` 和 unavailable 原因；不能根据组名或 `robot_name` 参数推断。没有新增未知 DDS topic 或写服务去查询型号。

正常完成、超时、部分订阅创建失败及 Ctrl+C 均调用自有 DDSInterface.close。0.1.8 的 close 返回 None，且 SDK 内部仅记录部分 native 清理错误；不能据此声称所有 native 清理结果已由 SDK 确认。关闭诊断只是停止本机接收，不是机器人物理停止或急停。

## 实际验证结果

所有命令使用 skin Python 3.10.20：

| 命令/检查 | 真实结果 |
|---|---|
| `python -m pip install --upgrade pip` | 成功，26.2.1 |
| `python -m pip install "fourier_aurora_client==0.1.8"` | 成功，默认源找到目标版本，无需后备源 |
| `python -m pip install -r requirements-dev.txt` | 成功，含项目本地 flexarm wheel |
| `python -m pip show fourier_aurora_client` | 0.1.8，路径为 skin site-packages |
| `python -m pip check` | `No broken requirements found.` |
| import smoke：平台/metadata/module/AuroraClient | 成功，`AuroraClient available: True` |
| `python tools/aurora_sdk_doctor.py` | exit 0，offline，import_ok=true，dds_started=false |
| 无 SDK 的 base 中 `--help` | exit 0，不导入 SDK |
| `python -m pytest -q tests/test_aurora_sdk_doctor.py` | 30 passed，1 个原目录 cache 权限警告 |
| `python -m pytest -q` | **315 passed, 1 failed**，15.51 秒；2 个 cache 权限警告 |
| `--connect` 真机测试 | **未运行**，无 DDS 或机器人验证结论 |

唯一全量失败是原有 `tests/test_imu770_upper_arm_twist_demo.py::test_run_calibrates_records_live_pairs_and_closes_both_ports`：假串口在 0.10 秒标定期间结束，随后打开预期 `live.csv` 得到 FileNotFoundError。将未修改 HEAD 的测试和对应工具导出到 `/tmp/aurora018-head-regression-jhox7t0b`，使用相同 skin Python 单独运行，**同样 1 failed**。没有改动或放宽原测试。已有 `.pytest_cache/v/cache` 无写权限，仅记录警告，未改变用户目录权限。

新增测试不启动 DDS，通过忠实模拟 0.1.8 的方法/Subscriber 返回合同和假时钟，覆盖未安装/导入失败、版本和 API 检测、默认不创建 session、帮助信息、显式连接分离、两个只读订阅、禁止 client/writer/service/lease、错误与 Ctrl+C 清理、超时、非法参数、坏反馈和缓存新鲜度；另在实际安装包上阻断 participant/client 构造器验证离线 doctor。

## 修改范围和下一阶段边界

- 新增 `tools/aurora_sdk_doctor.py`：标准库 CLI、离线审计、显式两订阅只读诊断。
- 新增 `tests/test_aurora_sdk_doctor.py`：完全离线的 SDK 合同和安全边界测试。
- 新增本文；更新 README、requirements-aurora.txt，以及 requirements.txt/requienment.txt 的旧注释；旧 aurora_control/aurora_audit 文档加版本边界并纠正安装入口。
- 没有改动 RobotArm、传感器协议、模型、标定、DyMotor native ABI 或已有安全测试。

安装阶段曾保留只接受 1.0.1 的旧 backend。后续实现已将 `AuroraRobotArm` / `aurora_session.py` 替换为独立 0.1.8 adapter，保留严格版本检查，没有混用或回退到另一套 API。真实控制仍须完整验证的 profile、execute 和操作员确认；参见当前控制文档。

下一阶段实施 backend 前必须现场确认：机器人 robot_type、hardware_type、end-effector 类型；实际 Aurora Server/消息协议版本；Domain ID、namespace、ROS 命名和网络可达性；真实组名、左右臂向量长度、完整关节顺序；方向、零位、限位、速度/跟踪约束及允许的 FSM；真实控制权机制与停止策略。doctor 可观测的组名和维度不等于语义映射或安全参数已确认。型号字段缺失应由设备记录/厂家资料补证，不能猜测。

import 成功不代表 DDS 连接、Aurora Server 连接、机器人已就绪或控制功能验证通过。实际只读连接和所有真实动作本轮均未执行。
