# 数字菜单手臂 SDK 调试脚本

`tools/aurora_arm_debug.py` 用于检查 Aurora SDK 的连接、完整 group 命令下发和
反馈到位。不需要袖套、IMU 或模型。默认只做离线预览；只有显式 `--execute`
才连接真机。启用后每条动作显示目标并直接执行，无需输入 `YES`。

## 三种运行方式

在 WSL/Linux 的项目根目录运行。默认预览无需 SDK 或硬件：

```bash
python tools/aurora_arm_debug.py
```

离线模拟完整的发送和反馈流程（模拟时间，不按真实秒数等待）：

```bash
python tools/aurora_arm_debug.py --backend fake --simulate
```

真机使用已经完成现场校准/验证的 profile：

```bash
python tools/aurora_arm_debug.py --profile configs/aurora.site.yaml --execute
```

`aurora.site.yaml` 需按 [Aurora 配置说明](aurora_control.md) 从模板建立。
原始模板的验证标记仍为 false，脚本会在导入 SDK/连接 DDS 前拒绝执行；
脚本不会替用户确认方向、零位、限位或控制权。

当前固定 SDK 依赖由 `requirements-aurora.txt` 提供。也可以先用已有工具分层检查：

```bash
# 只检查安装、版本与 API，不连接 DDS。
python tools/aurora_sdk_doctor.py
# 现场执行的只读 DDS 检查，domain 需与机器人一致；不发送动作。
python tools/aurora_sdk_doctor.py --connect --domain-id 123
```

## 数字菜单与角度含义

默认进入右臂菜单，不需要输入中文指令。先选方向编号，再输入角度数字：

```text
右臂动作菜单：
  1. 向前抬
  2. 向后抬
  3. 侧摆（向外抬）
  4. 向内收
  5. 屈肘
  6. 伸肘
  8. 查看当前角度
  0. 退出
请选择动作编号：1
已选择：右臂向前抬
请输入角度（度，0～180，直接回车取消）：30
```

例如向后抬 30°，输入 `2`、回车，再输入 `30`、回车；侧摆 20° 则依次输入
`3` 和 `20`。`8` 直接显示反馈，不询问角度；`0` 退出。

| 编号 | 语义关节和角度 |
| --- | --- |
| 1 / 2 | 肩前屈正 / 负角度 |
| 3 / 4 | 肩外展正 / 负角度 |
| 5 / 6 | 肘屈曲正 / 负角度 |

角度支持小数和全角数字，只输入数值，不输入“度”。空角度取消本次操作；
非法数值会提示重新输入。默认平滑模式把超范围位置目标收回 profile 的边界，并逐步执行。
这里的“侧摆”是到达指定外展角度，不是自动往复摆动。
这是单关节动作，其他关节保持，不会自动复位上一个动作涉及的不同关节。

默认 `--reference neutral`：角度是相对 profile 已标定零位的**目标姿态**。
例如先选 `1` 输入 `30`，再选 `2` 输入 `30`，目标从 +30° 变为 -30°，实际行程
约 60°；反复选 `1` 输入 `30` 不会累加。

如果要在当前姿态基础上再动指定角度：

```bash
python tools/aurora_arm_debug.py --backend fake --simulate --reference current
```

例如先选 `5` 输入 `30`，再选 `6` 输入 `10`，从初始 0° 最后到约 20°。
在默认 neutral 模式下选 `6` 输入 `10` 要求到 -10°，可能超出 GR3 的肘部范围。
增量目标根据展示预览时的新鲜反馈计算一次，执行前重新读取反馈不会改变目标。

默认只连接右臂。左臂使用 `--side left`，菜单会显示为左臂；同一次会话不切换侧别。

单次调用使用编号和角度参数：

```bash
python tools/aurora_arm_debug.py --backend fake --simulate --action 1 --angle-deg 30
```

原中文 `--command` 参数已移除，迁移为 `--action 编号 --angle-deg 角度`。
`--action 8` 仅查看状态，`--action 0` 结束；这两项不接受角度参数。
不提供 `--action` 就进入交互菜单。

每次动作都以当前 profile 允许的最大速度推进：每个控制周期的最大步幅为
`min(max_velocity × 实际周期时间, max_position_step)`，实际周期时间最多按
`1 / control_hz` 计算。到达目标即结束，不再设置固定运动耗时、至少 2 秒等待
或额外 25% 时间余量，也不再接受 `--duration` 参数；旧命令中删除该参数即可。
这会从首个周期开始按最大允许步幅发送，不再使用先加速后减速的三次插值。
正常运行配置 `robot_aurora.yaml` 和 `aurora_196_shoulder.yaml` 已采用官方速度上限：
肩前屈/外展 7.75 rad/s，上臂旋转/肘屈曲 6.28 rad/s。100 Hz 下的步长分别为
0.0775 和 0.0628 rad，避免原来 0.01 rad 的步长再次压低速度。
默认 `smooth_limits: true`：位置超限收回到边界，跟踪误差限制目标超前量，
速度与步长限制每次变化；启动位置越界逐步回收，不因这些数值超限退出。
若跟踪目标长期无进展，报告尚未到位并返回菜单。断流、无效数据和到位超时仍独立处理。

`max_velocity` 限制指令位置的变化速度，实测反馈用独立的
`max_feedback_velocity` 在旧的 `smooth_limits: false` 模式中判断超速。GR3 手臂未填写该项时，按
[官方关节参数](https://support-old.fftai.com/docs/GR-X-Humanoid-Robot/GR3/SDK/Aurora-SDK/reference/robot_specs/)
使用对应关节上限：肩前屈/外展 7.75、上臂旋转/肘屈曲/腕 yaw 6.28、腕 pitch/roll
9.2153 rad/s；也可在关节 `limits` 中设置更低的 `max_feedback_velocity`，但不能低于
指令 `max_velocity`。例如 `max_velocity: 0.3`、`max_feedback_velocity: 0.6`
分别表示指令最多 0.3 rad/s、旧模式下实测超过 0.6 rad/s 报错。非 GR3 手臂未填写时沿用
`max_velocity` 作为反馈上限。

原来 `velocity exceeds max_velocity` 报错把指令限速也当作实测速度阈值；按限速运行时，
反馈超过 0.3 rad/s 就会终止。现在默认平滑模式不因实测速度超限终止；启动时的
`smooth_limits` 与 `velocity_limits_rad_s` 分别显示策略和配置值。指令限速始终生效。
`DDSInterface closed` 是随后清理客户端的日志，不是这个超速错误的原因。

官方 [MoveCommand 示例](https://support-old.fftai.com/docs/GR-X-Humanoid-Robot/GR3/SDK/Aurora-SDK/examples/move_command_example/)
的 `expect_vel` 是最大速度的千分比，并要求全身 FSM 3 / 上身 FSM 4。
当前脚本继续使用 [PdStand 下的直接关节命令](https://support-old.fftai.com/docs/GR-X-Humanoid-Robot/GR3/SDK/Aurora-SDK/examples/joint_command_example/)，
不因这次反馈阈值修复自动切换控制模式。

`debug.sh` 使用更新后的肩部配置；`control.sh` 使用更新后的双臂通用配置，
其传感器控制循环也按 100 Hz 推进最新的新鲜目标。完整限位核对和仍属应用层的
跟踪误差/超时参数见 [Aurora 配置说明](aurora_control.md#正常运行配置的官方限位核对)。

## 输出能证明什么

脚本先打印解析结果、目标 group/index、完整向量前后值和阻止原因。执行后报告：

- `submitted`：SDK 调用返回，不能单独证明硬件收到命令。
- `arrived`：控制层观察到提交后的新鲜位置反馈，误差在 profile 容差内。
- `target_deg`、`feedback_deg`、`error_deg`：语义角度目标、实测值和误差。
- `feedback_velocity_rad_s`：到位检查后的实测语义关节速度。
- `delivery_confirmed: false`：0.1.8 没有消息送达回执，不能伪造成功确认。
- `simulation: true`：仅为模拟结果，不能证明真机 SDK/网络/电机已工作。

真机要求 PdStand/FSM 2 和 `stable_level > minimum_stable_level`，不自动切换 FSM。
动作经现有 `AuroraMotionService → SafeArmController → AuroraRobotArm → AuroraSession`
执行，以应用层插值调用 `get_group_state + set_group_cmd`；保持 7DoF 完整向量。
没有引入 MoveCommand，也没有改动实时传感器控制循环。

整个交互使用同一个 SDK 会话；每个动作结束后禁用本程序的新目标发送，等待下一条
输入时不持续发旧目标。下一条动作会重新获取新鲜反馈并检查模式。动作失败或
Ctrl+C 会结束会话，不在已经锁存故障的客户端上继续尝试。
退出停止发布并关闭客户端，不回零、不切 FSM，也不等于硬件急停。

本脚本仅经过 fake/离线测试；真机 SDK 可用性仍须通过现场反馈和实际动作确认。
