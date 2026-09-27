# 中文手臂 SDK 调试脚本

`tools/aurora_arm_debug.py` 用于检查 Aurora SDK 的连接、完整 group 命令下发和
反馈到位。不需要袖套、IMU 或模型。默认只做离线预览；只有显式 `--execute`
才连接真机，且每条动作都显示目标并要求准确输入 `YES`。

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

## 中文输入与角度含义

```text
动作> 右臂向上抬30度
动作> 右臂向后30度
动作> 右臂向外20度
动作> 右臂屈肘30度
动作> 状态
动作> 退出
```

| 输入方向 | 语义关节和角度 |
| --- | --- |
| 向上抬、向上、向前抬、向前、前举 | 肩前屈正角度 |
| 向后抬、向后、后伸 | 肩前屈负角度 |
| 向外抬、向外、外展、侧举 | 肩外展正角度 |
| 向内收、向内、内收 | 肩外展负角度 |
| 屈肘、弯肘 | 肘屈曲正角度 |
| 伸肘 | 肘屈曲负角度 |

使用阿拉伯数字，支持小数和全角数字；一次仅接受一个完整动作，不推测自由文本。
“向上”明确表示向身体前方抬臂；侧方抬臂用“向外”。这是单关节动作，不是手掌
笛卡尔位置控制；其他关节保留原值，不会自动复位上一个动作涉及的不同关节。

默认 `--reference neutral`：角度是相对 profile 已标定零位的**目标姿态**。
例如先“向前30度”再“向后30度”，目标从 +30° 变为 -30°，实际行程约 60°；
反复输入“向前30度”不会累加。

如果要在当前姿态基础上再动指定角度：

```bash
python tools/aurora_arm_debug.py --backend fake --simulate --reference current
```

例如先“右臂屈肘30度”，再“右臂伸肘10度”，从初始 0° 最后到约 20°。
在默认 neutral 模式下“伸肘10度”要求到 -10°，可能超出 GR3 的肘部范围而被拒绝。
增量目标根据展示预览时的新鲜反馈计算一次，确认等待期间不会悄悄改成另一个目标。

默认只连接右臂。左臂使用 `--side left`，然后输入“左臂向前30度”等；不接受
在同一个已绑定右臂的会话里输入左臂动作。启动时只验证所选侧。

也可以仅执行一条命令：

```bash
python tools/aurora_arm_debug.py --backend fake --simulate --command "右臂向上抬30度"
```

默认按 profile 的速度/步长限制自动计算运动耗时；`--duration 5` 可指定 5 秒，
耗时不足会拒绝。位置超限直接拒绝，预览中的限幅值仅用于诊断。

## 输出能证明什么

脚本先打印解析结果、目标 group/index、完整向量前后值和阻止原因。执行后报告：

- `submitted`：SDK 调用返回，不能单独证明硬件收到命令。
- `arrived`：控制层观察到提交后的新鲜位置反馈，误差在 profile 容差内。
- `target_deg`、`feedback_deg`、`error_deg`：语义角度目标、实测值和误差。
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
