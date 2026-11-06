# 模拟传感器延迟测试

`tools/simulate_control_latency.py` 默认离线运行，不需要传感器、机械臂或 Aurora SDK，不打开串口或连接网络。
使用虚拟时间运行实际 `SensorRuntime`、双 IMU 肩部估计器、上臂旋转估计器、肘部滤波器、Bend5/Bend6 映射、`ModelControlApp`、安全控制器和 Aurora 后端；最终指令由内存中的 fake SDK 接收。
零位、前抬方向和关节限值均为合成测试参数，不用于真机。

脚本也支持**模拟传感器＋真实 Aurora**。该模式使用真实时钟和指定的机器人 profile，不使用离线合成限值，见下方说明。

在项目根目录运行（依赖项目已有的 NumPy 和 PyYAML，无需安装 pytest）：

```powershell
python tools/simulate_control_latency.py
```

默认每个场景模拟 4 秒，运行全部七个场景：

| 场景 | 输入 | 预期 |
| --- | --- | --- |
| `normal` | 所有传感器每 10 ms 更新 | 持续提交 |
| `sleeve250` | sleeve 每 250 ms 更新 | 肘部按 sleeve 更新，IMU/手套继续更新 |
| `sleeve_dropout` | sleeve 在第 1～2.5 秒断流 | 肘部沿用缓存，其他目标继续，随后恢复 |
| `glove_dropout` | 手套在第 1～2.5 秒断流 | 手指沿用缓存，IMU 继续，随后恢复 |
| `combined` | sleeve 和手套同时断流 | IMU 继续更新，随后恢复 |
| `imu_only_glove_dropout` | 无 sleeve，手套断流 | 肩部与上臂旋转继续更新 |
| `imu_timeout` | IMU 在第 1～2.5 秒断流 | 按现有 IMU 策略报错并停止；验证该行为也记为 PASS |

测试控制周期固定 10 ms；默认 sleeve/IMU 超时阈值为 200 ms，手套为 250 ms。
这些是脚本的显式测试参数，与当前 `configs/phase3.yaml` 的现场阈值无关。`--timeout-ms` 可调整测试中的 sleeve/IMU 阈值。
脚本输出 PASS/FAIL，全部符合预期时退出码为 0，否则为 1。

## 自定义到达间隔与断流

```powershell
# 单独复现 sleeve 每 250 ms 才有新值
python tools/simulate_control_latency.py --scenario sleeve250

# sleeve 250 ms，手套 100 ms；各路额外加入 0～20 ms 的随机到达间隔
python tools/simulate_control_latency.py --scenario custom --sleeve-ms 250 --glove-ms 100 --jitter-ms 20

# 不启用 sleeve；手套从第 1 秒开始断流 1.5 秒
python tools/simulate_control_latency.py --scenario custom --no-sleeve --dropout glove:1:1.5

# 可重复 --dropout，分别控制各路；也可切换到 Bend5
python tools/simulate_control_latency.py --scenario custom --dropout sleeve:1:1 --dropout glove:2:1 --duration 5 --glove-backend bend5
```

`--imu-ms` 控制模拟 IMU 的更新间隔；每对 IMU 使用同步的合成四元数。
`--seed` 固定随机种子，默认 1。断流起点从模拟传感器启动时计算，需大于 0，启动阶段仍要求有效数据。
断流期间丢弃对应到达事件，不在恢复时重放积压帧。`--jitter-ms` 表示到达间隔抖动，不是完整的无线链路模型。
自定义 IMU 断流触发保护时，会显示 FAIL；只有专用 `imu_timeout` 场景将此保护退出作为预期结果。
测试时长应覆盖断流后的恢复阶段；否则恢复检查会失败。

## 保存结果

```powershell
python tools/simulate_control_latency.py --output-dir recordings/control_latency
```

- `summary.json`：场景参数、检查结果、警告和故障原因。
- 每个场景的 `.csv`：每次实际 fake SDK 提交时刻、sleeve 帧号/年龄、手套时间戳/年龄、人体目标和限速后提交的目标。

重复运行会覆盖该目录内的同名报告。`recordings/` 已被 Git 忽略。

检查包括：确实调用了 fake SDK、提交是否连续、同一帧输入是否保持同一目标、超时期间 IMU 是否继续更新、断流后是否恢复，以及资源是否关闭。

## 模拟传感器驱动真实 Aurora

在已配置 Aurora SDK/通信环境的机器人控制电脑上运行。传感器始终由脚本模拟，无需接入 sleeve、IMU 或手套。
仅控制右侧，输入为合成的肩前屈约 10～30°、肘屈曲约 18～72°（启用 sleeve 时）、上臂旋转 ±10°，以及各手指约 20～80% 的归一化输入。
最终指令仍经过实际 profile 的位置、步长、速度和反馈检查；未控制的关节沿用原有目标。

先查看不连接机器人的预览：

```powershell
python tools/simulate_control_latency.py --robot aurora --aurora-profile configs/robot_aurora.yaml --scenario sleeve250 --duration 10
```

执行 sleeve 延迟场景：

```powershell
python tools/simulate_control_latency.py --robot aurora --aurora-profile configs/robot_aurora.yaml --scenario sleeve250 --duration 10 --execute --output-dir recordings/real_sleeve250
```

执行无 sleeve、手套断流场景：

```powershell
python tools/simulate_control_latency.py --robot aurora --aurora-profile configs/robot_aurora.yaml --scenario imu_only_glove_dropout --duration 10 --execute --output-dir recordings/real_glove_dropout
```

`--execute` 后仍需完成现有的 `YES` 交互确认。profile 中配置的 FSM 必须已就绪，脚本不自动切换 FSM。
延迟场景从机器人准备完成后开始，不包含等待确认的时间。真机每次执行一个场景，最长 60 秒，可用 Ctrl+C 提前结束。
达到最后有效目标后，延迟期间会继续提交该目标，不应把输入停更理解为物理急停。

真机报告包含实际提交间隔的均值、P95、最大值；`--max-gap-ms` 设置允许的最大间隔（默认 50 ms），超过后报告 FAIL。
该项是事后检查，不会代替已有的实时机器人保护。提交成功仅表示 SDK 调用成功，不确认机械运动已到位。
真机也使用本脚本的测试超时参数：sleeve/IMU 默认 200 ms、手套 250 ms；不会修改现场配置文件。

## 结果的适用范围

离线模式的虚拟时钟用于快速、可重复地验证调度和解耦逻辑。离线正常场景中的 10 ms 间隔是虚拟时间结果，不能证明电脑实际能稳定运行 100 Hz，也不能测量真实串口、DDS、机器人响应或机械卡顿。
脚本注入到达帧，未经过真实串口字节解析；fake 机器人反馈跟随指令，不包含电机动力学。
真机模式可观察模拟数据到机器人运动的链路；它仍不能复现真实传感器内部采样或无线传输缓存。
