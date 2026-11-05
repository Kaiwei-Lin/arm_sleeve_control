# Bend5 / Bend6 手套与 Aurora 灵巧手

`tools/run_model_control.py` 可同时用 IMU 控制手臂、Bend5 或 Bend6 控制同侧灵巧手；
`tools/debug_glove.py` 只使用一个手套，不需要 Sleeve 或 IMU。
默认仍为 Bend5；Bend6 配置、标定和完整命令见本文后半部分。

## 配置

修改 `configs/sensors.yaml` 中的 `sensors.glove.port`（如 `/dev/ttyUSB0` 或 `COM5`）。
`enabled: true` 会让主程序自动启用手套；也可以保持默认关闭，启动时显式传入
`--glove real`。`--glove none` 始终只运行原有手臂流程。

手套解析、启动调零、逐指/矩阵 JSON 校准和滤波均由本项目的
`sleeve_arm/sources/glove.py` 实现。独立部署只需本项目及 `requirements.txt`
中的依赖，不需要相邻项目、GUI 或 Bridge。`calibration_path` 相对
`sensors.yaml` 所在目录解析；已有校准 JSON 可以复制到本项目的 `calibrations/`
目录，例如配置为 `../calibrations/bend5.json`。

有现成 Bend5 校准 JSON 时设置 `calibration_path`，并保留
`source_options.per_finger_max_delta: null`，以使用文件里的逐指范围。
没有校准文件时，启动后先保持五指伸直，采集 30 帧作为零点；
`per_finger_max_delta` 默认 1000，可改成单个数或按
`[小指, 无名指, 中指, 食指, 拇指]` 排列的五个值。
`source_options` 支持 `bend_direction`、`deadzone_value`、`filter_alpha`、
`adaptive_baseline`、`adaptive_baseline_alpha`、`sensor_map`、
`renderer_remap_from_current` 以及 `calibration_mode`（auto/matrix/per_finger/none）。
非法串口记录或串口读取失败会结束采集并停止控制，避免错误通道继续驱动手指。

默认每帧 11 个值，支持 `v0,v1,...,v10;` 和连续 `v0;v1;...;v10;`。
前五路按小指到拇指解释，六个尾部通道保留在调试读数中。
Aurora 下发顺序按参考示例为：
`[拇指弯曲, 食指, 中指, 无名指, 小指, 拇指横摆]`。
Bend5 只有一个拇指传感器，因此拇指弯曲与横摆联动。
这与直接连接 FDH6 的 Ethernet SDK 顺序不同。

`configs/robot_aurora.yaml` 已包含双侧六关节手部定义；其他仅有手臂的 profile
不能用于手套执行。手部速度、步长、跟踪误差可在该 profile 调整。
Aurora 默认 `smooth_limits: true`，手臂和灵巧手的数值限位均用于平滑约束：
位置目标收回到上下限，速度与步长限制每次指令变化，跟踪误差限制目标超前量。
实测位置或速度超限、启动时轻微移动不再触发退出；启动位置越界会逐步回到范围内。
右手按此前桥接配置设为 `max_velocity: 3.0`、`max_position_step: 0.03`，
100 Hz 时每次最多变化 0.03 rad；左手保留 `max_velocity: 0.8` 和步长 0.02，
100 Hz 时由速度限制到每次最多 0.008 rad。两侧反馈速度阈值均保留 3.14 rad/s，
该阈值是现场配置值，不是厂商额定速度。
启动日志各关节显示 `smooth_limits: True`。断连、过期反馈、非有限数值和设备故障仍会停止控制。
如需缩小手势幅度，可在 `sensors.glove` 设置六项
`open_pose_rad` / `closed_pose_rad`，顺序同上；超出手部范围的配置会自动收回到边界。

## 快速查找手套串口

在 Linux 的项目目录运行，无需预先配置手套端口：

```bash
python tools/identify_glove_ports.py
```

启动时枚举所有 `/dev/ttyUSB*`，按端口编号排序，以 115200 / 8N1 同时读取，
按 Bend5 的 11 路格式解析，每秒刷新 5 次。每个端口单独显示一行：
`bend5` 是前五路原始值（小指、无名指、中指、食指、拇指），`tail` 是其余六路。
无有效数据、超过 1 秒未收到新帧、打开或读取失败时显示 `NONE` 及原因。
各端口独立采集，某个端口没有数据不会阻塞其他端口。

逐个弯曲手指确认变化，再把对应路径填入 `configs/sensors.yaml` 的
`sensors.glove.port`。按 `Ctrl+C` 退出并关闭串口；只扫描读取，不连接机器人。
运行前先退出占用这些串口的采集或控制程序。
若某行出现“读取失败”，该端口的采集已结束；排除原因后重新运行脚本。

需要限时采集或保留输出时：

```bash
python tools/identify_glove_ports.py --duration 10 --no-clear
```

可用 `--prefix` 改设备路径前缀，`--baudrate` 改波特率，
`--print-hz` 改刷新频率，`--stale-seconds` 改无数据超时。

## 单手套调试

以下命令在 `sleeve_arm_control` 仓库根目录运行。
默认只读取手套并打印原始五路值、归一化弯曲量、目标角度，不连接机器人：

```bash
python tools/debug_glove.py --duration 30
```

完全离线测试解析、映射及假机器人限速：

```bash
python tools/debug_glove.py --glove fake --robot aurora-fake --duration 3
```

控制单只真实手（仍需现场输入 `YES`）：

```bash
python tools/debug_glove.py --side right --execute --duration 30
```

`--side left` 选择左手。直接运行 Python 时默认只下发所选
`left_hand` / `right_hand`，不会抬臂；要求机器人处于 profile 的 `allowed_fsm` 第一项，当前默认 FSM 11。
只有显式添加 `--prepare-aurora-fsm` 才请求原有 FSM 准备流程。

手掌被大腿挡住时，可以同时让同侧肩部前抬到相对零位的 **45°**：

```bash
bash debug_glove.sh
# 等价的主要选项：
python tools/debug_glove.py --side right --execute --duration 3000 --arm-forward-deg 45 --arm-move-seconds 5
```

启动脚本默认控制右手，时长 3000 秒（50 分钟）。手套完成伸直调零并在现场输入
`YES` 后，从最新反馈姿态平滑前抬，默认至少用 5 秒到达目标命令，随后持续下发
45° 肩部目标并同步跟随手套。该角度是相对机器人标定零位的绝对角度，
不是在当前角度上再增加 45°；按当前 profile，SDK 的 shoulder pitch 目标为
`-0.7854 rad`。只改变同侧肩前屈和手指目标，其余手臂关节保持启动姿态。
抬臂仍经过现有限速、限位和反馈检查，调试时长包含抬臂过程；
循环延迟或 profile 限速可能延长抬臂时间。

脚本会透传参数，例如 `bash debug_glove.sh --duration 60 --side left`。
`bash debug_glove.sh --arm-forward-deg 0` 恢复仅控制手指，**不会主动放下手臂**。
Python 入口的 `--arm-forward-deg` 默认是 0，保持原来的仅手部行为。
按 `Ctrl+C`、到时或传感器/机器人故障后停止下发，不自动放下手臂或恢复原姿态。

抬臂与手指联动的离线验证：

```bash
python tools/debug_glove.py --glove fake --robot aurora-fake --duration 7 --arm-forward-deg 45
```

## IMU 与手套同时控制

只读预览（使用真实 IMU 和手套，不连接机器人）：

```bash
python tools/run_model_control.py --print-only --glove real --duration 30
```

真实右臂与右手同步控制，沿用原有 IMU 校准和交互确认：

```bash
python tools/run_model_control.py --robot aurora --aurora-profile configs/robot_aurora.yaml --side right --source-side right --glove real --execute --duration 30
```

`control.sh` 默认使用本机 Bend6 标定执行配置，启用真实 IMU 与手套，默认不使用袖套，
完成交互确认后准备 profile 指定的 FSM。运行 `bash control.sh --duration 30` 即可。
主程序的肩部估计仍只支持右臂；`--sleeve real` 可启用肘部输入，`--glove none` 可关闭手套。
仅预览 IMU：`bash control.sh --print-only --glove none --duration 30`；
仅用 IMU 控制右臂：`bash control.sh --glove none --duration 30`。
离线联调可使用 `--robot aurora-fake --side right --source-side right --imus fake --glove fake`。
真实机器人执行不能使用假手套。

联合模式使用一个 AuroraSession 和一个 SafeArmController，单次命令同时包含
右臂与右手；保留完整手臂向量中的未控制关节。
手套的 250 ms 默认超时按串口包到达的单调时间计算，重复读取旧帧不会刷新时间。
启用的手套断流或其他控制故障会结束整条控制链路；退出时关闭所有已打开的源和会话。
停止策略仍为停止下发，不自动张手、复位或恢复启动姿态。

离线回归：`python -m pytest -q tests/test_glove_control.py`。

## Bend6：独立采集和协议边界

本地组织为：`sources/glove.py` 保留 Bend5；`sources/bend6.py` 提供
`Bend6Parser`、`Bend6Calibration`、`Bend6Filter`、`Bend6Reading` 和
`Bend6GloveSource`。`runtime/glove_control.py` 根据 `sensors.glove.backend`
选择 Source，再统一进入现有 RobotRuntime / AuroraSession / SafeArmController。
没有引入 electronic_skin_project、FDH6 Ethernet SDK 或新的机器人后端。

参考了上游的[协议定义](https://github.com/e-skin-whut/electronic_skin_project/blob/master/electronic_skin/domain/bend6.py)、
[采集和标定逻辑](https://github.com/e-skin-whut/electronic_skin_project/blob/master/electronic_skin/sources/gloves/bend6.py)；
没有复制其现场标定数值。源对象和配置加载不打开串口，只有 `start()` 才打开；
`latest()` 不刷新旧帧时间，使用本项目 `time.monotonic()` 语义。

| 索引 | 通道 | 意义 | 默认 Aurora 目标 |
|---|---|---|---|
| 0 | `little_ip` | 小指指间弯曲 | `pinky_flexion` |
| 1 | `little_ring_between` | 小指与无名指之间 | 保留，不控制 |
| 2 | `ring_ip` | 无名指指间弯曲 | `ring_flexion` |
| 3 | `middle_ip` | 中指指间弯曲 | `middle_flexion` |
| 4 | `index_middle_between` | 食指与中指之间 | 保留，不控制 |
| 5 | `index_mcp` | 食指掌指关节 | 保留，不控制 |
| 6 | `index_ip` | 食指指间关节 | `index_flexion` |
| 7 | `thumb_mcp` | 拇指掌指关节 | `thumb_swing` |
| 8 | `thumb_tip_joint` | 拇指指尖关节 | `thumb_bend` |
| 9 | `reserved_10` | 保留字段 | 禁止控制 |
| 10 | `reserved_11` | 保留字段 | 禁止控制 |

默认 115200 / 8N1、串口超时 0.02 秒，严格 ASCII、每包恰好 11 个有限数。
Reading 的 `raw` 始终包含 11 个原值，`raw_channels` 为前九个原始具名值，
`channels` 为九个经过标定和可选滤波的具名值，另带 `timestamp`、`status`、
`side`、`sequence`、`calibration_mode`、`calibrated`、`filter_enabled`、`synthetic`。
Bend5 的 GloveReading 及其 `.curls` 保持不变，Bend6 不压缩成五指 curls。

默认 `source_options.packet_delimiter: ";"` 接收：

```text
0,1,2,3,4,5,6,7,8,9,10;
0 1 2 3 4 5 6 7 8 9 10;
```

包内部的空白、制表符、换行都可分隔字段；不同读取批次、跨行片段会累计至
分号结束，多个包粘连也可解析。若设备**以分号分隔字段、换行结束整包**，设置：

```yaml
source_options:
  side: right
  packet_delimiter: "\n"
  filter: {enabled: false}
```

此模式接收 `0;1;2;3;4;5;6;7;8;9;10\n`（也可用逗号或空白分隔）。
包结束符必须与设备实际格式一致，不能仅凭数值数量推断边界。
特别是没有额外整包边界的 `v0;v1;...;v10;v0;...` 连续单值流：
中途接入后无法判断哪个值是索引 0，Bend6 不将其凑成可控制帧。
这点比 Bend5 的历史累计方式严格；需要设备提供整包分号或整包换行边界。

每次开启或人工重启 Source 都丢弃第一个包结束符终止的记录，哪怕它恰好有
11 个数。坏 ASCII、空标点字段、少/多字段、NaN/Inf、非法数字及超过 8192 字节
的无边界记录均丢弃，不把剩余字段拼接到下一包。损坏包不刷新时间；下一完整包
恢复解析并重置滤波。持续坏包或断流按 `stale_policy` 处理（见下文）。串口异常传播到上层并
停止整条控制链路；不在执行中自动重连，需排障后重新启动和确认。

## Bend6：配置、标定与映射

使用完整的 `configs/sensors_bend6.example.yaml`，其中保留原 Sleeve、四个 IMU、
肩部/旋转估计、同步和记录配置。`enabled: false`、`port: null`、无标定、滤波关闭
是安全默认值。其余设备端口沿用原示例，实际使用前也须核实。

`--glove none/real/fake` 仍表示是否启用以及真实/模拟来源；协议只由 YAML
`backend: bend5|bend6` 决定。不应把 Bend5 的 `zero_frames`、五指矩阵、
`per_finger_max_delta` 等选项带到 Bend6，加载时会拒绝。切回默认 Bend5 使用
`--sensor-config configs/sensors.yaml`；不要覆盖原文件。

### Bend6 断流时保持上一帧

在所用传感器 YAML 的 `sensors.glove` 下配置：

```yaml
stale_timeout_s: 0.25
stale_policy: hold_last
```

`stale_policy` 默认 `error`，保留原来的过期退出行为。`hold_last` 仅支持 Bend6：
运行中超过 `stale_timeout_s` 没有收到新的有效包时，继续使用最后一个有效帧计算
手部目标，不因这次断流退出；持续保持至新有效包到达，没有额外的断流退出时限。
坏包仍被丢弃，不能覆盖最后一个有效帧。新有效包到达后自动更新目标。
上一目标仍经过原来的速度、步长和关节限位控制；若还没有到达上一目标，
会继续按限速走到该目标并保持，并非冻结断流瞬间的实测位置。

帧的 `timestamp`、`sequence`、原始值和滤波结果不会因重复使用而刷新。
输出会显示 `input_state=holding_last` 和持续增长的 `age`；恢复后显示 `input_state=fresh`。
首次启动仍需在 `startup_timeout_s` 内收到新鲜有效帧。串口读取异常／线程退出、
非法映射、无效时间戳和非有限目标仍会退出；raw 数据仍只读。
标定与逐指检查仍必须采集新帧，不能用保留帧代替现场测量。

已按本机操作需求在 `configs/bend6_calibration.measured.sensors.yaml` 和
`configs/sensors_bend6.local.yaml` 中启用 `hold_last`，使用这两个文件的原命令即可生效。
其他配置及 Bend5 默认行为不变；改回 `stale_policy: error` 可恢复过期退出。
此策略只针对手套输入，机器人反馈检查和 IMU/Sleeve watchdog 继续生效。
联合控制时手部保持上一目标，手臂仍由新鲜的 IMU/Sleeve 输入控制。

`calibration_path: null` 为 raw：保留数值及正负号，不猜测量程，不生成机器人目标。
也支持标定文件显式写 `mode: raw`。raw 可进入只读调试或联合 `--print-only`，
不允许进入 aurora-fake 或真实机器人控制；假机器人使用显式 `--glove fake`，
其合成九通道使用测试用 normalized 标定，输出标记 `synthetic=True`，不打开串口。
真实 `--execute` 继续禁止 fake 输入。

独立标定文件可为 `.yaml`、`.yml` 或 `.json`，推荐结构为
`glove.bend6.calibration`，也支持 `bend6.calibration`、顶层 `calibration` 或
直接 `{mode, channels}`。文件路径相对**传感器 YAML 所在目录**解析。
只读取 calibration 部分；设备端口和滤波始终来自传感器 YAML，避免两个文件相互覆盖。

`configs/bend6_calibration.template.yaml` 为待填写模板，九通道端点全是 `null`，
**不能直接用于执行**。在当前手套、当前佩戴方式下逐通道测量后填写：

- `mode: normalized`；九个通道配置必须全部存在。
- 两个 `between` 通道可明确配置为 `{mode: raw}`：保留原始值，不做归一化、
  不通过归一化滤波、不允许映射到机器人关节。此时 Reading 显示 `calibration=partial`
  和 `raw_only_channels`，不声称九路全部标定。其余七路必须具有有效归一化标定；
  只要六轴映射都使用已标定通道，就可正常预览和控制。若要用 between 控制关节，
  需单独实测其张合端点并提供有效标定。
- 每通道填写有限且不同的 `raw_min`、`raw_max`，以及布尔值 `invert`。
  线性公式为 `clip((raw - raw_min)/(raw_max - raw_min), 0, 1)`，然后按需 `1 - value`。
  支持下降端点；不要同时倒置端点和 invert 而无意反转两次。
- 可选 `points: [[raw, normalized], ...]`：至少两点，raw 严格递增，输出从
  0 到 1 非递减（允许零区平台），区间内线性插值，区间外裁剪，最后应用 invert。
  points 存在时优先于线性端点公式，但端点仍必须有效。点值须来自现场测量。
- 完成标定后仍需用只读预览检查自然张开、逐指动作、握拳和拇指组合姿态。

滤波是标定之后的独立阶段，默认关闭。启用后每通道先对变化量应用 deadband，
再限制 `max_delta_per_sample`，最后 EMA：`previous + alpha * delta`。
`alpha` 范围 `(0,1]`，`deadband >= 0`，最大变化量为正数或 null。
deadband 和最大变化量的单位随模式而定（raw 单位或 normalized 单位）。
状态按 Source 实例隔离，首次样本直接通过；启动、关闭和重新同步后清空。

六轴 mapping 必须完整且仅含以下六个目标键；值只能选九个有效通道名称：

```yaml
bend6_mapping:
  thumb_bend: thumb_tip_joint
  index_flexion: index_ip
  middle_flexion: middle_ip
  ring_flexion: ring_ip
  pinky_flexion: little_ip
  thumb_swing: thumb_mcp
```

按应用层的 `HAND_JOINTS` 语义顺序，以名称查找九通道值，计算
`open_pose_rad + normalized * (closed_pose_rad - open_pose_rad)`，再沿用现有
HAND_LIMITS、SafeArmController 的速度/步长/反馈保护。未使用的三个通道始终保留。
`HAND_JOINTS` 及两个 pose 数组仍按拇指弯曲、食指、中指、无名指、小指、拇指横摆排列，
不代表 SDK 数组顺序。RobotRuntime 传递具名目标，AuroraRobotArm 按选中 profile 的
`joints[].index` 组装 SDK 数组，并使用同一索引解析反馈。
未知或缺失关节、reserved 映射、非有限输入/目标会拒绝控制。

上述示例映射中拇指弯曲与横摆有两个独立输入，但传感器仍有姿态相关的机械串扰。
本版**没有移植上下文补偿，也没有验证完整手姿的解耦精度**。
未来补偿应放在 `GloveRuntime._bend6_values()` 映射边界，保留完整原始采集数据。

### 本机：加大闭合目标，并用指尖弯曲同时控制拇指两轴

按操作需求，当前执行文件 `configs/bend6_calibration.measured.sensors.yaml` 使用：
同一组 pose 和 mapping 也写入 `configs/sensors_bend6.local.yaml`，作为本机标定输入。
因此即使执行文件不存在，从这份本机输入重新生成也会保留拇指方向、联动和完整闭合目标。

```yaml
sensors:
  glove:
    # 依次为拇指弯曲、食指、中指、无名指、小指、拇指朝掌心；不是 SDK 顺序。
    open_pose_rad: [0.35, 0.22, 0.22, 0.22, 0.22, 0.05]
    closed_pose_rad: [1.28, 1.78, 1.78, 1.78, 1.78, 1.68]
    bend6_mapping:
      thumb_bend: thumb_tip_joint
      index_flexion: index_ip
      middle_flexion: middle_ip
      ring_flexion: ring_ip
      pinky_flexion: little_ip
      thumb_swing: thumb_tip_joint
```

这两个 pose 数组分别表示**归一化输入为 0 和 1 时的目标**。2026-11-03 重采后，
九通道都是伸直对应 0、握拳对应 1；指尖原始中位数为 250 → 505，`invert: false`。
因此拇指弯曲目标为伸直 0.35、握拳 1.28 rad，朝掌心轴同样读取 `thumb_tip_joint`，
目标为伸直 0.05、握拳 1.68 rad。四指为伸直 0.22、握拳 1.78 rad。
此前使用的反向拇指标定及相应倒置 pose 数组不再适用于此次标定。

以上闭合端点来自现有 profile 限位，未扩大物理限位；断流保持策略继续生效。
闭合端点调整本身没有改变速度，后续的右手提速见下一节。
此处的闭合目标修改只应用于上述本机执行配置，Bend5 和原始预览配置不变。
本机代码的缺省映射也已设为两个拇指轴共用 `thumb_tip_joint`；显式 YAML 的
`bend6_mapping` 优先，因此示例文件仍可选择 `thumb_mcp` 独立控制横摆。
`thumb_mcp` 和其他九通道原始数据仍完整采集，但该配置下 `thumb_mcp` 不再控制机器人。
本机映射主动联动了两个拇指轴，因此不再提供这两个轴的独立输入控制。

重新标定应继承当前执行配置的 pose 和 mapping，操作命令见 D 节。
程序自动设置 `invert` 使伸直为 0、握拳为 1；不要单独翻转它来补偿机器人的方向，
应按实际动作设置对应轴的两个 pose 端点。此处只做离线映射验证，实际机械闭合程度
仍需现场检查；归一化没到端点、反馈尚未跟上或机械串扰，也会影响最终姿态。

### 本机：按旧桥接配置提高右手跟随速度

旧 `aurora_hand_bridge.control` 中 `control_hz: 100.0`、`max_rad_step: 0.03`
等价于目标每秒最多变化 3 rad。当前入口使用 `configs/robot_aurora.yaml`，
不读取那份桥接配置。现已将该 profile 的右手六轴设置为：

```yaml
# 每个 right_hand 关节的 limits；不替换整个 profile。
max_velocity: 3.0
max_position_step: 0.03
max_feedback_velocity: 3.14  # 保留原值
```

目标变化速度上限从 0.8 提高到 3 rad/s，是原来的 3.75 倍。
当前握拳动作最长行程为朝掌心轴的 1.63 rad；假设输入立即到端点、100 Hz 调度、
反馈及时跟上，目标走完该行程约需 0.55 s，原来约需 2.04 s。
这是指令变化时间，不能等同于真实手的运动时间，也不能据此宣称达到硬件满速。
延迟周期仍只允许最多 0.03 rad，短于 10 ms 的周期按实际时间限速；
关节限位、跟踪误差约束、机器人反馈 watchdog、FSM 和人工确认均保留。
此速度配置适用于使用该 profile 的右手控制，包括 Bend5；手套解析和映射不变。
左手和双臂配置没有改变。旧桥接配置中的 `ema_alpha: 0.2` 未迁入；
当前实测 Bend6 配置继续关闭滤波。

停止旧进程后，用原命令重新启动即可加载新速度（执行仍需现场输入 `YES`）：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH python tools/debug_glove.py \
  --sensor-config configs/bend6_calibration.measured.sensors.yaml \
  --glove real --robot aurora --aurora-profile configs/robot_aurora.yaml \
  --side right --execute --duration 15
```

这次只做离线假机器人测试，实际跟随速度和机械表现仍需现场验证。

## Bend6 完整命令（仓库根目录）

以下均使用已有 Python CLI；示例端口需要替换成 A 中实际识别的端口。
不要同时运行两个程序占用同一串口。

### A. 查找串口，无需机器人

```bash
python tools/identify_glove_ports.py --backend bend6 --duration 10 --no-clear
# 若设备使用分号字段、换行整包：
python tools/identify_glove_ports.py --backend bend6 --bend6-packet-delimiter newline --duration 10 --no-clear
```

显示九个具名原始通道、两个 reserved、帧序号及年龄；多个端口各有独立解析器。
该工具只读串口，默认协议仍为 bend5。

### B. 九通道 raw 预览，无需机器人

```bash
cp configs/sensors_bend6.example.yaml configs/sensors_bend6.local.yaml
# /dev/ttyUSB0 仅是端口示例，替换为 A 中确认的设备：
GLOVE_PORT=/dev/ttyUSB0 python - <<'PY'
import os
from pathlib import Path
import yaml
p = Path('configs/sensors_bend6.local.yaml')
data = yaml.safe_load(p.read_text())
data['sensors']['glove']['port'] = os.environ['GLOVE_PORT']
p.write_text(yaml.safe_dump(data, sort_keys=False))
PY
python tools/debug_glove.py --sensor-config configs/sensors_bend6.local.yaml --glove real --duration 30
```

会输出全部 raw、raw_channels、channels、标定/滤波状态、sequence、age 和
`target_rad=unavailable`。默认 Python 调试入口不连接机器人。

如果 `error` 策略下出现 `glove watchdog`，或 `hold_last` 下长时间保持旧帧，
可把手套靠近接收器，再用只读诊断观察：

```bash
python tools/debug_glove.py --sensor-config configs/sensors_bend6.local.yaml --glove real --diagnose-serial --duration 10
```

此选项不创建机器人或目标角度，在过期时标记 `STALE` 并继续观察；普通预览、
执行和联合控制按各自配置的 `stale_policy` 处理手套旧帧。
`bytes_received` 不再增加且 `byte_age_s` 增大，
说明没有新串口字节；字节持续增加而 `invalid` 增加，则查看 `last_error` 和
`last_invalid_record`；`pending_bytes` 持续增大通常需要核实整包结束符。
`last_chunk`、`last_invalid_record` 为最多 160 字节的转义原文；错误计数与
`last_error` 是本次启动以来的历史统计，恢复后不会清零。源重新启动会清空统计。

### C. Bend6 fake + aurora-fake，完全离线

```bash
python tools/debug_glove.py --sensor-config configs/sensors_bend6.example.yaml --glove fake --robot aurora-fake --duration 3
```

此命令不需要填写端口或现场标定文件，假数据明确标记为 synthetic。

### D. 加载现场标定并预览六关节角度

推荐先运行两动作标定程序，只打开配置中的 Bend6 串口，不导入或连接机器人：

```bash
python tools/calibrate_bend6_glove.py --sensor-config configs/sensors_bend6.local.yaml --output configs/bend6_calibration.measured.yaml
```

按提示操作两次：

1. 伸直五指，保持自然张开，按 Enter。等待 1 秒稳定后，继续保持 3 秒。
2. 握拳，保持想要作为闭合端点的姿势，按 Enter。同样等待并保持。

程序直接采集未经标定、未滤波的 11 字段数据，忽略输入配置里的旧标定路径。
每通道取两组数据的中位数，将较小值写入 `raw_min`、较大值写入 `raw_max`。
如果握拳值比伸直值小，自动设置 `invert: true`，因此伸直映射到 0、握拳映射到 1。
两个 reserved 字段保留在采集报告中，不进入标定通道。

成功时生成三个新文件，原传感器配置保持不变：

- `configs/bend6_calibration.measured.yaml`：兼容 `glove.bend6.calibration` 的标定，包含全部九路配置。
- `configs/bend6_calibration.measured.report.json`：两动作逐帧原始数据、单调时间戳、序号、中位数、噪声和质量检查结果。
- `configs/bend6_calibration.measured.sensors.yaml`：原传感器配置的副本，标定路径指向新文件。

预览六个目标关节角度：

```bash
python tools/debug_glove.py --sensor-config configs/bend6_calibration.measured.sensors.yaml --glove real --duration 30
```

默认遇到同名输出会在打开串口前拒绝覆盖。要覆盖原标定，可直接运行：

```bash
python tools/calibrate_bend6_glove.py \
  --sensor configs/sensors_bend6.local.yaml \
  --output configs/bend6_calibration.measured.yaml --overwrite
```

`--sensor` 是 `--sensor-config` 的别名。只有采集完成且七个非 between 通道质量检查通过，
才更新同名标定和报告。已有 `.sensors.yaml` 的设置（包括 pose、mapping 和保持策略）
会被保留，仅更新其中的标定路径；本次采集仍使用 `--sensor` 指定文件的串口。
如果 `.sensors.yaml` 不存在，则从输入配置生成。取消或采集失败不会改动原输出，
质量不合格时另存 `.failed-<时间>.report.json`，原标定及报告保留。
写入采用临时文件替换；遇到替换异常会尝试恢复此前已替换的文件。

**如果要输出到新的文件名，应选择带有正确 pose/mapping 的传感器配置作为输入**，
这样新文件才会继承已调整的机器人角度和六轴映射。例如：

```bash
python tools/calibrate_bend6_glove.py \
  --sensor-config configs/bend6_calibration.measured.sensors.yaml \
  --output configs/bend6_calibration.recalibrated.yaml
```

之后使用新生成的 `configs/bend6_calibration.recalibrated.sensors.yaml` 预览和执行。
本机 `sensors_bend6.local.yaml` 已同步上述定制 pose/mapping，也可继续使用原命令。
其他未配置 pose/mapping 的预览文件仍会使用默认值，不能自动找回另一文件的设置。
生成的执行文件总会显式保存实际采用的两端角度和映射，终端也会打印这三项。
本工具的串口来自 `--sensor-config`，不支持 `--port` 或 `--sensor-config-output`；
传感器副本文件名由 `--output` 自动派生。

也可省略 `--output`，自动保存到 `calibrations/bend6_<时间>.yaml` 并打印完整预览命令。
`--output` 同时支持 JSON；`--duration 5` 可延长每个动作的采集时间，
`--settle-seconds` 调整回车后的稳定等待时间。每动作默认至少 20 个新帧，
可用 `--min-samples` 调整但不能少于 5；重复读取同一帧不重复计数。

`little_ring_between` 和 `index_middle_between` 不参加伸直/握拳的幅度和噪声检查，
包括两姿态数值完全相同的情况。它们优先保留旧文件中的有效标定（包括 points 和 invert）；
没有旧标定则写成 `{mode: raw}`，不根据静态噪声猜测量程。旧文件优先使用覆盖目标，
其次使用输入执行配置所引用的标定。两个通道的 11 字段原始记录和具名读数始终保留。

其余七通道须通过质量检查：端点不同、差值至少为 `--min-span`（默认 1 个原始单位），
且差值大于两动作静态噪声中较大者的 3 倍。噪声使用每姿态的 P90−P10。
这些是软件质量阈值，不是手套物理量程。动作不稳定或某通道幅度不足时，只保存
报告并指出失败通道，不生成 normalized 标定及其传感器配置。
采集期间断流、旧帧、读取异常或新帧不足会终止；请检查距离/连接后重新采集。

**两动作标定仅定义伸直和握拳两个姿态，不能证明九个通道的独立全量程。**
`thumb_mcp` 横摆和 `index_mcp` 可能没有充分变化；若未通过检查，
需要调整姿态重采或为这些通道单独测量端点，不能用猜测范围补齐。
即使文件成功生成，仍须只读验证拇指两轴和各手指方向，不代表完整姿态已解耦。

#### 手指对应错误时，先做逐指只读检查

伸直／握拳标定只确定量程和正反方向，不能识别实际手指与串口字段的对应关系，
也不能验证机器人 SDK 数组的物理顺序。若出现弯拇指却动机器人食指等情况，
先停止执行模式，用下面的工具比较动作前后数据（不连接机器人）：

```bash
python tools/check_bend6_mapping.py \
  --sensor-config configs/bend6_calibration.measured.sensors.yaml \
  --joint thumb_bend \
  --output /tmp/bend6_thumb_mapping.json
```

第一次按提示自然伸直手指，第二次只弯曲拇指指尖，尽量保持拇指横摆和其余四指不动。
每次回车后等待 1 秒，再采集 2 秒。显示所有 11 个字段的中位数、原始差值、静态噪声，
以及九个通道在当前 `bend6_mapping` 中控制的关节。已有有效 normalized 标定时，
额外显示归一化差值；采集始终使用未滤波 raw 数据。串口旧帧和断流仍会终止采集。
可省略 `--output` 只打印；若同名报告已存在，换文件名，工具不会覆盖它。
报告保留原始帧，不更改标定或传感器配置。

`--joint` 可换为 `index_flexion`（食指）、`middle_flexion`（中指）、
`ring_flexion`（无名指）、`pinky_flexion`（小指）、`thumb_swing`（拇指向掌心横摆）。
索引从 0 开始。以 `sensors_bend6.example.yaml` 的独立拇指映射为例，
当前 `robot_aurora.yaml` 路由如下；本机联动配置的两个拇指目标都读取 `raw[8]`：

| 手套字段 | 逻辑关节 | 右手 SDK 索引（本次修正） | 左手 SDK 索引（保留原配置） |
|---|---|---|---|
| `raw[8] thumb_tip_joint` | `thumb_bend` | 4 | 0 |
| `raw[6] index_ip` | `index_flexion` | 0 | 1 |
| `raw[3] middle_ip` | `middle_flexion` | 1 | 2 |
| `raw[2] ring_ip` | `ring_flexion` | 2 | 3 |
| `raw[0] little_ip` | `pinky_flexion` | 3 | 4 |
| `raw[7] thumb_mcp` | `thumb_swing` | 5 | 5 |

2026-10-28 现场反馈：右手原来呈现“拇指动作→机器人食指，食指→中指，
中指→无名指，无名指→小指，小指→拇指”，横摆正常；随后只弯拇指的采集显示
`raw[8]` 从 244 增至 1187（差值 943，静态 P90−P10 为 50.8），
支持拇指输入字段正确。结合五指现场观察，右手 profile 的 SDK 顺序修正为
食指、中指、无名指、小指、拇指弯曲、拇指横摆。各具名关节的原有限位数值、
方向、速度和步长保持不变，完整 SDK 限位数组随关节索引重排。
现有 Bend6 mapping、标定和 pose 数组继续按原语义使用；索引修正本身无需重排这些数据。
该 profile 同时供 Bend5/Bend6 使用，两者的手套解析和具名关节映射保持兼容。
左手没有此次现场证据，保留旧配置，不据右手推断左手顺序。
本次修正仅做离线验证，修正后的实物逐指方向及拇指串扰仍需现场复核。
该次观测中的拇指值 1187 已超过本地标定上限 622，因此会被裁剪到 1；
这可能造成提前到达闭合目标，但不是五指错位的原因。若要覆盖这一动作范围，
应重新采集覆盖实际使用姿态的标定，复核拇指弯曲／横摆组合下的串扰。

例如，只弯拇指时 `raw[8]` 明显变化，而现场机器人食指运动，说明要进一步核查
GR3 槽位顺序；若主要改变的是 `raw[6]`，则要核查手套字段与实际佩戴／接线关系。
各通道增益、量程和机械串扰不同，不能只按最大原始差值自动重排。
若错位在机器人侧，应同时核实该槽位的方向与限位；不要通过重命名手套通道
掩盖问题，否则拇指和其他手指可能套用错误的角度范围。
上面的自动生成配置可替换 E/F 命令里的 `--sensor-config`，真实执行授权要求不变。

需要手动填写或加入分段点时，仍可使用以下模板方式：

先复制模板，用编辑器把九个通道的 null 全部替换为当前手套的实测端点，
填写方向，按需添加实测分段点；不能照抄其他手套的数值：

```bash
cp configs/bend6_calibration.template.yaml configs/bend6_calibration.local.yaml
# 在编辑器中完成 configs/bend6_calibration.local.yaml 后再执行下面的校验和配置：
python - <<'PY'
from pathlib import Path
import yaml
from sleeve_arm.sources.bend6 import Bend6Calibration, load_bend6_calibration
c = Bend6Calibration(load_bend6_calibration('configs/bend6_calibration.local.yaml'))
assert c.mode == 'normalized'
data = yaml.safe_load(Path('configs/sensors_bend6.local.yaml').read_text())
data['sensors']['glove']['calibration_path'] = 'bend6_calibration.local.yaml'
Path('configs/sensors_bend6.calibrated.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
PY
python tools/debug_glove.py --sensor-config configs/sensors_bend6.calibrated.yaml --glove real --duration 30
```

只打印六个 Aurora 目标弧度，不连接机器人；先逐轴核对默认 mapping 和拇指串扰。

### E. 现场验证后执行真实 GR3 手部

确认标定、目标方向、机器人 profile、FSM、现场空间和急停后才运行：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH python tools/debug_glove.py --sensor-config configs/bend6_calibration.measured.sensors.yaml --glove real --robot aurora --aurora-profile configs/robot_aurora.yaml --side right --execute --duration 15
```

这里使用两动作标定工具生成的 `bend6_calibration.measured.sensors.yaml`；
若按 D 节手工填写标定，改用 `configs/sensors_bend6.calibrated.yaml`。
`env` 前缀仅清除本次进程的两个环境变量，避免本机 `/opt/fftai` 的 DDS 库
覆盖 pip SDK 配套库。若遇到 `_PolicyServiceWrapper.so: undefined symbol`，
先运行 `env -u LD_LIBRARY_PATH -u PYTHONPATH python tools/aurora_sdk_doctor.py`
进行离线导入检查，详见 [SDK 环境说明](aurora_sdk_environment.md)。

仍需现场输入 `YES`。默认只控制右手；`--side left` 选择左手，并应同步核实
`source_options.side` 与实际佩戴侧。可选肩部前抬仍用 `--arm-forward-deg` 和
`--arm-move-seconds`。`debug_glove.sh` 的默认真实执行、45° 前抬和 3000 秒行为
保持不变；它不是 raw 预览入口。

### F. Bend6 + IMU 联合控制

先配置本地 YAML 的真实 IMU 端口和安装关系。raw 手套与 IMU 只读预览：

```bash
python tools/run_model_control.py --sensor-config configs/sensors_bend6.local.yaml --print-only --imus real --glove real --duration 30
```

完整链路离线模拟（保留原 IMU 校准提示，按提示回车）：

```bash
python tools/run_model_control.py --sensor-config configs/sensors_bend6.example.yaml --robot aurora-fake --side right --source-side right --imus fake --glove fake --calibration-seconds 0.1 --duration 3
```

已完成现场标定和逐轴验证后，真实右臂与右手联合执行：

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH python tools/run_model_control.py --sensor-config configs/bend6_calibration.measured.sensors.yaml --robot aurora --aurora-profile configs/robot_aurora.yaml --side right --source-side right --sleeve real --imus real --glove real --prepare-aurora-fsm --execute --duration 15
```

仍使用 Sensor Runtime → Intent Pipeline → ModelControlApp → RobotRuntime，
共享单个机器人会话，不改变 IMU/Sleeve 算法或执行授权。需要 Sleeve 肘部输入时
显式加 `--sleeve real` 并确认其端口。手套旧帧按 `stale_policy` 处理；线程异常、
无效目标或其他传感器／机器人故障仍会停止整条链路并释放串口及会话；
退出不自动恢复姿态。

## Bend6 离线验证

依赖本项目运行依赖与 pytest，不需要真实串口、机器人或实际 Aurora SDK：

```bash
python -m pytest -q tests/test_bend6_glove.py
python -m pytest -q tests/test_bend6_stale_policy.py
python -m pytest -q tests/test_calibrate_bend6_glove.py
python -m pytest -q tests/test_check_bend6_mapping.py
python -m pytest -q tests/test_glove_control.py
python -m pytest -q tests/test_identify_glove_ports.py
python -m pytest -q
```

测试使用 fake serial、fake clock 和 aurora-fake，并在隔离项目副本中禁止导入
electronic_skin、真实 Aurora SDK 或串口模块运行 fake 两入口。测试里的单位量程
只用于合成数据，不能当作现场归一化参数。

### 本次离线验证记录（2026-10-28）

首次接入 Bend6 时，在 `/tmp/bend6-test-venv` 临时环境运行专项及全量 pytest（Python 3.10、
pytest 9.2.0、NumPy 2.2.6、SciPy 1.15.3、scikit-learn 1.7.2），
仅复用本机运行依赖，不安装或加载真实 Aurora SDK，不连接真实设备。

| 范围 | 结果 |
|---|---|
| `test_bend6_glove.py` | 96 passed |
| `test_glove_control.py` | 35 passed，1 failed（原有失败） |
| `test_identify_glove_ports.py` | 14 passed |
| 全量 | 891 passed，7 failed，2 skipped |
| 修改前副本全量，包含用户原有未提交版本 | 793 passed，完全相同的 7 failed、2 skipped |

7 项原有失败：`test_aurora_smooth_limits.py` 的停滞目标预期（0.47 与现有配置的
1.17）；`test_glove_control.py` 的速度阈值预期（2 小于现有 3.14）；
`test_model_target_limits.py` 两项严格拒绝预期与当前 smooth-limits 行为不同；
`test_upper_arm_rotation.py` 三项 IMU 角色预期与当前传感器 YAML 不同。
这些旧测试、物理限位、安全策略和现场配置均保持原样。

两项 skipped 为真实 SDK 安装契约检查，不计为通过。初次运行还遇到默认 Python
缺 pytest、隔离子进程缺 PyYAML、系统 NumPy/SciPy ABI 不匹配、已安装可选 SDK
动态库符号错误；最终用上述无 SDK 临时环境完成离线测试。沙箱最初阻止三个
TCP/UDP 回环测试创建 socket，经授权重跑后通过。没有执行真实串口或机器人测试。

后续加入串口诊断和两动作标定程序后，标定专项 `test_calibrate_bend6_glove.py`
33 项通过，Bend6/诊断专项 113 项通过，端口识别 14 项通过。
全量复测为 941 passed、相同的 7 项既有 failed、2 项 SDK 检查 skipped。
未连接真实手套或机器人，未替换用户本地配置或填入模拟现场标定值。

逐指检查工具加入后，`test_check_bend6_mapping.py` 30 项通过，
`test_bend6_glove.py` 115 项通过，标定 33 项及端口识别 14 项回归通过。
新增左右手逐字段至 SDK 数组的离线检查，仅证明软件按当前 profile 路由，
不能代替 GR3 实际槽位顺序的现场确认。本次未重跑全量测试。

随后根据五指现场观察及独立拇指原始数据，修正右手 profile 索引和 SDK 限位数组。
补充左右手命令／反馈往返及各关节限位跟随索引的检查，原有平滑约束测试中的
右手食指／拇指槽位随配置更新，数值断言保持不变。全量复测为
974 passed、相同的 7 项既有 failed、2 项 SDK 检查 skipped。
已离线对比确认双臂、左手配置和右手各具名关节的限位／方向／零点不变，
本地手套标定与传感器配置未修改。未连接真实设备或执行修正后的运动测试。

按后续操作需求加入 `stale_policy: hold_last` 后，新增断流保持专项 26 项全部通过，
覆盖长时间旧帧保持、新帧恢复、原始预览、启动新鲜度、假机器人调试／联合控制，
以及串口、目标、机器人反馈和 IMU 故障仍退出。相关回归为 211 passed、
1 项既有 failed；全量为 1000 passed、相同的 7 项既有 failed、2 项 SDK 检查 skipped。
本机两个 Bend6 传感器配置已显式启用保持策略，标定参数没有随此次修改调整。

随后提高当前执行配置的握拳目标，并让两个拇指轴共用指尖输入，保留当前
`invert: true` 的标定方向。用已保存的两姿态报告离线复算，伸直目标不变，
握拳目标为拇指弯曲 1.28、四指各 1.78、朝掌心 1.68 rad。新增三姿态及
假机器人限速到达测试通过；相关专项 209 passed，全量 1004 passed、
相同的 7 项既有 failed、2 项 SDK 检查 skipped。未更改标定或机器人限位，
未执行真实设备运动测试。

参照操作人员提供的旧桥接配置，将右手速度和步长分别调整到 3 rad/s、0.03 rad。
离线逐项对比确认 profile 只改变右手六轴的这 12 个字段；新增/更新测试覆盖
55 个 10 ms 周期内完整张开与闭合、零时间保持、短周期按时间限速、延迟周期
仍限制单步，以及断流保持后恢复。手套和限速专项为 229 passed、2 项既有 failed；
全量为 1007 passed、相同的 7 项既有 failed、2 项 SDK 检查 skipped。
使用隔离环境 `/tmp/bend6-test-venv/bin/python`，没有连接真实手套或机器人。

2026-11-03 重采后，执行配置丢失定制 pose/mapping，恢复为默认目标。
按新报告的伸直 0、握拳 1 修正本机执行配置，恢复完整闭合目标和拇指两轴联动，
原始标定文件与报告保持不变。用报告中位数离线通过 Source/GloveRuntime 复算，
伸直目标为 `[0.35, 0.22, 0.22, 0.22, 0.22, 0.05]`，握拳目标为
`[1.28, 1.78, 1.78, 1.78, 1.78, 1.68]`，均在现有 profile 限位内。
`test_bend6_glove.py` 和 `test_calibrate_bend6_glove.py` 合计 157 passed，
使用 `/tmp/bend6-recalibration-venv/bin/python`；本次未重跑全量测试，未连接真实设备。

随后将两个 between 通道排除出两动作的幅度/噪声检查；无旧标定时以 raw-only
保留，运行时只允许已归一化通道进入六轴映射。已保存的失败采集报告离线复算通过，
没有重写现场数据。标定、Bend6、映射检查和断流专项合计 225 passed。
全量 1022 passed、8 failed、2 skipped：除此前 7 项失败外，端口识别工具当前
默认 backend 为 bend6，与测试要求 bend5 不一致；本次未修改该工具的默认值。
两项跳过仍为真实 SDK 安装检查。本次未连接真实手套或机器人。

后续确认用户持续使用的本机标定输入未包含定制 pose，导致执行文件缺失时重新
生成了默认角度。现已同时修正 `sensors_bend6.local.yaml` 和当前 measured 执行配置，
标定工具将解析后的 pose/mapping 显式写入输出并打印。使用真实本机输入离线验证
首次生成与覆盖均保留设置，并回放最新报告经过 GloveRuntime 到 aurora-fake：
握拳 SDK 顺序目标为 `[1.78, 1.78, 1.78, 1.78, 1.28, 1.68]`，再伸直能返回原目标。
现场标定数值和报告未修改。标定与 Bend6 专项 170 passed，本次未重跑全量测试，
未连接真实设备。
