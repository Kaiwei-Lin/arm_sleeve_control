# Bend5 手套与 Aurora 灵巧手

`tools/run_model_control.py` 可同时用 IMU 控制手臂、Bend5 控制同侧灵巧手；
`tools/debug_glove.py` 只使用一个手套，不需要 Sleeve 或 IMU。

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
如需缩小手势幅度，可在 `sensors.glove` 设置六项
`open_pose_rad` / `closed_pose_rad`，顺序同上，且不能超出 Aurora 手部限位。

## 单手套调试

以下命令在 `sleeve_arm_control_v2` 目录运行。
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

`--side left` 选择左手。单手调试只下发所选 `left_hand` / `right_hand`，
不会抬臂或修改其他机器人关节；默认要求机器人已处于可控制的 PdStand。
只有显式添加 `--prepare-aurora-fsm` 才请求原有 FSM 准备流程。

## IMU 与手套同时控制

只读预览（使用真实 IMU 和手套，不连接机器人）：

```bash
python tools/run_model_control.py --print-only --glove real --duration 30
```

真实右臂与右手同步控制，沿用原有 IMU 校准和交互确认：

```bash
python tools/run_model_control.py --robot aurora --aurora-profile configs/robot_aurora.yaml --side right --source-side right --glove real --execute --duration 30
```

现有 `control.sh` 会透传选项，也可以运行 `bash control.sh --glove real --duration 30`。
主程序的肩部估计仍只支持右臂；`--sleeve real` 可额外启用原有肘部输入。
离线联调可使用 `--robot aurora-fake --side right --source-side right --imus fake --glove fake`。
真实机器人执行不能使用假手套。

联合模式使用一个 AuroraSession 和一个 SafeArmController，单次命令同时包含
右臂与右手；保留完整手臂向量中的未控制关节。
手套的 250 ms 默认超时按串口包到达的单调时间计算，重复读取旧帧不会刷新时间。
启用的手套断流或其他控制故障会结束整条控制链路；退出时关闭所有已打开的源和会话。
停止策略仍为停止下发，不自动张手、复位或恢复启动姿态。

离线回归：`python -m pytest -q tests/test_glove_control.py`。
