# GR3 机器人开机、联网与 SDK 调试交接

更新日期：2026-09-27。

本文按现场操作顺序整理。开机方式、网络地址、`viewturbo` 和已安装 Codex 等
信息由交接人提供，本次未登录机器人核实。项目命令已与当前代码核对。
先完成不依赖传感器的 SDK 调试，再接入 IMU 和袖套。

```text
电池开机 → 下方开关绿灯闪烁 → 连接机器人 WiFi → 配置机器人互联网
    → SSH → viewturbo + 代理环境变量 → 更新仓库
    → SDK 安装检查 → 只读反馈检查 → 菜单动作调试 → IMU/袖套联调
```

## 1. 开机与连接机器人 WiFi

1. 长按**电池的开关**，等待设备有反应。
2. 再长按**电池下方的开关**，直到绿灯闪烁。
3. 等待机器人 WiFi 出现，将操作电脑连接到机器人 WiFi。
4. 记录实际 WiFi 名称和设备编号。交接人记得编号可能为 **0197 或 0196**，
   SSH 用户名需要据此与现场设备记录核对，不能直接认定为 0197。

这里记录的是开机步骤，不是机器人站立模式的准备步骤。绿灯闪烁、能 SSH 登录，
都不能代替后面 SDK 对 PdStand 和稳定度的检查。

## 2. 给机器人接入互联网

电脑保持连接机器人 WiFi，在浏览器地址栏打开：

```text
192.168.237.2
```

这是交接人提供的 **WiFi 管理地址**。在管理页面里让机器人连接可用的互联网
WiFi；管理页面账号、密码和实际菜单名称需现场确认。改动网络后若连接中断，
重新连接机器人 WiFi，再继续 SSH。

两个地址用途不同：

| 地址 | 用途 |
| --- | --- |
| `192.168.237.2` | WiFi 管理页面，配置机器人接入互联网 WiFi |
| `192.168.137.220` | 交接人提供的机器人 SSH 地址 |

这两个地址由现场配置决定；不要因网段不同就擅自改成同一个地址。
仅配置 HTTP 代理不能代替连接机器人 WiFi。

## 3. SSH 登录机器人

在**操作电脑的终端**执行。若确认设备为 0197：

```bash
ssh gr301aa0197@192.168.137.220
```

若现场确认实际账号为 0196，改用：

```bash
ssh gr301aa0196@192.168.137.220
```

按 SSH 提示交互输入交接人提供的密码；输入时终端不显示字符是正常现象。
登录密码通过交接附页单独提供，不存入 Git，也不写在 shell 命令中。

登录后确认连接到的主机、账号与系统架构：

```bash
whoami
hostname
uname -m
uname -r
```

后续 `viewturbo`、环境变量、Git、Python 和 Codex 命令，除非特别说明，
都在这个**机器人上的 SSH 终端**执行，不是在操作电脑本地执行。

## 4. 启动 viewturbo 并设置代理环境变量

交接人已在机器人上安装代理，入口命令是：

```bash
viewturbo
```

按程序界面启用代理，确认启动成功和监听端口。交接人提供的端口为 **15732**，
但未提供监听协议；以下 HTTP 和 SOCKS5 示例按实际协议选择一组。
如果 `viewturbo` 以前台方式运行，需要保留该终端，另开一个 SSH 终端继续操作。

可检查端口是否监听：

```bash
ss -lntp 'sport = :15732'
```

若界面显示该端口支持 **HTTP 或 mixed 代理**，在将要运行 Git/Codex 的终端设置：

```bash
export http_proxy="http://127.0.0.1:15732"
export https_proxy="http://127.0.0.1:15732"
export all_proxy="http://127.0.0.1:15732"
export HTTP_PROXY="$http_proxy"
export HTTPS_PROXY="$https_proxy"
export ALL_PROXY="$all_proxy"
```

`https_proxy` 的值仍可为 `http://...`：它表示 HTTP 代理的地址，目标 HTTPS
连接通过代理建立。如果实际端口仅支持 **SOCKS5**，改用这一组：

```bash
export http_proxy="socks5h://127.0.0.1:15732"
export https_proxy="socks5h://127.0.0.1:15732"
export all_proxy="socks5h://127.0.0.1:15732"
export HTTP_PROXY="$http_proxy"
export HTTPS_PROXY="$https_proxy"
export ALL_PROXY="$all_proxy"
```

再让已知的本地地址绕过代理：

```bash
export no_proxy="localhost,127.0.0.1,::1,192.168.137.220,192.168.237.2,gr301aa0197,gr301aa0196"
export NO_PROXY="$no_proxy"
curl -I --max-time 15 https://github.com
```

这些变量只作用于当前 shell 及其随后启动的程序。**新开 SSH 终端要重新设置**，
或者使用现场已配置好的环境脚本。必须在同一个已设置代理的终端里启动 Codex。
如果 `curl` 报连接失败，先检查 viewturbo 进程、端口和 HTTP/SOCKS5 协议是否对应。
SOCKS5 能否被其他客户端使用，仍需按该客户端支持情况确认。

## 5. 进入机器人上已有的仓库并更新

机器人上已拉取仓库。交接人记得目录名为 `arm_sleeve_contro`，可能存在拼写差异；
先找到现有目录，不要直接重新克隆覆盖。可先查看家目录：

```bash
ls -d ~/arm_sleeve* ~/sleeve_arm* 2>/dev/null
```

确认目录后进入。例如只有确认目录确实存在时才执行：

```bash
cd ~/arm_sleeve_contro
pwd
git status --short
git remote -v
git branch --show-current
git log -1 --oneline
```

需要获取的项目是 **`Kaiwei-Lin/arm_sleeve_control` 的 `master` 分支**。
编写本文时，上一版已推送的功能提交为 `644a80a`，包含菜单动作调试脚本、
本地估计器及 runtime 重构；后续提交可能更新，不要把此 hash 当成永久最新版本。

确认当前为 master，且它的 upstream 指向上述仓库后，正常更新即可：

```bash
git pull --ff-only
```

若配置了多个远端，明确选 URL 对应这个仓库的远端。比如正确远端名为 `skin`：

```bash
git pull --ff-only skin master
```

**SSH 格式的 Git 远端不会因为设置了 `http_proxy` 就自动走 HTTP 代理。**
如果 GitHub SSH 无法访问，而上一步 HTTPS 已通，可在 master 分支使用下面的
HTTPS 拉取命令；它不修改已有 remote 配置，私有仓库仍需已有的 GitHub 访问权限：

```bash
git pull --ff-only https://github.com/Kaiwei-Lin/arm_sleeve_control.git master
```

如果工作区有修改、分支不对或无法快进，先保留现场配置并与维护人核对，再合并。
不要为完成更新而覆盖串口、零位和限位配置。更新后确认文件已存在：

```bash
git log -1 --oneline
ls tools/aurora_arm_debug.py tools/aurora_sdk_doctor.py
```

机器人上已经安装 Codex。如需用它继续维护，在已设置代理且位于仓库根目录的
SSH 终端启动即可；不要把本地模拟测试成功当成机器人现场验证成功。

```bash
codex
```

## 6. 确认机器人上的 Python 环境

先激活机器人上已有的项目环境。**环境名称和路径待现场确认**，本机开发环境
叫 `skin` 不代表机器人上的环境也叫这个名字。

```bash
which python
python --version
python -m pip show fourier-aurora-client
python tools/aurora_sdk_doctor.py
```

项目使用 Python 3.10+，当前 Aurora backend 固定为
`fourier_aurora_client==0.1.8`。默认 doctor 不启动 DDS；核对输出里的版本、API
兼容性以及 `dds_started: false`。当前 0.1.8 只读连接实现要求 Linux x86_64；
若 `uname -m` 显示其他架构，不要认定现有 wheel 可以直接运行，需另行核对部署包。

缺依赖时，在已确认的项目 Python 环境中安装：

```bash
python -m pip install -r requirements.txt -r requirements-aurora.txt
python -m pip check
```

`DualImuArmEstimator` 和 `FlexArmEstimator` 已在仓库内，无需另外安装
`flexarm-estimator` wheel。详见 [估计器迁移说明](local_estimators.md)。

## 7. 调试 Aurora SDK

### 7.1 先读取机器人状态

确认机器人侧 Aurora 服务已按现场流程运行。服务启动命令尚未提供，
不要猜服务名或用反复切 FSM 代替服务排查。

现有已验证 backend 使用原生 DDS；在运行只读 doctor 的终端显式设置：

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export FOURIERDDS_ROS_COMPATIBLE=false
export FOURIERDDS_USE_DISCOVERY_SERVER=false
python tools/aurora_sdk_doctor.py --connect --domain-id 123 --no-ros-compatible
```

`123` 来自当前 GR3 profile，仍应核对现场 DDS domain。此命令只订阅状态，
不会调用 `set_group_cmd` 或切换 FSM。重点检查：

- `feedback_valid: true`，状态端点匹配，反馈未过期。
- `right_manipulator` / `left_manipulator` 组存在，各自位置向量长度为 7。
- 全身 FSM 符合现场状态。后面的站立动作调试要求 **PdStand / FSM 2**，
  并由 backend 检查 `get_stand_pose()` 中的稳定度严格大于 100。

只读订阅成功证明读取路径可用；完整控制客户端的 publisher 匹配和实际动作，
仍要在下一步确认。`allow_missing_velocity_cmd` 只容许已核实的唯一
`velocity_cmd` publisher 缺失，其他 DDS mismatch 仍会失败。

### 7.2 确认数字菜单与模拟流程

```bash
python tools/aurora_arm_debug.py --backend fake --simulate
```

先输入方向编号，再输入角度。例如向前抬 30°：

```text
请选择动作编号：1
请输入角度（度，0～180，直接回车取消）：30
```

编号 `1` 向前、`2` 向后、`3` 侧摆（向外）、`4` 向内收、`5` 屈肘、`6` 伸肘；
`8` 查看当前角度，`0` 退出。不需要输入中文。
默认角度相对已标定零位，不是每次累加；需要增量动作时
启动参数加 `--reference current`。模拟输出带 `simulation: true`，不证明真机运动。

### 7.3 真机小角度验证

先确认已有现场 profile，例如 `configs/aurora.site.yaml`。若没有，可从
`configs/robot_aurora.yaml` 复制一份再逐项核对；模板的 `verified: false` 会阻止
执行。需填写现场核对的方向、零位、限位、控制权和验证记录，不能只批量改成 true。
具体字段见 [Aurora 控制说明](aurora_control.md)。

确认机器人已处于 PdStand 且稳定后运行：

```bash
python tools/aurora_arm_debug.py --profile configs/aurora.site.yaml --execute
```

先选 `8` 查看当前角度，再选择当前姿态允许的 3～5° 小动作测试，例如在零位附近
选 `1`（向前），角度输入 `3`。核对程序显示的目标后输入 `YES`。这里的 3° 仍是绝对目标；
若要从当前姿态增加 3°，启动时加 `--reference current`。

观察实际运动，并检查 `submitted`、`arrived`、`feedback_deg`、`error_deg`。
`submitted: true` 仅表示调用返回；`arrived: true` 表示观察到下发后的新鲜关节
反馈到位。SDK 没有送达回执，所以 `delivery_confirmed` 保持 false；这个字段本身
不表示运动失败。
确认方向和反馈后，再逐步测试 30° 等目标。

此工具只通过已有限速轨迹调用 `get_group_state + set_group_cmd`，不自动切 FSM，
也不连接 IMU/袖套。退出停止发布并关闭客户端，不代表硬件急停。
详细指令见 [数字菜单手臂 SDK 调试](aurora_arm_debug.md)。

## 8. 接入 IMU：驱动安装待补充

交接人说明：**连接 IMU 所需驱动应安装在机器人系统上**，而不是只装在操作电脑。
目前未收到驱动包或下载链接，因此不能给出经过确认的安装命令。

当前配置的 IMU 设备节点以 `/dev/ttyCH9344USB*` 命名，提示应核对 CH9344 系列
USB 转串口设备；具体芯片、驱动版本和安装步骤必须以实物、`lsusb` 和驱动说明为准。

安装前在机器人上记录：

```bash
uname -m
uname -r
lsusb
```

拿到驱动包后，按对应系统架构、内核版本的说明安装。安装路径、是否要加载模块、
是否要重启均待驱动说明确认；不要在机器人运动中进行驱动安装或重启。

接入 IMU 后检查枚举与权限：

```bash
ls -l /dev/ttyCH9344USB* /dev/ttyUSB* 2>/dev/null
id
python tools/list_serial_ports.py
```

若设备节点不存在，先排查线缆、USB 枚举和驱动；节点存在但无法打开，再核对
串口权限和是否被其他采集程序占用。

确认每枚实物 IMU 对应哪个串口：

```bash
python tools/identify_imu_ports.py
```

该脚本同时解析所有 `/dev/ttyCH9344USB*`，逐个晃动 IMU，观察哪一行数据变化。
无有效数据的端口显示 `NONE`，默认超过 1 秒没有新有效帧也会显示 `NONE`。
先退出其他采集程序；识别完成后按 `Ctrl+C` 释放串口，再启动模型控制。
刷新频率、运行时长及输出字段见 [IMU 串口识别](identify_imu_ports.md)。

编辑 [configs/sensors.yaml](../configs/sensors.yaml)，把端口改成**机器人上实际出现的值**。
仓库现有值只是之前机器的配置，不保证 USB 插到机器人后编号相同：

| 设备 | 当前配置端口 | 波特率 | 作用 |
| --- | --- | --- | --- |
| 袖套 | `/dev/ttyUSB2` | 115200 | CH2 肘部输入；可选 CH3/4/5 肩部模型输入 |
| IMU1 | `/dev/ttyCH9344USB3` | 460800 | 右上臂，肩部估计 |
| IMU2 | `/dev/ttyCH9344USB0` | 460800 | 胸部，肩部参考 |
| IMU3 | `/dev/ttyCH9344USB5` | 460800 | 大臂旋转输入 |
| IMU4 | `/dev/ttyCH9344USB4` | 460800 | 大臂旋转参考 |

先单独读取一枚 IMU，不连接机器人控制客户端：

```bash
python tools/read_imu.py --imu imu1 --config configs/sensors.yaml
```

四枚 IMU 都配置好且 `upper_arm_rotation.enabled: true` 时，再做四 IMU 标定/显示：

```bash
python tools/test_imu_motion.py --config configs/sensors.yaml --duration 60 --imu-debug
```

该工具不读取袖套、不连接机器人。只有两枚肩部 IMU 时先逐个验证读取；接入模型
控制前关闭 `upper_arm_rotation.enabled`，不要运行要求四枚 IMU 的上述工具。

## 9. IMU、袖套与机器人联调

SDK 动作测试通过后，接好袖套并核对 `configs/phase3.yaml` 的 CH2 标定。
先用真实传感器驱动模拟 Aurora：

```bash
python tools/run_model_control.py \
  --robot aurora-fake --side right --source-side right \
  --shoulder-predictor dual_imu --sleeve real --imus real \
  --duration 30 --imu-debug
```

`dual_imu` 使用 IMU1/2 控制肩部，CH2 控制肘部；启用旋转时另用 IMU3/4。
按提示完成自然下垂、正前方抬起 45～60°、旋转零位的标定。
这个路径不需要模型权重。三柔性传感器肩部模式 `flexarm_estimator` 另需完整权重，
目前提供的源项目 `models/` 缺 `.joblib` 文件，不能直接假定该模式已准备好。

现场参数和传感器方向核对完成后，才运行真机跟随：

```bash
python tools/run_model_control.py \
  --robot aurora --aurora-profile configs/aurora.site.yaml \
  --side right --source-side right \
  --shoulder-predictor dual_imu --sleeve real --imus real \
  --duration 10 --execute
```

当前入口只支持右臂传感器映射到右臂。默认不切 FSM；传感器失效或预测异常会
阻止继续下发。`--robot aurora` 不加 `--execute` 时仅做配置预览，不读取传感器。

注意两项现有配置差异：CH2 人体角度上限是 135°，Aurora 模板肘部上限约 130°，
需要核对可用范围；`elbow_flexion_limit_deg` 的旋转归零规则目前仅由 DyMotor
mapper 使用，Aurora mapper 不应用该规则。

## 10. 交接时需要补全的记录

| 项目 | 当前情况 / 待记录 |
| --- | --- |
| 实际 WiFi SSID、设备编号 | 待现场确认 0197 / 0196 |
| SSH 用户名、主机名 | 登录后记录 `whoami` 和 `hostname` |
| WiFi 管理登录信息 | 未提供，单独交接 |
| 仓库绝对路径、remote 和分支 | 目录疑似 `arm_sleeve_contro`；目标 Kaiwei-Lin/arm_sleeve_control master |
| viewturbo 监听协议 | 端口 15732；HTTP/mixed 或 SOCKS5 待确认 |
| Python 环境名称和路径 | 待现场确认 |
| Aurora 服务启动方式 | 待现场确认 |
| IMU 驱动包、来源、版本、安装步骤 | 待交接人补充 |
| profile、串口映射及 CH2 标定 | 需在机器人现场核对 |
| SDK 实机结果 | 记录版本、只读反馈、实际动作、到位误差及日期 |

本次文档整理没有连接机器人、安装驱动、切换 FSM 或执行真实动作。
