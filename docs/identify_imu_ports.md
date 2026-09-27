# 通过晃动 IMU 确认串口

在连接 IMU 的那台机器上，进入仓库并激活项目 Python 环境，运行：

```bash
python tools/identify_imu_ports.py
```

脚本自动扫描所有 `/dev/ttyCH9344USB*`，按编号排列，用独立线程同时读取。
默认串口参数为 **460800 / 8N1**，每秒刷新终端 5 次。它复用已有 IMU770 协议
解析器，不读取 `sensors.yaml`，不需要预先知道 IMU1～IMU4 的端口。
仅读取串口，不向传感器发送命令，也不导入或连接机器人 SDK。

示意输出（实际数值随 IMU 输出字段变化）：

```text
/dev/ttyCH9344USB0 | TID=7 age=8ms quat(wxyz)=(+0.9980,+0.0632,+0.0000,+0.0000)
/dev/ttyCH9344USB1 | NONE (等待有效 IMU770 帧)
/dev/ttyCH9344USB2 | TID=8 age=5ms gyro(deg/s)=(+25.000,-0.200,+0.100) accel(m/s2)=(+0.100,+0.200,+9.800)
/dev/ttyCH9344USB3 | NONE (打开失败: ...)
```

1. 先停止其他正在读取这些串口的采集程序。
2. 一次只晃动一枚 IMU，观察哪一行四元数、角速度、加速度或欧拉角随之变化。
3. 记录实物标签与该行串口，例如“胸部 IMU → `/dev/ttyCH9344USB0`”。
4. 按 `Ctrl+C` 退出并释放全部串口，再将确认的映射填入
   [configs/sensors.yaml](../configs/sensors.yaml) 中对应 IMU 的 `port`。

有四元数时显示 `quat(wxyz)`；有其他字段时显示 `gyro(deg/s)`、`accel(m/s2)`、
`euler(deg)`，三轴顺序均为 x/y/z。只输出四元数或只输出其他运动字段的 IMU770
也可以识别。`TID` 是设备帧序号，`age` 是最近有效帧距离现在的时间。

`NONE` 表示该串口当前没有可用的 IMU770 数据：尚未收到有效帧、校验/解析失败、
距离上次有效帧超过默认 1 秒，或串口打开/读取失败。错误原因显示在括号里。
显示的有效数据来自最近一帧，不把旧数据永久当成在线数据。
其他协议（例如袖套 ASCII 或 WT901）不会按 IMU770 解码。

常用选项：

```bash
# 运行 30 秒后退出
python tools/identify_imu_ports.py --duration 30

# 更快刷新，允许 2 秒的数据间隔
python tools/identify_imu_ports.py --print-hz 10 --stale-seconds 2

# 换用其他转串口设备前缀或波特率（只按 IMU770 协议解析）
python tools/identify_imu_ports.py --prefix /dev/ttyUSB --baudrate 460800

# 保留每次输出，便于保存日志
python tools/identify_imu_ports.py --no-clear --duration 10 > imu_ports.log
```

扫描在启动时执行一次；插拔后重新运行。找不到任何端口时检查 USB 连接、驱动及
设备节点。遇到 `Permission denied` 时检查串口权限；若现场需要 sudo，使用当前
环境解释器的绝对路径，例如 `sudo /path/to/env/bin/python tools/identify_imu_ports.py`。
不要同时启动控制程序和这个脚本读取同一串口。
