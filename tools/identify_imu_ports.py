#!/usr/bin/env python3
"""Read all matching serial ports to identify physical IMU770 sensors by motion."""

from __future__ import annotations

import argparse
import glob
import math
import sys
import threading
import time
from pathlib import Path
from typing import Callable, TextIO

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tools.demo_upper_arm_twist_imu770 import Imu770Sample, Imu770SerialReader


def discover_ports(prefix: str) -> list[str]:
    """Scan once and keep USB2 before USB10; no sensor configuration is needed."""
    def sort_key(port: str) -> tuple[int, int | str]:
        suffix = port[len(prefix):]
        return (0, int(suffix)) if suffix.isdigit() else (1, port)

    return sorted(
        (port for port in glob.glob(glob.escape(prefix) + "*") if not Path(port).is_dir()),
        key=sort_key,
    )


class PortMonitor:
    """Each port has its own reader, parser and latest valid sample."""

    def __init__(self, port: str, baudrate: int, serial_factory: Callable | None = None) -> None:
        self.port = port
        self._sample: Imu770Sample | None = None
        self._lock = threading.Lock()
        self._open_error: str | None = None
        self.reader = Imu770SerialReader(
            port=port,
            baudrate=baudrate,
            timeout_s=0.1,
            on_sample=self._receive,
            serial_factory=serial_factory,
            require_quaternion=False,
        )

    def _receive(self, sample: Imu770Sample) -> None:
        with self._lock:
            self._sample = sample

    def start(self) -> None:
        try:
            self.reader.start()
        except Exception as exc:
            self._open_error = f"打开失败: {exc}"

    def close(self) -> None:
        self.reader.close()

    def format_line(self, now_ns: int, stale_seconds: float) -> str:
        error = self._open_error
        if error is None and self.reader.error is not None:
            error = f"读取失败: {self.reader.error}"
        if error is not None:
            return f"{self.port} | NONE ({' '.join(error.splitlines())})"
        with self._lock:
            sample = self._sample
        if sample is None:
            return f"{self.port} | NONE (等待有效 IMU770 帧)"
        age = max(0.0, (now_ns - sample.host_timestamp_ns) / 1e9)
        if age > stale_seconds:
            return f"{self.port} | NONE (数据过期，上次有效帧 {age:.1f}s 前)"

        fields = [f"{self.port} | TID={sample.tid} age={age * 1000:.0f}ms"]
        for name, vector, precision in (
            ("quat(wxyz)", sample.quaternion_wxyz, 4),
            ("gyro(deg/s)", sample.gyro_dps, 3),
            ("accel(m/s2)", sample.accel_mps2, 3),
            ("euler(deg)", sample.euler_deg, 3),
        ):
            if vector is not None:
                values = ",".join(f"{value:+.{precision}f}" for value in vector)
                fields.append(f"{name}=({values})")
        return " ".join(fields)


def positive_float(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("必须是大于 0 的有限数值")
    return number


def positive_int(value: str) -> int:
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("必须是正整数")
    return number


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="同时读取所有匹配串口，晃动 IMU 来确认对应端口。")
    parser.add_argument("--prefix", default="/dev/ttyCH9344USB", help="串口路径前缀（不带 *）")
    parser.add_argument("--baudrate", type=positive_int, default=460800, help="默认 460800 / 8N1")
    parser.add_argument("--print-hz", type=positive_float, default=5.0, help="终端刷新频率，默认 5 Hz")
    parser.add_argument("--stale-seconds", type=positive_float, default=1.0, help="超过此时间无有效帧则显示 NONE")
    parser.add_argument("--duration", type=positive_float, help="运行秒数；默认一直运行，Ctrl+C 退出")
    parser.add_argument("--no-clear", action="store_true", help="保留每次输出，不刷新覆盖终端")
    args = parser.parse_args(argv)
    if not args.prefix:
        parser.error("--prefix 不能为空")
    return args


def run(
    args: argparse.Namespace,
    *,
    serial_factory: Callable | None = None,
    output: TextIO | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    output = sys.stdout if output is None else output
    ports = discover_ports(args.prefix)
    if not ports:
        print(f"未找到 {args.prefix}*；请检查 USB 连接、驱动和设备节点。", file=output)
        return 1

    monitors = [PortMonitor(port, args.baudrate, serial_factory) for port in ports]
    clear = output.isatty() and not args.no_clear
    result = 0
    try:
        for monitor in monitors:
            monitor.start()
        started = clock()
        deadline = None if args.duration is None else started + args.duration
        while True:
            now = clock()
            lines = [
                f"IMU770 端口识别 | {len(ports)} 个端口 | {args.baudrate} / 8N1 | {now - started:.1f}s",
                "逐个晃动 IMU，观察对应行的 quat / gyro / accel / euler。Ctrl+C 退出。",
                *(monitor.format_line(int(now * 1e9), args.stale_seconds) for monitor in monitors),
            ]
            print(("\033[H\033[2J" if clear else "") + "\n".join(lines), file=output, flush=True)
            after_print = clock()
            if deadline is not None and after_print >= deadline:
                break
            delay = 1.0 / args.print_hz
            if deadline is not None:
                delay = min(delay, deadline - after_print)
            sleep(delay)
    except KeyboardInterrupt:
        print("\n已停止端口识别。", file=output)
    finally:
        for monitor in monitors:
            try:
                monitor.close()
            except Exception as exc:
                result = 1
                print(f"{monitor.port} 关闭失败: {exc}", file=output)
    return result


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
