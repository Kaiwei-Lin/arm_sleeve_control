#!/usr/bin/env python3
"""Read matching serial ports as Bend5 streams to locate a glove by finger motion."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Callable, TextIO

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.sources.glove import Bend5GloveSource
from tools.identify_imu_ports import discover_ports, positive_float, positive_int


class PortMonitor:
    def __init__(self, port: str, baudrate: int, serial_factory: Callable | None = None) -> None:
        self.port = port
        self._open_error: str | None = None
        self.reader = Bend5GloveSource(
            port, baudrate, timeout=0.1, zero_on_start=False,
            adaptive_baseline=False, filter_alpha=1.0, serial_factory=serial_factory,
        )

    def start(self) -> None:
        try:
            self.reader.start()
        except Exception as exc:
            self._open_error = f"打开失败: {exc}"

    def close(self) -> None:
        self.reader.close()

    def format_line(self, now: float, stale_seconds: float) -> str:
        error = self._open_error
        reading = None
        if error is None:
            try:
                reading = self.reader.latest()
            except Exception as exc:
                error = f"读取失败: {exc}"
        if error is not None:
            return f"{self.port} | NONE ({' '.join(error.splitlines())})"
        if reading is None:
            return f"{self.port} | NONE (等待有效 Bend5 帧)"
        age = max(0.0, now - reading.timestamp)
        if age > stale_seconds:
            return f"{self.port} | NONE (数据过期，上次有效帧 {age:.1f}s 前)"
        fingers = ",".join(f"{value:g}" for value in reading.raw[:5])
        tail = ",".join(f"{value:g}" for value in reading.raw[5:])
        return f"{self.port} | age={age * 1000:.0f}ms | bend5=({fingers}) | tail=({tail})"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="按 Bend5 格式读取所有匹配串口，弯曲手指确认手套端口。")
    parser.add_argument("--prefix", default="/dev/ttyUSB", help="串口路径前缀（不带 *）")
    parser.add_argument("--baudrate", type=positive_int, default=115200, help="默认 115200 / 8N1")
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
                f"Bend5 手套端口识别 | {len(ports)} 个端口 | {args.baudrate} / 8N1 | {now - started:.1f}s",
                "逐个弯曲手指，观察 bend5（小指,无名指,中指,食指,拇指）；tail 为其余六路。Ctrl+C 退出。",
                *(monitor.format_line(now, args.stale_seconds) for monitor in monitors),
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
