#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_PHASE3_CONFIG_PATH, DEFAULT_SENSOR_CONFIG_PATH, load_phase3_config, load_sensor_config
from sleeve_arm.sources import FakeSleeveSource, create_sleeve_source


def collect(source, channel_index: int, duration_s: float) -> list[float]:
    values: list[float] = []
    last_timestamp = None
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        frame = source.latest()
        if frame is not None and frame.timestamp != last_timestamp:
            if channel_index >= len(frame.channels):
                raise ValueError(f"configured channel is missing from {len(frame.channels)}-channel frame")
            values.append(frame.channels[channel_index])
            last_timestamp = frame.timestamp
        time.sleep(0.001)
    if not values:
        raise RuntimeError("no Sleeve samples received")
    return values


def summary(values: list[float]) -> dict[str, float]:
    return {
        "median": statistics.median(values),
        "min": min(values),
        "max": max(values),
        "std": statistics.pstdev(values),
    }


def monitor(source, duration_s=None, print_hz=5., *, clock=None, print_fn=print):
    clock = clock or time
    deadline = None if duration_s is None else clock.monotonic() + duration_s
    last_timestamp = None
    while deadline is None or clock.monotonic() < deadline:
        frame = source.latest()
        if frame is None:
            print_fn("[Sleeve] WAITING: 等待串口数据", flush=True)
        else:
            state = "NEW" if frame.timestamp != last_timestamp else "HELD"
            channels = "  ".join(f"CH{i}={value:.6g}" for i, value in enumerate(frame.channels, 1))
            print_fn(f"[Sleeve {state}] age={clock.monotonic() - frame.timestamp:.3f}s  {channels}", flush=True)
            last_timestamp = frame.timestamp
        remaining = math.inf if deadline is None else deadline - clock.monotonic()
        if remaining > 0:
            clock.sleep(min(1. / print_hz, remaining))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Calibrate the Sleeve elbow channel or monitor all channels; no robot connection.")
    parser.add_argument("--monitor", action="store_true", help="实时显示所有 CH 通道，不进行姿态标定；Ctrl+C 退出")
    parser.add_argument("--duration", type=float, help="标定时每姿态采集秒数（默认 3）；监看总秒数（默认持续运行）")
    parser.add_argument("--print-hz", type=float, default=5., help="监看刷新频率，默认 5 Hz")
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be finite and positive")
    if not math.isfinite(args.print_hz) or args.print_hz <= 0:
        parser.error("--print-hz must be finite and positive")

    source = None
    try:
        phase3 = None if args.monitor else load_phase3_config(args.phase3_config)
        source = FakeSleeveSource() if args.fake else create_sleeve_source(load_sensor_config(args.sensor_config))
        source.start()
        if args.monitor:
            print("Sleeve 全通道只读监看：不连接机器人、不修改标定；NEW=新帧，HELD=沿用旧帧。Ctrl+C 退出。")
            monitor(source, args.duration, args.print_hz)
            return 0
        duration = 3. if args.duration is None else args.duration
        channel = phase3.elbow.sleeve_channel
        index = channel - 1
        if not args.fake:
            input(f"Keep the elbow extended, then press Enter to sample CH{channel}...")
        extension = summary(collect(source, index, duration))
        if not args.fake:
            input("Flex the elbow to the planned calibration posture, then press Enter...")
        flexion = summary(collect(source, index, duration))
        print(f"extension: {extension}")
        print(f"flexion:   {flexion}")
        low, high = sorted((extension["median"], flexion["median"]))
        print("\nSuggested configuration (verify posture and direction before use):")
        print(f"input_min: {low:.9g}")
        print(f"input_max: {high:.9g}")
        print(f"invert_input: {'true' if flexion['median'] < extension['median'] else 'false'}")
        print("angle_range:")
        print("  min_deg: null  # measured human angle at the extension posture")
        print("  max_deg: null  # measured human angle at the flexion posture")
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nSleeve monitoring/calibration stopped.")
        return 130
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
