#!/usr/bin/env python3
from __future__ import annotations

import argparse
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate Sleeve CH2 without connecting the robot.")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")

    source = None
    try:
        phase3 = load_phase3_config(args.phase3_config)
        source = FakeSleeveSource() if args.fake else create_sleeve_source(load_sensor_config(args.sensor_config))
        source.start()
        channel = phase3.elbow.sleeve_channel
        index = channel - 1
        if not args.fake:
            input(f"Keep the elbow extended, then press Enter to sample CH{channel}...")
        extension = summary(collect(source, index, args.duration))
        if not args.fake:
            input("Flex the elbow to the planned calibration posture, then press Enter...")
        flexion = summary(collect(source, index, args.duration))
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
        print("\nCalibration cancelled.")
        return 130
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
