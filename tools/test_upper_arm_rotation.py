#!/usr/bin/env python3
"""Calibrate and inspect dual-IMU upper-arm rotation without opening a robot."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_sensor_config
from sleeve_arm.estimation import UpperArmRotationEstimator
from sleeve_arm.sources import FakeImuSource, ImuSource, create_imu_source
from sleeve_arm.sync import SensorSynchronizer


def main() -> int:
    parser = argparse.ArgumentParser(description="Read existing dual IMU sources and print upper-arm twist difference.")
    parser.add_argument("--config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--print-hz", type=float, default=10.0)
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.print_hz <= 0.0:
        parser.error("--print-hz must be positive")

    sources: dict[str, ImuSource] = {}
    try:
        sensor_config = load_sensor_config(args.config)
        config = sensor_config.upper_arm_rotation
        if not config.enabled:
            raise RuntimeError("set upper_arm_rotation.enabled=true before running this tool")
        for name in (config.upper_imu, config.reference_imu):
            source = (
                FakeImuSource(timestamp_offset_s=(0.0 if name == config.upper_imu else 0.005))
                if args.fake
                else create_imu_source(name, getattr(sensor_config, name))
            )
            if source is None:
                raise RuntimeError(f"could not create configured {name} source")
            sources[name] = source
            source.start()

        max_sync_ms = (
            sensor_config.synchronization.max_time_delta_ms
            if config.max_sync_ms is None
            else config.max_sync_ms
        )
        estimator = UpperArmRotationEstimator(config.ema_alpha, max_sync_ms, config.twist_axis)

        def synchronizer() -> SensorSynchronizer:
            return SensorSynchronizer(
                sensor_config.synchronization.max_time_delta_ms,
                sensor_config.synchronization.buffer_duration_ms,
                imu1_enabled=True,
                imu2_enabled=True,
            )

        sync = synchronizer()
        last_pair: tuple[float, float] | None = None

        def next_pair():
            nonlocal last_pair
            for source, add in zip(sources.values(), (sync.add_imu1, sync.add_imu2)):
                frame = source.latest()
                if frame is None:
                    return None
                add(frame)
            pair = sync.latest_imu_pair(max_sync_ms)
            if pair is None:
                return None
            upper, reference = pair
            identity = (upper.timestamp, reference.timestamp)
            if identity == last_pair:
                return None
            estimator.validate_pair(upper, reference)
            last_pair = identity
            return upper, reference

        startup_deadline = time.monotonic() + config.startup_timeout_s
        while time.monotonic() < startup_deadline:
            try:
                ready = next_pair()
            except ValueError:
                ready = None
            if ready is not None:
                break
            time.sleep(0.001)
        else:
            raise RuntimeError("no fresh valid dual-IMU quaternion pair before startup timeout")

        input("请保持大臂旋转零位并静止，按回车开始 IMU 零位标定：")
        print(f"正在标定 {config.calibration_seconds:g} 秒；不要求手臂水平...")
        sync = synchronizer()
        last_pair = None
        minimum_timestamp = time.monotonic()
        pairs = []
        deadline = minimum_timestamp + config.calibration_seconds
        while time.monotonic() < deadline:
            try:
                pair = next_pair()
            except ValueError:
                pair = None
            if pair is not None and min(pair[0].timestamp, pair[1].timestamp) >= minimum_timestamp:
                pairs.append(pair)
            time.sleep(0.001)
        estimator.calibrate(pairs)
        print(f"Calibration complete: {len(pairs)} synchronized pairs. Ctrl+C to stop.")

        started = time.monotonic()
        stop_at = None if args.duration is None else started + args.duration
        next_print = started
        valid = rejected = 0
        while stop_at is None or time.monotonic() < stop_at:
            try:
                pair = next_pair()
                if pair is None:
                    time.sleep(0.001)
                    continue
                result = estimator.update(*pair)
            except ValueError:
                rejected += 1
                continue
            valid += 1
            now = time.monotonic()
            if now >= next_print:
                upper_fps = sources[config.upper_imu].stats.estimated_fps
                reference_fps = sources[config.reference_imu].stats.estimated_fps
                print(
                    f"world={result.world_filtered_deg:+8.2f} deg  "
                    f"relative={result.relative_filtered_deg:+8.2f} deg  "
                    f"difference={result.difference_deg:+8.2f} deg  "
                    f"sync={result.sync_gap_ms:5.2f} ms  "
                    f"fps=({upper_fps:5.1f},{reference_fps:5.1f})  "
                    f"valid={valid} rejected={rejected} "
                    f"sync_rejected={estimator.sync_rejected_count}"
                )
                next_print = now + 1.0 / args.print_hz
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        for source in reversed(tuple(sources.values())):
            try:
                source.close()
            except Exception as exc:
                print(f"ERROR: source close: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
