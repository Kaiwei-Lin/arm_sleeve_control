#!/usr/bin/env python3
"""Print shoulder raise and upper-arm rotation from four IMUs without a robot."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import (
    DEFAULT_PHASE3_CONFIG_PATH,
    DEFAULT_SENSOR_CONFIG_PATH,
    load_phase3_config,
    load_sensor_config,
)
from sleeve_arm.estimation import UpperArmRotationEstimator
from sleeve_arm.predictor import DualImuShoulderPredictor
from sleeve_arm.sources import FakeImuSource, ImuSource, create_imu_source
from tools.run_model_control import (
    _add_latest_pair,
    _collect_imu_pairs,
    _pair_synchronizer,
    _require_fresh_pair,
    prepare_dual_imu_estimator,
)


def _pair_key(pair) -> tuple[int, int]:
    return int(pair[0].host_timestamp_ns), int(pair[1].host_timestamp_ns)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read four IMUs and print shoulder direction/magnitude plus upper-arm rotation; no robot or Sleeve is opened."
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--calibration-seconds", type=float)
    parser.add_argument("--imu-debug", action="store_true")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.print_hz <= 0.0:
        parser.error("--print-hz must be positive")
    if args.calibration_seconds is not None and args.calibration_seconds <= 0.0:
        parser.error("--calibration-seconds must be positive")

    sources: dict[str, ImuSource] = {}
    try:
        sensor_config = load_sensor_config(args.config)
        phase3 = load_phase3_config(args.phase3_config)
        shoulder_config = sensor_config.shoulder_imu
        rotation_config = sensor_config.upper_arm_rotation
        if not rotation_config.enabled:
            raise RuntimeError("set upper_arm_rotation.enabled=true before running this tool")

        shoulder_names = (shoulder_config.arm_imu, shoulder_config.chest_imu)
        rotation_names = (rotation_config.upper_imu, rotation_config.reference_imu)
        delayed_names = {shoulder_names[1], rotation_names[1]}
        for name in shoulder_names + rotation_names:
            endpoint = getattr(sensor_config, name)
            if not endpoint.enabled:
                raise ValueError(f"four-IMU test requires sensors.{name}.enabled=true")
            source = (
                FakeImuSource(timestamp_offset_s=0.005 if name in delayed_names else 0.0)
                if args.fake
                else create_imu_source(name, endpoint)
            )
            if source is None:
                raise RuntimeError(f"could not create configured {name} source")
            sources[name] = source
            source.start()

        timeout_s = phase3.sensor_timeout_ms / 1000.0
        shoulder_estimator, _ = prepare_dual_imu_estimator(
            sources,
            sensor_config,
            timeout_s,
            calibration_seconds=args.calibration_seconds,
            input_fn=input,
        )
        shoulder_predictor = DualImuShoulderPredictor(shoulder_estimator, timeout_s)

        shoulder_max_sync_ms = (
            sensor_config.synchronization.max_time_delta_ms
            if shoulder_config.max_sync_ms is None
            else shoulder_config.max_sync_ms
        )
        rotation_max_sync_ms = (
            sensor_config.synchronization.max_time_delta_ms
            if rotation_config.max_sync_ms is None
            else rotation_config.max_sync_ms
        )
        rotation_estimator = UpperArmRotationEstimator(
            rotation_config.ema_alpha,
            rotation_max_sync_ms,
            rotation_config.twist_axis,
        )
        rotation_pairs = _collect_imu_pairs(
            sources,
            sensor_config,
            rotation_names,
            rotation_config,
            timeout_s,
            calibration_seconds=args.calibration_seconds,
            prompt="请保持大臂旋转零位并静止，按回车开始 IMU3/IMU4 旋转零位标定：",
            status="大臂旋转 IMU3/IMU4",
            input_fn=input,
        )
        rotation_estimator.calibrate(rotation_pairs)
        print(f"大臂旋转零位标定完成：samples={len(rotation_pairs)}")
        print(
            f"开始四 IMU 测试：shoulder={shoulder_names}, rotation={rotation_names}, "
            f"twist_axis={rotation_config.twist_axis}. Ctrl+C 退出。"
        )

        shoulder_sync = _pair_synchronizer(sensor_config)
        rotation_sync = _pair_synchronizer(sensor_config)
        started = next_print = time.monotonic()
        deadline = None if args.duration is None else started + args.duration
        last_shoulder_valid = last_rotation_valid = started
        last_shoulder_key = last_rotation_key = None
        last_shoulder_pair = last_rotation_pair = None
        shoulder_result = rotation_result = None
        shoulder_error = rotation_error = None
        hard_timeout_s = phase3.hard_timeout_ms / 1000.0

        while deadline is None or time.monotonic() < deadline:
            _add_latest_pair(shoulder_sync, sources, shoulder_names)
            _add_latest_pair(rotation_sync, sources, rotation_names)
            now = time.monotonic()

            try:
                shoulder_pair = _require_fresh_pair(
                    shoulder_sync.latest_imu_pair(shoulder_max_sync_ms),
                    timeout_s,
                    "shoulder",
                )
                key = _pair_key(shoulder_pair)
                if key != last_shoulder_key:
                    shoulder_predictor.predict_imu_pair(*shoulder_pair)
                    shoulder_result = shoulder_predictor.last_result
                    last_shoulder_pair = shoulder_pair
                    last_shoulder_key = key
                    last_shoulder_valid = now
                    shoulder_error = None
            except ValueError as exc:
                shoulder_error = exc

            try:
                rotation_pair = _require_fresh_pair(
                    rotation_sync.latest_imu_pair(rotation_max_sync_ms),
                    timeout_s,
                    "upper-arm rotation",
                )
                key = _pair_key(rotation_pair)
                if key != last_rotation_key:
                    rotation_result = rotation_estimator.update(*rotation_pair)
                    last_rotation_pair = rotation_pair
                    last_rotation_key = key
                    last_rotation_valid = now
                    rotation_error = None
            except ValueError as exc:
                rotation_error = exc

            if now - last_shoulder_valid > hard_timeout_s:
                raise RuntimeError(f"shoulder IMU pair hard timeout: {shoulder_error}")
            if now - last_rotation_valid > hard_timeout_s:
                raise RuntimeError(
                    f"upper-arm rotation IMU pair hard timeout: {rotation_error}"
                )

            if shoulder_result is not None and rotation_result is not None and now >= next_print:
                assert last_shoulder_pair is not None
                debug = ""
                if args.imu_debug:
                    arm, chest = shoulder_predictor.last_frames or (None, None)
                    if arm is not None and chest is not None and last_rotation_pair is not None:
                        upper, reference = last_rotation_pair
                        debug = (
                            f" shoulder_q=({arm.quat_w}, {arm.quat_x}, {arm.quat_y}, {arm.quat_z})"
                            f" chest_q=({chest.quat_w}, {chest.quat_x}, {chest.quat_y}, {chest.quat_z})"
                            f" upper_q=({upper.quat_w}, {upper.quat_x}, {upper.quat_y}, {upper.quat_z})"
                            f" reference_q=({reference.quat_w}, {reference.quat_x}, {reference.quat_y}, {reference.quat_z})"
                        )
                print(
                    f"shoulder_direction={shoulder_result.direction} "
                    f"shoulder_magnitude_deg={shoulder_result.magnitude_deg:.2f} "
                    f"shoulder_confidence={shoulder_result.confidence:.3f} "
                    f"rotation_signed_deg={rotation_result.difference_deg:+.2f} "
                    f"rotation_world_deg={rotation_result.world_filtered_deg:+.2f} "
                    f"rotation_relative_deg={rotation_result.relative_filtered_deg:+.2f} "
                    f"shoulder_sync_ms={abs(last_shoulder_pair[0].timestamp - last_shoulder_pair[1].timestamp) * 1000.0:.2f} "
                    f"rotation_sync_ms={rotation_result.sync_gap_ms:.2f}{debug}"
                )
                next_print = now + 1.0 / args.print_hz
            time.sleep(0.001)
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
