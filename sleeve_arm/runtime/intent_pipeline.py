"""Calibrate and combine existing estimators into MotionIntent, without robot knowledge."""
from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path
from typing import Any

from sleeve_arm.config import FlexModelConfig, SensorConfig
from sleeve_arm.domain import ImuFrame, MotionIntent, SensorSample
from sleeve_arm.estimation import UpperArmRotationEstimator
from sleeve_arm.predictor import (
    DualImuShoulderPredictor, FlexModelPredictor, RuleBasedPredictor,
    calibrate_dual_imu_forward, calibrate_dual_imu_estimator, imu_quaternion,
)
from sleeve_arm.predictor.calibration import collect_calibration_samples
from sleeve_arm.sources import ImuSource
from sleeve_arm.runtime.sensors import _latest_imu_pair, _pair_synchronizer, _require_fresh_pair


def prepare_flexarm_predictor(
    source: Any,
    config: FlexModelConfig,
    *,
    predictor: FlexModelPredictor | Any | None = None,
    reuse_calibration: bool = False,
    calibration_seconds: float | None = None,
    calibration_output: Path | None = None,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[[str], None] = print,
    monotonic: Callable[[], float] = time.monotonic,
) -> FlexModelPredictor:
    """Prepare the shared three-flex-sensor shoulder predictor."""
    prepared = FlexModelPredictor(config) if predictor is None else predictor
    output = config.calibration_file if calibration_output is None else calibration_output
    if reuse_calibration:
        prepared.reuse_calibration(output)
        print_fn(f"Reused FlexArm calibration: {output}")
        return prepared

    duration = config.calibration_seconds if calibration_seconds is None else calibration_seconds
    input_fn(
        f"Let the arm hang naturally and keep still. Press Enter to collect "
        f"{duration:g} seconds of calibration data: "
    )
    rows = collect_calibration_samples(
        source,
        config.sleeve_channels,
        duration,
        monotonic=monotonic,
    )
    calibration = prepared.calibrate(rows, output)
    print_fn(
        f"Calibration complete: samples={calibration.sample_count}, "
        f"baseline={list(calibration.baseline)}, scale={list(calibration.scale)}"
    )
    print_fn(f"Calibration saved to: {output}")
    return prepared


def _collect_imu_pairs(
    imu_sources: dict[str, ImuSource],
    sensor_config: SensorConfig,
    names: tuple[str, str],
    pair_config: Any,
    sensor_timeout_s: float,
    *,
    calibration_seconds: float | None = None,
    prompt: str,
    status: str,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[[str], None] = print,
) -> list[tuple[ImuFrame, ImuFrame]]:
    duration = pair_config.calibration_seconds if calibration_seconds is None else calibration_seconds
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("dual-IMU calibration duration must be positive and finite")
    max_sync_ms = (
        sensor_config.synchronization.max_time_delta_ms
        if pair_config.max_sync_ms is None
        else pair_config.max_sync_ms
    )

    sync = _pair_synchronizer(sensor_config)
    startup_deadline = time.monotonic() + pair_config.startup_timeout_s
    while time.monotonic() < startup_deadline:
        pair = _latest_imu_pair(imu_sources, sync, names, max_sync_ms)
        if pair is not None:
            try:
                if any(abs(time.monotonic() - frame.timestamp) > sensor_timeout_s for frame in pair):
                    raise ValueError("dual-IMU pair is stale")
                imu_quaternion(pair[0])
                imu_quaternion(pair[1])
            except ValueError:
                pass
            else:
                break
        time.sleep(0.001)
    else:
        raise RuntimeError(f"no fresh valid {status} quaternion pair before startup timeout")

    input_fn(prompt)
    print_fn(f"正在采集 {duration:g} 秒 {status} 同步四元数...")
    sync = _pair_synchronizer(sensor_config)
    pairs: list[tuple[ImuFrame, ImuFrame]] = []
    last_pair_key: tuple[int, int] | None = None
    minimum_timestamp = time.monotonic()
    deadline = minimum_timestamp + duration
    while time.monotonic() < deadline:
        pair = _latest_imu_pair(imu_sources, sync, names, max_sync_ms)
        if pair is not None:
            pair_key = (int(pair[0].host_timestamp_ns), int(pair[1].host_timestamp_ns))
            try:
                if any(abs(time.monotonic() - frame.timestamp) > sensor_timeout_s for frame in pair):
                    raise ValueError("dual-IMU pair is stale")
                if min(frame.timestamp for frame in pair) < minimum_timestamp:
                    raise ValueError("dual-IMU pair predates calibration prompt")
                imu_quaternion(pair[0])
                imu_quaternion(pair[1])
            except ValueError:
                pass
            else:
                if pair_key != last_pair_key:
                    pairs.append(pair)
                    last_pair_key = pair_key
        time.sleep(0.001)
    return pairs


def prepare_dual_imu_estimator(
    imu_sources: dict[str, ImuSource],
    sensor_config: SensorConfig,
    sensor_timeout_s: float,
    *,
    estimator: Any | None = None,
    calibration_seconds: float | None = None,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[[str], None] = print,
    calibration_stage: Callable[[str], None] | None = None,
) -> tuple[Any, list[tuple[ImuFrame, ImuFrame]]]:
    config = sensor_config.shoulder_imu
    if estimator is None:
        from sleeve_arm.estimation.flexarm import DualImuArmEstimator

        estimator = DualImuArmEstimator(
            down_axis=(-1, 0, 0),
            forward_axis=(0, 0, 1),
            lateral_axis=(0, 1, 0),
            rest_threshold_deg=5,
            dominance_ratio=1.1,
        )
    if calibration_stage is not None:
        calibration_stage("rest")
    rest_pairs = _collect_imu_pairs(
        imu_sources,
        sensor_config,
        (config.arm_imu, config.chest_imu),
        config,
        sensor_timeout_s,
        calibration_seconds=calibration_seconds,
        prompt="请自然站立、右臂自然下垂并保持静止，按回车开始双 IMU 肩部零位标定：",
        status="肩部 IMU1/IMU2",
        input_fn=input_fn,
        print_fn=print_fn,
    )
    calibration = calibrate_dual_imu_estimator(estimator, rest_pairs)
    sample_count = int(getattr(calibration, "sample_count", len(rest_pairs)))
    print_fn(f"双 IMU 肩部零位标定完成：samples={sample_count}")

    if calibration_stage is not None:
        calibration_stage("forward")
    forward_pairs = _collect_imu_pairs(
        imu_sources,
        sensor_config,
        (config.arm_imu, config.chest_imu),
        config,
        sensor_timeout_s,
        calibration_seconds=calibration_seconds,
        prompt="请将右臂向身体正前方抬起并保持在 45–60°，按回车开始双 IMU 肩部前抬方向标定：",
        status="肩部 IMU1/IMU2 前抬方向",
        input_fn=input_fn,
        print_fn=print_fn,
    )
    calibrate_dual_imu_forward(estimator, forward_pairs)
    print_fn(f"双 IMU 肩部前抬方向标定完成：samples={len(forward_pairs)}")
    return estimator, rest_pairs


class RotationPredictionError(ValueError):
    """A rotation failure with the original partial intent and held rotation.

    The app owns whether this partial result may be used. The prediction layer
    has no knowledge of which robot is attached.
    """

    def __init__(self, cause, intent):
        super().__init__(str(cause))
        self.intent = intent


class IntentPipeline:
    def __init__(self, configs, *, reuse_calibration=False, calibration_seconds=None,
                 calibration_output=None, input_fn=None, print_fn=print):
        self.configs = configs
        self.backend = configs.shoulder_backend
        self.reuse_calibration = reuse_calibration
        self.calibration_seconds = calibration_seconds
        self.calibration_output = calibration_output
        self.input_fn = input_fn
        self.print = print_fn
        self.sensors = None
        self.shoulder = None
        self.elbow = None
        self.rotation = None
        self.rotation_result = None
        self.last_rotation_frames = None
        self.last_rotation_rad = None

    def prepare(self, sensors):
        self.sensors = sensors
        config = self.configs.sensors
        input_fn = self.input_fn or input
        timeout = self.configs.phase3.sensor_timeout_ms / 1000.0
        if self.backend == "dual_imu":
            estimator, _ = prepare_dual_imu_estimator(
                sensors.imu_sources, config, timeout,
                calibration_seconds=self.calibration_seconds, input_fn=input_fn, print_fn=self.print,
                calibration_stage=sensors.set_fake_calibration_pose,
            )
            self.shoulder = DualImuShoulderPredictor(estimator, timeout)
            sensors.set_fake_calibration_pose("rest")
        else:
            self.shoulder = prepare_flexarm_predictor(
                sensors.source, self.configs.phase4.flex_model,
                reuse_calibration=self.reuse_calibration, calibration_seconds=self.calibration_seconds,
                calibration_output=self.calibration_output, input_fn=input_fn, print_fn=self.print,
            )
        self.print(f"shoulder_predictor={self.backend}")
        self.elbow = RuleBasedPredictor(self.configs.elbow)
        rotation = config.upper_arm_rotation
        if rotation.enabled:
            self.rotation = UpperArmRotationEstimator(
                rotation.ema_alpha, sensors.max_sync_ms(rotation), rotation.twist_axis,
            )
            pairs = _collect_imu_pairs(
                sensors.imu_sources, config, sensors.rotation_names, rotation, timeout,
                calibration_seconds=self.calibration_seconds,
                prompt="请保持大臂旋转零位并静止，按回车开始 IMU3/IMU4 旋转零位标定：",
                status="大臂旋转 IMU3/IMU4", input_fn=input_fn, print_fn=self.print,
            )
            self.rotation.calibrate(pairs)
            self.print(f"大臂旋转零位标定完成：samples={len(pairs)}")
            self.last_rotation_rad = 0.0
        sensors.reset_synchronization()

    def predict(self, sample: SensorSample) -> MotionIntent:
        if self.sensors.imu_error is not None:
            raise self.sensors.imu_error
        shoulder = self.shoulder.predict(sample)
        elbow = self.elbow.predict(sample)
        intent = replace(shoulder, elbow_flexion=elbow.elbow_flexion)
        if self.rotation is not None:
            try:
                self.last_rotation_frames = _require_fresh_pair(
                    self.sensors.rotation_pair, self.configs.phase3.sensor_timeout_ms / 1000.0,
                    "upper-arm rotation",
                )
                self.rotation_result = self.rotation.update(*self.last_rotation_frames)
                self.last_rotation_rad = math.radians(self.rotation_result.difference_deg)
            except Exception as exc:
                raise RotationPredictionError(
                    exc, replace(intent, upper_arm_rotation_rad=self.last_rotation_rad),
                ) from exc
            intent = replace(intent, upper_arm_rotation_rad=self.last_rotation_rad)
        return intent

    def diagnostics(self):
        return dict(backend=self.backend, rotation_result=self.rotation_result,
                    rotation_rad=self.last_rotation_rad,
                    sync_rejected=0 if self.rotation is None else self.rotation.sync_rejected_count,
                    shoulder_frames=getattr(self.shoulder, "last_frames", None),
                    rotation_frames=self.last_rotation_frames)


def create_intent_pipeline(args, configs):
    if configs.offline_preview:
        return None
    return IntentPipeline(configs, reuse_calibration=args.reuse_calibration,
                          calibration_seconds=args.calibration_seconds,
                          calibration_output=args.calibration_output)
