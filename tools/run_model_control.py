#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable
from dataclasses import replace
from enum import Enum, auto
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import (
    DEFAULT_CONFIG_PATH, DEFAULT_PHASE3_CONFIG_PATH, DEFAULT_PHASE4_CONFIG_PATH,
    DEFAULT_SENSOR_CONFIG_PATH, load_phase3_config, load_phase4_config,
    load_robot_config, load_sensor_config, FlexModelConfig, SensorConfig,
    UpperArmRotationConfig,
)
from sleeve_arm.control import ArmMapper, SafeArmController, SensorWatchdog
from sleeve_arm.domain import ImuFrame, SensorSample
from sleeve_arm.estimation import UpperArmRotationEstimator, UpperArmRotationResult
from sleeve_arm.predictor import ArmMotionPredictor, FlexModelPredictor, RuleBasedPredictor
from sleeve_arm.predictor.calibration import collect_calibration_samples
from sleeve_arm.robot import DyMotorArm, FakeRobotArm
from sleeve_arm.sources import (
    FakeImuSource,
    FakeSleeveSource,
    ImuSource,
    create_imu_source,
    create_sleeve_source,
)
from sleeve_arm.sync import SensorSynchronizer


class RuntimeState(Enum):
    INIT = auto()
    ROBOT_READY = auto()
    SENSOR_READY = auto()
    ARMED = auto()
    RUNNING = auto()
    STALE = auto()
    FAULT = auto()
    STOPPING = auto()


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


def _add_latest_imus(sync: SensorSynchronizer, sources: dict[str, ImuSource]) -> None:
    for name, source in sources.items():
        frame = source.latest()
        if frame is None:
            continue
        (sync.add_imu1 if name == "imu1" else sync.add_imu2)(frame)


def _rotation_pair(
    sample: SensorSample,
    config: UpperArmRotationConfig,
    max_age_s: float | None = None,
) -> tuple[ImuFrame, ImuFrame]:
    upper = getattr(sample, config.upper_imu)
    reference = getattr(sample, config.reference_imu)
    if upper is None or reference is None:
        raise ValueError("upper-arm rotation requires latest upper and reference IMU frames")
    if max_age_s is not None:
        now = time.monotonic()
        if any(abs(now - frame.timestamp) > max_age_s for frame in (upper, reference)):
            raise ValueError("upper-arm rotation IMU pair is stale")
    return upper, reference


def _attach_rotation_pair(
    sample: SensorSample,
    sync: SensorSynchronizer,
    config: UpperArmRotationConfig,
) -> SensorSample:
    pair = sync.latest_imu_pair(config.max_sync_ms)
    if pair is None:
        return replace(sample, imu1=None, imu2=None)
    return replace(sample, imu1=pair[0], imu2=pair[1])


def _synchronize_latest(
    sleeve_source: Any,
    imu_sources: dict[str, ImuSource],
    sync: SensorSynchronizer,
    last_sleeve_timestamp: float | None,
    rotation_config: UpperArmRotationConfig,
) -> SensorSample | None:
    _add_latest_imus(sync, imu_sources)
    sleeve = sleeve_source.latest()
    if sleeve is None or sleeve.timestamp == last_sleeve_timestamp:
        return None
    return _attach_rotation_pair(sync.synchronize(sleeve), sync, rotation_config)


def prepare_upper_arm_rotation(
    sleeve_source: Any,
    imu_sources: dict[str, ImuSource],
    sensor_config: SensorConfig,
    sensor_timeout_s: float,
    *,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[[str], None] = print,
) -> UpperArmRotationEstimator:
    config = sensor_config.upper_arm_rotation
    estimator = UpperArmRotationEstimator(
        config.ema_alpha,
        config.max_sync_ms,
        config.twist_axis,
    )

    def synchronizer() -> SensorSynchronizer:
        return SensorSynchronizer(
            sensor_config.synchronization.max_time_delta_ms,
            sensor_config.synchronization.buffer_duration_ms,
            imu1_enabled=True,
            imu2_enabled=True,
        )

    sync = synchronizer()
    last_timestamp = None
    startup_deadline = time.monotonic() + config.startup_timeout_s
    while time.monotonic() < startup_deadline:
        sample = _synchronize_latest(
            sleeve_source, imu_sources, sync, last_timestamp, config
        )
        if sample is not None:
            last_timestamp = sample.timestamp
            if 0.0 <= time.monotonic() - sample.timestamp <= sensor_timeout_s:
                try:
                    estimator.validate_pair(*_rotation_pair(sample, config, sensor_timeout_s))
                    break
                except ValueError:
                    pass
        time.sleep(0.001)
    else:
        raise RuntimeError("no fresh valid dual-IMU quaternion pair before startup timeout")

    input_fn("请保持大臂旋转零位并静止，按回车开始 IMU 零位标定：")
    print_fn(f"正在标定 {config.calibration_seconds:g} 秒；当前姿态定义为 upper_arm_rotation=0°...")
    sync = synchronizer()  # discard every pre-prompt frame
    last_timestamp = None
    pairs: list[tuple[ImuFrame, ImuFrame]] = []
    deadline = time.monotonic() + config.calibration_seconds
    while time.monotonic() < deadline:
        sample = _synchronize_latest(
            sleeve_source, imu_sources, sync, last_timestamp, config
        )
        if sample is not None:
            last_timestamp = sample.timestamp
            if 0.0 <= time.monotonic() - sample.timestamp <= sensor_timeout_s:
                try:
                    pair = _rotation_pair(sample, config, sensor_timeout_s)
                    estimator.validate_pair(*pair)
                except ValueError:
                    pass
                else:
                    pairs.append(pair)
        time.sleep(0.001)
    estimator.calibrate(pairs)
    zero = estimator.update(*pairs[-1])
    print_fn(
        f"IMU 零位标定完成：pairs={len(pairs)}, difference={zero.difference_deg:+.3f}°"
    )
    return estimator


def main() -> int:
    parser = argparse.ArgumentParser(description="Flex model + CH2 elbow control; real motion requires --execute.")
    parser.add_argument("--sleeve", choices=("fake", "real"), default="real")
    parser.add_argument("--imus", choices=("fake", "real"), default="real")
    parser.add_argument("--robot", choices=("fake", "dymotor"), default="dymotor")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument(
        "--reuse-calibration",
        action="store_true",
        help="reuse the configured saved calibration instead of collecting a fresh baseline",
    )
    parser.add_argument("--calibration-seconds", type=float)
    parser.add_argument("--calibration-output", type=Path)
    parser.add_argument("--library", type=Path)
    parser.add_argument(
        "--bridge-diagnostics",
        action="store_true",
        help="print raw C-side PVCT values during DyMotor reads",
    )
    parser.add_argument("--imu-debug", action="store_true", help="include both quaternions in 1 Hz telemetry")
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.calibration_seconds is not None and args.calibration_seconds <= 0:
        parser.error("--calibration-seconds must be positive")
    if args.robot == "dymotor" and args.execute and args.sleeve != "real":
        parser.error("real robot execution requires --sleeve real")

    state = RuntimeState.INIT
    source = controller = robot = shoulder_predictor = rotation_estimator = None
    imu_sources: dict[str, ImuSource] = {}
    rotation_result: UpperArmRotationResult | None = None
    last_rotation_frames: tuple[ImuFrame, ImuFrame] | None = None
    last_rotation_rad: float | None = None
    invalid = consecutive_errors = stale = cycles = predictions = 0
    rotation_invalid = rotation_consecutive_errors = 0
    try:
        phase3 = load_phase3_config(args.phase3_config)
        phase4 = load_phase4_config(args.phase4_config)
        sensor_config = load_sensor_config(args.sensor_config)
        rotation_config = sensor_config.upper_arm_rotation
        if phase4.predictor_backend != "flexarm_estimator":
            raise ValueError(
                "run_model_control requires predictor.backend=flexarm_estimator; "
                "use run_sleeve_elbow for rule_based"
            )
        assert phase4.flex_model is not None
        elbow_config = phase3.elbow
        if args.sleeve == "fake" and elbow_config.input_min is None:
            elbow_config = replace(
                elbow_config, input_min=0.0, input_max=2.0,
                angle_min_deg=0.0, angle_max_deg=90.0,
            )
        watchdog = SensorWatchdog(phase3.sensor_timeout_ms, phase3.hard_timeout_ms)

        robot_config = load_robot_config(args.robot_config)
        if args.robot == "dymotor" and args.execute:
            controlled = ["shoulder_flexion", "shoulder_abduction", "elbow_flexion"]
            if rotation_config.enabled:
                controlled.append("upper_arm_rotation")
            for name in controlled:
                joint = robot_config.joints[name]
                if joint.zero_position is None or joint.min_position is None or joint.max_position is None:
                    raise ValueError(
                        f"{name}: absolute model control requires calibrated "
                        "zero_position, min_position, and max_position"
                    )
        robot = (
            FakeRobotArm(robot_config)
            if args.robot == "fake"
            else DyMotorArm(robot_config, args.library, diagnostics=args.bridge_diagnostics)
        )
        controller = SafeArmController(robot, robot_config)
        controller.connect()
        controller.read_joint_states()
        if isinstance(robot, DyMotorArm):
            print(f"loaded_so: {robot.loaded_library_path}")
        state = RuntimeState.ROBOT_READY

        # Match the proven Phase 1 tools: connect DyMotor before opening serial
        # sources or importing/initializing the external model. The vendor
        # connection pipeline itself performs Servo On; --execute only gates
        # post-startup generated targets.
        source = (
            FakeSleeveSource()
            if args.sleeve == "fake"
            else create_sleeve_source(sensor_config)
        )
        if rotation_config.enabled:
            for name in (rotation_config.upper_imu, rotation_config.reference_imu):
                endpoint = getattr(sensor_config, name)
                created = (
                    FakeImuSource(timestamp_offset_s=(0.0 if name == rotation_config.upper_imu else 0.005))
                    if args.imus == "fake"
                    else create_imu_source(name, endpoint)
                )
                if created is None:
                    raise RuntimeError(f"upper_arm_rotation could not create configured {name} source")
                imu_sources[name] = created
        source.start()
        for imu_source in imu_sources.values():
            imu_source.start()
        shoulder_predictor = prepare_flexarm_predictor(
            source,
            phase4.flex_model,
            reuse_calibration=args.reuse_calibration,
            calibration_seconds=args.calibration_seconds,
            calibration_output=args.calibration_output,
        )
        predictor = ArmMotionPredictor(RuleBasedPredictor(elbow_config), shoulder_predictor)

        if rotation_config.enabled:
            rotation_estimator = prepare_upper_arm_rotation(
                source,
                imu_sources,
                sensor_config,
                phase3.sensor_timeout_ms / 1000.0,
            )
            last_rotation_rad = 0.0

        sync = SensorSynchronizer(
            sensor_config.synchronization.max_time_delta_ms,
            sensor_config.synchronization.buffer_duration_ms,
            imu1_enabled=rotation_config.enabled,
            imu2_enabled=rotation_config.enabled,
        )
        readiness_deadline = time.monotonic() + phase3.hard_timeout_ms / 1000.0
        sample = intent = None
        while time.monotonic() < readiness_deadline:
            _add_latest_imus(sync, imu_sources)
            frame = source.latest()
            if frame is not None:
                sample = sync.synchronize(frame)
                if rotation_estimator is not None:
                    sample = _attach_rotation_pair(sample, sync, rotation_config)
                try:
                    intent = predictor.predict(sample)
                    if rotation_estimator is not None:
                        last_rotation_frames = _rotation_pair(
                            sample, rotation_config, phase3.sensor_timeout_ms / 1000.0
                        )
                        rotation_result = rotation_estimator.update(*last_rotation_frames)
                        last_rotation_rad = math.radians(rotation_result.difference_deg)
                        intent = replace(intent, upper_arm_rotation_rad=last_rotation_rad)
                    break
                except Exception as exc:
                    invalid += 1
                    last_error = exc
            time.sleep(0.001)
        if sample is None or intent is None:
            raise RuntimeError(f"no valid model prediction before readiness timeout: {locals().get('last_error')}")
        state = RuntimeState.SENSOR_READY

        startup_states = controller.read_joint_states()
        startup = {name: item.position for name, item in startup_states.items()}
        mapper = ArmMapper(elbow_config, startup)
        motion_enabled = args.robot == "fake" or args.execute
        if motion_enabled:
            controller.enable()
            controller.set_joint_positions(startup, dt=1.0 / phase3.control_hz)
            state = RuntimeState.ARMED
        else:
            print("DRY RUN: vendor startup used Servo On; no post-startup model target is sent.")

        period = 1.0 / phase3.control_hz
        started = next_tick = last_print = time.monotonic()
        deadline = None if args.duration is None else started + args.duration
        last_frame_timestamp = sample.timestamp
        last_safe = startup
        state = RuntimeState.RUNNING
        while deadline is None or time.monotonic() < deadline:
            rotation_source_error: Exception | None = None
            try:
                _add_latest_imus(sync, imu_sources)
            except Exception as exc:
                rotation_source_error = exc
            frame = source.latest()
            now = time.monotonic()
            cycles += 1
            if frame is not None:
                sample = sync.synchronize(frame)
                if rotation_estimator is not None:
                    sample = _attach_rotation_pair(sample, sync, rotation_config)
                age = watchdog.age(sample.timestamp, now)
                if watchdog.is_hard_timeout(sample.timestamp, now):
                    raise RuntimeError(f"Sleeve hard timeout: {age * 1000:.1f} ms")
                if watchdog.is_stale(sample.timestamp, now):
                    if state is not RuntimeState.STALE:
                        print(f"WARNING: Sleeve stale ({age * 1000:.1f} ms); holding last safe target")
                    state = RuntimeState.STALE
                    stale += 1
                elif sample.timestamp != last_frame_timestamp:
                    try:
                        intent = predictor.predict(sample)
                    except Exception as exc:
                        invalid += 1
                        consecutive_errors += 1
                        last_frame_timestamp = sample.timestamp
                        print(f"WARNING: prediction rejected; holding last safe target: {exc}", file=sys.stderr)
                        if consecutive_errors >= phase4.max_consecutive_prediction_errors:
                            raise RuntimeError(f"too many consecutive prediction errors: {exc}") from exc
                    else:
                        if watchdog.is_hard_timeout(sample.timestamp, time.monotonic()):
                            raise RuntimeError("prediction completed after Sleeve hard timeout")
                        if rotation_estimator is not None:
                            try:
                                if rotation_source_error is not None:
                                    raise rotation_source_error
                                last_rotation_frames = _rotation_pair(
                                    sample, rotation_config, phase3.sensor_timeout_ms / 1000.0
                                )
                                rotation_result = rotation_estimator.update(*last_rotation_frames)
                                last_rotation_rad = math.radians(rotation_result.difference_deg)
                            except Exception as exc:
                                rotation_invalid += 1
                                rotation_consecutive_errors += 1
                                print(
                                    f"WARNING: upper-arm rotation rejected; holding last safe target: {exc}",
                                    file=sys.stderr,
                                )
                                if rotation_consecutive_errors >= phase4.max_consecutive_prediction_errors:
                                    raise RuntimeError(
                                        f"too many consecutive upper-arm rotation errors: {exc}"
                                    ) from exc
                            else:
                                rotation_consecutive_errors = 0
                            assert last_rotation_rad is not None
                            intent = replace(intent, upper_arm_rotation_rad=last_rotation_rad)
                        mapped = mapper.map(intent)
                        last_safe = (
                            controller.set_joint_positions(mapped, dt=period)
                            if motion_enabled else controller.preview_positions(mapped, dt=period)
                        )
                        predictions += 1
                        consecutive_errors = 0
                        last_frame_timestamp = sample.timestamp
                        state = RuntimeState.RUNNING
                if now - last_print >= 1.0:
                    states = controller.read_joint_states()
                    positions = {name: value.position for name, value in states.items()}
                    tracking = {name: last_safe[name] - positions[name] for name in last_safe}
                    rotation_telemetry = ""
                    if rotation_estimator is not None and rotation_result is not None:
                        rotation_telemetry = (
                            f" rotation={rotation_result.difference_deg:+.2f}deg"
                            f" world={rotation_result.world_filtered_deg:+.2f}deg"
                            f" relative={rotation_result.relative_filtered_deg:+.2f}deg"
                            f" imu_sync={rotation_result.sync_gap_ms:.2f}ms"
                            f" rotation_rad={last_rotation_rad:+.4f}"
                            f" rotation_target={last_safe['upper_arm_rotation']:+.4f}"
                            f" rotation_position={positions['upper_arm_rotation']:+.4f}"
                            f" imu_sync_rejected={rotation_estimator.sync_rejected_count}"
                            f" rotation_invalid={rotation_invalid}"
                        )
                        if args.imu_debug and last_rotation_frames is not None:
                            upper, reference = last_rotation_frames
                            rotation_telemetry += (
                                f" upper_q={(upper.quat_w, upper.quat_x, upper.quat_y, upper.quat_z)}"
                                f" reference_q={(reference.quat_w, reference.quat_x, reference.quat_y, reference.quat_z)}"
                            )
                    # print(
                    #     f"state={state.name} sleeve_fps={source.stats.estimated_fps:.1f} "
                    #     f"control_fps={cycles / max(now-started, 1e-9):.1f} "
                    #     f"model_fps={predictions / max(now-started, 1e-9):.1f} age_ms={age*1000:.1f} "
                    #     f"flex={shoulder_predictor.last_flex} action={intent.model_action} "
                    #     f"action_conf={intent.confidence:.3f} angle_conf={intent.angle_confidence:.3f} "
                    #     f"moving={intent.moving} angle_deg={intent.angle_deg:.2f} "
                    #     f"inference_ms={intent.inference_ms:.3f} "
                    #     f"targets_rad={last_safe} positions_rad={positions} tracking_rad={tracking} "
                    #     f"invalid={invalid} stale={stale}{rotation_telemetry}"
                    # )
                    last_print = now
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
        return 0
    except KeyboardInterrupt:
        print("\nStopping on Ctrl+C...")
        return 0
    except Exception as exc:
        state = RuntimeState.FAULT
        print(f"ERROR [{state.name}] {type(exc).__name__}: {exc}", file=sys.stderr)
        if isinstance(robot, DyMotorArm):
            print(f"loaded_so: {robot.loaded_library_path}", file=sys.stderr)
        return 1
    finally:
        state = RuntimeState.STOPPING
        if controller is not None:
            try:
                controller.shutdown()
            except Exception as exc:
                print(f"ERROR [{state.name}] shutdown: {exc}", file=sys.stderr)
        close_errors: list[Exception] = []
        if source is not None:
            try:
                source.close()
            except Exception as exc:
                close_errors.append(exc)
        for imu_source in reversed(tuple(imu_sources.values())):
            try:
                imu_source.close()
            except Exception as exc:
                close_errors.append(exc)
        for exc in close_errors:
            print(f"ERROR [{state.name}] source close: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
