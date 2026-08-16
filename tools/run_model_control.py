#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from collections.abc import Callable, Sequence
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
from sleeve_arm.domain import ArmAction, ImuFrame, MotionIntent, SensorSample
from sleeve_arm.estimation import UpperArmRotationEstimator, UpperArmRotationResult
from sleeve_arm.predictor import FlexModelPredictor, RuleBasedPredictor
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
    """Legacy helper retained for the standalone flex calibration/test tools."""
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


def _imu_quaternion(frame: ImuFrame) -> tuple[float, float, float, float]:
    values = (frame.quat_w, frame.quat_x, frame.quat_y, frame.quat_z)
    if any(value is None for value in values):
        raise ValueError("IMU quaternion is missing")
    quaternion = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError("IMU quaternion must contain four finite WXYZ values")
    if math.sqrt(sum(value * value for value in quaternion)) <= 1e-12:
        raise ValueError("IMU quaternion norm must be nonzero")
    return quaternion  # type: ignore[return-value]


def _role_pair(
    imu1: ImuFrame,
    imu2: ImuFrame,
    config: UpperArmRotationConfig,
) -> tuple[ImuFrame, ImuFrame]:
    frames = {"imu1": imu1, "imu2": imu2}
    return frames[config.upper_imu], frames[config.reference_imu]


def _rotation_pair(
    sample: SensorSample,
    config: UpperArmRotationConfig,
    max_age_s: float | None = None,
) -> tuple[ImuFrame, ImuFrame]:
    if sample.imu1 is None or sample.imu2 is None:
        raise ValueError("dual-IMU shoulder estimation requires synchronized chest and arm frames")
    upper, reference = _role_pair(sample.imu1, sample.imu2, config)
    if max_age_s is not None:
        now = time.monotonic()
        if any(abs(now - frame.timestamp) > max_age_s for frame in (upper, reference)):
            raise ValueError("dual-IMU shoulder pair is stale")
    return upper, reference


def _attach_rotation_pair(
    sample: SensorSample,
    sync: SensorSynchronizer,
    config: UpperArmRotationConfig,
    max_sync_ms: float,
) -> SensorSample:
    pair = sync.latest_imu_pair(max_sync_ms)
    if pair is None:
        return replace(sample, imu1=None, imu2=None)
    return replace(sample, imu1=pair[0], imu2=pair[1])


def _latest_imu_pair(
    imu_sources: dict[str, ImuSource],
    sync: SensorSynchronizer,
    config: UpperArmRotationConfig,
    max_sync_ms: float,
) -> tuple[ImuFrame, ImuFrame] | None:
    _add_latest_imus(sync, imu_sources)
    pair = sync.latest_imu_pair(max_sync_ms)
    if pair is None:
        return None
    return _role_pair(*pair, config)


def calibrate_dual_imu_estimator(
    estimator: Any,
    pairs: Sequence[tuple[ImuFrame, ImuFrame]],
) -> Any:
    if len(pairs) < 2:
        raise ValueError("dual-IMU neutral calibration needs at least 2 synchronized pairs")
    chest_rest_samples = [_imu_quaternion(reference) for _, reference in pairs]
    arm_rest_samples = [_imu_quaternion(upper) for upper, _ in pairs]
    return estimator.calibrate(chest_rest_samples, arm_rest_samples)


def prepare_dual_imu_estimator(
    imu_sources: dict[str, ImuSource],
    sensor_config: SensorConfig,
    sensor_timeout_s: float,
    *,
    estimator: Any | None = None,
    calibration_seconds: float | None = None,
    input_fn: Callable[[str], str] = input,
    print_fn: Callable[[str], None] = print,
) -> tuple[Any, list[tuple[ImuFrame, ImuFrame]]]:
    config = sensor_config.upper_arm_rotation
    duration = config.calibration_seconds if calibration_seconds is None else calibration_seconds
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("dual-IMU calibration duration must be positive and finite")
    if estimator is None:
        from flexarm import DualImuArmEstimator

        estimator = DualImuArmEstimator()
    max_sync_ms = (
        sensor_config.synchronization.max_time_delta_ms
        if config.max_sync_ms is None
        else config.max_sync_ms
    )

    def synchronizer() -> SensorSynchronizer:
        return SensorSynchronizer(
            sensor_config.synchronization.max_time_delta_ms,
            sensor_config.synchronization.buffer_duration_ms,
            imu1_enabled=True,
            imu2_enabled=True,
        )

    sync = synchronizer()
    startup_deadline = time.monotonic() + config.startup_timeout_s
    while time.monotonic() < startup_deadline:
        pair = _latest_imu_pair(imu_sources, sync, config, max_sync_ms)
        if pair is not None:
            try:
                if any(abs(time.monotonic() - frame.timestamp) > sensor_timeout_s for frame in pair):
                    raise ValueError("dual-IMU shoulder pair is stale")
                _imu_quaternion(pair[0])
                _imu_quaternion(pair[1])
            except ValueError:
                pass
            else:
                break
        time.sleep(0.001)
    else:
        raise RuntimeError("no fresh valid dual-IMU quaternion pair before startup timeout")

    input_fn("请自然站立、右臂自然下垂并保持静止，按回车开始双 IMU 肩部零位标定：")
    print_fn(f"正在采集 {duration:g} 秒胸部/大臂同步四元数...")
    sync = synchronizer()  # discard every pre-prompt frame
    pairs: list[tuple[ImuFrame, ImuFrame]] = []
    last_pair_key: tuple[int, int] | None = None
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        pair = _latest_imu_pair(imu_sources, sync, config, max_sync_ms)
        if pair is not None:
            pair_key = (int(pair[0].host_timestamp_ns), int(pair[1].host_timestamp_ns))
            try:
                if any(abs(time.monotonic() - frame.timestamp) > sensor_timeout_s for frame in pair):
                    raise ValueError("dual-IMU shoulder pair is stale")
                _imu_quaternion(pair[0])
                _imu_quaternion(pair[1])
            except ValueError:
                pass
            else:
                if pair_key != last_pair_key:
                    pairs.append(pair)
                    last_pair_key = pair_key
        time.sleep(0.001)
    calibration = calibrate_dual_imu_estimator(estimator, pairs)
    sample_count = int(getattr(calibration, "sample_count", len(pairs)))
    print_fn(f"双 IMU 肩部零位标定完成：samples={sample_count}")
    return estimator, pairs


class DualImuShoulderPredictor:
    """Adapt DualImuArmEstimator direction/magnitude output to robot semantics."""

    _ACTIVE = {
        "Forward": (ArmAction.FORWARD, 1.0, 0.0),
        "Backward": (ArmAction.BACKWARD, -1.0, 0.0),
        "Lateral": (ArmAction.LATERAL, 0.0, 1.0),
    }

    def __init__(
        self,
        estimator: Any,
        config: UpperArmRotationConfig,
        max_age_s: float | None,
    ) -> None:
        self.estimator = estimator
        self.config = config
        self.max_age_s = max_age_s
        self.last_result: Any | None = None
        self.last_frames: tuple[ImuFrame, ImuFrame] | None = None
        self._last_targets: tuple[float | None, float | None] = (None, None)

    def predict(self, sample: SensorSample) -> MotionIntent:
        upper, chest = _rotation_pair(sample, self.config, self.max_age_s)
        chest_q = _imu_quaternion(chest)
        arm_q = _imu_quaternion(upper)
        started = time.perf_counter()
        result = self.estimator.update(chest_q, arm_q)
        inference_ms = (time.perf_counter() - started) * 1000.0

        direction = getattr(result, "direction", None)
        if direction not in (*self._ACTIVE, "Rest", "Transition"):
            raise ValueError(f"invalid dual-IMU shoulder direction: {direction!r}")
        magnitude_deg = float(getattr(result, "magnitude_deg", math.nan))
        confidence = float(getattr(result, "confidence", math.nan))
        if not math.isfinite(magnitude_deg) or not 0.0 <= magnitude_deg <= 180.0:
            raise ValueError("dual-IMU shoulder magnitude_deg must be finite and in [0, 180]")
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("dual-IMU shoulder confidence must be finite and in [0, 1]")

        action = None
        if direction in self._ACTIVE:
            action, flexion_sign, abduction_sign = self._ACTIVE[direction]
            magnitude_rad = math.radians(magnitude_deg)
            targets = (flexion_sign * magnitude_rad, abduction_sign * magnitude_rad)
            self._last_targets = targets
        elif direction == "Rest":
            targets = (0.0, 0.0)
            self._last_targets = targets
        else:
            # Direction + magnitude cannot uniquely decompose a diagonal transition.
            targets = self._last_targets

        self.last_result = result
        self.last_frames = (upper, chest)
        print(f"Predicted motion: {direction} with magnitude {magnitude_deg:.1f}°")
        return MotionIntent(
            timestamp=sample.timestamp,
            shoulder_flexion_rad=targets[0],
            shoulder_abduction_rad=targets[1],
            action=action,
            confidence=confidence,
            angle_deg=magnitude_deg,
            inference_ms=inference_ms,
            model_action=direction,
            moving=direction != "Rest",
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Dual-IMU shoulder + CH2 elbow control; real motion requires --execute.")
    parser.add_argument("--sleeve", choices=("fake", "real"), default="real")
    parser.add_argument("--imus", choices=("fake", "real"), default="real")
    parser.add_argument("--robot", choices=("fake", "dymotor"), default="dymotor")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument("--calibration-seconds", type=float, help="override neutral dual-IMU calibration duration")
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
    if args.robot == "dymotor" and args.execute:
        if args.sleeve != "real":
            parser.error("real robot execution requires --sleeve real")
        if args.imus != "real":
            parser.error("real robot execution requires --imus real")

    state = RuntimeState.INIT
    source = controller = robot = shoulder_estimator = shoulder_predictor = rotation_estimator = None
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
        if phase4.predictor_backend != "dual_imu":
            raise ValueError(
                "run_model_control requires predictor.backend=dual_imu; "
                "use run_sleeve_elbow for rule_based"
            )
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
        for name in (rotation_config.upper_imu, rotation_config.reference_imu):
            endpoint = getattr(sensor_config, name)
            if not endpoint.enabled:
                raise ValueError(f"dual-IMU shoulder estimation requires sensors.{name}.enabled=true")
            created = (
                FakeImuSource(timestamp_offset_s=(0.0 if name == rotation_config.upper_imu else 0.005))
                if args.imus == "fake"
                else create_imu_source(name, endpoint)
            )
            if created is None:
                raise RuntimeError(f"dual-IMU shoulder estimation could not create configured {name} source")
            imu_sources[name] = created
        source.start()
        for imu_source in imu_sources.values():
            imu_source.start()
        shoulder_estimator, neutral_pairs = prepare_dual_imu_estimator(
            imu_sources,
            sensor_config,
            phase3.sensor_timeout_ms / 1000.0,
            calibration_seconds=args.calibration_seconds,
        )
        shoulder_predictor = DualImuShoulderPredictor(
            shoulder_estimator,
            rotation_config,
            phase3.sensor_timeout_ms / 1000.0,
        )
        elbow_predictor = RuleBasedPredictor(elbow_config)

        max_sync_ms = (
            sensor_config.synchronization.max_time_delta_ms
            if rotation_config.max_sync_ms is None
            else rotation_config.max_sync_ms
        )
        if rotation_config.enabled:
            rotation_estimator = UpperArmRotationEstimator(
                rotation_config.ema_alpha,
                max_sync_ms,
                rotation_config.twist_axis,
            )
            rotation_estimator.calibrate(neutral_pairs)
            last_rotation_rad = 0.0

        sync = SensorSynchronizer(
            sensor_config.synchronization.max_time_delta_ms,
            sensor_config.synchronization.buffer_duration_ms,
            imu1_enabled=True,
            imu2_enabled=True,
        )
        readiness_deadline = time.monotonic() + phase3.hard_timeout_ms / 1000.0
        sample = intent = None
        while time.monotonic() < readiness_deadline:
            _add_latest_imus(sync, imu_sources)
            frame = source.latest()
            if frame is not None:
                sample = sync.synchronize(frame)
                sample = _attach_rotation_pair(sample, sync, rotation_config, max_sync_ms)
                try:
                    shoulder_intent = shoulder_predictor.predict(sample)
                    elbow_intent = elbow_predictor.predict(sample)
                    intent = replace(shoulder_intent, elbow_flexion=elbow_intent.elbow_flexion)
                    last_rotation_frames = shoulder_predictor.last_frames
                    if rotation_estimator is not None:
                        assert last_rotation_frames is not None
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
            imu_source_error: Exception | None = None
            try:
                _add_latest_imus(sync, imu_sources)
            except Exception as exc:
                imu_source_error = exc
            frame = source.latest()
            now = time.monotonic()
            cycles += 1
            if frame is not None:
                sample = sync.synchronize(frame)
                sample = _attach_rotation_pair(sample, sync, rotation_config, max_sync_ms)
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
                        if imu_source_error is not None:
                            raise imu_source_error
                        shoulder_intent = shoulder_predictor.predict(sample)
                        elbow_intent = elbow_predictor.predict(sample)
                        intent = replace(shoulder_intent, elbow_flexion=elbow_intent.elbow_flexion)
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
                        last_rotation_frames = shoulder_predictor.last_frames
                        if rotation_estimator is not None:
                            try:
                                assert last_rotation_frames is not None
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
                    shoulder_result = shoulder_predictor.last_result
                    assert shoulder_result is not None
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
                        upper, chest = last_rotation_frames
                        rotation_telemetry += (
                            f" arm_q={(upper.quat_w, upper.quat_x, upper.quat_y, upper.quat_z)}"
                            f" chest_q={(chest.quat_w, chest.quat_x, chest.quat_y, chest.quat_z)}"
                        )
                    print(
                        f"state={state.name} sleeve_fps={source.stats.estimated_fps:.1f} "
                        f"control_fps={cycles / max(now-started, 1e-9):.1f} "
                        f"model_fps={predictions / max(now-started, 1e-9):.1f} age_ms={age*1000:.1f} "
                        f"direction={shoulder_result.direction} "
                        f"magnitude_deg={shoulder_result.magnitude_deg:.2f} "
                        f"confidence={shoulder_result.confidence:.3f} "
                        f"inference_ms={intent.inference_ms:.3f} "
                        f"targets_rad={last_safe} positions_rad={positions} tracking_rad={tracking} "
                        f"invalid={invalid} stale={stale}{rotation_telemetry}"
                    )
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
