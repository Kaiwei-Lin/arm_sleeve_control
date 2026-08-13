#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import replace
from enum import Enum, auto
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import (
    DEFAULT_CONFIG_PATH, DEFAULT_PHASE3_CONFIG_PATH, DEFAULT_PHASE4_CONFIG_PATH,
    DEFAULT_SENSOR_CONFIG_PATH, load_phase3_config, load_phase4_config,
    load_robot_config, load_sensor_config,
)
from sleeve_arm.control import ArmMapper, SafeArmController, SensorWatchdog
from sleeve_arm.predictor import ArmMotionPredictor, FlexModelPredictor, RuleBasedPredictor
from sleeve_arm.robot import DyMotorArm, FakeRobotArm
from sleeve_arm.sources import FakeSleeveSource, create_sleeve_source
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


def main() -> int:
    parser = argparse.ArgumentParser(description="Flex model + CH2 elbow control; real motion requires --execute.")
    parser.add_argument("--sleeve", choices=("fake", "real"), default="real")
    parser.add_argument("--robot", choices=("fake", "dymotor"), default="fake")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument("--library", type=Path)
    parser.add_argument(
        "--bridge-diagnostics",
        action="store_true",
        help="print raw C-side PVCT values during DyMotor reads",
    )
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.robot == "dymotor" and args.execute and args.sleeve != "real":
        parser.error("real robot execution requires --sleeve real")

    state = RuntimeState.INIT
    source = controller = robot = shoulder_predictor = None
    invalid = consecutive_errors = stale = cycles = predictions = 0
    try:
        phase3 = load_phase3_config(args.phase3_config)
        phase4 = load_phase4_config(args.phase4_config)
        if phase4.predictor_backend != "flex_model":
            raise ValueError("run_model_control requires predictor.backend=flex_model; use run_sleeve_elbow for rule_based")
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
            for name in ("shoulder_flexion", "shoulder_abduction", "elbow_flexion"):
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

        # Keep the vendor SDK load/connect order identical to the proven Phase 1
        # tools. Model initialization is still completed before Sensor start or
        # Servo On, so a model failure remains motion-free.
        shoulder_predictor = FlexModelPredictor(phase4.flex_model)
        predictor = ArmMotionPredictor(RuleBasedPredictor(elbow_config), shoulder_predictor)

        source = FakeSleeveSource() if args.sleeve == "fake" else create_sleeve_source(load_sensor_config(args.sensor_config))
        sync = SensorSynchronizer()
        source.start()
        readiness_deadline = time.monotonic() + phase3.hard_timeout_ms / 1000.0
        sample = intent = None
        while time.monotonic() < readiness_deadline:
            frame = source.latest()
            if frame is not None:
                sample = sync.synchronize(frame)
                try:
                    intent = predictor.predict(sample)
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
            print("DRY RUN: DyMotor PVCT only; no Servo On and no position command.")

        period = 1.0 / phase3.control_hz
        started = next_tick = last_print = time.monotonic()
        deadline = None if args.duration is None else started + args.duration
        last_frame_timestamp = sample.timestamp
        last_safe = startup
        state = RuntimeState.RUNNING
        while deadline is None or time.monotonic() < deadline:
            frame = source.latest()
            now = time.monotonic()
            cycles += 1
            if frame is not None:
                sample = sync.synchronize(frame)
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
                    action = None if intent.action is None else intent.action.name
                    states = controller.read_joint_states()
                    positions = {name: value.position for name, value in states.items()}
                    tracking = {name: last_safe[name] - positions[name] for name in last_safe}
                    print(
                        f"state={state.name} sleeve_fps={source.stats.estimated_fps:.1f} "
                        f"control_fps={cycles / max(now-started, 1e-9):.1f} "
                        f"model_fps={predictions / max(now-started, 1e-9):.1f} age_ms={age*1000:.1f} "
                        f"flex={shoulder_predictor.last_flex} action={action} confidence={intent.confidence:.3f} "
                        f"angle_deg={intent.angle_deg:.2f} inference_ms={intent.inference_ms:.3f} "
                        f"targets_rad={last_safe} positions_rad={positions} tracking_rad={tracking} "
                        f"invalid={invalid} stale={stale}"
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
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
