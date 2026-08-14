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
    DEFAULT_CONFIG_PATH, DEFAULT_PHASE3_CONFIG_PATH, DEFAULT_SENSOR_CONFIG_PATH,
    load_phase3_config, load_robot_config, load_sensor_config,
)
from sleeve_arm.control import ArmMapper, SafeArmController, SensorWatchdog
from sleeve_arm.predictor import RuleBasedPredictor
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


def wait_for_sample(source, sync, predictor, timeout_s: float):
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        frame = source.latest()
        if frame is not None:
            sample = sync.synchronize(frame)
            try:
                predictor.predict(sample)
                return sample
            except ValueError as exc:
                last_error = exc
        time.sleep(0.001)
    raise RuntimeError(f"no valid fresh Sleeve sample before readiness timeout: {last_error}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 3 CH2-to-elbow control; real motion requires --execute.")
    parser.add_argument("--sleeve", choices=("fake", "real"), default="fake")
    parser.add_argument("--robot", choices=("fake", "dymotor"), default="fake")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--library", type=Path)
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    if args.robot == "dymotor" and args.execute and args.sleeve != "real":
        parser.error("real robot execution requires --sleeve real")

    state = RuntimeState.INIT
    source = None
    controller = None
    invalid_samples = stale_count = cycles = 0
    try:
        phase3 = load_phase3_config(args.phase3_config)
        elbow_config = phase3.elbow
        if args.sleeve == "fake" and elbow_config.input_min is None:
            # FakeSleeve CH2 is 1 + sin(...), so its deterministic range is [0, 2].
            elbow_config = replace(
                elbow_config, input_min=0.0, input_max=2.0,
                angle_min_deg=0.0, angle_max_deg=90.0,
            )
        predictor = RuleBasedPredictor(elbow_config)
        watchdog = SensorWatchdog(phase3.sensor_timeout_ms, phase3.hard_timeout_ms)
        robot_config = load_robot_config(args.robot_config)
        if args.robot == "dymotor" and args.execute:
            joint = robot_config.joints["elbow_flexion"]
            if joint.zero_position is None or joint.min_position is None or joint.max_position is None:
                raise ValueError(
                    "elbow_flexion: absolute control requires calibrated "
                    "zero_position, min_position, and max_position"
                )
        robot = FakeRobotArm(robot_config) if args.robot == "fake" else DyMotorArm(robot_config, args.library)
        controller = SafeArmController(robot, robot_config)

        # Read and validate all three joints before opening the Sleeve or enabling anything.
        controller.connect(prepare_feedback=args.robot == "dymotor" and args.execute)
        startup_states = controller.read_joint_states()
        startup = {name: item.position for name, item in startup_states.items()}
        state = RuntimeState.ROBOT_READY

        source = FakeSleeveSource() if args.sleeve == "fake" else create_sleeve_source(load_sensor_config(args.sensor_config))
        sync = SensorSynchronizer()
        source.start()
        sample = wait_for_sample(source, sync, predictor, phase3.hard_timeout_ms / 1000.0)
        state = RuntimeState.SENSOR_READY

        # Refresh startup positions immediately before the only possible Servo On.
        startup_states = controller.read_joint_states()
        startup = {name: item.position for name, item in startup_states.items()}
        mapper = ArmMapper(elbow_config, startup)

        motion_enabled = args.robot == "fake" or args.execute
        if motion_enabled:
            controller.enable()
            controller.set_joint_positions(startup, dt=1.0 / phase3.control_hz)
            state = RuntimeState.ARMED
        elif args.robot == "dymotor":
            print("DRY RUN: DyMotor connected for PVCT only; Servo On is disabled.")

        period = 1.0 / phase3.control_hz
        started = next_tick = last_print = time.monotonic()
        deadline = None if args.duration is None else started + args.duration
        # The readiness sample only validates the pipeline; motion starts from
        # the measured robot position and waits for the next fresh Sleeve frame.
        last_frame_timestamp = sample.timestamp
        last_safe = startup
        state = RuntimeState.RUNNING
        while deadline is None or time.monotonic() < deadline:
            frame = source.latest()
            now = time.monotonic()
            cycles += 1
            if frame is None:
                if now >= next_tick:
                    next_tick += period
                time.sleep(max(0.0, min(0.001, next_tick - time.monotonic())))
                continue
            sample = sync.synchronize(frame)
            age = watchdog.age(sample.timestamp, now)
            if watchdog.is_hard_timeout(sample.timestamp, now):
                raise RuntimeError(f"Sleeve hard timeout: {age * 1000:.1f} ms")
            if watchdog.is_stale(sample.timestamp, now):
                if state is not RuntimeState.STALE:
                    print(f"WARNING: Sleeve stale ({age * 1000:.1f} ms); holding last safe target")
                state = RuntimeState.STALE
                stale_count += 1
            elif sample.timestamp != last_frame_timestamp:
                try:
                    intent = predictor.predict(sample)
                    mapped = mapper.map(intent)
                    if motion_enabled:
                        last_safe = controller.set_joint_positions(mapped, dt=period)
                    else:
                        last_safe = controller.preview_positions(mapped, dt=period)
                    last_frame_timestamp = sample.timestamp
                    state = RuntimeState.RUNNING
                except ValueError as exc:
                    invalid_samples += 1
                    print(f"WARNING: rejected sensor sample: {exc}", file=sys.stderr)

            if now - last_print >= 1.0:
                elbow = startup["elbow_flexion"]
                target = last_safe["elbow_flexion"]
                stats = source.stats
                print(
                    f"state={state.name} sleeve_fps={stats.estimated_fps:.1f} "
                    f"control_fps={cycles / max(now - started, 1e-9):.1f} age_ms={age * 1000:.1f} "
                    f"CH{elbow_config.sleeve_channel}={predictor.last_raw!r} "
                    f"normalized={predictor.last_normalized!r} filtered={predictor.last_filtered!r} "
                    f"absolute_angle={math.degrees(predictor.last_angle_rad or 0.0):.2f}deg "
                    f"target={target:.6f}rad startup_delta={math.degrees(target - elbow):.2f}deg "
                    f"invalid={invalid_samples} stale={stale_count}"
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
