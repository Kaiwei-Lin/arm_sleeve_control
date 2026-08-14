#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_CONFIG_PATH, load_robot_config
from sleeve_arm.control import SafeArmController
from sleeve_arm.domain import JointCommand
from sleeve_arm.robot import DyMotorArm


THREE_JOINT_NAMES = ("shoulder_flexion", "shoulder_abduction", "elbow_flexion")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dry-run or execute one small three-joint batch move.")
    parser.add_argument("--shoulder-flexion-delta-deg", type=float, default=0.0)
    parser.add_argument("--shoulder-abduction-delta-deg", type=float, default=0.0)
    parser.add_argument("--elbow-delta-deg", type=float, default=0.0)
    parser.add_argument("--execute", action="store_true", help="actually Servo On and send the batch")
    parser.add_argument("--monitor-seconds", type=float, default=1.0)
    parser.add_argument("--command-dt", type=float, default=0.05, help="safety control period in seconds")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--library", type=Path, help="path to libdymotor_bridge.so")
    args = parser.parse_args()
    deltas = (
        args.shoulder_flexion_delta_deg,
        args.shoulder_abduction_delta_deg,
        args.elbow_delta_deg,
    )
    if not all(math.isfinite(value) for value in deltas):
        parser.error("all delta values must be finite")
    if not math.isfinite(args.monitor_seconds) or args.monitor_seconds < 0:
        parser.error("--monitor-seconds must be finite and non-negative")
    if not math.isfinite(args.command_dt) or args.command_dt <= 0:
        parser.error("--command-dt must be finite and positive")
    return args


def main() -> int:
    args = parse_args()
    config = load_robot_config(args.config)
    controller = SafeArmController(DyMotorArm(config, args.library), config)
    print(f"Config: {config}")
    print(f"Library path: {args.library}")
    exit_code = 0
    try:
        controller.connect()
        states = controller.read_joint_states()
        command = JointCommand(
            shoulder_flexion=states["shoulder_flexion"].position
            + math.radians(args.shoulder_flexion_delta_deg),
            shoulder_abduction=states["shoulder_abduction"].position
            + math.radians(args.shoulder_abduction_delta_deg),
            elbow_flexion=states["elbow_flexion"].position
            + math.radians(args.elbow_delta_deg),
        )
        targets = controller.preview_positions(command.as_dict(), dt=args.command_dt)

        print("Three-joint relative move preview (one SDK batch):")
        for name in THREE_JOINT_NAMES:
            joint = config.joints[name]
            print(f"\n{name} (motor {joint.motor_id}, CAN {joint.can_id})")
            print(f"  current: {states[name].position:.6f} rad")
            print(f"  requested: {command.as_dict()[name]:.6f} rad")
            print(f"  safety-limited: {targets[name]:.6f} rad")
            print(f"  direction status: {joint.direction_status}")

        if not args.execute:
            print("\nDRY-RUN: no Servo On and no motion command were issued.")
            print("Joint directions, zero positions, and limits require hardware validation.")
        else:
            print("\nEXECUTE requested: sending all three targets with one batch flush.")
            controller.enable()
            sent = controller.set_joint_positions(targets, dt=args.command_dt)
            for name in THREE_JOINT_NAMES:
                print(f"Sent {name}: {sent[name]:.6f} rad")
            final = controller.monitor(args.monitor_seconds)
            for name in THREE_JOINT_NAMES:
                print(f"Final {name}: {final[name].position:.6f} rad; error={final[name].error}")
    except KeyboardInterrupt:
        print("\nInterrupted; entering safe shutdown.")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        try:
            controller.shutdown()
        except BaseException as exc:
            print(f"SHUTDOWN ERROR: {exc}", file=sys.stderr)
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
