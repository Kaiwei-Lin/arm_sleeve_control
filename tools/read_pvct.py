#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_CONFIG_PATH, load_robot_config
from sleeve_arm.control import SafeArmController
from sleeve_arm.domain import JOINT_NAMES
from sleeve_arm.robot import DyMotorArm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read three DyMotor PVCT streams without motion.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--interval", type=float, default=0.2, help="print interval in seconds")
    parser.add_argument("--library", type=Path, help="path to libdymotor_bridge.so")
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error("--interval must be positive")
    return args


def print_states(controller: SafeArmController) -> None:
    states = controller.read_joint_states()
    for name in JOINT_NAMES:
        joint = controller.config.joints[name]
        state = states[name]
        current = "unavailable (SDK Fast PVCT has no current field)" if state.current is None else state.current
        print(f"{name}:")
        print(f"  id: {joint.motor_id}")
        print(f"  can: {joint.can_id}")
        print(f"  position: {state.position:.6f} rad")
        print(f"  velocity: {state.velocity:.6f} rad/s")
        print(f"  current: {current}")
        print(f"  torque: {state.torque:.6f} N.m")
        print(f"  state: {state.state}")
        print(f"  error: {state.error}")
        print()


def main() -> int:
    args = parse_args()
    config = load_robot_config(args.config)
    controller = SafeArmController(DyMotorArm(config, args.library), config)
    exit_code = 0
    try:
        controller.connect()
        print("Connected; Servo remains OFF. Press Ctrl+C to stop.\n")
        while True:
            print_states(controller)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping PVCT readout...")
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
