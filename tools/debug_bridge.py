#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_CONFIG_PATH, load_robot_config
from sleeve_arm.domain import JOINT_NAMES
from sleeve_arm.robot import DyMotorArm


def main() -> int:
    parser = argparse.ArgumentParser(description="Read raw DyMotor bridge PVCT without Servo On or motion.")
    parser.add_argument("--joint", choices=("all", *JOINT_NAMES), default="all")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--library", type=Path, help="exact libdymotor_bridge.so to load")
    args = parser.parse_args()
    if args.samples < 1 or args.interval < 0:
        parser.error("--samples must be >= 1 and --interval must be >= 0")

    config = load_robot_config(args.config)
    robot = DyMotorArm(config, args.library)
    try:
        robot.connect()
        robot.set_bridge_diagnostics(True)
        print(f"loaded_so: {robot.loaded_library_path}")
        names = JOINT_NAMES if args.joint == "all" else (args.joint,)
        success = True
        for sample in range(1, args.samples + 1):
            for name in names:
                joint = config.joints[name]
                raw = robot.read_bridge_joint_state(name)
                print(f"sample: {sample}")
                print(f"joint: {name}")
                print(f"motor_id: {joint.motor_id}")
                print(f"can_id: {joint.can_id}")
                print(f"wrapper_status: {raw.wrapper_status}")
                print(f"position: {raw.position}")
                print(f"velocity: {raw.velocity}")
                print(f"current: {'unavailable' if raw.current is None else raw.current}")
                print(f"torque: {raw.torque}")
                print(f"state: {raw.state} (0x{raw.state:08X})")
                print(f"bus: {raw.bus}")
                print(f"motor_error: {raw.motor_error_code} (0x{raw.motor_error_code:08X})")
                print()
                success = success and raw.wrapper_status == 0 and raw.motor_error_code == 0
            if sample < args.samples:
                time.sleep(args.interval)
        return 0 if success else 1
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        # A never-enabled session closes without any Servo or position command.
        robot.close()


if __name__ == "__main__":
    raise SystemExit(main())
