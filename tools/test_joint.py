#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_CONFIG_PATH, load_robot_config
from sleeve_arm.control import SafeArmController
from sleeve_arm.domain import JOINT_NAMES
from sleeve_arm.robot import DyMotorArm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Dry-run or execute one small relative joint move.")
    parser.add_argument("--joint", required=True, choices=JOINT_NAMES)
    parser.add_argument("--delta-deg", required=True, type=float)
    parser.add_argument("--execute", action="store_true", help="actually Servo On and send the move")
    parser.add_argument("--monitor-seconds", type=float, default=1.0)
    parser.add_argument("--command-dt", type=float, default=0.05, help="safety control period in seconds")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--library", type=Path, help="path to libdymotor_bridge.so")
    args = parser.parse_args()
    if not math.isfinite(args.delta_deg):
        parser.error("--delta-deg must be finite")
    if not math.isfinite(args.monitor_seconds) or args.monitor_seconds < 0:
        parser.error("--monitor-seconds must be finite and non-negative")
    if not math.isfinite(args.command_dt) or args.command_dt <= 0:
        parser.error("--command-dt must be finite and positive")
    return args


def main() -> int:
    args = parse_args()
    config = load_robot_config(args.config)
    controller = SafeArmController(DyMotorArm(config, args.library), config)
    exit_code = 0
    try:
        controller.connect(prepare_feedback=args.execute)
        state = controller.read_joint_state(args.joint)
        joint = config.joints[args.joint]
        requested = state.position + math.radians(args.delta_deg)
        target = controller.preview_positions({args.joint: requested}, dt=args.command_dt)[args.joint]

        print(f"Joint: {args.joint}")
        print(f"Motor ID: {joint.motor_id}")
        print(f"CAN ID: {joint.can_id}")
        print(f"Direction status: {joint.direction_status}")
        print()
        print(f"Current position: {state.position:.6f} rad")
        print(f"Requested target: {requested:.6f} rad")
        print(f"Target position: {target:.6f} rad (after safety limits)")
        print(f"Delta: {args.delta_deg:.6f} deg (requested)")

        if not args.execute:
            print("\nDRY-RUN: no Servo On and no motion command were issued.")
            print("Re-run with --execute only after checking the setup and emergency stop.")
        else:
            print("\nEXECUTE requested: direction/zero/limits still require hardware validation.")
            controller.enable()
            sent = controller.set_joint_position(args.joint, target, dt=args.command_dt)
            print(f"Sent target: {sent:.6f} rad")
            final = controller.monitor(args.monitor_seconds)[args.joint]
            print(f"Final feedback: {final.position:.6f} rad; error={final.error}")
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
