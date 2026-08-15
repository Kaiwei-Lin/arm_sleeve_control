#!/usr/bin/env python3
"""Drive the normal mapper/safety/robot path from one manually supplied model output."""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_CONFIG_PATH, DEFAULT_PHASE3_CONFIG_PATH, load_phase3_config, load_robot_config
from sleeve_arm.control import ArmMapper, SafeArmController
from sleeve_arm.control.safety import clamp_position
from sleeve_arm.domain import ArmAction, JOINT_NAMES
from sleeve_arm.robot import DyMotorArm, FakeRobotArm
from sleeve_arm.robot.dymotor import semantic_to_sdk_position
from tools.debug_model_mapping import manual_intent


def main() -> int:
    parser = argparse.ArgumentParser(description="Manual model output through the production mapper/safety/robot path.")
    parser.add_argument("--action", choices=("Forward", "Lateral", "Backward"), required=True)
    parser.add_argument("--shoulder-angle-deg", type=float, required=True)
    parser.add_argument("--elbow-angle-deg", type=float, required=True)
    parser.add_argument(
        "--upper-arm-rotation-deg",
        type=float,
        help="absolute semantic ID24 rotation; omitted means hold startup position",
    )
    parser.add_argument("--robot", choices=("fake", "dymotor"), default="fake")
    parser.add_argument("--execute", action="store_true", help="Servo On and move; otherwise PVCT/preview only")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--tolerance-deg", type=float, default=0.5)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    parser.add_argument("--library", type=Path)
    args = parser.parse_args()

    if not all(math.isfinite(value) for value in (args.shoulder_angle_deg, args.elbow_angle_deg)):
        parser.error("angles must be finite")
    if args.upper_arm_rotation_deg is not None and not math.isfinite(args.upper_arm_rotation_deg):
        parser.error("--upper-arm-rotation-deg must be finite")
    if args.shoulder_angle_deg < 0:
        parser.error("--shoulder-angle-deg is a non-negative regression magnitude")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if not math.isfinite(args.tolerance_deg) or args.tolerance_deg <= 0:
        parser.error("--tolerance-deg must be finite and positive")

    phase3 = load_phase3_config(args.phase3_config)
    config = load_robot_config(args.robot_config)
    if args.robot == "dymotor" and args.execute:
        for name in JOINT_NAMES:
            joint = config.joints[name]
            if joint.zero_position is None or joint.min_position is None or joint.max_position is None:
                parser.error(f"{name} requires calibrated zero_position/min_position/max_position")

    action = ArmAction[args.action.upper()]
    intent = manual_intent(action, args.shoulder_angle_deg, args.elbow_angle_deg, args.upper_arm_rotation_deg)
    backend = FakeRobotArm(config) if args.robot == "fake" else DyMotorArm(config, args.library)
    controller = SafeArmController(backend, config)
    period = 1.0 / phase3.control_hz
    exit_code = 0
    try:
        controller.connect()
        if isinstance(backend, DyMotorArm):
            print(f"loaded_so: {backend.loaded_library_path}")
        states = controller.read_joint_states()
        startup = {name: state.position for name, state in states.items()}
        mapped = ArmMapper(phase3.elbow, startup).map(intent)
        # import pdb;pdb.set_trace()
        # if args.upper_arm_rotation_deg is not None:
        #     mapped["upper_arm_rotation"] = math.radians(args.upper_arm_rotation_deg)
        goals = {name: clamp_position(config.joints[name], mapped[name]) for name in JOINT_NAMES}
        first = controller.preview_positions(mapped, dt=period)

        upper_arm_output = (
            "hold startup" if args.upper_arm_rotation_deg is None
            else f"{args.upper_arm_rotation_deg:.3f} deg absolute"
        )
        print(f"Manual output: action={args.action}, shoulder={args.shoulder_angle_deg:.3f} deg, "
              f"elbow={args.elbow_angle_deg:.3f} deg, upper_arm_rotation={upper_arm_output}")
        for name in JOINT_NAMES:
            joint = config.joints[name]
            print(f"\n{name} (motor {joint.motor_id}, CAN {joint.can_id})")
            print(f"  feedback semantic: {startup[name]: .6f} rad ({math.degrees(startup[name]): .3f} deg)")
            print(f"  mapper target:     {mapped[name]: .6f} rad ({math.degrees(mapped[name]): .3f} deg)")
            print(f"  limit-clamped goal:{goals[name]: .6f} rad ({math.degrees(goals[name]): .3f} deg)")
            print(f"  first safe command:{first[name]: .6f} rad")
            print(f"  first SDK request: {semantic_to_sdk_position(joint, first[name]): .6f} rad")

        if not args.execute:
            print("\nDRY RUN: vendor startup used Servo On; no post-startup target was issued.")
        else:
            print("\nEXECUTE requested: enabling and sending the complete manual absolute targets through SafetyController.")
            controller.enable()
            controller.set_joint_positions(startup, dt=period)  # first command holds measured positions
            deadline = time.monotonic() + args.timeout
            tolerance = math.radians(args.tolerance_deg)
            last_print = 0.0
            while True:
                sent = controller.set_joint_positions(mapped, dt=period)
                now = time.monotonic()
                feedback = controller.read_joint_states()
                if now - last_print >= 0.5:
                    print("commanded:", {name: round(sent[name], 6) for name in JOINT_NAMES})
                    print("feedback: ", {name: round(feedback[name].position, 6) for name in JOINT_NAMES})
                    last_print = now
                if all(abs(feedback[name].position - goals[name]) <= tolerance for name in JOINT_NAMES):
                    print("Targets reached within tolerance; entering Servo Off.")
                    break
                if now >= deadline:
                    raise RuntimeError("manual target timeout before all joints reached tolerance")
                time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("\nInterrupted; entering safe shutdown.")
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
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
