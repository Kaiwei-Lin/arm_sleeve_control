#!/usr/bin/env python3
"""Offline manual model-output to SDK-request diagnostic; performs no I/O."""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_CONFIG_PATH, DEFAULT_PHASE3_CONFIG_PATH, load_phase3_config, load_robot_config
from sleeve_arm.control import ArmMapper
from sleeve_arm.control.safety import safe_target
from sleeve_arm.domain import ArmAction, JOINT_NAMES, MotionIntent
from sleeve_arm.robot.dymotor import semantic_to_sdk_position


def manual_intent(action: ArmAction, shoulder_angle_deg: float, elbow_angle_deg: float) -> MotionIntent:
    shoulder = math.radians(shoulder_angle_deg)
    flexion = shoulder if action is ArmAction.FORWARD else -shoulder if action is ArmAction.BACKWARD else 0.0
    abduction = shoulder if action is ArmAction.LATERAL else 0.0
    return MotionIntent(
        timestamp=0.0,
        elbow_flexion=math.radians(elbow_angle_deg),
        shoulder_flexion_rad=flexion,
        shoulder_abduction_rad=abduction,
        action=action,
        angle_deg=shoulder_angle_deg,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline model-output → mapper → safety → SDK request diagnostic.")
    parser.add_argument("--action", choices=("Forward", "Lateral", "Backward"), required=True)
    parser.add_argument("--shoulder-angle-deg", type=float, required=True)
    parser.add_argument("--elbow-angle-deg", type=float, required=True)
    parser.add_argument("--current-shoulder-flexion-deg", type=float, default=0.0)
    parser.add_argument("--current-shoulder-abduction-deg", type=float, default=0.0)
    parser.add_argument("--current-elbow-deg", type=float, default=0.0)
    parser.add_argument("--dt", type=float, help="one simulated control period; defaults to phase3 control_hz")
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--phase3-config", type=Path, default=DEFAULT_PHASE3_CONFIG_PATH)
    args = parser.parse_args()

    numeric = (
        args.shoulder_angle_deg, args.elbow_angle_deg,
        args.current_shoulder_flexion_deg, args.current_shoulder_abduction_deg,
        args.current_elbow_deg,
    )
    if not all(math.isfinite(value) for value in numeric):
        parser.error("all angles must be finite")
    if args.shoulder_angle_deg < 0:
        parser.error("--shoulder-angle-deg is a non-negative regression magnitude")

    phase3 = load_phase3_config(args.phase3_config)
    robot = load_robot_config(args.robot_config)
    dt = 1.0 / phase3.control_hz if args.dt is None else args.dt
    if not math.isfinite(dt) or dt <= 0:
        parser.error("--dt must be finite and positive")

    current = {
        "shoulder_flexion": math.radians(args.current_shoulder_flexion_deg),
        "shoulder_abduction": math.radians(args.current_shoulder_abduction_deg),
        "elbow_flexion": math.radians(args.current_elbow_deg),
    }
    action = ArmAction[(args.action.upper())]
    intent = manual_intent(action, args.shoulder_angle_deg, args.elbow_angle_deg)
    mapped = ArmMapper(phase3.elbow, current).map(intent)
    safe = {
        name: safe_target(robot.joints[name], current[name], mapped[name], dt)
        for name in JOINT_NAMES
    }

    print("OFFLINE ONLY: no Sleeve, model, serial port, bridge library, or robot was opened.\n")
    print(f"Manual model output: action={args.action}, shoulder_angle={args.shoulder_angle_deg:.6f} deg")
    print(f"Manual elbow absolute angle: {args.elbow_angle_deg:.6f} deg")
    print(f"Simulated control dt: {dt:.6f} s")
    sdk_requests: list[float] = []
    for name in JOINT_NAMES:
        joint = robot.joints[name]
        sdk = semantic_to_sdk_position(joint, safe[name])
        sdk_requests.append(sdk)
        print(f"\n{name} (motor {joint.motor_id}, CAN {joint.can_id})")
        print(f"  current semantic: {current[name]: .6f} rad ({math.degrees(current[name]): .3f} deg)")
        print(f"  mapper requested: {mapped[name]: .6f} rad ({math.degrees(mapped[name]): .3f} deg)")
        print(f"  safety output:    {safe[name]: .6f} rad ({math.degrees(safe[name]): .3f} deg)")
        print(f"  zero/direction:   zero={joint.zero_position!r}, direction={joint.direction:+d}")
        print(f"  SDK request:      {sdk: .6f} rad")
        print(f"  semantic limits:  [{joint.min_position!r}, {joint.max_position!r}] rad")
    print(f"\nSDK batch positions (joint order): {sdk_requests}")
    print("SDK batch mask: 0x7 (all three joints)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
