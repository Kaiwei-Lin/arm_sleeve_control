#!/usr/bin/env python3
"""Single Bend5 glove control with optional forward arm pose; no Sleeve/IMU required."""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, PROJECT_ROOT, load_phase3_config, load_sensor_config
from sleeve_arm.runtime.configuration import ModelControlConfigs
from sleeve_arm.runtime.glove_control import GloveRuntime
from sleeve_arm.runtime.robot_runtime import RobotRuntime


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--glove", choices=("real", "fake"), default="real")
    parser.add_argument("--robot", choices=("aurora", "aurora-fake"), default="aurora")
    parser.add_argument("--aurora-profile", type=Path, default=PROJECT_ROOT / "configs/robot_aurora.yaml")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--execute", action="store_true", help="move the selected hand/optional arm pose after interactive YES")
    parser.add_argument("--arm-forward-deg", type=float, default=0.0,
                        help="absolute forward shoulder angle in degrees; 0 leaves the arm uncontrolled (default)")
    parser.add_argument("--arm-move-seconds", type=float, default=5.0,
                        help="minimum duration of the smooth arm lift (default: 5 seconds)")
    parser.add_argument("--prepare-aurora-fsm", action="store_true")
    parser.add_argument("--duration", type=float)
    parser.set_defaults(library=None, bridge_diagnostics=False, source_side=None)
    args = parser.parse_args(argv)
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be positive and finite")
    if args.execute and args.duration is None:
        parser.error("--execute requires a bounded --duration")
    if args.execute and args.robot == "aurora" and args.glove != "real":
        parser.error("real robot execution requires --glove real")
    if args.prepare_aurora_fsm and not args.execute:
        parser.error("--prepare-aurora-fsm requires --execute")
    if not math.isfinite(args.arm_forward_deg) or args.arm_forward_deg < 0:
        parser.error("--arm-forward-deg must be nonnegative and finite")
    if not math.isfinite(args.arm_move_seconds) or args.arm_move_seconds <= 0:
        parser.error("--arm-move-seconds must be positive and finite")
    return args


class ForwardArmPose:
    """Smooth a fixed shoulder target without involving the IMU intent mapper."""

    def __init__(self, robot, args):
        self.key = robot.robot.joint_key(args.side, "shoulder_flexion")
        self.limits = robot.robot.config.joints[self.key]
        self.target = math.radians(args.arm_forward_deg)
        if not self.limits.min_position <= self.target <= self.limits.max_position:
            raise ValueError("--arm-forward-deg exceeds the selected shoulder's calibrated limits")
        self.duration = args.arm_move_seconds
        self.elapsed = 0.0

    def start(self, startup, period):
        # Use the fresh pose captured AFTER operator confirmation/FSM preparation.
        self.initial = startup[self.key]
        speed = min(self.limits.max_velocity, self.limits.max_position_step / period)
        self.duration = max(self.duration, 1.5 * abs(self.target - self.initial) / speed)

    def advance(self, dt):
        self.elapsed = min(self.elapsed + dt, self.duration)
        alpha = self.elapsed / self.duration
        smooth = alpha * alpha * (3.0 - 2.0 * alpha)
        return {"shoulder_flexion": self.initial + (self.target - self.initial) * smooth}


def main(argv=None):
    args = parse_args(argv)
    glove = robot = None
    try:
        config = load_sensor_config(args.sensor_config)
        glove = GloveRuntime(config.glove, mode=args.glove)
        phase3 = load_phase3_config()
        arm_pose = None
        if args.execute or args.robot == "aurora-fake":
            configs = ModelControlConfigs(sensors=config, phase3=phase3, glove_mode=args.glove)
            parts = ("arm", "hand") if args.arm_forward_deg else ("hand",)
            robot = RobotRuntime(args, configs, parts=parts)
            robot.connect()
            if args.arm_forward_deg:
                arm_pose = ForwardArmPose(robot, args)
                print(f"ARM POSE: {args.side} shoulder forward to {args.arm_forward_deg:g} deg "
                      f"from neutral over at least {args.arm_move_seconds:g}s, then hold during glove control.")
        print("Bend5: keep fingers straight during startup calibration.")
        if robot is None:
            print("PRINT ONLY: no robot connection; showing glove and target radians.")
            if args.arm_forward_deg:
                print(f"Requested arm pose: {args.side} forward {args.arm_forward_deg:g} deg (NO MOTION).")
        glove.start()
        glove.wait_ready()
        if robot is not None and not robot.prepare(map_arm_intents=False):
            return 0
        period = 1.0 / phase3.control_hz
        if arm_pose is not None:
            arm_pose.start(robot.startup, period)
        started = time.monotonic()
        next_tick = started + period
        previous_tick = started
        last_print = started - 1.0
        while args.duration is None or time.monotonic() - started < args.duration:
            # prepare() already submitted the startup pose; allow one control
            # period before the first new hand/arm command.
            time.sleep(max(0.0, next_tick - time.monotonic()))
            if args.duration is not None and time.monotonic() - started >= args.duration:
                break
            targets = glove.targets()
            if robot is not None:
                robot.check_health()
                now = time.monotonic()
                # A delayed loop must not jump ahead along the lifting trajectory.
                arm_targets = None if arm_pose is None else arm_pose.advance(min(period, now - previous_tick))
                robot.apply_hand(targets, period, arm_targets=arm_targets)
                previous_tick = now
            if time.monotonic() - last_print >= 0.2:
                glove.report()
                print(f"[{args.side}_hand] target_rad={ {k: round(v, 3) for k, v in targets.items()} }")
                last_print = time.monotonic()
            next_tick += period
        return 0
    except KeyboardInterrupt:
        print("\nStopping on Ctrl+C...")
        return 0
    except Exception as exc:
        print(f"ERROR [GLOVE] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        for resource in (robot, glove):
            if resource is not None:
                try:
                    resource.shutdown() if resource is robot else resource.close()
                except BaseException as exc:
                    print(f"ERROR [STOPPING] {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
