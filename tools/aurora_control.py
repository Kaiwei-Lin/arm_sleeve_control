#!/usr/bin/env python3
"""Offline by default; real Aurora reads and execution are explicitly separated."""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def parser():
    p = argparse.ArgumentParser(description="Aurora angular control; default backend is offline fake")
    p.add_argument("--backend", choices=("fake", "aurora"), default="fake")
    p.add_argument("--profile", type=Path)
    p.add_argument("--execute", action="store_true", help="authorize real command endpoints and lease acquisition")
    p.add_argument("--confirm", choices=("EXECUTE_AURORA",), help="explicit operator confirmation for real execution")
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("doctor", help="read-only raw identity/FSM/groups/freshness; never takes a lease")
    for name in ("move", "joint", "swing", "hand-joints"):
        cmd = sub.add_parser(name)
        cmd.add_argument("--side", choices=("left", "right"), required=True)
        if name == "swing":
            cmd.add_argument("--joint", required=True)
            cmd.add_argument("--amplitude-deg", type=float, required=True)
            cmd.add_argument("--period", type=float, required=True)
            cmd.add_argument("--cycles", type=int, required=True)
            cmd.add_argument("--center-rad", type=float, help="default: current; explicit center is semantic radians")
            cmd.add_argument("--settle-duration", type=float, default=2.0)
        else:
            cmd.add_argument("--duration", type=float, default=2.0)
            if name == "hand-joints":
                cmd.add_argument("--angle", nargs=2, action="append", metavar=("JOINT", "DEG"), required=True)
            else:
                cmd.add_argument("--angle-deg", type=float, required=True)
                cmd.add_argument("--reference", choices=("neutral", "current"), default="neutral")
                if name == "move":
                    cmd.add_argument("--direction", choices=("forward", "backward", "outward", "inward"), required=True)
                else:
                    cmd.add_argument("--joint", required=True)
    return p


def main(argv=None):
    p = parser()
    args = p.parse_args(argv)
    if args.command == "doctor" and args.execute:
        p.error("doctor is always read-only; remove --execute")
    if args.backend == "aurora" and args.profile is None:
        p.error("--backend aurora requires --profile")
    if args.backend == "aurora" and args.execute and args.confirm != "EXECUTE_AURORA":
        p.error("real execute requires --confirm EXECUTE_AURORA after operator review")
    from sleeve_arm.control.controller import SafeArmController
    from sleeve_arm.control.upper_limb import UpperLimbService
    from sleeve_arm.robot.aurora_fake import fake_session
    from sleeve_arm.robot.aurora_profile import load_aurora_profile
    from sleeve_arm.robot.aurora_session import AuroraSession
    from sleeve_arm.robot.factory import create_robot
    controller = session = None
    token = object()
    attached = False
    result_code = 0
    try:
        execute = args.command != "doctor" and (args.backend == "fake" or args.execute)
        if args.backend == "fake":
            session = fake_session(execute=execute, hands=args.command == "hand-joints")
        else:
            profile = load_aurora_profile(args.profile)
            session = AuroraSession.real(profile, execute=execute, operator_confirmed=args.confirm == "EXECUTE_AURORA")
        if not execute:
            session.attach(token, set())
            attached = True
            print(json.dumps(session.doctor(), indent=2, ensure_ascii=False))
            if args.command != "doctor":
                print("READ ONLY: requested motion was not executed or claimed complete. No lease/FSM/command publisher.")
        else:
            robot = create_robot("aurora-fake" if args.backend == "fake" else "aurora", session=session,
                                 side=args.side, parts=("hand",) if args.command == "hand-joints" else ("arm",))
            controller = SafeArmController(robot, robot.config, clock=session.clock)
            controller.connect()
            controller.enable()
            service = UpperLimbService(controller)
            if args.command == "move":
                result = service.move_arm(side=args.side, direction=args.direction, angle_deg=args.angle_deg,
                                          reference=args.reference, duration_s=args.duration)
            elif args.command == "joint":
                result = service.move_joint(side=args.side, joint=args.joint, angle_deg=args.angle_deg,
                                            reference=args.reference, duration_s=args.duration)
            elif args.command == "swing":
                result = service.swing_arm(side=args.side, joint=args.joint, amplitude_deg=args.amplitude_deg,
                                           period_s=args.period, cycles=args.cycles,
                                           center="current" if args.center_rad is None else args.center_rad,
                                           settle_duration_s=args.settle_duration)
            else:
                angles = dict(args.angle)
                if len(angles) != len(args.angle):
                    raise ValueError("duplicate hand joint")
                result = service.hand_joints(side=args.side, angles_deg={n: float(v) for n, v in angles.items()},
                                             duration_s=args.duration)
            print(json.dumps({"simulation": args.backend == "fake", **asdict(result)}, indent=2))
    except KeyboardInterrupt:
        print("Interrupted: application publishing is ending; physical stop is not confirmed.", file=sys.stderr)
        result_code = 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        result_code = 1
    finally:
        try:
            if controller is not None:
                controller.shutdown()
            elif attached:
                session.detach(token)
        except BaseException as exc:
            print(f"ERROR: cleanup failed: {exc}", file=sys.stderr)
            result_code = 1
    return result_code


if __name__ == "__main__":
    raise SystemExit(main())
