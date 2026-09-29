#!/usr/bin/env python3
"""Single Bend5 glove preview or Aurora hand control; no Sleeve/IMU required."""
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
    parser.add_argument("--execute", action="store_true", help="move the selected real hand after interactive YES")
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
    return args


def main(argv=None):
    args = parse_args(argv)
    glove = robot = None
    try:
        config = load_sensor_config(args.sensor_config)
        glove = GloveRuntime(config.glove, mode=args.glove)
        phase3 = load_phase3_config()
        if args.execute or args.robot == "aurora-fake":
            configs = ModelControlConfigs(sensors=config, phase3=phase3, glove_mode=args.glove)
            robot = RobotRuntime(args, configs, parts=("hand",))
            robot.connect()
        print("Bend5: keep fingers straight during startup calibration.")
        if robot is None:
            print("PRINT ONLY: no robot connection; showing glove and target radians.")
        glove.start()
        glove.wait_ready()
        if robot is not None and not robot.prepare():
            return 0
        period = 1.0 / phase3.control_hz
        started = next_tick = time.monotonic()
        last_print = started - 1.0
        while args.duration is None or time.monotonic() - started < args.duration:
            targets = glove.targets()
            if robot is not None:
                robot.check_health()
                robot.apply_hand(glove.targets(), period)
            if time.monotonic() - last_print >= 0.2:
                glove.report()
                print(f"[{args.side}_hand] target_rad={ {k: round(v, 3) for k, v in targets.items()} }")
                last_print = time.monotonic()
            next_tick += period
            time.sleep(max(0.0, next_tick - time.monotonic()))
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
