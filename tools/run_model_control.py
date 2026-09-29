#!/usr/bin/env python3
"""Compose the sensor, intent and robot runtimes for model control."""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.runtime.configuration import load_configs, parse_args
from sleeve_arm.runtime.intent_pipeline import create_intent_pipeline
from sleeve_arm.runtime.model_control import ModelControlApp
from sleeve_arm.runtime.robot_runtime import create_robot_runtime
from sleeve_arm.runtime.sensors import create_sensor_runtime
from sleeve_arm.runtime.telemetry import Telemetry


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        configs = load_configs(args)
        sensors = create_sensor_runtime(args, configs)
        intents = create_intent_pipeline(args, configs)
        robot = create_robot_runtime(args, configs)
    except Exception as exc:
        print(f"ERROR [INIT] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    app = ModelControlApp(
        sensors=sensors,
        intents=intents,
        robot=robot,
        phase3=configs.phase3,
        phase4=configs.phase4,
        duration=args.duration,
        telemetry=Telemetry(imu_debug=args.imu_debug, print_only=args.print_only),
    )
    return app.run()


if __name__ == "__main__":
    raise SystemExit(main())
