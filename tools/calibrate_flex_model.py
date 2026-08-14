#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import (
    DEFAULT_PHASE4_CONFIG_PATH,
    DEFAULT_SENSOR_CONFIG_PATH,
    load_phase4_config,
    load_sensor_config,
)
from sleeve_arm.sources import create_sleeve_source
from tools.run_model_control import prepare_flexarm_predictor


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Calibrate FlexArmEstimator CH3/CH4/CH5 without connecting a robot."
    )
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument("--output", type=Path, help="override predictor.calibration_file")
    parser.add_argument("--calibration-seconds", type=float)
    args = parser.parse_args(argv)
    if args.calibration_seconds is not None and args.calibration_seconds <= 0:
        parser.error("--calibration-seconds must be positive")

    source = None
    try:
        phase4 = load_phase4_config(args.phase4_config)
        if phase4.flex_model is None:
            raise ValueError("calibration requires predictor.backend=flexarm_estimator")
        source = create_sleeve_source(load_sensor_config(args.sensor_config))
        source.start()
        prepare_flexarm_predictor(
            source,
            phase4.flex_model,
            calibration_seconds=args.calibration_seconds,
            calibration_output=args.output,
        )
        return 0
    except (KeyboardInterrupt, EOFError):
        print("\nCalibration cancelled.")
        return 130
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
