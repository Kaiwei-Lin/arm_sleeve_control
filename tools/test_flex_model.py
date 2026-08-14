#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_PHASE4_CONFIG_PATH, DEFAULT_SENSOR_CONFIG_PATH, load_phase4_config, load_sensor_config
from sleeve_arm.domain import SensorSample, SleeveFrame
from sleeve_arm.predictor import FlexModelPredictor
from sleeve_arm.sources import create_sleeve_source


def print_prediction(predictor: FlexModelPredictor, intent) -> None:
    assert intent.model_action is not None and intent.angle_deg is not None
    print(f"Flex: {list(predictor.last_flex or ())}")
    print(f"Action: {intent.model_action} moving={intent.moving}")
    print(f"Action confidence: {intent.confidence:.6f}")
    print(f"Angle confidence: {intent.angle_confidence:.6f}")
    print(f"Angle: {intent.angle_deg:.6f} deg / {math.radians(intent.angle_deg):.6f} rad")
    print(f"Inference: {intent.inference_ms:.3f} ms\n")


def replay(path: Path, predictor: FlexModelPredictor) -> int:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        required = {"timestamp", "sleeve_ch_3", "sleeve_ch_4", "sleeve_ch_5"}
        if rows.fieldnames is None or not required.issubset(rows.fieldnames):
            raise ValueError(f"recording must contain: {', '.join(sorted(required))}")
        count = 0
        for row in rows:
            channel_fields = sorted(
                (name for name in rows.fieldnames if name.startswith("sleeve_ch_")),
                key=lambda name: int(name.rsplit("_", 1)[1]),
            )
            channels = tuple(float(row[name]) for name in channel_fields)
            sample = SensorSample(float(row["timestamp"]), SleeveFrame(float(row["timestamp"]), channels))
            print_prediction(predictor, predictor.predict(sample))
            count += 1
    print(f"replayed samples: {count}")
    return 0 if count else 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Run FlexPredictor without connecting any robot.")
    parser.add_argument("--input", type=Path, help="Phase 2 samples.csv; skips serial Sleeve")
    parser.add_argument("--refresh", type=float, default=0.5)
    parser.add_argument("--sensor-config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--phase4-config", type=Path, default=DEFAULT_PHASE4_CONFIG_PATH)
    parser.add_argument("--reuse-calibration", action="store_true")
    parser.add_argument("--calibration-seconds", type=float)
    parser.add_argument("--calibration-output", type=Path)
    args = parser.parse_args()
    if args.refresh <= 0:
        parser.error("--refresh must be positive")
    source = None
    try:
        config = load_phase4_config(args.phase4_config)
        if config.flex_model is None:
            raise ValueError("test_flex_model requires predictor.backend=flexarm_estimator")
        if args.input is not None:
            if not args.reuse_calibration:
                raise ValueError("offline replay requires --reuse-calibration")
            predictor = FlexModelPredictor(config.flex_model)
            predictor.reuse_calibration(
                config.flex_model.calibration_file
                if args.calibration_output is None
                else args.calibration_output
            )
            return replay(args.input, predictor)
        source = create_sleeve_source(load_sensor_config(args.sensor_config))
        source.start()
        from tools.run_model_control import prepare_flexarm_predictor

        predictor = prepare_flexarm_predictor(
            source,
            config.flex_model,
            reuse_calibration=args.reuse_calibration,
            calibration_seconds=args.calibration_seconds,
            calibration_output=args.calibration_output,
        )
        last_timestamp = None
        while True:
            frame = source.latest()
            if frame is not None and frame.timestamp != last_timestamp:
                print_prediction(predictor, predictor.predict(SensorSample(frame.timestamp, frame)))
                last_timestamp = frame.timestamp
            time.sleep(args.refresh)
    except KeyboardInterrupt:
        print("\nStopping model test...")
        return 0
    except Exception as exc:
        print(f"ERROR: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
