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
    assert intent.action is not None and intent.angle_deg is not None
    print(f"Flex: {list(predictor.last_flex or ())}")
    print(f"Action: {intent.action.name.title()} (raw={int(predictor.last_raw_action)})")
    print(f"Probabilities: {list(intent.action_probabilities or ())}")
    print(f"Confidence: {intent.confidence:.6f}")
    print(f"Angle: {intent.angle_deg:.6f} deg / {math.radians(intent.angle_deg):.6f} rad")
    print(f"Inference: {intent.inference_ms:.3f} ms\n")


def replay(path: Path, predictor: FlexModelPredictor) -> int:
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = csv.DictReader(stream)
        required = {"timestamp", "sleeve_ch_1", "sleeve_ch_2", "sleeve_ch_3"}
        if rows.fieldnames is None or not required.issubset(rows.fieldnames):
            raise ValueError(f"recording must contain: {', '.join(sorted(required))}")
        count = 0
        for row in rows:
            channels = tuple(float(row[f"sleeve_ch_{index}"]) for index in range(len(rows.fieldnames)) if f"sleeve_ch_{index}" in row)
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
    args = parser.parse_args()
    if args.refresh <= 0:
        parser.error("--refresh must be positive")
    source = None
    try:
        config = load_phase4_config(args.phase4_config)
        if config.flex_model is None:
            raise ValueError("test_flex_model requires predictor.backend=flex_model")
        predictor = FlexModelPredictor(config.flex_model)
        if args.input is not None:
            return replay(args.input, predictor)
        source = create_sleeve_source(load_sensor_config(args.sensor_config))
        source.start()
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
