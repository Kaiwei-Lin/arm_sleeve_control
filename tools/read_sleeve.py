#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_sensor_config
from sleeve_arm.sources import create_sleeve_source


def main() -> int:
    parser = argparse.ArgumentParser(description="Read the complete sleeve frame without connecting the robot.")
    parser.add_argument("--config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    parser.add_argument("--refresh", type=float, default=0.5)
    args = parser.parse_args()
    if args.refresh <= 0:
        parser.error("--refresh must be positive")
    source = None
    try:
        config = load_sensor_config(args.config)
        source = create_sleeve_source(config)
        source.start()
        print("Sleeve source started; robot is not connected. Press Ctrl+C to stop.")
        while True:
            frame = source.latest()
            stats = source.stats
            if frame is not None:
                print(f"timestamp={frame.timestamp:.6f} fps={stats.estimated_fps:.1f} channels={len(frame.channels)} invalid={stats.invalid_frames}")
                print(frame.channels)
            time.sleep(args.refresh)
    except KeyboardInterrupt:
        print("\nStopping sleeve source...")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
