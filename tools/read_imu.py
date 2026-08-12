#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_sensor_config
from sleeve_arm.sources import FakeImuSource, create_imu_source


def main() -> int:
    parser = argparse.ArgumentParser(description="Read one optional IMU without connecting the robot.")
    parser.add_argument("--imu", required=True, choices=("imu1", "imu2"))
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    args = parser.parse_args()
    source = None
    try:
        config = load_sensor_config(args.config)
        endpoint = getattr(config, args.imu)
        if args.fake:
            source = FakeImuSource()
        else:
            source = create_imu_source(args.imu, endpoint)
            if source is None:
                raise RuntimeError(f"real {args.imu} backend is disabled or not configured")
        source.start()
        print(f"{args.imu} source started; robot is not connected. Press Ctrl+C to stop.")
        last = None
        while True:
            frame = source.latest()
            if frame is not None and frame.timestamp != last:
                last = frame.timestamp
                print(f"t={frame.timestamp:.6f} accel=({frame.accel_x:.3f}, {frame.accel_y:.3f}, {frame.accel_z:.3f}) gyro=({frame.gyro_x:.3f}, {frame.gyro_y:.3f}, {frame.gyro_z:.3f})")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping IMU source...")
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())
