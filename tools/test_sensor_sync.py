#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_sensor_config
from sleeve_arm.sources import FakeImuSource, FakeSleeveSource
from sleeve_arm.sync import SensorSynchronizer


def main() -> int:
    parser = argparse.ArgumentParser(description="Exercise sleeve-anchored timestamp synchronization.")
    parser.add_argument("--fake", action="store_true", help="required: no real sync runner exists in this diagnostic")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    args = parser.parse_args()
    if not args.fake:
        parser.error("use --fake; real sources are tested individually before combined recording")
    if args.samples <= 0:
        parser.error("--samples must be positive")
    config = load_sensor_config(args.config)
    sleeve, imu1, imu2 = FakeSleeveSource(), FakeImuSource(), FakeImuSource(timestamp_offset_s=0.005)
    sync = SensorSynchronizer(config.synchronization.max_time_delta_ms, config.synchronization.buffer_duration_ms, True, True)
    for source in (sleeve, imu1, imu2):
        source.start()
    try:
        last = None
        produced = 0
        while produced < args.samples:
            one, two, anchor = imu1.latest(), imu2.latest(), sleeve.latest()
            if one is not None:
                sync.add_imu1(one)
            if two is not None:
                sync.add_imu2(two)
            if anchor is not None and anchor.timestamp != last:
                sample = sync.synchronize(anchor)
                last = anchor.timestamp
                produced += 1
                d1 = None if sample.imu1 is None else (sample.imu1.timestamp - anchor.timestamp) * 1000
                d2 = None if sample.imu2 is None else (sample.imu2.timestamp - anchor.timestamp) * 1000
                print(f"sleeve={anchor.timestamp:.6f} imu1={None if sample.imu1 is None else f'{sample.imu1.timestamp:.6f}'} delta1_ms={d1} imu2={None if sample.imu2 is None else f'{sample.imu2.timestamp:.6f}'} delta2_ms={d2}")
            time.sleep(0.002)
    finally:
        for source in (imu2, imu1, sleeve):
            source.close()
    print(sync.stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
