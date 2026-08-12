#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, SensorConfig, load_sensor_config
from sleeve_arm.recording import SensorRecorder
from sleeve_arm.sources import FakeImuSource, FakeSleeveSource, create_imu_source, create_sleeve_source
from sleeve_arm.sync import SensorSynchronizer


def metadata(config: SensorConfig, fake: bool) -> dict[str, object]:
    def endpoint(name: str) -> dict[str, object]:
        item = getattr(config, name)
        return {
            "enabled": True if fake else item.enabled,
            "backend": "fake" if fake else item.backend,
            "port": None if fake else item.port,
            "baudrate": item.baudrate,
        }
    return {
        "sleeve": endpoint("sleeve"), "imu1": endpoint("imu1"), "imu2": endpoint("imu2"),
        "synchronization": {
            "max_time_delta_ms": config.synchronization.max_time_delta_ms,
            "buffer_duration_ms": config.synchronization.buffer_duration_ms,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Record SensorSamples without connecting the robot.")
    parser.add_argument("--fake", action="store_true")
    parser.add_argument("--duration", type=float, default=None, help="optional recording duration in seconds")
    parser.add_argument("--config", type=Path, default=DEFAULT_SENSOR_CONFIG_PATH)
    args = parser.parse_args()
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be positive")
    config = load_sensor_config(args.config)
    if args.fake:
        sleeve = FakeSleeveSource()
        imu1 = FakeImuSource()
        imu2 = FakeImuSource(timestamp_offset_s=0.005)
        imu1_enabled = imu2_enabled = True
    else:
        sleeve = create_sleeve_source(config)
        imu1 = create_imu_source("imu1", config.imu1)
        imu2 = create_imu_source("imu2", config.imu2)
        imu1_enabled, imu2_enabled = imu1 is not None, imu2 is not None
    sources = [source for source in (sleeve, imu1, imu2) if source is not None]
    sync = SensorSynchronizer(config.synchronization.max_time_delta_ms, config.synchronization.buffer_duration_ms, imu1_enabled, imu2_enabled)
    recorder: SensorRecorder | None = None
    try:
        for source in sources:
            source.start()
        deadline = None if args.duration is None else time.monotonic() + args.duration
        last_sleeve = None
        started = time.monotonic()
        while deadline is None or time.monotonic() < deadline:
            one = None if imu1 is None else imu1.latest()
            two = None if imu2 is None else imu2.latest()
            if one is not None:
                sync.add_imu1(one)
            if two is not None:
                sync.add_imu2(two)
            anchor = sleeve.latest()
            if anchor is not None and anchor.timestamp != last_sleeve:
                if recorder is None:
                    recorder = SensorRecorder(config.recording.output_dir, len(anchor.channels), metadata(config, args.fake))
                    recorder.start()
                    print(f"recording directory: {recorder.session_dir}")
                recorder.write(sync.synchronize(anchor))
                last_sleeve = anchor.timestamp
                if recorder.samples_written % 50 == 0:
                    elapsed = max(time.monotonic() - started, 1e-9)
                    misses = sync.stats.imu1_missed + sync.stats.imu2_missed
                    print(f"samples={recorder.samples_written} rate={recorder.samples_written / elapsed:.1f} Hz unsynchronized={misses}")
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\nStopping recording...")
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        if recorder is not None:
            recorder.close()
        for source in reversed(sources):
            source.close()
    if recorder is None:
        print("No sleeve frame received; no recording was created.", file=sys.stderr)
        return 1
    print(f"samples written: {recorder.samples_written}")
    print(f"sync stats: {sync.stats}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
