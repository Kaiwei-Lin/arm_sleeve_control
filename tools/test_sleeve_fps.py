#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_sensor_config
from sleeve_arm.sources import create_sleeve_source


def percentile(values: list[float], p: float) -> float:
    """简单计算百分位数。"""
    if not values:
        return 0.0

    values = sorted(values)

    index = (len(values) - 1) * p
    lower = int(index)
    upper = min(lower + 1, len(values) - 1)

    if lower == upper:
        return values[lower]

    weight = index - lower
    return values[lower] * (1 - weight) + values[upper] * weight


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure sleeve sensor effective frame rate without connecting the robot."
    )

    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_SENSOR_CONFIG_PATH,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Measurement duration in seconds (default: 30)",
    )

    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.001,
        help="latest() polling interval in seconds (default: 0.001)",
    )

    parser.add_argument(
        "--report-interval",
        type=float,
        default=1.0,
        help="Live FPS report interval in seconds (default: 1)",
    )

    args = parser.parse_args()

    if args.duration <= 0:
        parser.error("--duration must be positive")

    if args.poll_interval < 0:
        parser.error("--poll-interval must be >= 0")

    if args.report_interval <= 0:
        parser.error("--report-interval must be positive")

    source = None

    try:
        config = load_sensor_config(args.config)
        source = create_sleeve_source(config)
        source.start()

        print("Sleeve source started; robot is not connected.")
        print(f"Measurement duration: {args.duration:.1f} s")
        print(f"Polling interval:      {args.poll_interval * 1000:.3f} ms")
        print("Press Ctrl+C to stop early.")
        print()

        # Give the serial source a short warm-up period.
        time.sleep(1.0)

        start_time = time.perf_counter()
        end_time = start_time + args.duration

        last_frame_timestamp: float | None = None

        frame_count = 0
        frame_timestamps: list[float] = []

        report_start = start_time
        report_frame_count = 0

        while time.perf_counter() < end_time:
            frame = source.latest()

            if frame is not None:
                # latest() may repeatedly return the same frame.
                # Only count it when the timestamp changes.
                if frame.timestamp != last_frame_timestamp:
                    last_frame_timestamp = frame.timestamp

                    frame_count += 1
                    report_frame_count += 1
                    frame_timestamps.append(frame.timestamp)

            now = time.perf_counter()

            if now - report_start >= args.report_interval:
                elapsed = now - report_start
                live_fps = report_frame_count / elapsed

                stats = source.stats

                print(
                    f"[{now - start_time:6.1f}s] "
                    f"observed_fps={live_fps:7.2f} "
                    f"source_fps={stats.estimated_fps:7.2f} "
                    f"frames={frame_count:6d} "
                    f"invalid={stats.invalid_frames}"
                )

                report_start = now
                report_frame_count = 0

            if args.poll_interval > 0:
                time.sleep(args.poll_interval)

        actual_elapsed = time.perf_counter() - start_time

        # --------------------------------------------------
        # Calculate frame intervals from source timestamps
        # --------------------------------------------------

        intervals = [
            frame_timestamps[i] - frame_timestamps[i - 1]
            for i in range(1, len(frame_timestamps))
            if frame_timestamps[i] > frame_timestamps[i - 1]
        ]

        observed_fps = frame_count / actual_elapsed

        stats = source.stats

        print()
        print("=" * 60)
        print("Sleeve frame-rate measurement result")
        print("=" * 60)

        print(f"Duration:             {actual_elapsed:.3f} s")
        print(f"Observed frames:      {frame_count}")
        print(f"Observed FPS:         {observed_fps:.3f} Hz")
        print(f"Source estimated FPS: {stats.estimated_fps:.3f} Hz")
        print(f"Invalid frames:       {stats.invalid_frames}")

        if intervals:
            intervals_ms = [v * 1000.0 for v in intervals]

            avg_interval = statistics.mean(intervals_ms)
            std_interval = (
                statistics.stdev(intervals_ms)
                if len(intervals_ms) >= 2
                else 0.0
            )

            timestamp_fps = 1000.0 / avg_interval if avg_interval > 0 else 0.0

            print()
            print("Frame interval statistics")
            print("-" * 60)
            print(f"Timestamp-based FPS:  {timestamp_fps:.3f} Hz")
            print(f"Average interval:     {avg_interval:.3f} ms")
            print(f"Std deviation:        {std_interval:.3f} ms")
            print(f"Minimum interval:     {min(intervals_ms):.3f} ms")
            print(f"Maximum interval:     {max(intervals_ms):.3f} ms")
            print(f"P50 interval:         {percentile(intervals_ms, 0.50):.3f} ms")
            print(f"P95 interval:         {percentile(intervals_ms, 0.95):.3f} ms")
            print(f"P99 interval:         {percentile(intervals_ms, 0.99):.3f} ms")

        print("=" * 60)

        return 0

    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
        return 0

    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    finally:
        if source is not None:
            source.close()


if __name__ == "__main__":
    raise SystemExit(main())