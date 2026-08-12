from __future__ import annotations

import csv
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any, TextIO

import yaml

from sleeve_arm.domain.sensor import ImuFrame, SensorSample


IMU_FIELDS = ("ax", "ay", "az", "gx", "gy", "gz", "qw", "qx", "qy", "qz")


class SensorRecorder:
    def __init__(self, output_root: str | Path, channel_count: int, metadata: dict[str, Any] | None = None, session_name: str | None = None) -> None:
        if channel_count <= 0:
            raise ValueError("channel_count must be positive")
        self.channel_count = channel_count
        root = Path(output_root).expanduser().resolve()
        name = session_name or datetime.now().astimezone().strftime("%Y%m%d_%H%M%S_%f")
        self.session_dir = root / name
        self.csv_path = self.session_dir / "samples.csv"
        self.metadata_path = self.session_dir / "metadata.yaml"
        self.samples_written = 0
        self._file: TextIO | None = None
        self._writer: csv.writer | None = None
        self._metadata = dict(metadata or {})

    def start(self) -> Path:
        if self._file is not None:
            return self.session_dir
        self.session_dir.mkdir(parents=True, exist_ok=False)
        self._file = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        header = ["timestamp", *(f"sleeve_ch_{i}" for i in range(self.channel_count))]
        header.extend(f"imu1_{field}" for field in IMU_FIELDS)
        header.extend(f"imu2_{field}" for field in IMU_FIELDS)
        self._writer.writerow(header)
        metadata = {
            "recording_started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "software": {"git_commit": self._git_commit()},
            "channel_count": self.channel_count,
            **self._metadata,
        }
        with self.metadata_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False, allow_unicode=True)
        return self.session_dir

    def write(self, sample: SensorSample) -> None:
        if self._writer is None:
            raise RuntimeError("recorder is not started")
        if len(sample.sleeve.channels) != self.channel_count:
            raise ValueError("sleeve channel count changed during recording")
        self._writer.writerow([sample.timestamp, *sample.sleeve.channels, *self._imu_values(sample.imu1), *self._imu_values(sample.imu2)])
        self.samples_written += 1

    def flush(self) -> None:
        if self._file is not None:
            self._file.flush()

    def close(self) -> None:
        if self._file is not None:
            self._file.flush()
            self._file.close()
        self._file = None
        self._writer = None

    @staticmethod
    def _imu_values(frame: ImuFrame | None) -> tuple[float | str, ...]:
        if frame is None:
            return ("",) * len(IMU_FIELDS)
        return (
            frame.accel_x, frame.accel_y, frame.accel_z,
            frame.gyro_x, frame.gyro_y, frame.gyro_z,
            "" if frame.quat_w is None else frame.quat_w,
            "" if frame.quat_x is None else frame.quat_x,
            "" if frame.quat_y is None else frame.quat_y,
            "" if frame.quat_z is None else frame.quat_z,
        )

    @staticmethod
    def _git_commit() -> str | None:
        try:
            result = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True, check=False)
        except OSError:
            return None
        return result.stdout.strip() or None

    def __enter__(self) -> SensorRecorder:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_value: object, traceback: object) -> None:
        self.close()
