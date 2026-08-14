from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import time
from typing import Any

import numpy as np


def collect_calibration_samples(
    source: Any,
    channels: tuple[int, int, int],
    duration_s: float,
    *,
    monotonic: Callable[[], float] = time.monotonic,
) -> np.ndarray:
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    if len(channels) != 3 or any(channel < 1 for channel in channels):
        raise ValueError("channels must contain three positive one-based indexes")

    indexes = tuple(channel - 1 for channel in channels)
    deadline = monotonic() + duration_s
    last_timestamp: float | None = None
    rows: list[tuple[float, float, float]] = []
    while monotonic() < deadline:
        frame = source.latest()
        if frame is None or frame.timestamp == last_timestamp:
            time.sleep(0.001)
            continue
        last_timestamp = frame.timestamp
        if max(indexes) >= len(frame.channels):
            continue
        values = tuple(float(frame.channels[index]) for index in indexes)
        if not all(math.isfinite(value) for value in values):
            continue
        rows.append(values)

    if len(rows) < 2:
        raise RuntimeError(
            f"calibration received only {len(rows)} valid samples; check the Sleeve connection"
        )
    return np.asarray(rows, dtype=float)


def calibrate_estimator(estimator: Any, rows: np.ndarray, output: Path) -> Any:
    calibration = estimator.calibrate(rows)
    calibration.save(output)
    estimator.reset()
    return calibration
