from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .config import SUPPORTED_WINDOW_SIZES
from .types import MotionPhase


FEATURE_NAMES = (
    "f1_median",
    "f2_median",
    "f3_median",
    "f1_slope",
    "f2_slope",
    "f3_slope",
    "f2_minus_f3",
    "f1_plus_f3",
)


@dataclass(frozen=True)
class FeatureVector:
    values: np.ndarray
    current: np.ndarray
    medians: np.ndarray
    slopes: np.ndarray
    phase: MotionPhase
    timestamp_ns: int


class CausalFeatureBuffer:
    def __init__(self, window_size: int = 5, *, phase_slope_deadband: float = 0.02) -> None:
        if window_size not in SUPPORTED_WINDOW_SIZES:
            raise ValueError(f"window_size must be one of {SUPPORTED_WINDOW_SIZES}")
        if not math.isfinite(phase_slope_deadband) or phase_slope_deadband <= 0.0:
            raise ValueError("phase_slope_deadband must be finite and positive")
        self.window_size = int(window_size)
        self.phase_slope_deadband = float(phase_slope_deadband)
        self._values: deque[np.ndarray] = deque(maxlen=window_size)
        self._timestamps: deque[int] = deque(maxlen=window_size)

    @property
    def ready(self) -> bool:
        return len(self._values) == self.window_size

    @property
    def size(self) -> int:
        return len(self._values)

    @property
    def window(self) -> np.ndarray:
        if not self._values:
            return np.empty((0, 3), dtype=float)
        return np.asarray(self._values, dtype=float).copy()

    def append(self, values: Sequence[float], timestamp_ns: int) -> None:
        sample = np.asarray(values, dtype=float)
        if sample.shape != (3,) or not np.isfinite(sample).all():
            raise ValueError("Flex values must contain three finite numbers")
        if not isinstance(timestamp_ns, (int, np.integer)) or int(timestamp_ns) < 0:
            raise ValueError("timestamp_ns must be a non-negative integer")
        timestamp = int(timestamp_ns)
        if self._timestamps and timestamp <= self._timestamps[-1]:
            raise ValueError("timestamps must be strictly increasing")
        self._values.append(sample.copy())
        self._timestamps.append(timestamp)

    def extract(self) -> FeatureVector:
        if not self.ready:
            raise RuntimeError("a complete causal window is required")
        samples = np.asarray(self._values, dtype=float)
        timestamps = np.asarray(self._timestamps, dtype=np.int64)
        elapsed_s = (timestamps - timestamps[0]).astype(float) / 1e9
        if elapsed_s[-1] <= 0.0:
            raise RuntimeError("window timestamps have no positive duration")
        medians = np.median(samples, axis=0)
        slopes = np.asarray(
            [np.polyfit(elapsed_s, samples[:, sensor], 1)[0] for sensor in range(3)],
            dtype=float,
        )
        values = np.asarray(
            [
                *medians,
                *slopes,
                medians[1] - medians[2],
                medians[0] + medians[2],
            ],
            dtype=float,
        )
        phase = self._phase(medians, slopes)
        return FeatureVector(values, samples[-1].copy(), medians, slopes, phase, int(timestamps[-1]))

    def _phase(self, medians: np.ndarray, slopes: np.ndarray) -> MotionPhase:
        if float(np.linalg.norm(slopes)) <= self.phase_slope_deadband:
            return MotionPhase.HOLDING
        radial_velocity = float(np.dot(medians, slopes))
        if abs(radial_velocity) <= self.phase_slope_deadband:
            dominant = float(slopes[np.argmax(np.abs(slopes))])
            radial_velocity = dominant
        return MotionPhase.RAISING if radial_velocity > 0.0 else MotionPhase.LOWERING

    def reset(self) -> None:
        self._values.clear()
        self._timestamps.clear()


def select_features(vector: FeatureVector, feature_names: Sequence[str]) -> np.ndarray:
    available = dict(zip(FEATURE_NAMES, vector.values))
    available.update(
        {
            "f1_current": vector.current[0],
            "f2_current": vector.current[1],
            "f3_current": vector.current[2],
        }
    )
    try:
        selected = np.asarray([available[str(name)] for name in feature_names], dtype=float)
    except KeyError as exc:
        raise ValueError(f"unsupported runtime feature: {exc.args[0]}") from exc
    if not np.isfinite(selected).all():
        raise ValueError("selected runtime features must be finite")
    return selected
