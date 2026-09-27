from __future__ import annotations

from typing import Sequence

import numpy as np

from .calibration import FlexCalibration, RestThresholds


class MotionGate:
    def __init__(self, thresholds: RestThresholds) -> None:
        self.thresholds = thresholds

    @classmethod
    def from_calibration(cls, calibration: FlexCalibration) -> "MotionGate":
        return cls(calibration.rest_thresholds)

    def classify(self, window: Sequence[Sequence[float]], slopes: Sequence[float]) -> bool:
        samples = np.asarray(window, dtype=float)
        slope_values = np.asarray(slopes, dtype=float)
        if samples.ndim != 2 or samples.shape[1:] != (3,) or samples.shape[0] < 1:
            raise ValueError("window must have shape (n, 3)")
        if slope_values.shape != (3,):
            raise ValueError("slopes must contain three values")
        if not np.isfinite(samples).all() or not np.isfinite(slope_values).all():
            raise ValueError("Motion Gate inputs must be finite")
        magnitude = float(np.linalg.norm(samples[-1]))
        window_range = float(np.max(np.ptp(samples, axis=0)))
        slope_magnitude = float(np.linalg.norm(slope_values))
        return bool(
            magnitude > self.thresholds.magnitude
            or window_range > self.thresholds.window_range
            or slope_magnitude > self.thresholds.slope
        )
