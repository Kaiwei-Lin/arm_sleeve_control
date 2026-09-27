from __future__ import annotations

from typing import Sequence

import numpy as np

from .calibration import FlexCalibration


class FlexNormalizer:
    def __init__(self, calibration: FlexCalibration, *, eps: float = 1e-9) -> None:
        if not np.isfinite(eps) or eps <= 0.0:
            raise ValueError("eps must be finite and positive")
        self.calibration = calibration
        self._baseline = np.asarray(calibration.baseline, dtype=float)
        self._scale = np.asarray(calibration.scale, dtype=float)
        self._eps = float(eps)

    def normalize(self, raw: Sequence[float]) -> np.ndarray:
        values = np.asarray(raw, dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all():
            raise ValueError("raw Flex input must contain three finite values")
        return (values - self._baseline) / (self._scale + self._eps)

    def normalize_many(self, raw_samples: Sequence[Sequence[float]]) -> np.ndarray:
        values = np.asarray(raw_samples, dtype=float)
        if values.ndim != 2 or values.shape[1:] != (3,) or not np.isfinite(values).all():
            raise ValueError("raw Flex samples must have shape (n, 3) and be finite")
        return (values - self._baseline) / (self._scale + self._eps)

