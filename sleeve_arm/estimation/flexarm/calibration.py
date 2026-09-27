from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Sequence

import numpy as np


def _robust_limit(values: np.ndarray, multiplier: float, floor: float) -> float:
    center = float(np.median(values))
    mad = float(np.median(np.abs(values - center)))
    return max(float(floor), center + multiplier * 1.4826 * mad)


@dataclass(frozen=True)
class RestThresholds:
    magnitude: float
    window_range: float
    slope: float
    phase_slope_deadband: float

    def __post_init__(self) -> None:
        values = (self.magnitude, self.window_range, self.slope, self.phase_slope_deadband)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("Rest thresholds must be finite and positive")


@dataclass(frozen=True)
class FlexCalibration:
    baseline: tuple[float, float, float]
    scale: tuple[float, float, float]
    rest_thresholds: RestThresholds
    sample_count: int
    algorithm_version: str = "flexarm-calibration-v1"

    @classmethod
    def fit(
        cls,
        raw_samples: Sequence[Sequence[float]],
        *,
        scales: Sequence[float] | None = None,
        threshold_multiplier: float = 6.0,
        threshold_floor: float = 0.02,
    ) -> "FlexCalibration":
        samples = np.asarray(raw_samples, dtype=float)
        if samples.ndim != 2 or samples.shape[1:] != (3,) or samples.shape[0] < 2:
            raise ValueError("raw_samples must have shape (n, 3) with n >= 2")
        if not np.isfinite(samples).all():
            raise ValueError("raw_samples must be finite")
        if scales is None:
            raise ValueError("scales are required and must come from training or configuration")
        scale = np.asarray(scales, dtype=float)
        if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0.0):
            raise ValueError("scales must contain three finite positive values")
        if not math.isfinite(threshold_multiplier) or threshold_multiplier <= 0.0:
            raise ValueError("threshold_multiplier must be finite and positive")
        if not math.isfinite(threshold_floor) or threshold_floor <= 0.0:
            raise ValueError("threshold_floor must be finite and positive")

        baseline = np.median(samples, axis=0)
        normalized = (samples - baseline) / scale
        magnitudes = np.linalg.norm(normalized, axis=1)
        if len(normalized) > 1:
            steps = np.diff(normalized, axis=0)
            step_magnitudes = np.linalg.norm(steps, axis=1)
            sensor_steps = np.max(np.abs(steps), axis=1)
        else:  # guarded above; retained for numerical clarity
            step_magnitudes = np.zeros(1)
            sensor_steps = np.zeros(1)
        ranges = np.max(normalized, axis=0) - np.min(normalized, axis=0)
        thresholds = RestThresholds(
            magnitude=_robust_limit(magnitudes, threshold_multiplier, threshold_floor),
            window_range=_robust_limit(ranges, threshold_multiplier, threshold_floor),
            slope=_robust_limit(step_magnitudes, threshold_multiplier, threshold_floor),
            phase_slope_deadband=_robust_limit(sensor_steps, threshold_multiplier, threshold_floor),
        )
        return cls(
            baseline=tuple(float(value) for value in baseline),  # type: ignore[arg-type]
            scale=tuple(float(value) for value in scale),  # type: ignore[arg-type]
            rest_thresholds=thresholds,
            sample_count=int(samples.shape[0]),
        )

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "algorithm_version": self.algorithm_version,
            "sample_count": self.sample_count,
            "motion_gate": {
                "magnitude_threshold": self.rest_thresholds.magnitude,
                "range_threshold": self.rest_thresholds.window_range,
                "slope_threshold": self.rest_thresholds.slope,
                "phase_slope_deadband": self.rest_thresholds.phase_slope_deadband,
            },
        }
        for index, (baseline, scale) in enumerate(zip(self.baseline, self.scale), start=1):
            result[f"flex{index}"] = {"baseline": baseline, "scale": scale}
        return result

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> "FlexCalibration":
        try:
            baseline = tuple(float(payload[f"flex{i}"]["baseline"]) for i in (1, 2, 3))
            scale = tuple(float(payload[f"flex{i}"]["scale"]) for i in (1, 2, 3))
            gate = payload["motion_gate"]
            thresholds = RestThresholds(
                magnitude=float(gate["magnitude_threshold"]),
                window_range=float(gate["range_threshold"]),
                slope=float(gate["slope_threshold"]),
                phase_slope_deadband=float(gate["phase_slope_deadband"]),
            )
            sample_count = int(payload["sample_count"])
            algorithm_version = str(payload["algorithm_version"])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("invalid calibration payload") from exc
        if len(baseline) != 3 or len(scale) != 3 or any(value <= 0 for value in scale):
            raise ValueError("invalid calibration baseline or scale")
        if not all(math.isfinite(value) for value in (*baseline, *scale)):
            raise ValueError("calibration values must be finite")
        if sample_count < 2:
            raise ValueError("calibration sample_count must be at least 2")
        return cls(
            baseline=baseline,  # type: ignore[arg-type]
            scale=scale,  # type: ignore[arg-type]
            rest_thresholds=thresholds,
            sample_count=sample_count,
            algorithm_version=algorithm_version,
        )

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        partial = destination.with_suffix(destination.suffix + ".partial")
        partial.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")
        partial.replace(destination)

    @classmethod
    def load(cls, path: str | Path) -> "FlexCalibration":
        try:
            payload = json.loads(Path(path).read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"cannot load calibration from {path}") from exc
        if not isinstance(payload, dict):
            raise ValueError("calibration JSON root must be an object")
        return cls.from_dict(payload)

