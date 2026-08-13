from __future__ import annotations

import math

from sleeve_arm.config import Phase3ElbowConfig
from sleeve_arm.domain import MotionIntent, SensorSample
from sleeve_arm.predictor.base import MotionPredictor


class RuleBasedPredictor(MotionPredictor):
    """Temporary CH2 calibration rule; outputs semantics, never motor units."""

    def __init__(self, config: Phase3ElbowConfig) -> None:
        if config.input_min is None or config.input_max is None:
            raise ValueError("real Sleeve control requires calibrated input_min and input_max")
        self.config = config
        self.channel_index = config.sleeve_channel - 1
        self.last_raw: float | None = None
        self.last_normalized: float | None = None
        self.last_filtered: float | None = None

    def predict(self, sample: SensorSample) -> MotionIntent:
        if self.channel_index >= len(sample.sleeve.channels):
            raise ValueError(
                f"CH{self.config.sleeve_channel} is missing from "
                f"{len(sample.sleeve.channels)}-channel SleeveFrame"
            )
        raw = float(sample.sleeve.channels[self.channel_index])
        if not math.isfinite(raw):
            raise ValueError(f"CH{self.config.sleeve_channel} is not finite")
        assert self.config.input_min is not None and self.config.input_max is not None
        normalized = min(max((raw - self.config.input_min) / (self.config.input_max - self.config.input_min), 0.0), 1.0)
        if self.config.invert_input:
            normalized = 1.0 - normalized
        if abs(normalized - 0.5) < self.config.deadzone:
            normalized = 0.5

        filtered = normalized
        if self.config.filter.type == "ema" and self.last_filtered is not None:
            assert self.config.filter.alpha is not None
            filtered = self.config.filter.alpha * normalized + (1.0 - self.config.filter.alpha) * self.last_filtered
        self.last_raw = raw
        self.last_normalized = normalized
        self.last_filtered = filtered
        return MotionIntent(timestamp=sample.timestamp, elbow_flexion=filtered)
