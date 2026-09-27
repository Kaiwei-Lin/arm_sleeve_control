from __future__ import annotations

from dataclasses import dataclass
import math


SUPPORTED_WINDOW_SIZES = (3, 5, 7, 9)


@dataclass(frozen=True)
class FlexArmConfig:
    window_size: int = 5
    ema_alpha: float = 0.25
    max_velocity_deg_s: float = 180.0
    debounce_frames: int = 3
    action_confidence_threshold: float = 0.55
    low_confidence_grace_frames: int = 2
    max_timestamp_gap_s: float = 1.0
    eps: float = 1e-9

    def validate(self) -> None:
        if self.window_size not in SUPPORTED_WINDOW_SIZES:
            raise ValueError(f"window_size must be one of {SUPPORTED_WINDOW_SIZES}")
        if not math.isfinite(self.ema_alpha) or not 0.0 < self.ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be finite and in (0, 1]")
        if not math.isfinite(self.max_velocity_deg_s) or self.max_velocity_deg_s <= 0.0:
            raise ValueError("max_velocity_deg_s must be finite and positive")
        if self.debounce_frames < 1:
            raise ValueError("debounce_frames must be at least 1")
        if not 0.0 <= self.action_confidence_threshold <= 1.0:
            raise ValueError("action_confidence_threshold must be in [0, 1]")
        if self.low_confidence_grace_frames < 0:
            raise ValueError("low_confidence_grace_frames must be non-negative")
        if not math.isfinite(self.max_timestamp_gap_s) or self.max_timestamp_gap_s <= 0.0:
            raise ValueError("max_timestamp_gap_s must be finite and positive")
        if not math.isfinite(self.eps) or self.eps <= 0.0:
            raise ValueError("eps must be finite and positive")

