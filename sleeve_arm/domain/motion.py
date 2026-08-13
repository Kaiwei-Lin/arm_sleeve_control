from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class MotionIntent:
    timestamp: float
    elbow_flexion: float | None = None
    shoulder_abduction: float | None = None
    shoulder_flexion: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp):
            raise ValueError("MotionIntent timestamp must be finite")
        for name in ("elbow_flexion", "shoulder_abduction", "shoulder_flexion"):
            value = getattr(self, name)
            if value is not None and (not math.isfinite(value) or not 0 <= value <= 1):
                raise ValueError(f"{name} must be null or normalized to [0, 1]")
