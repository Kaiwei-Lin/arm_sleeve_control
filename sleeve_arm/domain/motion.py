from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum


class ArmAction(IntEnum):
    FORWARD = 0
    LATERAL = 1
    BACKWARD = 2


@dataclass(frozen=True, slots=True)
class MotionIntent:
    timestamp: float
    elbow_flexion: float | None = None
    shoulder_flexion_rad: float | None = None
    shoulder_abduction_rad: float | None = None
    action: ArmAction | None = None
    confidence: float | None = None
    action_probabilities: tuple[float, ...] | None = None
    angle_deg: float | None = None
    inference_ms: float | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.timestamp):
            raise ValueError("MotionIntent timestamp must be finite")
        if self.elbow_flexion is not None and not math.isfinite(self.elbow_flexion):
            raise ValueError("elbow_flexion must be null or an absolute finite semantic angle in radians")
        for name in ("shoulder_flexion_rad", "shoulder_abduction_rad", "angle_deg", "inference_ms"):
            value = getattr(self, name)
            if value is not None and not math.isfinite(value):
                raise ValueError(f"{name} must be null or finite")
        if self.confidence is not None and (
            not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1
        ):
            raise ValueError("confidence must be null or in [0, 1]")
        if self.action is not None and not isinstance(self.action, ArmAction):
            raise ValueError("action must be an ArmAction or null")
        if self.action_probabilities is not None:
            probabilities = tuple(float(value) for value in self.action_probabilities)
            if len(probabilities) < 3 or not all(math.isfinite(value) and 0 <= value <= 1 for value in probabilities):
                raise ValueError("action_probabilities must contain at least three finite values in [0, 1]")
            object.__setattr__(self, "action_probabilities", probabilities)
