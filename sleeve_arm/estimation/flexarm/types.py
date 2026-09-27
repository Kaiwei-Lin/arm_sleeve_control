from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


VALID_ACTIONS = ("Forward", "Backward", "Lateral", "Rest", "Unknown")
ACTIVE_ACTIONS = VALID_ACTIONS[:3]


class MotionPhase(str, Enum):
    RAISING = "Raising"
    LOWERING = "Lowering"
    HOLDING = "Holding"


@dataclass(frozen=True)
class FlexArmPrediction:
    action: str
    angle_deg: float
    action_confidence: float
    angle_confidence: float
    moving: bool

    def __post_init__(self) -> None:
        if self.action not in VALID_ACTIONS:
            raise ValueError(f"unsupported action: {self.action}")
        if not 0.0 <= self.action_confidence <= 1.0:
            raise ValueError("action_confidence must be in [0, 1]")
        if not 0.0 <= self.angle_confidence <= 1.0:
            raise ValueError("angle_confidence must be in [0, 1]")
