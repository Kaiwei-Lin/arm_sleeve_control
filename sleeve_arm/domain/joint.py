from __future__ import annotations

from dataclasses import dataclass


JOINT_NAMES = (
    "shoulder_flexion",
    "shoulder_abduction",
    "elbow_flexion",
)


@dataclass(frozen=True)
class JointState:
    name: str
    position: float
    velocity: float
    current: float | None
    torque: float
    state: int
    bus: int
    error: int


@dataclass(frozen=True)
class JointCommand:
    shoulder_flexion: float
    shoulder_abduction: float
    elbow_flexion: float

    def as_dict(self) -> dict[str, float]:
        return {
            "shoulder_flexion": self.shoulder_flexion,
            "shoulder_abduction": self.shoulder_abduction,
            "elbow_flexion": self.elbow_flexion,
        }
