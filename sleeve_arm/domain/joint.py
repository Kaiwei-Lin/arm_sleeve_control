from __future__ import annotations

from dataclasses import dataclass


# Fixed DyMotor native ABI order. Never extend this tuple for other robots.
DYMOTOR_JOINT_NAMES = (
    "shoulder_flexion",
    "shoulder_abduction",
    "elbow_flexion",
    "upper_arm_rotation",
)


JOINT_NAMES = DYMOTOR_JOINT_NAMES  # Historical public alias.


@dataclass(frozen=True)
class JointState:
    name: str
    position: float
    velocity: float | None
    current: float | None
    torque: float | None
    state: int | None
    bus: int | None
    error: int | None
    received_at: float | None = None


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
