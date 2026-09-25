"""Backend-independent safety configuration (no motor or network addresses)."""
from dataclasses import dataclass

from sleeve_arm.config import SafetyConfig


@dataclass(frozen=True)
class JointSafetyParameters:
    name: str
    min_position: float | None = None
    max_position: float | None = None
    max_velocity: float | None = None
    max_current: float | None = None
    max_tracking_error: float | None = None
    max_position_step: float | None = None
    require_motor_error: bool = False


@dataclass(frozen=True)
class ControlConfig:
    safety: SafetyConfig
    joints: dict[str, JointSafetyParameters]
