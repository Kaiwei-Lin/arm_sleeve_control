from __future__ import annotations

import math
from collections.abc import Mapping

from sleeve_arm.config import Phase3ElbowConfig
from sleeve_arm.domain import MotionIntent


class ArmMapper:
    """Map normalized elbow semantics into a small startup-relative radian window."""

    def __init__(
        self,
        config: Phase3ElbowConfig,
        startup_positions: Mapping[str, float],
        shoulder_flexion_max_delta_deg: float | None = None,
        shoulder_abduction_max_delta_deg: float | None = None,
    ) -> None:
        required = {"shoulder_flexion", "shoulder_abduction", "elbow_flexion"}
        if set(startup_positions) != required or not all(math.isfinite(value) for value in startup_positions.values()):
            raise ValueError("startup_positions must contain three finite semantic joint positions")
        self.config = config
        self.startup_positions = dict(startup_positions)
        self.shoulder_flexion_max_delta_rad = (
            None if shoulder_flexion_max_delta_deg is None else math.radians(shoulder_flexion_max_delta_deg)
        )
        self.shoulder_abduction_max_delta_rad = (
            None if shoulder_abduction_max_delta_deg is None else math.radians(shoulder_abduction_max_delta_deg)
        )

    def map(self, intent: MotionIntent) -> dict[str, float]:
        if intent.elbow_flexion is None:
            raise ValueError("elbow_flexion intent is required")
        value = 1.0 - intent.elbow_flexion if self.config.invert_output else intent.elbow_flexion
        minimum = math.radians(self.config.min_delta_deg)
        maximum = math.radians(self.config.max_delta_deg)
        delta = minimum * (1.0 - 2.0 * value) if value <= 0.5 else maximum * (2.0 * value - 1.0)
        elbow = self.startup_positions["elbow_flexion"] + delta
        shoulder_flexion = self._shoulder_target(
            "shoulder_flexion", intent.shoulder_flexion_rad, self.shoulder_flexion_max_delta_rad
        )
        shoulder_abduction = self._shoulder_target(
            "shoulder_abduction", intent.shoulder_abduction_rad, self.shoulder_abduction_max_delta_rad
        )
        return {
            "shoulder_flexion": shoulder_flexion,
            "shoulder_abduction": shoulder_abduction,
            "elbow_flexion": elbow,
        }

    def _shoulder_target(self, name: str, semantic_angle: float | None, limit: float | None) -> float:
        if semantic_angle is None:
            return self.startup_positions[name]
        if limit is None:
            raise ValueError("shoulder validation limits are required for shoulder MotionIntent")
        delta = min(max(semantic_angle, -limit), limit)
        return self.startup_positions[name] + delta


class SensorWatchdog:
    def __init__(self, timeout_ms: float, hard_timeout_ms: float) -> None:
        self.timeout_s = timeout_ms / 1000.0
        self.hard_timeout_s = hard_timeout_ms / 1000.0

    def age(self, sample_timestamp: float, now: float) -> float:
        age = now - sample_timestamp
        if not math.isfinite(age) or age < 0:
            raise ValueError("sensor timestamp is invalid or in the future")
        return age

    def is_stale(self, sample_timestamp: float, now: float) -> bool:
        return self.age(sample_timestamp, now) > self.timeout_s

    def is_hard_timeout(self, sample_timestamp: float, now: float) -> bool:
        return self.age(sample_timestamp, now) > self.hard_timeout_s
