from __future__ import annotations

import math
from collections.abc import Mapping

from sleeve_arm.config import Phase3ElbowConfig
from sleeve_arm.domain import JOINT_NAMES, MotionIntent


class ArmMapper:
    """Pass absolute semantic joint radians while holding inactive joints at startup."""

    def __init__(
        self,
        config: Phase3ElbowConfig,
        startup_positions: Mapping[str, float],
    ) -> None:
        required = set(JOINT_NAMES)
        if set(startup_positions) != required or not all(math.isfinite(value) for value in startup_positions.values()):
            raise ValueError("startup_positions must contain all finite semantic joint positions")
        self.config = config
        self.startup_positions = dict(startup_positions)

    def map(self, intent: MotionIntent) -> dict[str, float]:
        if intent.elbow_flexion is None:
            raise ValueError("elbow_flexion intent is required")
        elbow = intent.elbow_flexion
        shoulder_flexion = self._shoulder_target("shoulder_flexion", intent.shoulder_flexion_rad)
        shoulder_abduction = self._shoulder_target("shoulder_abduction", intent.shoulder_abduction_rad)
        upper_arm_rotation = self._shoulder_target(
            "upper_arm_rotation", intent.upper_arm_rotation_rad
        )
        if elbow > math.radians(self.config.elbow_flexion_limit_deg):
            upper_arm_rotation = 0
        return {
            "shoulder_flexion": shoulder_flexion,
            "shoulder_abduction": shoulder_abduction,
            "elbow_flexion": elbow,
            "upper_arm_rotation": upper_arm_rotation,
        }

    def _shoulder_target(self, name: str, semantic_angle: float | None) -> float:
        if semantic_angle is None:
            return self.startup_positions[name]
        if not math.isfinite(semantic_angle):
            raise ValueError(f"{name} absolute semantic angle must be finite")
        return semantic_angle


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


class AuroraIntentMapper:
    """Route a single explicitly sided semantic intent to one arm only.

    Current predictors/calibration prompts are right-arm contracts. Cross-side
    mirroring requires a future validated mapping, and is deliberately rejected.
    """
    def __init__(self, robot, *, source_side: str, target_side: str):
        if source_side != "right" or target_side != source_side:
            raise ValueError("current shoulder estimators require source_side=right and target_side=right; cross-side mapping is unverified")
        if robot.sides != (target_side,):
            raise ValueError("one sleeve must bind exactly one declared target side")
        self.robot = robot
        self.side = target_side

    def map(self, intent: MotionIntent) -> dict[str, float]:
        fields = {"elbow_flexion": intent.elbow_flexion,
                  "shoulder_flexion": intent.shoulder_flexion_rad,
                  "shoulder_abduction": intent.shoulder_abduction_rad,
                  "upper_arm_rotation": intent.upper_arm_rotation_rad}
        targets = {self.robot.joint_key(self.side, joint): value
                   for joint, value in fields.items() if value is not None}
        if not targets:
            raise ValueError("intent contains no supported joint target")
        return targets
