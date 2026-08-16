from __future__ import annotations

import math
import time
from collections.abc import Sequence
from typing import Any

from sleeve_arm.domain import ArmAction, ImuFrame, MotionIntent, SensorSample
from sleeve_arm.predictor.base import MotionPredictor


def imu_quaternion(frame: ImuFrame) -> tuple[float, float, float, float]:
    values = (frame.quat_w, frame.quat_x, frame.quat_y, frame.quat_z)
    if any(value is None for value in values):
        raise ValueError("IMU quaternion is missing")
    quaternion = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in quaternion):
        raise ValueError("IMU quaternion must contain four finite WXYZ values")
    if math.sqrt(sum(value * value for value in quaternion)) <= 1e-12:
        raise ValueError("IMU quaternion norm must be nonzero")
    return quaternion  # type: ignore[return-value]


def calibrate_dual_imu_estimator(
    estimator: Any,
    pairs: Sequence[tuple[ImuFrame, ImuFrame]],
) -> Any:
    """Calibrate from synchronized ``(arm, chest)`` frame pairs."""
    if len(pairs) < 2:
        raise ValueError("dual-IMU neutral calibration needs at least 2 synchronized pairs")
    chest_rest_samples = [imu_quaternion(chest) for _, chest in pairs]
    arm_rest_samples = [imu_quaternion(arm) for arm, _ in pairs]
    return estimator.calibrate(chest_rest_samples, arm_rest_samples)


class DualImuShoulderPredictor(MotionPredictor):
    """Adapt DualImuArmEstimator using ``sample.imu1=arm`` and ``sample.imu2=chest``."""

    _ACTIVE = {
        "Forward": (ArmAction.FORWARD, 1.0, 0.0),
        "Backward": (ArmAction.BACKWARD, -1.0, 0.0),
        "Lateral": (ArmAction.LATERAL, 0.0, 1.0),
    }

    def __init__(self, estimator: Any, max_age_s: float | None = None) -> None:
        self.estimator = estimator
        self.max_age_s = max_age_s
        self.last_result: Any | None = None
        self.last_frames: tuple[ImuFrame, ImuFrame] | None = None
        self._last_targets: tuple[float | None, float | None] = (None, None)

    def predict(self, sample: SensorSample) -> MotionIntent:
        if sample.imu1 is None or sample.imu2 is None:
            raise ValueError("dual-IMU shoulder estimation requires synchronized arm and chest frames")
        return self.predict_imu_pair(sample.imu1, sample.imu2, timestamp=sample.timestamp)

    def predict_imu_pair(
        self,
        arm: ImuFrame,
        chest: ImuFrame,
        *,
        timestamp: float | None = None,
    ) -> MotionIntent:
        """Predict directly from the synchronized ``(arm, chest)`` frames."""
        self._require_fresh((arm, chest))
        chest_q = imu_quaternion(chest)
        arm_q = imu_quaternion(arm)
        started = time.perf_counter()
        result = self.estimator.update(chest_q, arm_q)
        inference_ms = (time.perf_counter() - started) * 1000.0

        direction = getattr(result, "direction", None)
        if direction not in (*self._ACTIVE, "Rest", "Transition"):
            raise ValueError(f"invalid dual-IMU shoulder direction: {direction!r}")
        magnitude_deg = float(getattr(result, "magnitude_deg", math.nan))
        confidence = float(getattr(result, "confidence", math.nan))
        if not math.isfinite(magnitude_deg) or not 0.0 <= magnitude_deg <= 180.0:
            raise ValueError("dual-IMU shoulder magnitude_deg must be finite and in [0, 180]")
        if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
            raise ValueError("dual-IMU shoulder confidence must be finite and in [0, 1]")

        action = None
        if direction in self._ACTIVE:
            action, flexion_sign, abduction_sign = self._ACTIVE[direction]
            magnitude_rad = math.radians(magnitude_deg)
            targets = (flexion_sign * magnitude_rad, abduction_sign * magnitude_rad)
            self._last_targets = targets
        elif direction == "Rest":
            targets = (0.0, 0.0)
            self._last_targets = targets
        else:
            targets = self._last_targets

        self.last_result = result
        self.last_frames = (arm, chest)
        return MotionIntent(
            timestamp=max(arm.timestamp, chest.timestamp) if timestamp is None else timestamp,
            shoulder_flexion_rad=targets[0],
            shoulder_abduction_rad=targets[1],
            action=action,
            confidence=confidence,
            angle_deg=magnitude_deg,
            inference_ms=inference_ms,
            model_action=direction,
            moving=direction != "Rest",
        )

    def _require_fresh(self, frames: tuple[ImuFrame, ImuFrame]) -> None:
        if self.max_age_s is not None:
            now = time.monotonic()
            if any(abs(now - frame.timestamp) > self.max_age_s for frame in frames):
                raise ValueError("dual-IMU shoulder pair is stale")
