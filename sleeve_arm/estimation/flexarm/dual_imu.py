from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


Quaternion = Iterable[float]


def _unit_vector(value: Iterable[float], name: str) -> np.ndarray:
    vector = np.asarray(tuple(value), dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite 3-vector")
    norm = float(np.linalg.norm(vector))
    if norm < 1e-12:
        raise ValueError(f"{name} cannot be zero")
    return vector / norm


def _normalize_quaternion(value: Quaternion) -> np.ndarray:
    quaternion = np.asarray(tuple(value), dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("quaternion must contain four finite values [w,x,y,z]")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("quaternion cannot be zero")
    return quaternion / norm


def _multiply(left: Quaternion, right: Quaternion) -> np.ndarray:
    w1, x1, y1, z1 = _normalize_quaternion(left)
    w2, x2, y2, z2 = _normalize_quaternion(right)
    return _normalize_quaternion((
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ))


def _conjugate(value: Quaternion) -> np.ndarray:
    w, x, y, z = _normalize_quaternion(value)
    return np.asarray((w, -x, -y, -z), dtype=float)


def _relative(chest: Quaternion, arm: Quaternion) -> np.ndarray:
    return _multiply(_conjugate(chest), arm)


def _rotate(quaternion: Quaternion, vector: Iterable[float]) -> np.ndarray:
    q = _normalize_quaternion(quaternion)
    v = np.asarray(tuple(vector), dtype=float)
    qv = q[1:]
    cross = 2.0 * np.cross(qv, v)
    return v + q[0] * cross + np.cross(qv, cross)


def _quaternion_to_matrix(quaternion: Quaternion) -> np.ndarray:
    """Return the active 3-D rotation matrix for a ``[w, x, y, z]`` quaternion."""
    w, x, y, z = _normalize_quaternion(quaternion)
    return np.asarray((
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    ), dtype=float)


def _wrap_deg(angle_deg: float) -> float:
    return float((angle_deg + 180.0) % 360.0 - 180.0)


def _yxy_euler(rotation: np.ndarray, singularity_epsilon: float = 1e-7) -> tuple[float, float, float]:
    """Decompose ``R = Ry(alpha) Rx(beta) Ry(gamma)`` and return degrees.

    At ``beta`` near 0 or 180 degrees, alpha and gamma are not separately
    observable.  The singular branch chooses alpha=0 and puts the observable
    combined Y rotation in gamma; the result stays finite but must not be
    interpreted as a precise upper-arm rotation near a hanging-arm pose.
    """
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation must be a finite 3x3 matrix")
    beta = math.acos(float(np.clip(matrix[1, 1], -1.0, 1.0)))
    sin_beta = math.hypot(float(matrix[0, 1]), float(matrix[2, 1]))
    if sin_beta < singularity_epsilon:
        alpha = 0.0
        gamma = math.atan2(float(matrix[0, 2]), float(matrix[0, 0]))
    else:
        alpha = math.atan2(float(matrix[0, 1]), float(matrix[2, 1]))
        gamma = math.atan2(float(matrix[1, 0]), float(-matrix[1, 2]))
    return tuple(_wrap_deg(math.degrees(value)) for value in (alpha, beta, gamma))


def _samples(value: Iterable[Quaternion] | Quaternion) -> list[np.ndarray]:
    materialized = list(value)
    if len(materialized) == 4 and all(np.isscalar(item) for item in materialized):
        return [_normalize_quaternion(materialized)]
    return [_normalize_quaternion(item) for item in materialized]


def _mean_quaternion(values: Iterable[Quaternion]) -> np.ndarray:
    quaternions = [_normalize_quaternion(value) for value in values]
    if not quaternions:
        raise ValueError("at least one quaternion sample is required")
    reference = quaternions[0]
    aligned = [value if float(np.dot(value, reference)) >= 0.0 else -value for value in quaternions]
    return _normalize_quaternion(np.mean(aligned, axis=0))


@dataclass(frozen=True, slots=True)
class DualImuCalibration:
    chest_reference: tuple[float, float, float, float]
    arm_reference: tuple[float, float, float, float]
    relative_reference: tuple[float, float, float, float]
    sample_count: int


@dataclass(frozen=True, slots=True)
class DirectionCalibration:
    forward_axis: tuple[float, float, float]
    lateral_axis: tuple[float, float, float]
    representative_magnitude_deg: float
    sample_count: int


@dataclass(frozen=True, slots=True)
class ArmRaiseEstimate:
    forward_backward_deg: float
    lateral_deg: float
    upper_arm_rotation_deg: float
    direction: str
    magnitude_deg: float
    plane_deg: float
    confidence: float
    forward_component: float
    lateral_component: float
    arm_vector: tuple[float, float, float]

    @property
    def angle_deg(self) -> float:
        """Alias for ``magnitude_deg``."""
        return self.magnitude_deg


class DualImuArmEstimator:
    """Estimate continuous shoulder pose from chest and arm IMUs.

    Quaternions use ``[w, x, y, z]`` and must describe sensor-local to world
    orientation. Call :meth:`calibrate` while the arm is naturally hanging
    before calling :meth:`estimate`.
    """

    def __init__(
        self,
        *,
        down_axis: Iterable[float] = (0.0, 0.0, -1.0),
        forward_axis: Iterable[float] = (0.0, 1.0, 0.0),
        lateral_axis: Iterable[float] = (1.0, 0.0, 0.0),
        rest_threshold_deg: float = 8.0,
        dominance_ratio: float = 1.2,
        rotation_sign: float = 1.0,
    ) -> None:
        self._down = _unit_vector(down_axis, "down_axis")
        supplied_forward = _unit_vector(forward_axis, "forward_axis")
        supplied_lateral = _unit_vector(lateral_axis, "lateral_axis")
        if max(
            abs(float(np.dot(self._down, supplied_forward))),
            abs(float(np.dot(self._down, supplied_lateral))),
            abs(float(np.dot(supplied_forward, supplied_lateral))),
        ) > 0.15:
            raise ValueError("down, forward and lateral axes must be approximately orthogonal")
        self._forward = _unit_vector(
            supplied_forward - self._down * float(np.dot(supplied_forward, self._down)),
            "forward_axis",
        )
        right_handed_lateral = _unit_vector(np.cross(self._down, self._forward), "lateral_axis")
        if float(np.dot(right_handed_lateral, supplied_lateral)) <= 0.0:
            raise ValueError("down, forward and lateral axes must form a right-handed frame")
        self._lateral = right_handed_lateral
        if not 0.0 <= float(rest_threshold_deg) < 180.0:
            raise ValueError("rest_threshold_deg must be in [0, 180)")
        if float(dominance_ratio) < 1.0:
            raise ValueError("dominance_ratio must be at least 1")
        if float(rotation_sign) not in (-1.0, 1.0):
            raise ValueError("rotation_sign must be +1 or -1")
        self.rest_threshold_deg = float(rest_threshold_deg)
        self.dominance_ratio = float(dominance_ratio)
        # In the right-arm (forward, up, outward) frame, +Y turns forward
        # toward the body and is therefore internal rotation.  Set this one
        # knob to -1 if physical mounting verification shows reversed signs.
        self._rotation_sign = float(rotation_sign)
        self._configured_forward = self._forward.copy()
        self._configured_lateral = self._lateral.copy()
        self._calibration: DualImuCalibration | None = None
        self._direction_calibration: DirectionCalibration | None = None

    @property
    def is_calibrated(self) -> bool:
        return self._calibration is not None

    @property
    def calibration(self) -> DualImuCalibration | None:
        return self._calibration

    @property
    def direction_calibration(self) -> DirectionCalibration | None:
        return self._direction_calibration

    def reset_calibration(self) -> None:
        self._calibration = None
        self._direction_calibration = None
        self._forward = self._configured_forward.copy()
        self._lateral = self._configured_lateral.copy()

    def calibrate(
        self,
        chest_quaternions: Iterable[Quaternion] | Quaternion,
        arm_quaternions: Iterable[Quaternion] | Quaternion,
    ) -> DualImuCalibration:
        """Calibrate from one or more synchronized, naturally-hanging samples."""
        chest = _samples(chest_quaternions)
        arm = _samples(arm_quaternions)
        if len(chest) != len(arm):
            raise ValueError("chest and arm calibration sample counts must match")
        if not chest:
            raise ValueError("at least one synchronized calibration sample is required")
        relative = [_relative(chest_q, arm_q) for chest_q, arm_q in zip(chest, arm)]
        result = DualImuCalibration(
            chest_reference=tuple(float(value) for value in _mean_quaternion(chest)),
            arm_reference=tuple(float(value) for value in _mean_quaternion(arm)),
            relative_reference=tuple(float(value) for value in _mean_quaternion(relative)),
            sample_count=len(chest),
        )
        self._calibration = result
        return result

    def calibrate_forward(
        self,
        chest_quaternions: Iterable[Quaternion] | Quaternion,
        arm_quaternions: Iterable[Quaternion] | Quaternion,
        *,
        minimum_raise_deg: float = 15.0,
    ) -> DirectionCalibration:
        """Learn the anatomical forward axis from a held forward-raised pose."""
        if self._calibration is None:
            raise RuntimeError("calibrate() must be called before calibrate_forward()")
        if not 0.0 < float(minimum_raise_deg) < 180.0:
            raise ValueError("minimum_raise_deg must be in (0, 180)")
        chest = _samples(chest_quaternions)
        arm = _samples(arm_quaternions)
        if len(chest) != len(arm):
            raise ValueError("chest and arm direction calibration sample counts must match")
        tangents: list[np.ndarray] = []
        magnitudes: list[float] = []
        for chest_q, arm_q in zip(chest, arm):
            pose = self.estimate(chest_q, arm_q)
            if pose.magnitude_deg < minimum_raise_deg:
                continue
            vector = np.asarray(pose.arm_vector, dtype=float)
            tangent = vector - self._down * float(np.dot(vector, self._down))
            norm = float(np.linalg.norm(tangent))
            if norm > 1e-9:
                tangents.append(tangent / norm)
                magnitudes.append(pose.magnitude_deg)
        if not tangents:
            raise ValueError(
                f"forward calibration requires a raise of at least {minimum_raise_deg:g} degrees"
            )
        forward = _unit_vector(np.mean(tangents, axis=0), "learned forward axis")
        lateral = _unit_vector(np.cross(self._down, forward), "learned lateral axis")
        self._forward = forward
        self._lateral = lateral
        result = DirectionCalibration(
            forward_axis=tuple(float(value) for value in forward),
            lateral_axis=tuple(float(value) for value in lateral),
            representative_magnitude_deg=float(np.median(magnitudes)),
            sample_count=len(tangents),
        )
        self._direction_calibration = result
        return result

    def estimate(self, chest_quaternion: Quaternion, arm_quaternion: Quaternion) -> ArmRaiseEstimate:
        """Return FB, lateral, and upper-arm rotation angles in degrees."""
        if self._calibration is None:
            raise RuntimeError("calibrate() must be called before estimate()")
        current_relative = _relative(chest_quaternion, arm_quaternion)
        delta = _multiply(current_relative, _conjugate(self._calibration.relative_reference))
        vector = _rotate(delta, self._down)
        vector /= max(float(np.linalg.norm(vector)), 1e-12)
        forward = float(np.dot(vector, self._forward))
        lateral = float(np.dot(vector, self._lateral))
        down = float(np.clip(np.dot(vector, self._down), -1.0, 1.0))
        horizontal = math.hypot(forward, lateral)
        magnitude_rad = math.atan2(horizontal, down)
        magnitude = math.degrees(magnitude_rad)
        plane = math.degrees(math.atan2(lateral, forward))
        if horizontal < 1e-12:
            forward_backward_deg = lateral_deg = 0.0
        else:
            forward_backward_deg = math.degrees(magnitude_rad * forward / horizontal)
            lateral_deg = math.degrees(magnitude_rad * lateral / horizontal)

        up = -self._down
        anatomical_basis = np.column_stack((self._forward, up, self._lateral))
        if not np.allclose(anatomical_basis.T @ anatomical_basis, np.eye(3), atol=1e-9):
            raise RuntimeError("anatomical basis is not orthonormal")
        if not math.isclose(float(np.linalg.det(anatomical_basis)), 1.0, abs_tol=1e-9):
            raise RuntimeError("anatomical basis is not right-handed")
        anatomical_rotation = (
            anatomical_basis.T @ _quaternion_to_matrix(delta) @ anatomical_basis
        )
        _, _, gamma_deg = _yxy_euler(anatomical_rotation)
        upper_arm_rotation_deg = _wrap_deg(self._rotation_sign * gamma_deg)

        if magnitude < self.rest_threshold_deg:
            direction, confidence = "Rest", 1.0
        else:
            forward_abs, lateral_abs = abs(forward), abs(lateral)
            dominant, secondary = max(forward_abs, lateral_abs), min(forward_abs, lateral_abs)
            confidence = float(np.clip((dominant - secondary) / max(dominant, 1e-12), 0.0, 1.0))
            if forward_abs >= self.dominance_ratio * lateral_abs:
                direction = "Forward" if forward >= 0.0 else "Backward"
            elif lateral_abs >= self.dominance_ratio * forward_abs:
                direction = "Lateral"
            else:
                direction = "Transition"

        return ArmRaiseEstimate(
            forward_backward_deg=float(forward_backward_deg),
            lateral_deg=float(lateral_deg),
            upper_arm_rotation_deg=upper_arm_rotation_deg,
            direction=direction,
            magnitude_deg=float(magnitude),
            plane_deg=float(plane),
            confidence=confidence,
            forward_component=forward,
            lateral_component=lateral,
            arm_vector=tuple(float(value) for value in vector),
        )

    update = estimate


# Descriptive new name while preserving the existing public type name.
ShoulderPoseEstimate = ArmRaiseEstimate
