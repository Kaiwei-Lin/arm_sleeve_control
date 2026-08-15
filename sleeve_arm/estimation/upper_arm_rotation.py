from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from sleeve_arm.domain import ImuFrame


def normalize_quaternion(quaternion: Sequence[float]) -> np.ndarray:
    values = np.asarray(quaternion, dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError("quaternion must contain four finite [w,x,y,z] values")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("quaternion norm must be nonzero")
    return values / norm


def quaternion_inverse(quaternion: Sequence[float]) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    return np.array((q[0], -q[1], -q[2], -q[3]), dtype=float)


def quaternion_multiply(left: Sequence[float], right: Sequence[float]) -> np.ndarray:
    w1, x1, y1, z1 = normalize_quaternion(left)
    w2, x2, y2, z2 = normalize_quaternion(right)
    return normalize_quaternion((
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ))


def align_quaternion_sign(
    quaternion: Sequence[float],
    reference: Sequence[float],
) -> np.ndarray:
    current = normalize_quaternion(quaternion)
    return -current if float(np.dot(current, normalize_quaternion(reference))) < 0.0 else current


def average_quaternions(quaternions: Sequence[Sequence[float]]) -> np.ndarray:
    """Markley average for WXYZ unit quaternions with deterministic sign."""
    if not quaternions:
        raise ValueError("at least one quaternion is required")
    reference = normalize_quaternion(quaternions[0])
    accumulator = np.zeros((4, 4), dtype=float)
    for quaternion in quaternions:
        current = align_quaternion_sign(quaternion, reference)
        accumulator += np.outer(current, current)
    _, eigenvectors = np.linalg.eigh(accumulator)
    return align_quaternion_sign(eigenvectors[:, -1], reference)


def quaternion_slerp(
    start: Sequence[float],
    end: Sequence[float],
    fraction: float,
) -> np.ndarray:
    fraction = float(fraction)
    if not math.isfinite(fraction) or not 0.0 <= fraction <= 1.0:
        raise ValueError("SLERP fraction must be finite and in [0, 1]")
    q0 = normalize_quaternion(start)
    q1 = align_quaternion_sign(end, q0)
    dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
    if dot > 0.9995:
        return normalize_quaternion(q0 + fraction * (q1 - q0))
    angle = math.acos(dot)
    scale = math.sin(angle)
    return normalize_quaternion(
        math.sin((1.0 - fraction) * angle) / scale * q0
        + math.sin(fraction * angle) / scale * q1
    )


def quaternion_rotate_vector(
    quaternion: Sequence[float],
    vector: Sequence[float],
) -> np.ndarray:
    q = normalize_quaternion(quaternion)
    values = np.asarray(vector, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("vector must contain three finite values")
    q_vector = q[1:]
    return values + 2.0 * np.cross(q_vector, np.cross(q_vector, values) + q[0] * values)


def quaternion_from_axis_angle(axis: Sequence[float], angle_rad: float) -> np.ndarray:
    values = np.asarray(axis, dtype=float)
    angle_rad = float(angle_rad)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        raise ValueError("axis must contain three finite values")
    if not math.isfinite(angle_rad):
        raise ValueError("angle_rad must be finite")
    norm = float(np.linalg.norm(values))
    if norm <= 1e-12:
        raise ValueError("axis norm must be nonzero")
    half = angle_rad / 2.0
    return normalize_quaternion((math.cos(half), *(math.sin(half) * values / norm)))


def _axis_twist_degrees(quaternion: Sequence[float], axis: str) -> float:
    component = {"x": 1, "y": 2, "z": 3}.get(axis)
    if component is None:
        raise ValueError("twist axis must be x, y, or z")
    q = normalize_quaternion(quaternion)
    twist_norm = float(np.hypot(q[0], q[component]))
    if twist_norm <= 1e-12:
        return 0.0
    w, vector = q[0] / twist_norm, q[component] / twist_norm
    angle = math.degrees(2.0 * math.atan2(vector, w))
    return (angle + 180.0) % 360.0 - 180.0


def _x_twist_degrees(quaternion: Sequence[float]) -> float:
    return _axis_twist_degrees(quaternion, "x")


@dataclass(frozen=True, slots=True)
class TwistResult:
    raw_deg: float
    unwrapped_deg: float
    filtered_deg: float


class _FilteredTwist:
    def __init__(self, ema_alpha: float, axis: str = "x") -> None:
        if not 0.0 < ema_alpha <= 1.0:
            raise ValueError("ema_alpha must be in (0, 1]")
        if axis not in ("x", "y", "z"):
            raise ValueError("twist axis must be x, y, or z")
        self.ema_alpha = float(ema_alpha)
        self.axis = axis
        self._previous_raw: float | None = None
        self._unwrapped: float | None = None
        self._filtered: float | None = None

    def _reset_filter(self) -> None:
        self._previous_raw = None
        self._unwrapped = None
        self._filtered = None

    def _result(self, delta: Sequence[float]) -> TwistResult:
        raw = _axis_twist_degrees(delta, self.axis)
        if self._previous_raw is None:
            unwrapped = raw
        else:
            step = raw - self._previous_raw
            if step > 180.0:
                step -= 360.0
            elif step <= -180.0:
                step += 360.0
            unwrapped = float(self._unwrapped) + step
        filtered = unwrapped if self._filtered is None else (
            self.ema_alpha * unwrapped + (1.0 - self.ema_alpha) * self._filtered
        )
        self._previous_raw = raw
        self._unwrapped = unwrapped
        self._filtered = filtered
        return TwistResult(raw, unwrapped, filtered)


class TwistEstimator(_FilteredTwist):
    def __init__(self, ema_alpha: float, axis: str = "x") -> None:
        super().__init__(ema_alpha, axis)
        self._zero: np.ndarray | None = None

    def calibrate(self, quaternions: Sequence[Sequence[float]]) -> None:
        self._zero = average_quaternions(quaternions)
        self._reset_filter()

    def update(self, quaternion: Sequence[float]) -> TwistResult:
        if self._zero is None:
            raise RuntimeError("estimator must be calibrated before update")
        delta = quaternion_multiply(quaternion_inverse(self._zero), quaternion)
        return self._result(delta)


class RelativeTwistEstimator(_FilteredTwist):
    def __init__(self, ema_alpha: float, axis: str = "x") -> None:
        super().__init__(ema_alpha, axis)
        self._relative_zero: np.ndarray | None = None

    @staticmethod
    def _relative(upper: Sequence[float], reference: Sequence[float]) -> np.ndarray:
        return quaternion_multiply(quaternion_inverse(reference), upper)

    def calibrate(self, pairs: Sequence[tuple[Sequence[float], Sequence[float]]]) -> None:
        if not pairs:
            raise ValueError("at least one quaternion pair is required")
        relatives = [self._relative(upper, reference) for upper, reference in pairs]
        self._relative_zero = average_quaternions(relatives)
        self._reset_filter()

    def update(self, upper: Sequence[float], reference: Sequence[float]) -> TwistResult:
        if self._relative_zero is None:
            raise RuntimeError("estimator must be calibrated before update")
        relative = self._relative(upper, reference)
        delta = quaternion_multiply(quaternion_inverse(self._relative_zero), relative)
        return self._result(delta)


@dataclass(frozen=True, slots=True)
class UpperArmRotationResult:
    world_raw_deg: float
    world_unwrapped_deg: float
    world_filtered_deg: float
    relative_raw_deg: float
    relative_unwrapped_deg: float
    relative_filtered_deg: float
    difference_deg: float
    sync_gap_ms: float


class UpperArmRotationEstimator:
    """Estimate semantic upper-arm rotation from synchronized existing IMU frames."""

    def __init__(self, ema_alpha: float = 0.35, max_sync_ms: float | None = None, axis: str = "x") -> None:
        if max_sync_ms is not None and (not math.isfinite(max_sync_ms) or max_sync_ms <= 0.0):
            raise ValueError("max_sync_ms must be positive or null")
        self.max_sync_ms = None if max_sync_ms is None else float(max_sync_ms)
        self.world = TwistEstimator(ema_alpha, axis)
        self.relative = RelativeTwistEstimator(ema_alpha, axis)
        self.sync_rejected_count = 0

    def calibrate(self, pairs: Sequence[tuple[ImuFrame, ImuFrame]]) -> None:
        valid: list[tuple[np.ndarray, np.ndarray]] = []
        for upper, reference in pairs:
            try:
                valid.append(self._quaternions(upper, reference))
            except ValueError:
                continue
        if len(valid) < 2:
            raise ValueError(
                f"calibration needs at least 2 synchronized valid pairs; received {len(valid)}"
            )
        self.world.calibrate([upper for upper, _ in valid])
        self.relative.calibrate(valid)

    def update(self, upper: ImuFrame, reference: ImuFrame) -> UpperArmRotationResult:
        upper_q, reference_q = self._quaternions(upper, reference)
        world = self.world.update(upper_q)
        relative = self.relative.update(upper_q, reference_q)
        difference = world.filtered_deg - relative.filtered_deg
        if not math.isfinite(difference):
            raise ValueError("upper-arm rotation difference is not finite")
        return UpperArmRotationResult(
            world.raw_deg,
            world.unwrapped_deg,
            world.filtered_deg,
            relative.raw_deg,
            relative.unwrapped_deg,
            relative.filtered_deg,
            difference,
            abs(upper.timestamp - reference.timestamp) * 1000.0,
        )

    def validate_pair(self, upper: ImuFrame, reference: ImuFrame) -> float:
        self._quaternions(upper, reference)
        return abs(upper.timestamp - reference.timestamp) * 1000.0

    def _quaternions(self, upper: ImuFrame, reference: ImuFrame) -> tuple[np.ndarray, np.ndarray]:
        gap_ms = abs(upper.timestamp - reference.timestamp) * 1000.0
        if self.max_sync_ms is not None and gap_ms > self.max_sync_ms:
            self.sync_rejected_count += 1
            raise ValueError(
                f"upper/reference IMU sync gap {gap_ms:.3f} ms exceeds {self.max_sync_ms:.3f} ms"
            )
        return self._quaternion(upper), self._quaternion(reference)

    @staticmethod
    def _quaternion(frame: ImuFrame) -> np.ndarray:
        values = (frame.quat_w, frame.quat_x, frame.quat_y, frame.quat_z)
        if any(value is None for value in values):
            raise ValueError("IMU quaternion is missing")
        return normalize_quaternion(values)  # type: ignore[arg-type]
