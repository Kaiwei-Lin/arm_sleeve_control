from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from sleeve_arm.estimation import (
    align_quaternion_sign,
    average_quaternions,
    normalize_quaternion,
    quaternion_inverse,
    quaternion_multiply,
    quaternion_rotate_vector,
)
from sleeve_arm.shoulder_dataset_config import BodyFrameConfig, DirectionLabelConfig


@dataclass(frozen=True, slots=True)
class ShoulderAngles:
    signed_forward_backward_deg: float
    lateral_deg: float
    amplitude_deg: float
    derived_direction: str
    direction_confidence: float
    direction_valid: bool
    upper_arm_direction: tuple[float, float, float]


def torso_arm_relative_quaternion(
    torso_quaternion: Sequence[float],
    upper_arm_quaternion: Sequence[float],
) -> np.ndarray:
    """Arm orientation relative to torso for Sensor->World active rotations."""
    return quaternion_multiply(quaternion_inverse(torso_quaternion), upper_arm_quaternion)


def neutral_relative_quaternion(
    torso_arm_pairs: Sequence[tuple[Sequence[float], Sequence[float]]],
) -> np.ndarray:
    if len(torso_arm_pairs) < 2:
        raise ValueError("neutral calibration requires at least two torso/arm pairs")
    relatives = [
        torso_arm_relative_quaternion(torso, arm)
        for torso, arm in torso_arm_pairs
    ]
    return average_quaternions(relatives)


def shoulder_quaternion(
    torso_quaternion: Sequence[float],
    upper_arm_quaternion: Sequence[float],
    neutral_relative: Sequence[float],
    previous: Sequence[float] | None = None,
) -> np.ndarray:
    relative = torso_arm_relative_quaternion(torso_quaternion, upper_arm_quaternion)
    # q_relative = q_shoulder * q_zero, so the mounting offset is removed on the right.
    result = quaternion_multiply(relative, quaternion_inverse(neutral_relative))
    return result if previous is None else align_quaternion_sign(result, previous)


def shoulder_angles(
    quaternion: Sequence[float],
    body_frame: BodyFrameConfig,
    label: DirectionLabelConfig,
) -> ShoulderAngles:
    q = normalize_quaternion(quaternion)
    neutral = np.asarray(body_frame.upper_arm_axis_neutral, dtype=float)
    current = quaternion_rotate_vector(q, neutral)
    forward = float(np.dot(current, body_frame.forward_axis))
    lateral = float(np.dot(current, body_frame.lateral_axis))
    down = float(np.dot(current, neutral))

    signed_fb = math.degrees(math.atan2(forward, down))
    lateral_deg = max(0.0, math.degrees(math.atan2(lateral, down)))
    amplitude = math.degrees(math.acos(float(np.clip(np.dot(neutral, current), -1.0, 1.0))))

    if amplitude < label.min_amplitude_deg:
        direction = "neutral"
        confidence = 0.0
        valid = False
    else:
        horizontal_norm = math.hypot(forward, lateral)
        if horizontal_norm <= 1e-12:
            direction = "neutral"
            confidence = 0.0
            valid = False
        else:
            scores = {
                "forward": forward / horizontal_norm,
                "lateral": lateral / horizontal_norm,
                "backward": -forward / horizontal_norm,
            }
            direction, confidence = max(scores.items(), key=lambda item: item[1])
            confidence = max(0.0, min(1.0, float(confidence)))
            valid = confidence >= math.cos(math.radians(label.max_direction_error_deg))

    return ShoulderAngles(
        signed_forward_backward_deg=signed_fb,
        lateral_deg=lateral_deg,
        amplitude_deg=amplitude,
        derived_direction=direction,
        direction_confidence=confidence,
        direction_valid=valid,
        upper_arm_direction=tuple(float(value) for value in current),
    )
