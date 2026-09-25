from __future__ import annotations

import math

from sleeve_arm.config import JointConfig
from sleeve_arm.domain.joint import JointState


class SafetyError(RuntimeError):
    """A command or lifecycle operation violated a safety rule."""


class FeedbackError(SafetyError):
    """Joint feedback is missing, invalid, or reports a fault."""


def clamp_position(config: JointConfig, target: float) -> float:
    if not math.isfinite(target):
        raise SafetyError(f"{config.name}: target position is not finite")
    if config.min_position is not None:
        target = max(target, config.min_position)
    if config.max_position is not None:
        target = min(target, config.max_position)
    return target


def limit_position_change(
    config: JointConfig,
    current: float,
    target: float,
    dt: float | None = None,
) -> float:
    if not math.isfinite(current):
        raise FeedbackError(f"{config.name}: current position is not finite")

    allowed = config.max_position_step
    if config.max_velocity is not None:
        if dt is None or not math.isfinite(dt) or dt <= 0:
            raise SafetyError(f"{config.name}: a positive dt is required for max_velocity")
        velocity_step = config.max_velocity * dt
        allowed = velocity_step if allowed is None else min(allowed, velocity_step)

    if allowed is None:
        return target
    return min(max(target, current - allowed), current + allowed)


def safe_target(
    config: JointConfig,
    current: float,
    requested: float,
    dt: float | None = None,
) -> float:
    limited = limit_position_change(config, current, clamp_position(config, requested), dt)
    return clamp_position(config, limited)


def validate_feedback(
    state: JointState,
    config: JointConfig,
    expected_position: float | None = None,
) -> None:
    if state.name != config.name:
        raise FeedbackError(f"feedback name mismatch: {state.name} != {config.name}")
    if state.position is None:
        raise FeedbackError(f"{state.name}: position feedback is unavailable")
    values = {
        "position": state.position,
        "velocity": state.velocity,
        "torque": state.torque,
    }
    if state.current is not None:
        values["current"] = state.current
    for label, value in values.items():
        if value is not None and not math.isfinite(value):
            raise FeedbackError(f"{state.name}: {label} is not finite")
    if state.error is None and getattr(config, "require_motor_error", False):
        raise FeedbackError(f"{state.name}: motor error feedback is unavailable")
    if state.error is not None and state.error != 0:
        raise FeedbackError(f"{state.name}: motor error code {state.error}")
    if config.min_position is not None and state.position < config.min_position:
        raise FeedbackError(f"{state.name}: feedback is below min_position")
    if config.max_position is not None and state.position > config.max_position:
        raise FeedbackError(f"{state.name}: feedback is above max_position")
    if config.max_velocity is not None:
        if state.velocity is None:
            raise FeedbackError(f"{state.name}: velocity feedback is unavailable")
        if abs(state.velocity) > config.max_velocity:
            raise FeedbackError(f"{state.name}: velocity exceeds max_velocity")
    if config.max_current is not None:
        if state.current is None:
            raise FeedbackError(f"{state.name}: current feedback is unavailable")
        if abs(state.current) > config.max_current:
            raise FeedbackError(f"{state.name}: current exceeds max_current")
    if config.max_tracking_error is not None and expected_position is not None:
        if abs(state.position - expected_position) > config.max_tracking_error:
            raise FeedbackError(f"{state.name}: tracking error exceeds max_tracking_error")
