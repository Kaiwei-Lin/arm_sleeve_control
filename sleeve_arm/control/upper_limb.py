"""Bounded upper-limb trajectories through the existing safe controller."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping

from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.control.safety import SafetyError
from sleeve_arm.robot.aurora_profile import finite


@dataclass(frozen=True)
class MotionResult:
    submitted: bool
    arrived: bool
    targets_rad: dict[str, float]
    elapsed_s: float
    delivery_confirmed: bool = False


class UpperLimbService:
    """Robot-relative angular moves; no Cartesian control or IK.

    A service uses the controller's single owner and clock. Each relative target
    is resolved once from fresh measured feedback, before trajectory generation.
    """
    def __init__(self, controller: SafeArmController):
        self.controller = controller
        self.robot = controller.robot
        self.profile = self.robot.session.profile
        self.clock = controller.clock
        self.period = self.profile.control_period_s

    def _key(self, side, joint, *, capability="joint_position", part="arm"):
        return self.robot.joint_key(side, joint, capability=capability, part=part)

    def _bounds(self, key, target):
        target = finite(target, key)
        limits = self.controller.config.joints[key]
        if not limits.min_position <= target <= limits.max_position:
            raise ValueError(f"{key}: requested target outside calibrated limits")
        return target

    def _duration(self, duration):
        duration = finite(duration, "duration_s", positive=True)
        if duration < self.period or duration > 3600:
            raise ValueError("duration must be between one control period and 3600s")
        return duration

    def move_arm(self, *, side, direction, angle_deg, reference="neutral", duration_s=2.0):
        directions = {"forward": ("shoulder_flexion", 1), "backward": ("shoulder_flexion", -1),
                      "outward": ("shoulder_abduction", 1), "inward": ("shoulder_abduction", -1)}
        if direction not in directions:
            raise ValueError(f"unsupported direction: {direction}")
        angle = finite(angle_deg, "angle_deg")
        if angle < 0:
            raise ValueError("directional angle_deg must be nonnegative")
        joint, sign = directions[direction]
        return self.move_joint(side=side, joint=joint, angle_deg=sign * angle,
                               reference=reference, duration_s=duration_s)

    def move_joint(self, *, side, joint, angle_deg, duration_s=2.0, reference="neutral"):
        key = self._key(side, joint)
        return self.move_joints({key: math.radians(finite(angle_deg, "angle_deg"))},
                                duration_s=duration_s, reference=reference)

    def move_joints(self, targets_rad: Mapping[str, float], *, duration_s, reference="neutral"):
        """Batch semantic radians; dual-arm keys are 'left.name' / 'right.name'."""
        duration = self._duration(duration_s)
        if reference not in ("neutral", "current"):
            raise ValueError("reference must be neutral or current")
        if not targets_rad or set(targets_rad) - set(self.controller.config.joints):
            raise ValueError("empty or unsupported joint target")
        values = {k: finite(v, k) for k, v in targets_rad.items()}
        states = self.controller.read_joint_states()
        starts = {key: states[key].position for key in values}
        targets = {k: self._bounds(k, v + (starts[k] if reference == "current" else 0.0))
                   for k, v in values.items()}
        for key, target in targets.items():
            peak = 1.5 * abs(target - starts[key]) / duration  # cubic smoothstep derivative
            self._speed(key, peak)
        def path(elapsed):
            alpha = min(elapsed / duration, 1.0)
            smooth = alpha * alpha * (3.0 - 2.0 * alpha)
            return {k: starts[k] + (v - starts[k]) * smooth for k, v in targets.items()}
        return self._run(path, duration, targets)

    def _speed(self, key, peak):
        limits = self.controller.config.joints[key]
        if peak > limits.max_velocity or peak * self.period > limits.max_position_step:
            raise ValueError(f"{key}: trajectory exceeds velocity/step limits; increase duration/period")

    def swing_arm(self, *, side, joint, amplitude_deg, period_s, cycles, center="current", settle_duration_s=2.0):
        key = self._key(side, joint)
        amplitude = math.radians(finite(amplitude_deg, "amplitude_deg", positive=True))
        period = self._duration(period_s)
        if period < 20 * self.period:
            raise ValueError("swing period requires at least 20 control samples")
        if type(cycles) is not int or not 1 <= cycles <= 1000 or cycles * period > 3600:
            raise ValueError("cycles must be an integer in 1..1000 and total duration <= 3600s")
        states = self.controller.read_joint_states()
        current = states[key].position
        middle = current if center == "current" else finite(center, "center (semantic radians)")
        self._bounds(key, middle - amplitude)
        self._bounds(key, middle + amplitude)
        # Raised cosine: starts/ends at the lower endpoint with zero velocity,
        # visits both extrema once per cycle. Bounded entry/exit moves are explicit.
        self._speed(key, 2.0 * math.pi * amplitude / period)
        entry = middle - amplitude
        entry_duration = self._duration(settle_duration_s)
        self._speed(key, 1.5 * abs(entry - current) / entry_duration)
        self._speed(key, 1.5 * amplitude / entry_duration)
        began = self.clock.monotonic()
        self.move_joints({key: entry}, duration_s=entry_duration)
        self._run(lambda t: {key: middle - amplitude * math.cos(2 * math.pi * t / period)},
                  period * cycles, {key: entry})
        result = self.move_joints({key: middle}, duration_s=entry_duration)
        return MotionResult(result.submitted, result.arrived, result.targets_rad, self.clock.monotonic() - began)

    def hand_joints(self, *, side, angles_deg, duration_s=2.0):
        targets = {self._key(side, name, capability="hand_joints", part="hand"):
                   math.radians(finite(angle, name)) for name, angle in angles_deg.items()}
        return self.move_joints(targets, duration_s=duration_s)

    def hand_closure(self, **kwargs):
        raise ValueError("hand closure unsupported: no verified opening/closing calibration")

    def wrist_joints(self, *, side, angles_deg, duration_s=2.0):
        """Only named physical wrist joints; never an invented 'palm' joint."""
        targets = {self._key(side, name, capability="wrist_orientation"):
                   math.radians(finite(angle, name)) for name, angle in angles_deg.items()}
        return self.move_joints(targets, duration_s=duration_s)

    def _run(self, path, duration, targets):
        if not self.controller.enabled:
            raise SafetyError("trajectory requires an enabled controller")
        began = self.clock.monotonic()
        last = began
        next_tick = began + self.period
        try:
            while True:
                self.clock.sleep(max(0.0, next_tick - self.clock.monotonic()))
                now = self.clock.monotonic()
                dt = now - last
                # Never skip many trajectory steps or expand the allowed step after a delay.
                if dt > self.period * 1.5 or dt <= 0:
                    raise SafetyError("trajectory scheduling delay; request not completed")
                elapsed = min(now - began, duration)
                requested = path(elapsed)
                self.controller.set_joint_positions(requested, dt=min(dt, self.period), require_exact=True)
                last = now
                if elapsed >= duration - 1e-9:
                    break
                next_tick = min(next_tick + self.period, began + duration)
            deadline = self.clock.monotonic() + self.profile.arrival_timeout_s
            while True:
                states = self.controller.read_joint_states()
                # A publication is not arrival. Require a newer feedback sample as well as tolerance.
                if self.robot.arrival_feedback_is_new(targets) and all(states[k].received_at is not None and states[k].received_at >= last and
                       abs(states[k].position - v) <= self.profile.arrival_tolerance_rad for k, v in targets.items()):
                    return MotionResult(True, True, dict(targets), self.clock.monotonic() - began)
                if self.clock.monotonic() >= deadline:
                    raise SafetyError("submitted trajectory but target arrival timed out (SDK provides no delivery acknowledgement)")
                self.clock.sleep(self.period)
        except BaseException as exc:
            self.robot._latch(exc)
            # This also covers Ctrl+C, fake clock errors, and callers outside CLI finally blocks.
            self.controller.fault = "trajectory interrupted/failed; explicit reconstruction required"
            self.controller.shutdown()
            raise
