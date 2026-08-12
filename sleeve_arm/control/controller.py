from __future__ import annotations

import math
import time
from collections.abc import Mapping

from sleeve_arm.config import RobotConfig
from sleeve_arm.control.safety import SafetyError, safe_target, validate_feedback
from sleeve_arm.domain.joint import JOINT_NAMES, JointState
from sleeve_arm.robot.base import RobotArm, RobotError


class SafeArmController:
    """Mandatory lifecycle and command safety around a RobotArm backend."""

    def __init__(self, robot: RobotArm, config: RobotConfig) -> None:
        self.robot = robot
        self.config = config
        self.connected = False
        self.ready = False
        self.enabled = False
        self._last_targets: dict[str, float] = {}
        self._last_command_time: float | None = None

    def connect(self) -> None:
        if self.connected:
            raise SafetyError("controller is already connected")
        try:
            self.robot.connect()
            self.connected = True
            last_states: dict[str, JointState] = {}
            for sample in range(self.config.safety.stable_feedback_samples):
                states = self._read_all_valid()
                if last_states:
                    self._validate_startup_stability(last_states, states)
                last_states = states
                if sample + 1 < self.config.safety.stable_feedback_samples:
                    time.sleep(self.config.safety.stable_feedback_interval_s)
            self._last_targets = {name: state.position for name, state in last_states.items()}
            self.ready = True
        except BaseException:
            self._close_after_failed_connect()
            raise

    def enable(self) -> None:
        if not self.connected or not self.ready:
            raise SafetyError("stable feedback and motor discovery are required before Servo On")
        if self.enabled:
            return
        try:
            states = self._read_all_valid()
            self.robot.enable()
        except BaseException as exc:
            self._emergency_stop(exc)
        self.enabled = True
        self._last_targets = {name: state.position for name, state in states.items()}
        self._last_command_time = time.monotonic()

    def read_joint_state(self, joint_name: str) -> JointState:
        self._require_joint(joint_name)
        if not self.connected:
            raise SafetyError("controller is not connected")
        try:
            state = self.robot.read_joint_state(joint_name)
            expected = self._last_targets.get(joint_name) if self.enabled else None
            validate_feedback(state, self.config.joints[joint_name], expected)
            return state
        except BaseException as exc:
            if self.enabled:
                self._emergency_stop(exc)
            raise

    def read_joint_states(self) -> dict[str, JointState]:
        if not self.connected:
            raise SafetyError("controller is not connected")
        try:
            return self._read_all_valid(use_tracking_error=self.enabled)
        except BaseException as exc:
            if self.enabled:
                self._emergency_stop(exc)
            raise

    def preview_positions(
        self,
        targets: Mapping[str, float],
        dt: float | None = None,
    ) -> dict[str, float]:
        checked = self._check_targets(targets)
        states = self.read_joint_states()
        return {
            name: safe_target(self.config.joints[name], states[name].position, target, dt)
            for name, target in checked.items()
        }

    def set_joint_position(
        self,
        joint_name: str,
        position: float,
        dt: float | None = None,
    ) -> float:
        return self.set_joint_positions({joint_name: position}, dt)[joint_name]

    def set_joint_positions(
        self,
        targets: Mapping[str, float],
        dt: float | None = None,
    ) -> dict[str, float]:
        if not self.enabled:
            raise SafetyError("Servo On is required before sending a motion command")
        checked = self._check_targets(targets)
        now = time.monotonic()
        elapsed = dt
        if elapsed is None and self._last_command_time is not None:
            elapsed = now - self._last_command_time
        try:
            states = self._read_all_valid(use_tracking_error=True)
            safe = {
                name: safe_target(
                    self.config.joints[name],
                    states[name].position,
                    target,
                    elapsed,
                )
                for name, target in checked.items()
            }
            self.robot.set_joint_positions(safe)
        except BaseException as exc:
            self._emergency_stop(exc)
        self._last_targets.update(safe)
        self._last_command_time = now
        return safe

    def monitor(self, duration_s: float) -> dict[str, JointState]:
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("duration_s must be finite and non-negative")
        deadline = time.monotonic() + duration_s
        states = self.read_joint_states()
        while time.monotonic() < deadline:
            time.sleep(min(self.config.safety.monitor_interval_s, max(0.0, deadline - time.monotonic())))
            states = self.read_joint_states()
        return states

    def disable(self) -> None:
        try:
            if self.connected:
                self.robot.disable()
        finally:
            self.enabled = False
            self._last_command_time = None

    def close(self) -> None:
        try:
            if self.connected:
                self.robot.close()
        finally:
            self.connected = False
            self.ready = False
            self.enabled = False

    def shutdown(self) -> None:
        errors: list[str] = []
        try:
            self.disable()
        except BaseException as exc:  # close still has its own Servo Off attempt
            errors.append(f"Servo Off failed: {exc}")
        try:
            self.close()
        except BaseException as exc:
            errors.append(f"close failed: {exc}")
        if errors:
            raise RobotError("; ".join(errors))

    def _read_all_valid(self, use_tracking_error: bool = False) -> dict[str, JointState]:
        states: dict[str, JointState] = {}
        for name in JOINT_NAMES:
            state = self.robot.read_joint_state(name)
            expected = self._last_targets.get(name) if use_tracking_error else None
            validate_feedback(state, self.config.joints[name], expected)
            states[name] = state
        return states

    def _check_targets(self, targets: Mapping[str, float]) -> dict[str, float]:
        if not targets:
            raise SafetyError("at least one joint target is required")
        checked: dict[str, float] = {}
        for name, value in targets.items():
            self._require_joint(name)
            target = float(value)
            if not math.isfinite(target):
                raise SafetyError(f"{name}: target position is not finite")
            checked[name] = target
        return checked

    def _validate_startup_stability(
        self,
        previous: Mapping[str, JointState],
        current: Mapping[str, JointState],
    ) -> None:
        for name in JOINT_NAMES:
            max_change = self.config.joints[name].max_position_step
            if max_change is not None and abs(current[name].position - previous[name].position) > max_change:
                raise SafetyError(
                    f"{name}: startup feedback is not stable "
                    f"(change exceeds max_position_step)"
                )

    @staticmethod
    def _require_joint(joint_name: str) -> None:
        if joint_name not in JOINT_NAMES:
            raise SafetyError(f"unknown joint: {joint_name}")

    def _close_after_failed_connect(self) -> None:
        try:
            if self.connected:
                self.robot.disable()
        except BaseException:
            pass
        try:
            self.robot.close()
        except BaseException:
            pass
        self.connected = False
        self.ready = False
        self.enabled = False

    def _emergency_stop(self, cause: BaseException) -> None:
        stop_errors: list[str] = []
        try:
            self.robot.disable()
        except BaseException as exc:
            stop_errors.append(f"Servo Off failed: {exc}")
        try:
            self.robot.close()
            self.connected = False
        except BaseException as exc:
            stop_errors.append(f"close failed: {exc}")
        self.enabled = False
        self.ready = False
        suffix = f"; {'; '.join(stop_errors)}" if stop_errors else ""
        raise SafetyError(f"motion stopped: {cause}{suffix}") from cause
