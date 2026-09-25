"""Profile-driven semantic joint backend, sharing one AuroraSession."""
from __future__ import annotations

import math
from collections.abc import Mapping

from sleeve_arm.control.safety import validate_feedback
from sleeve_arm.domain.joint import JointState
from sleeve_arm.robot.aurora_session import AuroraSession
from sleeve_arm.robot.base import RobotArm, RobotError


class AuroraRobotArm(RobotArm):
    strict_limits = True

    def __init__(self, session: AuroraSession, *, sides, parts=("arm",)):
        self.session = session
        self.sides = tuple(sides)
        self.parts = tuple(parts)
        self.groups = session.profile.selected_groups(self.sides, self.parts)
        self.config = session.profile.control_config(self.sides, self.parts)
        self._joints = {}
        for group in self.groups:
            for joint in group.joints:
                key = f"{group.side}.{joint.name}" if len(self.sides) > 1 else joint.name
                self._joints[key] = (group, joint)
        self._token = object()
        self.connected = False
        self.enabled = False
        self.fault: str | None = None
        self._snapshot = None
        self._states = None
        self._targets: dict[str, list[float]] = {}
        self._last_times: dict[str, float] = {}

    def _require(self, *, enabled=False):
        if self.fault:
            raise RobotError(f"Aurora view fault latched: {self.fault}; close/reconfirm to recover")
        if not self.connected:
            raise RobotError("Aurora view not connected")
        if enabled and not self.enabled:
            raise RobotError("Aurora view not enabled")
        self.session._check_fault()

    def _latch(self, exc):
        self.fault = self.fault or str(exc)
        self.enabled = False

    def connect(self):
        if self.connected or self.fault:
            raise RobotError("view already connected or fault latched; create a new view to recover")
        self.session.attach(self._token, {g.name for g in self.groups})
        self.connected = True

    def enable(self):
        self._require()
        if self.enabled:
            return
        try:
            self.read_joint_states()
            self.session.enable(self._token)
            # Re-read after the authority round trip before establishing initial targets.
            self.read_joint_states()
            entries = {g.group_name: g for g in self._snapshot.groups.joint_state}
            self._targets = {g.name: list(entries[g.name].joint_position) for g in self.groups}
            now = self.session.clock.monotonic()
            self._last_times = {g.name: now for g in self.groups}
            self.enabled = True
        except BaseException as exc:
            self._latch(exc)
            try:
                self.session.disable(self._token)
            except Exception as cleanup:
                raise RobotError(f"{exc}; cleanup: {cleanup}") from exc
            raise

    def read_joint_states(self, joint_names=None):
        self._require()
        names = tuple(self.config.joints) if joint_names is None else tuple(joint_names)
        if any(name not in self._joints for name in names):
            raise RobotError("unknown joint")
        try:
            snapshot = self.session.snapshot(controlling=self.enabled)
            entries = {}
            for entry in snapshot.groups.joint_state:
                if entry.group_name in entries:
                    raise RobotError("duplicate control group feedback")
                entries[entry.group_name] = entry
            states = {}
            for group in self.groups:
                entry = entries.get(group.name)
                if entry is None:
                    raise RobotError(f"missing feedback group {group.name}")
                if group.count is None or len(entry.joint_position) != group.count:
                    raise RobotError(f"{group.name}: feedback dimension mismatch/unverified count")
                for field in ("joint_position", "joint_velocity", "joint_effort"):
                    values = getattr(entry, field)
                    if len(values) != group.count:
                        raise RobotError(f"{group.name}: {field} dimension mismatch")
                    if not all(math.isfinite(v) for v in values):
                        raise RobotError(f"{group.name}: non-finite {field} feedback")
            for key, (group, joint) in self._joints.items():
                entry = entries[group.name]
                state = JointState(
                    name=key, position=joint.from_sdk(entry.joint_position[joint.index]),
                    velocity=joint.sign * entry.joint_velocity[joint.index],
                    torque=joint.sign * entry.joint_effort[joint.index],
                    current=None, state=None, bus=None, error=None,
                    received_at=snapshot.received_at,
                )
                expected = (joint.from_sdk(self._targets[group.name][joint.index])
                            if self.enabled else None)
                validate_feedback(state, self.config.joints[key], expected)
                states[key] = state
            self._snapshot, self._states = snapshot, states
            return {name: states[name] for name in names}
        except BaseException as exc:
            self._latch(exc)
            raise

    def read_joint_state(self, joint_name):
        return self.read_joint_states((joint_name,))[joint_name]

    def set_joint_position(self, joint_name, position):
        self.set_joint_positions({joint_name: position})

    def set_joint_positions(self, targets: Mapping[str, float]):
        states = self.read_joint_states()
        self.set_joint_positions_checked(targets, states)

    def command_interval(self, targets):
        now = self.session.clock.monotonic()
        return min(self.session.profile.control_period_s,
                   *(now - self._last_times[self._joints[n][0].name] for n in targets))

    def set_joint_positions_checked(self, targets, states):
        self._require(enabled=True)
        try:
            if self._snapshot is None or self._states is None or any(
                name not in self._states or state is not self._states[name] for name, state in states.items()
            ) or set(states) != set(self._states):
                raise RobotError("command requires the current complete feedback snapshot")
            if not targets or set(targets) - set(self._joints):
                raise RobotError("empty target or unknown joint")
            updated = {}
            for name, value in targets.items():
                group, joint = self._joints[name]
                value = float(value)
                if not math.isfinite(value):
                    raise RobotError(f"{name}: non-finite target")
                vector = updated.setdefault(group.name, list(self._targets[group.name]))
                vector[joint.index] = joint.to_sdk(value)
            now = self.session.clock.monotonic()
            # Validate the entire batch, including preserved slots, before publishing any group.
            for group in self.groups:
                if group.name not in updated:
                    continue
                vector = updated[group.name]
                if len(vector) != group.count or not all(math.isfinite(v) for v in vector):
                    raise RobotError(f"{group.name}: invalid final command dimension/values")
                dt = min(now - self._last_times[group.name], self.session.profile.control_period_s)
                if dt < 0:
                    raise RobotError("monotonic clock regressed")
                for joint in group.joints:
                    value = joint.from_sdk(vector[joint.index])
                    old = joint.from_sdk(self._targets[group.name][joint.index])
                    limit = joint.limits
                    if not limit.min_position <= value <= limit.max_position:
                        raise RobotError(f"{joint.name}: target outside limits")
                    change = abs(value - old)
                    if change > limit.max_position_step + 1e-10 or change > limit.max_velocity * dt + 1e-10:
                        raise RobotError(f"{joint.name}: target step/velocity exceeds limit")
            self.session.publish(self._token, updated, self._snapshot)
            self._targets.update(updated)
            for name in updated:
                self._last_times[name] = now
        except BaseException as exc:
            self._latch(exc)
            raise

    def disable(self):
        self.enabled = False  # Immediately reject new application targets.
        if self.connected:
            self.session.disable(self._token)

    def close(self):
        self.enabled = False
        if self.connected:
            try:
                self.session.detach(self._token)
            finally:
                self.connected = False

    def joint_key(self, side, joint, *, capability="joint_position", part="arm"):
        if side not in self.sides:
            raise ValueError(f"side {side} is not bound to this backend")
        key = f"{side}.{joint}" if len(self.sides) > 1 else joint
        if key not in self._joints:
            raise ValueError(f"unsupported joint: {side}.{joint}")
        group, entry = self._joints[key]
        if capability == "wrist_orientation" and entry.kind != "wrist":
            raise ValueError("wrist orientation requires a calibrated physical wrist joint")
        if group.part != part or capability not in group.capabilities:
            raise ValueError(f"missing {part}/{capability} capability")
        return key
