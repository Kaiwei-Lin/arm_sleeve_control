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
    disable_operation_name = "application disable"

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
        self._command_versions = {}
        self._submitted_at = {}

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
        self.session.disable(self._token, fault=True)

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
            # Re-read after the FSM/application gate before establishing initial targets.
            self.read_joint_states()
            entries = self._snapshot.groups
            self._targets = {g.name: list(entries[g.name].position) for g in self.groups}
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
            reader = self.session.snapshot if self.enabled else self.session.fresh_snapshot
            snapshot = reader({g.name for g in self.groups})
            if self.enabled:
                self.session._check_fsm()
            entries = snapshot.groups
            states = {}
            for group in self.groups:
                entry = entries[group.name]
                group.check_vector(entry.position)
                if self.enabled and any(abs(a - b) > group.max_tracking_error
                                        for a, b in zip(entry.position, self._targets[group.name])):
                    raise RobotError(f"{group.name}: full group tracking error exceeds limit")
            for key, (group, joint) in self._joints.items():
                entry = entries[group.name]
                state = JointState(
                    name=key, position=joint.from_sdk(entry.position[joint.index]),
                    velocity=joint.sign * entry.velocity[joint.index] if entry.velocity else None,
                    torque=joint.sign * entry.effort[joint.index] if entry.effort else None,
                    current=None, state=None, bus=None, error=None,
                    received_at=entry.observed_at,
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
            self._command_versions.update({name: self._snapshot.groups[name].revision for name in updated})
            self._submitted_at.update({name: self.session.clock.monotonic() for name in updated})
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

    def read_group_positions(self, side):
        """Complete SDK-coordinate vector, copied from one cached group sample."""
        self._require()
        groups = [g for g in self.groups if g.side == side and g.part == "arm"]
        if len(groups) != 1:
            raise ValueError("unbound or unsupported arm side")
        try:
            snapshot = self.session.snapshot({groups[0].name})
            return list(snapshot.groups[groups[0].name].position)
        except BaseException as exc:
            self._latch(exc)
            raise

    def arrival_feedback_is_new(self, targets):
        for key in targets:
            name = self._joints[key][0].name
            sample = self._snapshot.groups[name]
            if (sample.revision <= self._command_versions.get(name, -1) or sample.observed_at is None
                    or sample.observed_at < self._submitted_at.get(name, float('inf'))):
                return False
        return True

    def set_hand_joint(self, side, joint, position):
        self.set_hand_joints(side, {joint: position})

    def set_hand_joints(self, side, targets):
        mapped = {self.joint_key(side, name, capability="hand_joints", part="hand"): value
                  for name, value in targets.items()}
        self.set_joint_positions(mapped)
