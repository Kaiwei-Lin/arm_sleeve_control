"""SDK 1.0.1 adapter and single-owner, reference-counted Aurora session.

No import, DDS startup, lease, or FSM operation happens at module import time.
"""
from __future__ import annotations

import importlib
import importlib.metadata
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any

from sleeve_arm.robot.aurora_profile import AuroraRobotProfile, SDK_VERSION
from sleeve_arm.robot.base import RobotError


class SystemClock:
    monotonic = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)
    time = staticmethod(time.time)


@dataclass(frozen=True)
class AuroraSnapshot:
    aurora: Any
    groups: Any
    received_at: float
    age_s: float


class AuroraSession:
    _registry_lock = threading.RLock()
    _real_session: AuroraSession | None = None

    @classmethod
    def real(cls, profile: AuroraRobotProfile, *, execute: bool = False,
             operator_confirmed: bool = False) -> AuroraSession:
        """Reuse the process singleton only for identical, explicitly authorized settings."""
        profile.validate(execute=execute)
        if execute and operator_confirmed is not True:
            raise RobotError("real execute requires explicit operator confirmation")
        with cls._registry_lock:
            old = cls._real_session
            if old is not None and not old.closed:
                if old.profile != profile or old.execute != execute:
                    raise RobotError("conflicting Aurora singleton configuration; close all existing views first")
                return old
            session = cls(profile, execute=execute, operator_confirmed=operator_confirmed)
            cls._real_session = session
            return session

    def __init__(self, profile: AuroraRobotProfile, *, execute: bool = False,
                 operator_confirmed: bool = False, sdk=None, client=None,
                 sdk_version: str | None = None, clock=None, simulation: bool = False):
        profile.validate(execute=execute, simulation=simulation)
        if execute and operator_confirmed is not True:
            raise RobotError("execute requires explicit operator confirmation")
        if simulation and (sdk is None or client is None):
            raise RobotError("simulation requires an injected SDK and client")
        if sdk_version is not None and sdk_version != SDK_VERSION:
            raise RobotError(f"unsupported SDK version {sdk_version}; require {SDK_VERSION}; no fallback")
        self.profile = profile
        self.execute = execute
        self.clock = clock or SystemClock()
        self.sdk = sdk
        self.client = client
        self.started = False
        self.closed = False
        self.fault: str | None = None
        self._owner_thread: int | None = None
        self._views: dict[object, set[str]] = {}
        self._active: dict[object, set[str]] = {}
        self._lease_attempted = False
        self._lease_lost = threading.Event()
        self._timestamps: dict[str, tuple[float, float]] = {}
        self._lock = threading.RLock()
        self._endpoints: list[str] = []

    def _fail(self, message: str):
        self.fault = self.fault or message
        raise RobotError(message)

    def _check_fault(self):
        if self._lease_lost.is_set():
            self._fail("lease lost (callback); explicit close/reconfirmation required")
        if self.fault:
            raise RobotError(f"Aurora session fault latched: {self.fault}")
        if self.closed:
            raise RobotError("Aurora session is closed; create a new explicitly confirmed session")

    def _owner(self):
        if self._owner_thread != threading.get_ident():
            self._fail("only the session command owner thread may change control state")

    def _call(self, name, *args):
        try:
            return getattr(self.client, name)(*args)
        except Exception as exc:
            self._fail(f"{name}: SDK exception {type(exc).__name__}: {exc}")

    def _operation(self, name, result):
        if result.success() is not True:
            self._fail(f"{name}: code={result.code} message={result.message}")

    def _void(self, name, *args):
        result = self._call(name, *args)
        if result is not None:
            self._fail(f"{name}: incompatible SDK return contract (expected None)")

    def _lease(self, name, resources=None, *, usable=True):
        result = self._call(name, resources) if resources is not None else self._call(name)
        self._operation(name, result.operation)
        if usable and result.usable() is not True:
            self._fail(f"{name}: lease unusable; code={result.operation.code} message={result.operation.message}")
        if usable and resources is not None and set(result.resources) != set(resources):
            self._fail(f"{name}: granted resource set differs from request")
        return result

    def _on_lease_changed(self, result):
        # SDK callback thread: only set a thread-safe latch. Never stop/release here.
        if result.state == self.sdk.LeaseState.LOST:
            self._lease_lost.set()

    def attach(self, token: object, groups: set[str]):
        with self._lock:
            self._check_fault()
            if token in self._views:
                raise RobotError("view already connected")
            if any(groups & owned for owned in self._views.values()):
                raise RobotError("control group already belongs to another session view")
            if not self.started:
                self._start()
            self._owner()
            self._views[token] = set(groups)

    def _start(self):
        self._owner_thread = threading.get_ident()
        if self.sdk is None:
            try:
                version = importlib.metadata.version("fourier-aurora-client")
            except importlib.metadata.PackageNotFoundError as exc:
                raise RobotError("Aurora SDK missing: install the platform's official fourier_aurora_client-1.0.1 wheel "
                                 "and matching DDS runtimes; see docs/aurora_control.md (no automatic installation)") from exc
            if version != SDK_VERSION:
                raise RobotError(f"unsupported SDK version {version}; require {SDK_VERSION}; no legacy fallback")
            try:
                self.sdk = importlib.import_module("fourier_aurora_client")
            except (ImportError, OSError) as exc:
                raise RobotError(f"Aurora SDK/runtime import failed: {exc}; see docs/aurora_control.md") from exc
        if self.client is None:
            self.client = self.sdk.AuroraClient.get_instance()
        if self._call("state") != self.sdk.ClientState.STOPPED:
            raise RobotError("Aurora singleton already running outside this session; refusing to reconfigure/stop it")
        self._endpoints = ["AURORA_STATE_SUBSCRIBER", "CONTROL_GROUP_STATE_SUBSCRIBER"]
        if self.execute:
            self._endpoints += ["ERROR_CODES_SUBSCRIBER", "MANAGE_LEASE_SERVICE", "CONTROL_GROUP_COMMAND_PUBLISHER"]
        options = self.sdk.ConnectionOptions()
        for name, value in self.profile.connection.items():
            setattr(options, name, value)
        options.enabled_endpoints = [getattr(self.sdk.Endpoint, name) for name in self._endpoints]
        options.allow_emergency_lease_priority = False
        try:
            self._void("configure", options)
            if self.execute:
                self._void("on_lease_changed", self._on_lease_changed)
            self._void("start")
            self.started = True
            if self._call("wait_for_endpoints", self.profile.endpoint_timeout_s) is not True:
                self._fail("wait_for_endpoints: required DDS endpoints did not match")
        except BaseException:
            try:
                self._void("stop")
            finally:
                self.started = False
                self.closed = True
            raise

    def _endpoints_ready(self):
        status = self._call("endpoint_match_status")
        for name in self._endpoints:
            entry = getattr(status, name.lower())
            if not entry.enabled or not entry.matched:
                self._fail(f"endpoint_match_status: required {name} disabled/unmatched")

    def _read(self, operation):
        result, value = self._call(operation)
        self._operation(operation, result)
        return value

    def _age(self, label, header):
        if not isinstance(header.received_time, datetime):
            self._fail(f"{label}: SDK received_time is missing/invalid")
        stamp = header.received_time.timestamp()
        now = self.clock.monotonic()
        wall_age = self.clock.time() - stamp
        if wall_age < -0.05:
            self._fail(f"{label}: receive timestamp is in the future")
        previous = self._timestamps.get(label)
        if previous is not None and stamp < previous[0]:
            self._fail(f"{label}: receive timestamp regressed")
        if previous is None or stamp > previous[0]:
            self._timestamps[label] = (stamp, now - max(0.0, wall_age))
        received = self._timestamps[label][1]
        age = max(now - received, wall_age)
        if age > self.profile.feedback_timeout_s:
            self._fail(f"{label}: stale feedback ({age:.3f}s)")
        return received, age

    def _health(self, *, controlling=False):
        self._check_fault()
        if not self.started:
            raise RobotError("Aurora session not connected")
        self._endpoints_ready()
        aurora = self._read("get_aurora_state")
        self._age("aurora_state", aurora.header)
        if controlling or self._active:
            for field in ("robot_type", "hardware_type", "end_effector_type"):
                expected = getattr(self.profile, field)
                if expected is not None and getattr(aurora.robot_info, field) != expected:
                    self._fail(f"robot identity mismatch: {field}")
            if aurora.current_state.id not in self.profile.allowed_fsm:
                self._fail(f"FSM {aurora.current_state.id} not allowed; no automatic FSM switching")
            errors = self._read("get_error_codes")
            if errors.error_codes:
                codes = [(e.high32, e.low32) for e in errors.error_codes]
                self._fail(f"asynchronous Aurora errors (not per-motor status): {codes}")
        if self._active:
            lease = self._lease("get_lease")
            if not set(self._resources(self._active_groups())) <= set(lease.resources):
                self._fail("get_lease: active resources lost")
        return aurora

    def snapshot(self, *, controlling=False) -> AuroraSnapshot:
        with self._lock:
            aurora = self._health(controlling=controlling)
            groups = self._read("get_control_group_state")
            received, age = self._age("control_group_state", groups.header)
            return AuroraSnapshot(aurora, groups, received, age)

    def _resources(self, names):
        resources = {g.resource for g in self.profile.groups if g.name in names}
        return [getattr(self.sdk.AuroraResource, r) for r in sorted(resources)]

    def _active_groups(self):
        return set().union(*self._active.values()) if self._active else set()

    def enable(self, token):
        with self._lock:
            self._owner()
            self._check_fault()
            if not self.execute:
                raise RobotError("read-only session cannot enable; --execute and confirmation required")
            self._health(controlling=True)
            if token not in self._views:
                raise RobotError("unconnected session view")
            if token in self._active:
                self._lease("get_lease")
                return
            resources = self._resources(self._active_groups() | self._views[token])
            operation = "replace_lease" if self._active else "register_lease"
            self._lease_attempted = True
            self._lease(operation, resources)
            self._check_fault()
            self._active[token] = self._views[token]

    def publish(self, token, positions: dict[str, list[float]], snapshot: AuroraSnapshot):
        with self._lock:
            self._owner()
            self._check_fault()
            if token not in self._active or not positions or not set(positions) <= self._active[token]:
                raise RobotError("publish outside this enabled view's groups")
            if self.clock.monotonic() - snapshot.received_at > self.profile.feedback_timeout_s:
                self._fail("publish: stale control group snapshot")
            self._health(controlling=True)
            required = self._resources(self._active_groups())
            lease = self._lease("get_lease")
            if not set(required) <= set(lease.resources):
                self._fail("get_lease: active resources lost")
            # All backend vectors have already been validated before constructing any message.
            command = self.sdk.ControlGroupCommand()
            entries = []
            for name, values in positions.items():
                group = next(g for g in self.profile.groups if g.name == name)
                entry = self.sdk.JointCommand()
                entry.group_name = name
                entry.control_type = self.sdk.JointControlType.JOINT
                entry.motor_mode = getattr(self.sdk.MotorMode, group.motor_mode)
                entry.position = list(values)
                entry.velocity = [0.0] * len(values)
                entry.effort = [0.0] * len(values)
                entries.append(entry)
            command.joint_cmd = entries
            self._check_fault()
            self._operation("publish_control_group_command", self._call("publish_control_group_command", command))

    def disable(self, token):
        with self._lock:
            self._owner()
            was_active = self._active.pop(token, None) is not None  # Block before any service call.
            if not was_active and self._active:
                return
            if not self._lease_attempted:
                return
            if self._active and not self.fault and not self._lease_lost.is_set():
                self._lease("replace_lease", self._resources(self._active_groups()))
            else:
                # Do not publish hold commands, reset pose, switch FSM, or servo off.
                self._lease("release_lease", usable=False)
                self._lease_attempted = False

    def detach(self, token):
        with self._lock:
            if token not in self._views:
                return
            errors = []
            try:
                self.disable(token)
            except BaseException as exc:
                errors.append(exc)
            self._views.pop(token, None)
            if not self._views:
                try:
                    if self._lease_attempted:
                        self._lease("release_lease", usable=False)
                        self._lease_attempted = False
                except BaseException as exc:
                    errors.append(exc)
                try:
                    self._void("stop")
                except BaseException as exc:
                    errors.append(exc)
                finally:
                    self.started = False
                    self.closed = True
            if errors:
                raise RobotError("Aurora cleanup: " + "; ".join(str(e) for e in errors))

    def doctor(self):
        snapshot = self.snapshot()
        info = snapshot.aurora.robot_info
        return {
            "api_family": self.profile.api_family, "sdk_version": self.profile.sdk_version,
            "robot_type": info.robot_type, "hardware_type": info.hardware_type,
            "end_effector_type": info.end_effector_type,
            "fsm": {"id": snapshot.aurora.current_state.id, "name": snapshot.aurora.current_state.name},
            "feedback_age_s": snapshot.age_s,
            "groups": {g.group_name: {"position_count": len(g.joint_position),
                                       "velocity_count": len(g.joint_velocity), "effort_count": len(g.joint_effort)}
                       for g in snapshot.groups.joint_state},
            "profile_verified": self.profile.verified, "simulated": self.profile.simulated,
            "execute_authorized": self.execute,
        }
