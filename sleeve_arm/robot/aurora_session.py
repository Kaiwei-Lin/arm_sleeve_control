"""Pinned 0.1.8 SDK adapter and shared process-client lifecycle.

Only public client calls are used. A read-only check of _instance prevents
adopting an externally initialized SDK singleton; it is never changed/reset.
"""
from __future__ import annotations

import importlib
import importlib.metadata
import logging
import threading
import time
from dataclasses import dataclass

from sleeve_arm.robot.aurora_profile import SDK_VERSION
from sleeve_arm.robot.base import RobotError


class SystemClock:
    monotonic = staticmethod(time.monotonic)
    sleep = staticmethod(time.sleep)


@dataclass(frozen=True)
class GroupSnapshot:
    position: tuple
    velocity: tuple
    effort: tuple
    observed_at: float | None
    read_at: float
    revision: int


@dataclass(frozen=True)
class Snapshot:
    fsm: int
    groups: dict[str, GroupSnapshot]
    read_at: float


class _SdkErrors(logging.Handler):
    def __init__(self):
        super().__init__(logging.ERROR)
        self.event = threading.Event()
        self.message = ''

    def emit(self, record):
        # SDK background callbacks/loggers only set a flag. No SDK operations.
        self.message = record.getMessage()
        self.event.set()


class AuroraSession:
    _process_session = None
    _registry_lock = threading.RLock()

    def __init__(self, profile, *, execute=False, operator_confirmed=False,
                 sdk=None, clock=None, simulation=False):
        profile.validate()
        self.profile = profile
        self.execute = bool(execute)
        self.operator_confirmed = bool(operator_confirmed)
        self.simulation = simulation
        self.sdk = sdk
        self.clock = clock or SystemClock()
        self.client = None
        self.closed = False
        self.fault = None
        self._refs = {}
        self._enabled = set()
        self._blocked_groups = set()
        self._owner_thread = None
        self._cache = {}
        self._lock = threading.RLock()
        self._errors = _SdkErrors()
        self._loggers = []

    @classmethod
    def real(cls, profile, *, execute=False, operator_confirmed=False):
        with cls._registry_lock:
            existing = cls._process_session
            if existing is not None:
                if existing.closed or existing.fault:
                    raise RobotError('0.1.8 singleton closed/faulted; restart process before reconfirming')
                if (existing.profile != profile or existing.execute != execute
                        or existing.operator_confirmed != operator_confirmed):
                    raise RobotError('conflicting Aurora process configuration/authorization')
                return existing
            result = cls(profile, execute=execute, operator_confirmed=operator_confirmed)
            cls._process_session = result
            return result

    def _check_fault(self):
        if self._errors.event.is_set():
            self.fault = self.fault or f'SDK error log: {self._errors.message}'
        if self.closed or self.fault:
            raise RobotError(f'Aurora session closed/fault latched: {self.fault}; reconstruct/reconfirm in a new process')

    def _load_sdk(self):
        if self.sdk is None:
            try:
                version = importlib.metadata.version('fourier_aurora_client')
            except importlib.metadata.PackageNotFoundError as exc:
                raise RobotError('Aurora SDK missing: python -m pip install -r requirements-aurora.txt; no automatic installation') from exc
            if version != SDK_VERSION:
                raise RobotError(f'unsupported SDK version {version}; requires 0.1.8; no automatic fallback')
            self.sdk = importlib.import_module('fourier_aurora_client')
        client_class = self.sdk.AuroraClient
        required = ('get_instance', 'get_fsm_state', 'get_group_state', 'set_group_cmd', 'close')
        if not all(callable(getattr(client_class, name, None)) for name in required):
            raise RobotError('incompatible 0.1.8 API surface')
        if any(hasattr(client_class, name) for name in ('configure', 'start', 'register_lease')):
            raise RobotError('mixed/new API family rejected')
        return client_class

    def _call(self, method, *args, **kwargs):
        self._check_fault()
        try:
            value = getattr(self.client, method)(*args, **kwargs)
        except KeyError as exc:
            raise RobotError(f'{method}: missing state/group: {exc}') from exc
        except Exception as exc:
            self.fault = f'{method}: {type(exc).__name__}: {exc}'
            raise RobotError(self.fault) from exc
        self._check_fault()
        return value

    def attach(self, token, groups):
        with self._lock:
            self._check_fault()
            selected = tuple(g for g in self.profile.groups if g.name in groups)
            if not groups or len(selected) != len(groups) or None in groups:
                raise RobotError('explicit configured group names required; use SDK doctor for discovery')
            if groups & self._blocked_groups:
                raise RobotError('group fault latched; restart process and reconfirm')
            if any(groups & other for other in self._refs.values()):
                raise RobotError('group already belongs to another application view')
            if self.profile.connection.get('domain_id') is None:
                raise RobotError('domain_id is unverified')
            if any(g.count is None for g in selected):
                raise RobotError('group DOF is unverified')
            if self.execute:
                self.profile.validate(execute=True, simulation=self.simulation, groups=selected)
            if self.client is None:
                try:
                    klass = self._load_sdk()
                    if getattr(klass, '_instance', None) is not None:
                        raise RobotError('external/stale Aurora singleton exists; restart with one owner')
                    if not self.simulation:
                        for name in ('fourier_aurora_client.client', 'fourier_aurora_client.dds_interface'):
                            logger = logging.getLogger(name)
                            logger.addHandler(self._errors)
                            self._loggers.append(logger)
                    self.client = klass.get_instance(**self.profile.connection)
                    if self.client is None:
                        raise RobotError('get_instance returned None: SDK initialization/discovery failed')
                    self._check_fault()
                except BaseException as exc:
                    self.fault = str(exc)
                    self._close_client()
                    raise
            self._refs[token] = set(groups)
            try:
                # First cache observation has unknown age. Require a replacement
                # by an actual callback before accepting a group as fresh.
                deadline = self.clock.monotonic() + self.profile.endpoint_timeout_s
                while True:
                    snapshot = self.snapshot(groups, require_fresh=False)
                    if all(g.observed_at is not None for g in snapshot.groups.values()):
                        self.check_snapshot(snapshot)
                        break
                    if self.clock.monotonic() >= deadline:
                        raise RobotError('fresh feedback timeout: no new cache sample observed')
                    self.clock.sleep(min(0.01, max(0, deadline - self.clock.monotonic())))
            except BaseException:
                self.detach(token)
                raise

    def snapshot(self, names, *, require_fresh=True):
        with self._lock:
            fsm = self._call('get_fsm_state')
            if type(fsm) is not int or fsm < 0:
                raise RobotError('get_fsm_state returned invalid FSM')
            groups = {}
            for name in names:
                raw = self._call('get_group_state', name, key='position')
                if not isinstance(raw, list):
                    raise RobotError('get_group_state(position) must return the 0.1.8 cached list')
                now = self.clock.monotonic()
                previous = self._cache.get(name)
                observed, revision = None, 0
                if previous:
                    old, last_read, observed, revision = previous
                    if raw is not old:
                        # Callback occurred after previous observation. Use that
                        # earlier time conservatively; never claim robot sample time.
                        observed, revision = last_read, revision + 1
                self._cache[name] = (raw, now, observed, revision)
                # Position is copied once for ALL semantic joints in this group.
                position = tuple(raw)
                group = next(g for g in self.profile.groups if g.name == name)
                group.check_vector(position)
                velocity = tuple(self._call('get_group_state', name, key='velocity'))
                effort = tuple(self._call('get_group_state', name, key='effort'))
                for label, values in (('velocity', velocity), ('effort', effort)):
                    if len(values) not in (0, group.count):
                        raise RobotError(f'{name}: {label} vector dimension mismatch')
                    from sleeve_arm.robot.aurora_profile import finite
                    for v in values:
                        finite(v, label)
                groups[name] = GroupSnapshot(position, velocity, effort, observed, now, revision)
            result = Snapshot(fsm, groups, self.clock.monotonic())
            if require_fresh:
                self.check_snapshot(result)
            return result

    def fresh_snapshot(self, names):
        """Reacquire two cache observations while application motion is disabled.

        After operator confirmation or long calibration, one read cannot bound
        the age of a replaced cache. Wait for a subsequent actual update.
        """
        deadline = self.clock.monotonic() + self.profile.endpoint_timeout_s
        while True:
            snapshot = self.snapshot(names, require_fresh=False)
            now = self.clock.monotonic()
            if all(g.observed_at is not None and 0 <= now - g.observed_at <= self.profile.feedback_timeout_s
                   for g in snapshot.groups.values()):
                return snapshot
            if now >= deadline:
                raise RobotError('fresh feedback timeout: cache did not update')
            self.clock.sleep(min(0.01, max(0, deadline - now)))

    def check_snapshot(self, snapshot):
        self._check_fault()
        now = self.clock.monotonic()
        for name, value in snapshot.groups.items():
            if value.observed_at is None or not 0 <= now - value.observed_at <= self.profile.feedback_timeout_s:
                raise RobotError(f'{name}: stale/unknown-age cache feedback')

    def _check_fsm(self):
        fsm = self._call('get_fsm_state')
        if type(fsm) is not int or fsm not in self.profile.allowed_fsm:
            self.fault = f'FSM {fsm} is not in verified allowed_fsm'
            raise RobotError(self.fault)

    def enable(self, token):
        with self._lock:
            self._check_fault()
            if token not in self._refs:
                raise RobotError('view not connected')
            if not self.execute or not self.operator_confirmed:
                raise RobotError('read-only: execute and explicit operator confirmation required')
            groups = [g for g in self.profile.groups if g.name in self._refs[token]]
            self.profile.validate(execute=True, simulation=self.simulation, groups=groups)
            if self._owner_thread is not None and self._owner_thread != threading.get_ident():
                raise RobotError('only one command owner thread is allowed')
            self._check_fsm()
            self._owner_thread = threading.get_ident()
            self._enabled.add(token)

    def publish(self, token, vectors, snapshot):
        with self._lock:
            self._check_fault()
            if token not in self._enabled or threading.get_ident() != self._owner_thread:
                raise RobotError('write blocked: disabled view or wrong command owner')
            if not vectors or set(vectors) - self._refs[token]:
                raise RobotError('command contains an unowned group')
            self.check_snapshot(snapshot)
            for name, vector in vectors.items():
                next(g for g in self.profile.groups if g.name == name).check_vector(vector)
            self._check_fsm()  # Immediately before every command; never changes FSM.
            result = self._call('set_group_cmd', position_cmd={k: list(v) for k, v in vectors.items()})
            if result is not None:
                self.fault = f'set_group_cmd: unexpected return {result!r}; 0.1.8 returns None (no acknowledgement)'
                raise RobotError(self.fault)
            # None means the call returned, NOT delivery or physical success.

    def disable(self, token, *, fault=False):
        with self._lock:
            self._enabled.discard(token)
            if fault:
                self._blocked_groups.update(self._refs.get(token, ()))

    def detach(self, token):
        with self._lock:
            self.disable(token)
            self._refs.pop(token, None)
            if not self._refs:
                self._close_client()

    def _close_client(self):
        if self.closed:
            return
        self.closed = True  # Block writes before cleanup, including cleanup failure.
        self._enabled.clear()
        try:
            if self.client is not None:
                result = self.client.close()
                if result is not None:
                    raise RobotError('close: unexpected SDK result')
        finally:
            for logger in self._loggers:
                logger.removeHandler(self._errors)
            self._loggers.clear()
