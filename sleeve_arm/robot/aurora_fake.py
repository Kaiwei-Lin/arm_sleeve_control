"""Offline simulator for the *verified SDK API*, never a DDS implementation.

Shapes and return contracts follow SDK 3d4e02e python_api.md/bind_types.cpp.
Geometry is intentionally synthetic and forbidden for real execution.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from enum import Enum, auto
from types import SimpleNamespace

from sleeve_arm.control.parameters import JointSafetyParameters
from sleeve_arm.robot.aurora_profile import API_FAMILY, SDK_VERSION, AuroraGroup, AuroraJoint, AuroraRobotProfile


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def time(self):
        return 1_800_000_000.0 + self.now

    def sleep(self, duration):
        if duration < 0:
            raise ValueError("negative sleep")
        self.now += duration


class ClientState(Enum):
    STOPPED = auto()
    RUNNING = auto()


class Endpoint(Enum):
    AURORA_STATE_SUBSCRIBER = auto()
    CONTROL_GROUP_STATE_SUBSCRIBER = auto()
    ERROR_CODES_SUBSCRIBER = auto()
    MANAGE_LEASE_SERVICE = auto()
    CONTROL_GROUP_COMMAND_PUBLISHER = auto()


class AuroraResource(Enum):
    LEFT_MANIPULATOR = auto()
    RIGHT_MANIPULATOR = auto()
    LEFT_END_EFFECTOR = auto()
    RIGHT_END_EFFECTOR = auto()


class LeaseState(Enum):
    UNREGISTERED = auto()
    ACTIVE = auto()
    LOST = auto()


@dataclass
class OperationResult:
    code: int = -1
    message: str = "CLIENT_ERROR"

    def success(self):
        return self.code == 0

    def __bool__(self):
        # Deliberately truthy, including failures, just like ordinary SDK objects.
        return True


@dataclass
class LeaseResult:
    operation: OperationResult = field(default_factory=OperationResult)
    state: LeaseState = LeaseState.UNREGISTERED
    resources: list = field(default_factory=list)
    remaining_duration: timedelta = timedelta(0)
    source_name: str = ""
    lease_id: str = ""

    def success(self):
        return self.operation.success()

    def usable(self):
        return self.state == LeaseState.ACTIVE and self.remaining_duration.total_seconds() > 0


class ConnectionOptions:
    def __init__(self):
        self.domain_id = 123
        self.namespace_name = None
        self.topic_prefix = "aurora/"
        self.dds_mode = 0
        self.is_ros_compatible = None
        self.use_namespace = True
        self.use_discovery_server = None
        self.service_timeout_ms = 1000
        self.source_name = ""
        self.allow_emergency_lease_priority = False
        self.enabled_endpoints = list(Endpoint)


@dataclass
class JointCommand:
    group_name: str = ""
    control_type: object = None
    motor_mode: object = None
    position: list = field(default_factory=list)
    velocity: list = field(default_factory=list)
    effort: list = field(default_factory=list)


@dataclass
class ControlGroupCommand:
    joint_cmd: list = field(default_factory=list)
    cartesian_cmd: list = field(default_factory=list)


JointControlType = Enum("JointControlType", "JOINT MOTOR")
MotorMode = Enum("MotorMode", "PD POSITION EFFORT")


class FakeAuroraClient:
    def __init__(self, profile, clock):
        self.profile, self.clock = profile, clock
        self.options = ConnectionOptions()
        self.running = False
        self.matched = True
        self.auto_feedback = True
        self.follow_commands = True
        self.failures = {}
        self.calls = []
        self.published = []
        self.callback = None
        self.lease = LeaseResult(OperationResult(0, "SUCCESS"))
        self._header = lambda: SimpleNamespace(received_time=datetime.fromtimestamp(self.clock.time()))
        self.aurora = SimpleNamespace(
            header=self._header(),
            robot_info=SimpleNamespace(robot_type=profile.robot_type, hardware_type=profile.hardware_type,
                                       end_effector_type=profile.end_effector_type),
            current_state=SimpleNamespace(id=profile.allowed_fsm[0], name="SIMULATION_CONTROL"),
        )
        self.groups = SimpleNamespace(header=self._header(), joint_state=[
            SimpleNamespace(group_name=g.name,
                            joint_position=[next(j for j in g.joints if j.index == i).to_sdk(0.0) for i in range(g.count)],
                            joint_velocity=[0.0] * g.count, joint_effort=[0.0] * g.count)
            for g in profile.groups
        ])
        self.errors = SimpleNamespace(header=self._header(), source=0, error_codes=[])

    def _result(self, operation):
        self.calls.append(operation)
        return self.failures.get(operation, OperationResult(0, "SUCCESS"))

    def state(self):
        return ClientState.RUNNING if self.running else ClientState.STOPPED

    def configure(self, options):
        self.calls.append("configure")
        assert not self.running
        self.options = deepcopy(options)

    def on_lease_changed(self, callback):
        assert not self.running
        self.callback = callback

    def start(self):
        self.calls.append("start")
        self.running = True

    def stop(self):
        self.calls.append("stop")
        self.running = False

    def wait_for_endpoints(self, timeout):
        self.calls.append("wait_for_endpoints")
        return self.matched

    def endpoint_match_status(self):
        return SimpleNamespace(**{e.name.lower(): SimpleNamespace(enabled=e in self.options.enabled_endpoints,
                                                                 matched=self.matched) for e in Endpoint})

    def _get(self, operation, value):
        if self.auto_feedback:
            value.header = self._header()
        return self._result(operation), deepcopy(value)

    def get_aurora_state(self):
        return self._get("get_aurora_state", self.aurora)

    def get_control_group_state(self):
        return self._get("get_control_group_state", self.groups)

    def get_error_codes(self):
        return self._get("get_error_codes", self.errors)

    def _acquire(self, operation, resources):
        result = self._result(operation)
        if result.success():
            self.lease = LeaseResult(result, LeaseState.ACTIVE, list(resources), timedelta(seconds=4),
                                     self.options.source_name, "fake-lease")
        else:
            return LeaseResult(result)
        if self.callback:
            self.callback(self.lease)
        return deepcopy(self.lease)

    def register_lease(self, resources, option=None):
        return self._acquire("register_lease", resources)

    def replace_lease(self, resources, option=None):
        return self._acquire("replace_lease", resources)

    def get_lease(self):
        result = self._result("get_lease")
        lease = deepcopy(self.lease)
        if not result.success():
            lease.operation = result
        return lease

    def release_lease(self):
        result = self._result("release_lease")
        if result.success():
            self.lease = LeaseResult(result)
        return LeaseResult(result, self.lease.state, self.lease.resources, self.lease.remaining_duration)

    def lose_lease(self):
        self.lease.state = LeaseState.LOST
        self.lease.remaining_duration = timedelta(0)
        if self.callback:
            self.callback(deepcopy(self.lease))

    def publish_control_group_command(self, command):
        result = self._result("publish_control_group_command")
        if result.success():
            assert self.lease.usable()
            self.published.append(deepcopy(command))
            if self.follow_commands:
                for entry in command.joint_cmd:
                    group = next(g for g in self.groups.joint_state if g.group_name == entry.group_name)
                    group.joint_position = list(entry.position)
        return result


def fake_profile(*, hands=False):
    """Synthetic layout, signs and limits; NOT a GR3 or other hardware profile."""
    groups = []
    for side in ("left", "right"):
        for part in (("arm", "hand") if hands else ("arm",)):
            names = (("shoulder_flexion", "shoulder_abduction", "elbow_flexion", "upper_arm_rotation",
                      "wrist_pitch", "wrist_yaw") if part == "arm" else ("index_flexion", "thumb_flexion", "thumb_opposition"))
            joints = tuple(AuroraJoint(
                name=name, index=i, sign=(-1 if (i + (side == "right")) % 3 == 0 else 1), zero=0.07 * (i + 1),
                limits=JointSafetyParameters(name, -2.0, 2.0, 0.8, None, 0.25, 0.04), verified=True,
                open_position=0.0 if part == "hand" else None,
                closed_position=0.5 if part == "hand" else None,
                kind="finger" if part == "hand" else ("wrist" if name.startswith("wrist_") else "arm"),
            ) for i, name in enumerate(names))
            group_name = f"{side}_" + ("manipulator" if part == "arm" else "end_effector")
            capabilities = ("joint_position", "wrist_orientation") if part == "arm" else ("joint_position", "hand_joints", "hand_closure")
            groups.append(AuroraGroup(group_name, side, part, group_name.upper(), len(joints), "PD", joints, capabilities, True))
    return AuroraRobotProfile(
        API_FAMILY, SDK_VERSION, "OFFLINE_SIMULATOR", "SYNTHETIC", "SYNTHETIC_HAND" if hands else None,
        {"domain_id": 0, "source_name": "sleeve-offline", "topic_prefix": "unused/", "dds_mode": 0, "use_namespace": False},
        tuple(groups), (999,), 0.25, 1.0, 0.02, 0.005, 2.0, "stop_publishing_release", True, True,
        "Synthetic fixture only; no physical hardware claims",
    )


def fake_session(*, execute=True, hands=False, clock=None):
    import sys
    from sleeve_arm.robot.aurora_session import AuroraSession
    clock = clock or FakeClock()
    profile = fake_profile(hands=hands)
    client = FakeAuroraClient(profile, clock)
    return AuroraSession(profile, execute=execute, operator_confirmed=True, sdk=sys.modules[__name__],
                         client=client, sdk_version=SDK_VERSION, clock=clock, simulation=True)
