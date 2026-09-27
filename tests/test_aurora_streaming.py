"""GR3 streaming and exact DDS compatibility contracts; injected SDKs only."""
from dataclasses import replace
import logging
from types import SimpleNamespace

import pytest

from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.robot.aurora import AuroraRobotArm
from sleeve_arm.robot.aurora_fake import fake_session, gr3_fake_profile
from sleeve_arm.robot.aurora_sdk import initialize_client, velocity_cmd_only_mismatch
from sleeve_arm.robot.aurora_session import _SdkErrors
from sleeve_arm.robot.base import RobotError


def connected(side="right", profile=None):
    session = fake_session(profile=profile or gr3_fake_profile())
    robot = AuroraRobotArm(session, sides=(side,))
    controller = SafeArmController(robot, robot.config, clock=session.clock)
    controller.connect()
    return session, robot, controller


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("joint,index", [("shoulder_flexion", 0), ("elbow_flexion", 3),
                                       ("shoulder_abduction", 1), ("upper_arm_rotation", 2)])
def test_gr3_routing_complete_vector_and_uncontrolled_joints(side, joint, index):
    session, robot, controller = connected(side)
    try:
        robot.prepare_control_mode(operator_confirmed=True)
        controller.enable()
        group = f"{side}_manipulator"
        before = list(session.client.groups[group]["position"])
        session.clock.sleep(.01)
        targets = controller.set_joint_positions({joint: .001}, dt=.01)
        assert set(session.client.commands[-1]) == {group}
        after = session.client.commands[-1][group]
        assert len(after) == 7
        entry = next(j for j in robot.groups[0].joints if j.name == joint)
        assert entry.index == index
        assert after[index] == pytest.approx(entry.to_sdk(targets[joint]))
        assert [v for i, v in enumerate(after) if i != index] == [v for i, v in enumerate(before) if i != index]
        assert "set_fsm_state" not in session.client.calls
    finally:
        controller.shutdown()


@pytest.mark.parametrize("pose", [[0, 0, 0, 100], [0, 0, 0, 99], [0, 0, 0, float("nan")], [0, 0, 101]])
def test_pdstand_requires_valid_stable_feedback_before_enable(pose):
    session, robot, controller = connected()
    try:
        session.client.stand_pose = pose
        with pytest.raises(RobotError):
            robot.prepare_control_mode(operator_confirmed=True)
        assert not session.client.commands
        assert "set_fsm_state" not in session.client.calls
    finally:
        controller.shutdown()


@pytest.mark.parametrize("fsm", [0, 3, 10, 11])
def test_wrong_standing_fsm_never_sends_or_switches_by_default(fsm):
    session, robot, controller = connected()
    try:
        session.client.fsm = fsm
        with pytest.raises(RobotError, match="FSM 2"):
            robot.prepare_control_mode(operator_confirmed=True)
        assert not session.client.commands and "set_fsm_state" not in session.client.calls
    finally:
        controller.shutdown()


@pytest.mark.parametrize("damage", ["fsm", "stable"])
def test_guard_is_rechecked_before_send_and_fault_blocks_future_commands(damage):
    profile = replace(gr3_fake_profile(), allowed_fsm=(2, 10))
    session, robot, controller = connected(profile=profile)
    try:
        robot.prepare_control_mode(operator_confirmed=True)
        controller.enable()
        if damage == "fsm":
            session.client.fsm = 10
        else:
            session.client.stand_pose[3] = 100
        session.clock.sleep(.01)
        with pytest.raises(Exception):
            controller.set_joint_positions({"shoulder_flexion": .001}, dt=.01)
        assert controller.fault and not session.client.commands
        with pytest.raises(Exception):
            controller.set_joint_positions({"shoulder_flexion": .001}, dt=.01)
    finally:
        controller.shutdown()


def test_explicit_prepare_requires_confirmation_and_waits_for_stance(monkeypatch):
    session, robot, controller = connected()
    try:
        session.client.fsm = 0
        with pytest.raises(RobotError, match="confirmation"):
            robot.prepare_control_mode(prepare_fsm=True)
        assert "set_fsm_state" not in session.client.calls
        began = session.clock.now
        monkeypatch.setattr(session.client, "get_stand_pose", lambda: [0, 0, 0, 101 if session.clock.now - began > .1 else 100])
        robot.prepare_control_mode(prepare_fsm=True, operator_confirmed=True)
        assert session.client.fsm == 2 and session.clock.now - began > .1
        assert session.client.calls.count("set_fsm_state") == 1
        assert not session.client.commands
    finally:
        controller.shutdown()


def test_explicit_prepare_timeout_never_enables_or_sends(monkeypatch):
    session, robot, controller = connected()
    try:
        session.client.fsm = 0
        requests = []
        monkeypatch.setattr(session.client, "set_fsm_state", requests.append)
        began = session.clock.now
        with pytest.raises(RobotError, match="timed out"):
            robot.prepare_control_mode(prepare_fsm=True, operator_confirmed=True)
        assert requests == [2]
        assert session.clock.now - began == pytest.approx(session.profile.stable_wait_seconds)
        assert not controller.enabled and not session.client.commands
    finally:
        controller.shutdown()


@pytest.mark.parametrize("names,allowed", [
    (["velocity_cmd"], True), (["gr3/velocity_cmd"], True),
    ([], False), (["velocity_cmd", "robot_control_group_cmd"], False),
    (["velocity_cmd", "velocity_cmd"], False), (["not_velocity_cmd"], False),
    (["velocity_cmd_extra"], False), (["robot_control_group_cmd"], False), (None, False),
    ("Unmatched publisher: velocity_cmd", False),
])
def test_velocity_exception_is_exactly_one_verified_publisher(names, allowed):
    assert velocity_cmd_only_mismatch(names) is allowed


@pytest.mark.parametrize("names", [["velocity_cmd"], ["velocity_cmd", "robot_control_group_cmd"],
                                  ["robot_control_group_cmd"], []])
def test_compatibility_checks_publishers_and_always_restores_sdk_method(monkeypatch, names):
    class Timeout(Exception):
        pass

    class Client:
        def _init_publishers(self):
            self._publisher_list = [SimpleNamespace(is_matched=False, _topic=SimpleNamespace(get_name=lambda name=name: name)) for name in names]
            raise Timeout("publishers")

        @classmethod
        def get_instance(cls, **kwargs):
            obj = cls()
            obj._init_publishers()
            return obj

    monkeypatch.setattr("sleeve_arm.robot.aurora_sdk.importlib.import_module", lambda _: SimpleNamespace(DDSMatchTimeout=Timeout))
    original = Client._init_publishers
    accepted = []
    profile = replace(gr3_fake_profile(), allow_missing_velocity_cmd=True)
    if names == ["velocity_cmd"]:
        assert initialize_client(Client, profile, accepted_velocity_mismatch=lambda: accepted.append(True))
        assert accepted == [True]
    else:
        with pytest.raises(Timeout):
            initialize_client(Client, profile, accepted_velocity_mismatch=lambda: accepted.append(True))
        assert accepted == []
    assert Client._init_publishers is original


@pytest.mark.parametrize("unrelated", ["Unmatched subscriber: velocity_cmd", "Unmatched service client: policy", "write failed"])
def test_velocity_log_exception_does_not_clear_any_other_sdk_error(unrelated):
    errors = _SdkErrors()
    for message in (unrelated, "Unmatched publisher: gr3/velocity_cmd"):
        errors.emit(logging.LogRecord("fourier_aurora_client.client", logging.ERROR, "", 0, message, (), None))
    errors.accept_velocity_mismatch()
    assert errors.event.is_set() and errors.message == unrelated
    errors = _SdkErrors()
    errors.emit(logging.LogRecord("fourier_aurora_client.client", logging.ERROR, "", 0,
                                 "Unmatched publisher: gr3/velocity_cmd", (), None))
    errors.accept_velocity_mismatch()
    assert not errors.event.is_set()


def test_dds_environment_is_set_before_sdk_factory_without_real_dds(monkeypatch):
    profile = replace(gr3_fake_profile(), simulated=False)
    session = fake_session(profile=profile)
    session.simulation = False
    seen = []
    factory = session.sdk.AuroraClient.get_instance

    def get_instance(**kwargs):
        import os
        seen.append(tuple(os.environ[n] for n in ("FASTDDS_BUILTIN_TRANSPORTS", "FOURIERDDS_ROS_COMPATIBLE", "FOURIERDDS_USE_DISCOVERY_SERVER")))
        assert kwargs == dict(domain_id=123, robot_name="gr3", is_ros_compatible=False)
        return factory(**kwargs)

    for name in ("FASTDDS_BUILTIN_TRANSPORTS", "FOURIERDDS_ROS_COMPATIBLE", "FOURIERDDS_USE_DISCOVERY_SERVER"):
        monkeypatch.setenv(name, "incorrect")
    monkeypatch.setattr(session.sdk.AuroraClient, "get_instance", staticmethod(get_instance))
    robot = AuroraRobotArm(session, sides=("right",))
    try:
        robot.connect()
        assert seen == [("UDPv4", "false", "false")]
        assert not session.client.commands
    finally:
        robot.close()


def test_gr3_profile_rejects_swapped_elbow_and_yaw_indices():
    profile = gr3_fake_profile()
    group = profile.groups[0]
    joints = tuple(replace(j, index=2 if j.name == "elbow_flexion" else 3 if j.name == "upper_arm_rotation" else j.index)
                   for j in group.joints)
    with pytest.raises(ValueError, match="physical order"):
        replace(profile, groups=(replace(group, joints=joints), profile.groups[1])).validate()


@pytest.mark.parametrize("failure", ["subscriber", "service", "other_exception", "uninspectable"])
def test_compatibility_does_not_swallow_other_initialization_failures(monkeypatch, failure):
    class Timeout(Exception):
        pass

    class Client:
        def _init_publishers(self):
            publisher = SimpleNamespace(is_matched=False, _topic=SimpleNamespace(get_name=lambda: "velocity_cmd"))
            self._publisher_list = [publisher] if failure != "uninspectable" else [object()]
            if failure == "other_exception":
                raise ValueError("other failure")
            raise Timeout("publisher")

        @classmethod
        def get_instance(cls, **kwargs):
            if failure == "subscriber":
                raise Timeout("subscriber")
            obj = cls()
            obj._init_publishers()
            if failure == "service":
                raise Timeout("service")
            return obj

    monkeypatch.setattr("sleeve_arm.robot.aurora_sdk.importlib.import_module", lambda _: SimpleNamespace(DDSMatchTimeout=Timeout))
    original = Client._init_publishers
    with pytest.raises((Timeout, ValueError)):
        initialize_client(Client, replace(gr3_fake_profile(), allow_missing_velocity_cmd=True))
    assert Client._init_publishers is original


def test_compatibility_is_opt_in_and_version_pinned(monkeypatch):
    class Client:
        @classmethod
        def get_instance(cls, **kwargs):
            raise RuntimeError("original mismatch")

    monkeypatch.setattr("sleeve_arm.robot.aurora_sdk.importlib.import_module", lambda _: pytest.fail("private hook imported"))
    with pytest.raises(RuntimeError, match="original mismatch"):
        initialize_client(Client, gr3_fake_profile())
    with pytest.raises(RobotError, match="exact SDK 0.1.8"):
        initialize_client(Client, replace(gr3_fake_profile(), sdk_version="0.1.9", allow_missing_velocity_cmd=True))
