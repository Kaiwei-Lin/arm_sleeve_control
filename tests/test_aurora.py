from __future__ import annotations

import importlib.util
import math
import subprocess
import sys
from dataclasses import replace
from datetime import timedelta
from pathlib import Path
from types import SimpleNamespace

import pytest

from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.control.mapper import AuroraIntentMapper
from sleeve_arm.control.safety import SafetyError
from sleeve_arm.control.upper_limb import UpperLimbService
from sleeve_arm.domain.joint import DYMOTOR_JOINT_NAMES, JOINT_NAMES
from sleeve_arm.domain.motion import MotionIntent
from sleeve_arm.robot.aurora import AuroraRobotArm
from sleeve_arm.robot.aurora_fake import (
    Endpoint, FakeAuroraClient, FakeClock, LeaseState, OperationResult, fake_profile, fake_session,
)
from sleeve_arm.robot.aurora_profile import load_aurora_profile
from sleeve_arm.robot.aurora_session import AuroraSession
from sleeve_arm.robot.base import RobotError
from sleeve_arm.robot.factory import create_robot
from tools import aurora_control

ROOT = Path(__file__).resolve().parents[1]


def ready(*, sides=("left",), hands=False, parts=("arm",)):
    session = fake_session(hands=hands)
    robot = AuroraRobotArm(session, sides=sides, parts=parts)
    controller = SafeArmController(robot, robot.config, clock=session.clock)
    controller.connect()
    controller.enable()
    return session, robot, controller, UpperLimbService(controller)


@pytest.fixture
def rig():
    session, robot, controller, service = ready()
    yield session, robot, controller, service
    controller.shutdown()


@pytest.mark.parametrize("tool", ["aurora_control.py", "run_model_control.py", "record_sensors.py", "read_imu.py"])
def test_help_never_imports_sdk(tool):
    code = """
import sys, runpy
class BlockSdk:
    def find_spec(self, fullname, *args):
        if fullname.startswith('fourier_aurora_client'):
            raise AssertionError('SDK imported during --help')
sys.meta_path.insert(0, BlockSdk())
sys.argv = [sys.argv[1], '--help']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
    result = subprocess.run([sys.executable, "-c", code, str(ROOT / "tools" / tool)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout


def test_missing_sdk_clear_error_and_no_runtime_install(monkeypatch):
    import importlib.metadata
    def missing(_):
        raise importlib.metadata.PackageNotFoundError
    monkeypatch.setattr(importlib.metadata, "version", missing)
    profile = load_aurora_profile(ROOT / "configs/aurora.unverified.yaml")
    session = AuroraSession(profile)
    with pytest.raises(RobotError, match="SDK missing.*official.*no automatic installation"):
        session.attach(object(), set())


@pytest.mark.parametrize("field,value", [("api_family", "legacy"), ("sdk_version", "1.0.0"), ("sdk_version", "1.0.2")])
def test_unsupported_api_and_version_rejected_before_start(field, value):
    profile = replace(fake_profile(), **{field: value})
    with pytest.raises(ValueError, match="unsupported api_family/version"):
        AuroraSession(profile)


def test_installed_version_checked_before_import(monkeypatch):
    monkeypatch.setattr("importlib.metadata.version", lambda _: "0.9.99")
    session = AuroraSession(load_aurora_profile(ROOT / "configs/aurora.unverified.yaml"))
    with pytest.raises(RobotError, match="unsupported SDK version.*no legacy fallback"):
        session.attach(object(), set())


def test_lifecycle_uses_singleton_without_kwargs(monkeypatch):
    from sleeve_arm.robot import aurora_fake as sdk
    profile, clock = fake_profile(), FakeClock()
    client = FakeAuroraClient(profile, clock)
    calls = []
    def get_instance():
        calls.append("get_instance")
        return client
    monkeypatch.setattr(sdk, "AuroraClient", SimpleNamespace(get_instance=get_instance), raising=False)
    session = AuroraSession(profile, sdk=sdk)
    session.clock = clock
    a, b = object(), object()
    session.attach(a, {"left_manipulator"})
    session.attach(b, {"right_manipulator"})
    assert calls == ["get_instance"]
    assert client.calls.count("configure") == client.calls.count("start") == 1
    session.detach(a)
    assert client.running
    session.detach(b)
    assert client.calls.count("stop") == 1


def test_external_running_singleton_not_reconfigured_or_stopped():
    session = fake_session()
    session.client.running = True
    with pytest.raises(RobotError, match="already running outside"):
        session.attach(object(), set())
    assert "configure" not in session.client.calls
    assert "stop" not in session.client.calls


def test_doctor_is_read_only_even_for_unverified_profile():
    session = fake_session(execute=False)
    session.profile = replace(session.profile, verified=False)
    token = object()
    session.attach(token, set())
    info = session.doctor()
    assert info["robot_type"] == "OFFLINE_SIMULATOR"
    assert info["groups"]["left_manipulator"]["position_count"] == 6
    assert set(session.client.options.enabled_endpoints) == {
        Endpoint.AURORA_STATE_SUBSCRIBER, Endpoint.CONTROL_GROUP_STATE_SUBSCRIBER}
    with pytest.raises(RobotError, match="read-only"):
        session.enable(token)
    session.detach(token)
    assert not {"register_lease", "replace_lease", "release_lease", "publish_control_group_command", "request_fsm_state"} & set(session.client.calls)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("direction,joint,sign", [("forward", "shoulder_flexion", 1), ("backward", "shoulder_flexion", -1),
                                                  ("outward", "shoulder_abduction", 1), ("inward", "shoulder_abduction", -1)])
def test_side_direction_transform_and_inverse(side, direction, joint, sign):
    session, robot, controller, service = ready(sides=(side,))
    result = service.move_arm(side=side, direction=direction, angle_deg=4, duration_s=1)
    target = sign * math.radians(4)
    assert result.arrived and result.targets_rad[joint] == pytest.approx(target)
    state = controller.read_joint_states()[joint]
    assert state.position == pytest.approx(target)
    assert state.current is state.error is state.state is state.bus is None
    group, entry = robot._joints[joint]
    for command in session.client.published:
        assert [x.group_name for x in command.joint_cmd] == [f"{side}_manipulator"]
    assert session.client.published[-1].joint_cmd[0].position[entry.index] == pytest.approx(entry.zero + entry.sign * target)
    controller.shutdown()


def test_current_relative_is_computed_once_not_accumulated(rig):
    _, _, controller, service = rig
    service.move_joint(side="left", joint="elbow_flexion", angle_deg=5, duration_s=1)
    result = service.move_joint(side="left", joint="elbow_flexion", angle_deg=3, reference="current", duration_s=1)
    assert result.targets_rad["elbow_flexion"] == pytest.approx(math.radians(8))
    assert controller.read_joint_states()["elbow_flexion"].position == pytest.approx(math.radians(8))
    service.move_joint(side="left", joint="elbow_flexion", angle_deg=2, duration_s=1)
    assert controller.read_joint_states()["elbow_flexion"].position == pytest.approx(math.radians(2))


def test_dynamic_joints_do_not_extend_native_abi(rig):
    from sleeve_arm.robot.dymotor import _ArmOpenConfig, _JOINT_COUNT
    _, robot, controller, service = rig
    assert len(controller.config.joints) == 6
    assert JOINT_NAMES is DYMOTOR_JOINT_NAMES
    assert _JOINT_COUNT == 4
    assert dict(_ArmOpenConfig._fields_)["motor_ids"]._length_ == 4
    service.wrist_joints(side="left", angles_deg={"wrist_pitch": 2}, duration_s=1)
    assert controller.read_joint_states()["wrist_pitch"].position == pytest.approx(math.radians(2))
    with pytest.raises(ValueError, match="physical wrist"):
        service.wrist_joints(side="left", angles_deg={"elbow_flexion": 2})


def test_partial_update_preserves_all_other_initial_then_owner_targets():
    session = fake_session()
    group = session.client.groups.joint_state[0]
    group.joint_position[5] = session.profile.groups[0].joints[5].to_sdk(0.17)
    robot = AuroraRobotArm(session, sides=("left",))
    ctl = SafeArmController(robot, robot.config, clock=session.clock)
    ctl.connect(); ctl.enable()
    service = UpperLimbService(ctl)
    service.move_joint(side="left", joint="elbow_flexion", angle_deg=3, duration_s=1)
    service.move_joint(side="left", joint="shoulder_flexion", angle_deg=2, duration_s=1)
    states = ctl.read_joint_states()
    assert states["wrist_yaw"].position == pytest.approx(0.17)
    assert states["elbow_flexion"].position == pytest.approx(math.radians(3))
    assert all(len(c.joint_cmd) == 1 for c in session.client.published)
    ctl.shutdown()


def test_batch_uses_one_control_group_snapshot_for_both_sides():
    session, _, ctl, service = ready(sides=("left", "right"))
    before = session.client.calls.count("get_control_group_state")
    session.clock.sleep(0.02)
    ctl.set_joint_positions({"left.elbow_flexion": 0.002, "right.elbow_flexion": 0.003}, dt=0.02)
    assert session.client.calls.count("get_control_group_state") - before == 1
    assert {g.group_name for g in session.client.published[-1].joint_cmd} == {"left_manipulator", "right_manipulator"}
    sent = len(session.client.published)
    with pytest.raises(SafetyError, match="outside limits"):
        ctl.set_joint_positions({"left.elbow_flexion": 0.003, "right.elbow_flexion": 100}, dt=0.02)
    assert len(session.client.published) == sent
    assert ctl.fault


def test_cached_samples_do_not_become_fresh_when_read_again(rig):
    session, _, ctl, _ = rig
    session.client.auto_feedback = False
    original = ctl.read_joint_states()["elbow_flexion"].received_at
    session.clock.sleep(0.1)
    assert ctl.read_joint_states()["elbow_flexion"].received_at == original
    session.clock.sleep(0.2)
    with pytest.raises(SafetyError, match="stale"):
        ctl.read_joint_states()
    assert session.closed and not ctl.enabled
    with pytest.raises(SafetyError, match="fault latched"):
        ctl.connect()


@pytest.mark.parametrize("operation", ["get_aurora_state", "get_control_group_state", "get_error_codes", "get_lease", "publish_control_group_command"])
def test_operation_failures_are_explicit_even_if_truthy(rig, operation):
    session, _, ctl, _ = rig
    result = OperationResult(42, "injected SDK failure")
    assert bool(result) and not result.success()
    session.client.failures[operation] = result
    session.clock.sleep(0.02)
    with pytest.raises(SafetyError, match=f"{operation}: code=42 message=injected SDK failure"):
        ctl.set_joint_positions({"elbow_flexion": 0.001}, dt=0.02)
    assert not session.client.published
    assert not ctl.enabled


@pytest.mark.parametrize("operation", ["register_lease", "replace_lease"])
def test_lease_rejected_before_any_publication(operation):
    session = fake_session()
    left, right = AuroraRobotArm(session, sides=("left",)), AuroraRobotArm(session, sides=("right",))
    left.connect(); right.connect()
    if operation == "replace_lease":
        left.enable()
    session.client.failures[operation] = OperationResult(8, "denied")
    with pytest.raises(RobotError, match=f"{operation}: code=8 message=denied"):
        (left if operation == "register_lease" else right).enable()
    assert not session.client.published
    left.close(); right.close()


@pytest.mark.parametrize("mode", ["callback", "unusable", "resources"])
def test_lost_lease_blocks_all_shared_writes(mode):
    session = fake_session()
    left, right = AuroraRobotArm(session, sides=("left",)), AuroraRobotArm(session, sides=("right",))
    left.connect(); right.connect(); left.enable(); right.enable()
    if mode == "callback":
        session.client.lose_lease()
        assert "stop" not in session.client.calls  # callback only set event
    elif mode == "unusable":
        session.client.lease.remaining_duration = timedelta(0)
    else:
        session.client.lease.resources = []
    session.clock.sleep(0.02)
    for robot in (left, right):
        with pytest.raises(RobotError, match="lease|resources"):
            robot.set_joint_position("elbow_flexion", 0.001)
    assert not session.client.published
    left.close(); right.close()


@pytest.mark.parametrize("damage,match", [("dimension", "dimension"), ("nan", "non-finite"), ("inf", "non-finite"),
                                         ("range", "above max_position"), ("tracking", "tracking error"),
                                         ("velocity", "velocity exceeds"), ("missing", "missing feedback")])
def test_feedback_faults_latch_without_publication(rig, damage, match):
    session, _, ctl, _ = rig
    entry = session.client.groups.joint_state[0]
    if damage == "dimension": entry.joint_position.pop()
    if damage == "nan": entry.joint_position[0] = float("nan")
    if damage == "inf": entry.joint_effort[0] = float("inf")
    if damage in ("range", "tracking"):
        joint = session.profile.groups[0].joints[0]
        entry.joint_position[0] = joint.to_sdk(3 if damage == "range" else 0.3)
    if damage == "velocity": entry.joint_velocity[0] = 4
    if damage == "missing": session.client.groups.joint_state.pop(0)
    with pytest.raises(SafetyError, match=match):
        ctl.read_joint_states()
    assert not session.client.published and ctl.fault


@pytest.mark.parametrize("damage,match", [("endpoint", "unmatched"), ("fsm", "FSM"), ("type", "identity mismatch"), ("async", "asynchronous")])
def test_session_faults_block_all_views(rig, damage, match):
    session, _, ctl, _ = rig
    if damage == "endpoint": session.client.matched = False
    if damage == "fsm": session.client.aurora.current_state.id = 13
    if damage == "type": session.client.aurora.robot_info.robot_type = "NOT_THIS_ROBOT"
    if damage == "async": session.client.errors.error_codes = [SimpleNamespace(high32=1, low32=2)]
    with pytest.raises(SafetyError, match=match):
        ctl.read_joint_states()
    assert session.fault


def test_endpoint_start_failure_stops_and_closes_without_lease():
    session = fake_session()
    session.client.matched = False
    with pytest.raises(RobotError, match="wait_for_endpoints"):
        session.attach(object(), set())
    assert session.closed
    assert "stop" in session.client.calls
    assert "register_lease" not in session.client.calls


def test_unverified_profile_and_simulation_block_real_execute_before_sdk(monkeypatch):
    profile = load_aurora_profile(ROOT / "configs/aurora.unverified.yaml")
    with pytest.raises(ValueError, match="verified"):
        AuroraSession.real(profile, execute=True, operator_confirmed=True)
    with pytest.raises(ValueError, match="simulated profile"):
        AuroraSession.real(fake_profile(), execute=True, operator_confirmed=True)
    with pytest.raises(RobotError, match="operator confirmation"):
        AuroraSession(replace(fake_profile(), simulated=False), execute=True)


@pytest.mark.parametrize("field,value", [("sign", None), ("zero", None), ("verified", False)])
def test_missing_joint_calibration_blocks_execute(field, value):
    profile = fake_profile()
    group = profile.groups[0]
    joint = replace(group.joints[0], **{field: value})
    profile = replace(profile, groups=(replace(group, joints=(joint, *group.joints[1:])), *profile.groups[1:]))
    with pytest.raises(ValueError, match="calibration/safety"):
        profile.validate(execute=True, simulation=True)


def test_unsupported_feedback_policy_is_not_silently_skipped():
    profile = fake_profile()
    group = profile.groups[0]
    joint = replace(group.joints[0], limits=replace(group.joints[0].limits, max_current=1))
    profile = replace(profile, groups=(replace(group, joints=(joint, *group.joints[1:])), *profile.groups[1:]))
    with pytest.raises(ValueError, match="unavailable feedback"):
        profile.validate(execute=True, simulation=True)


def test_hand_capabilities_and_calibrated_closure_share_session():
    session, robot, ctl, service = ready(hands=True, parts=("arm", "hand"))
    service.hand_joints(side="left", angles_deg={"thumb_opposition": 3}, duration_s=1)
    assert [g.group_name for g in session.client.published[-1].joint_cmd] == ["left_end_effector"]
    service.hand_closure(side="left", closure=0.2, duration_s=1)
    assert len(session.client.published[-1].joint_cmd[0].position) == 3
    assert ctl.read_joint_states()["index_flexion"].position == pytest.approx(0.1)
    assert session.client.calls.count("start") == session.client.calls.count("register_lease") == 1
    ctl.shutdown()


def test_hand_and_palm_missing_capability_rejected(rig):
    _, _, _, service = rig
    with pytest.raises(ValueError, match="unsupported joint"):
        service.hand_joints(side="left", angles_deg={"index_flexion": 1})
    with pytest.raises(ValueError, match="unavailable"):
        service.hand_closure(side="left", closure=0.5)
    with pytest.raises(ValueError, match="unsupported joint"):
        service.wrist_joints(side="left", angles_deg={"palm": 1})


def test_shared_close_does_not_stop_other_side_and_is_idempotent():
    session = fake_session()
    left, right = AuroraRobotArm(session, sides=("left",)), AuroraRobotArm(session, sides=("right",))
    left.connect(); right.connect(); left.enable(); right.enable()
    left.close(); left.close()
    assert session.client.running and not session.closed
    assert [r.name for r in session.client.lease.resources] == ["RIGHT_MANIPULATOR"]
    session.clock.sleep(0.02)
    right.set_joint_position("elbow_flexion", 0.002)
    assert session.client.published[-1].joint_cmd[0].group_name == "right_manipulator"
    right.close(); right.close()
    assert session.client.calls.count("stop") == 1
    assert session.client.calls.count("release_lease") == 1


def test_local_feedback_fault_does_not_fault_healthy_other_side():
    session = fake_session()
    left, right = AuroraRobotArm(session, sides=("left",)), AuroraRobotArm(session, sides=("right",))
    left.connect(); right.connect(); left.enable(); right.enable()
    session.client.groups.joint_state[0].joint_position[0] = float("nan")
    with pytest.raises(RobotError, match="non-finite"):
        left.read_joint_states()
    assert left.fault and session.fault is None
    left.close()
    session.clock.sleep(0.02)
    right.set_joint_position("elbow_flexion", 0.001)
    right.close()


def test_conflicting_process_config_rejected_without_getting_singleton(monkeypatch):
    monkeypatch.setattr(AuroraSession, "_real_session", None)
    profile = load_aurora_profile(ROOT / "configs/aurora.unverified.yaml")
    one = AuroraSession.real(profile)
    assert AuroraSession.real(profile) is one
    with pytest.raises(RobotError, match="conflicting"):
        AuroraSession.real(replace(profile, connection={**profile.connection, "domain_id": 66}))
    assert one.client is None


def test_second_command_owner_thread_is_rejected(rig):
    import threading
    session, robot, _, _ = rig
    errors = []
    def other():
        try:
            robot.set_joint_position("elbow_flexion", 0.0)
        except RobotError as exc:
            errors.append(str(exc))
    t = threading.Thread(target=other)
    t.start(); t.join()
    assert errors and "owner thread" in errors[0]
    assert not session.client.published


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_nonfinite_action_parameters_rejected_without_publish(rig, bad):
    session, _, _, service = rig
    for kwargs in ({"angle_deg": bad, "duration_s": 1}, {"angle_deg": 2, "duration_s": bad}):
        with pytest.raises(ValueError):
            service.move_joint(side="left", joint="elbow_flexion", **kwargs)
    with pytest.raises(ValueError):
        service.swing_arm(side="left", joint="elbow_flexion", amplitude_deg=bad, period_s=1, cycles=1)
    assert not session.client.published


@pytest.mark.parametrize("kwargs", [{"cycles": 0}, {"cycles": 1.5}, {"cycles": 1001}, {"period_s": 0},
                                     {"period_s": float("inf")}, {"amplitude_deg": -1}, {"center": float("nan")}])
def test_illegal_swing_parameters_rejected(rig, kwargs):
    session, _, _, service = rig
    args = dict(side="left", joint="elbow_flexion", amplitude_deg=3, period_s=2, cycles=2)
    with pytest.raises(ValueError):
        service.swing_arm(**{**args, **kwargs})
    assert not session.client.published


def test_finite_swing_exact_bounds_center_and_virtual_duration(rig):
    session, robot, ctl, service = rig
    result = service.swing_arm(side="left", joint="elbow_flexion", amplitude_deg=3, period_s=2, cycles=2)
    assert result.arrived and result.elapsed_s == pytest.approx(8)
    joint = robot._joints["elbow_flexion"][1]
    values = [joint.from_sdk(c.joint_cmd[0].position[joint.index]) for c in session.client.published]
    assert min(values) == pytest.approx(-math.radians(3))
    assert max(values) == pytest.approx(math.radians(3))
    assert ctl.read_joint_states()["elbow_flexion"].position == pytest.approx(0)
    assert len(values) <= 405


@pytest.mark.parametrize("case", ["speed", "range", "unknown", "side"])
def test_rejected_manual_requests_do_not_dispatch(rig, case):
    session, _, _, service = rig
    args = dict(side="left", joint="elbow_flexion", angle_deg=3, duration_s=1)
    if case == "speed": args.update(angle_deg=90, duration_s=0.02)
    if case == "range": args.update(angle_deg=200)
    if case == "unknown": args.update(joint="imagined_palm")
    if case == "side": args.update(side="right")
    with pytest.raises(ValueError):
        service.move_joint(**args)
    assert not session.client.published


def test_scheduling_delay_does_not_expand_steps(rig):
    session, _, ctl, service = rig
    original = session.clock.sleep
    session.clock.sleep = lambda duration: original(duration + 0.5)
    with pytest.raises(SafetyError, match="scheduling delay"):
        service.move_joint(side="left", joint="elbow_flexion", angle_deg=5, duration_s=1)
    assert not session.client.published and not ctl.enabled and session.closed


def test_publish_success_is_not_arrival_and_timeout_is_bounded(rig):
    session, _, ctl, service = rig
    session.client.follow_commands = False
    # Within tracking and per-step bounds, yet outside arrival tolerance.
    with pytest.raises(SafetyError, match="arrival timed out"):
        service.move_joint(side="left", joint="elbow_flexion", angle_deg=0.5, duration_s=1)
    assert session.client.published and ctl.fault and session.closed
    assert session.clock.monotonic() < 3.2


@pytest.mark.parametrize("error", [KeyboardInterrupt, RuntimeError])
def test_interrupt_or_thread_exception_releases_and_closes(rig, error):
    session, _, ctl, service = rig
    def broken(_):
        raise error("injected")
    session.clock.sleep = broken
    with pytest.raises(error):
        service.move_joint(side="left", joint="elbow_flexion", angle_deg=5, duration_s=1)
    assert session.closed and "release_lease" in session.client.calls
    assert ctl.fault and not ctl.enabled


def test_stop_error_reported_but_disable_blocks_immediately(rig):
    session, robot, ctl, _ = rig
    session.client.failures["release_lease"] = OperationResult(9, "release failed")
    with pytest.raises(RobotError, match="release_lease.*code=9"):
        ctl.shutdown()
    assert not robot.enabled and session.closed
    assert "stop" in session.client.calls


def test_profile_yaml_roundtrip_and_invalid_types(tmp_path):
    import yaml
    from dataclasses import asdict
    p = tmp_path / "profile.yaml"
    raw = asdict(fake_profile())
    for g in raw["groups"]:
        for j in g["joints"]:
            j["limits"].pop("name")
    p.write_text(yaml.safe_dump(raw))
    loaded = load_aurora_profile(p)
    loaded.validate(execute=True, simulation=True)
    raw["verified"] = "false"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="booleans"):
        load_aurora_profile(p)


def test_mapper_routes_only_explicit_source_side_and_allows_single_joint():
    session, robot, ctl, _ = ready(sides=("right",))
    mapper = AuroraIntentMapper(robot, source_side="right", target_side="right")
    assert mapper.map(MotionIntent(timestamp=0, shoulder_flexion_rad=0.02)) == {"shoulder_flexion": 0.02}
    with pytest.raises(ValueError, match="cross-side"):
        AuroraIntentMapper(robot, source_side="right", target_side="left")
    with pytest.raises(ValueError, match="cross-side"):
        AuroraIntentMapper(robot, source_side="left", target_side="left")
    session.clock.sleep(0.02)
    ctl.set_joint_positions(mapper.map(MotionIntent(timestamp=0, elbow_flexion=0.001)), dt=0.02)
    assert session.client.published[-1].joint_cmd[0].group_name == "right_manipulator"
    ctl.shutdown()


def test_cli_fake_doctor_and_joint(capsys):
    assert aurora_control.main(["doctor"]) == 0
    assert '"execute_authorized": false' in capsys.readouterr().out
    assert aurora_control.main(["joint", "--side", "right", "--joint", "wrist_pitch", "--angle-deg", "2"]) == 0
    assert '"arrived": true' in capsys.readouterr().out


def test_cli_real_requires_all_gates_before_sdk(capsys):
    with pytest.raises(SystemExit) as exc:
        aurora_control.main(["--backend", "aurora", "--profile", "configs/aurora.unverified.yaml", "--execute",
                             "joint", "--side", "left", "--joint", "elbow_flexion", "--angle-deg", "1"])
    assert exc.value.code == 2
    assert aurora_control.main(["--backend", "aurora", "--profile", "configs/aurora.unverified.yaml", "--execute",
                               "--confirm", "EXECUTE_AURORA", "joint", "--side", "left", "--joint", "elbow_flexion", "--angle-deg", "1"]) == 1
    assert "verified" in capsys.readouterr().err


@pytest.mark.parametrize("mode", ["normal", "prediction_error", "sensor_timeout"])
def test_model_entry_aurora_fake_routes_right_and_cleans_up(tmp_path, monkeypatch, mode):
    import yaml
    from tools import run_model_control
    from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH
    import sleeve_arm.robot.factory as factory
    from sleeve_arm.robot.aurora_session import SystemClock
    phase4 = tmp_path / "phase4.yaml"
    phase4.write_text(yaml.safe_dump({
        "predictor": {"backend": "flexarm_estimator", "flexarm_estimator": {
            "model_dir": str(tmp_path), "sleeve_channels": [3, 4, 5],
            "calibration_file": str(tmp_path / "unused.json"), "calibration_seconds": 0.1,
            "angle": {"min_deg": 0, "max_deg": 180}}},
        "phase4_validation": {"max_consecutive_prediction_errors": 3},
    }))
    sensors = yaml.safe_load(DEFAULT_SENSOR_CONFIG_PATH.read_text())
    sensors["upper_arm_rotation"]["enabled"] = False
    for name in ("imu1", "imu2", "imu3", "imu4"):
        sensors["sensors"][name]["enabled"] = False
    sensor_file = tmp_path / "sensors.yaml"
    sensor_file.write_text(yaml.safe_dump(sensors))
    session = fake_session(clock=SystemClock())
    actual_factory = factory.create_robot
    def create(backend, config=None, **kwargs):
        return actual_factory(backend, config, session=session, **kwargs)
    monkeypatch.setattr(factory, "create_robot", create)
    class Predictor:
        calls = 0
        def predict(self, sample):
            self.calls += 1
            if mode == "prediction_error" and self.calls > 1:
                raise ValueError("injected model failure")
            return MotionIntent(timestamp=sample.timestamp, shoulder_flexion_rad=0.03,
                                model_action="Forward", angle_deg=2, confidence=1, inference_ms=0.1)
    monkeypatch.setattr(run_model_control, "prepare_flexarm_predictor", lambda *a, **k: Predictor())
    if mode == "sensor_timeout":
        monkeypatch.setattr("sleeve_arm.control.SensorWatchdog.is_stale", lambda *a: True)
    monkeypatch.setattr(sys, "argv", [
        "run_model_control.py", "--robot", "aurora-fake", "--side", "right", "--source-side", "right",
        "--sleeve", "fake", "--imus", "fake", "--duration", "0.15",
        "--phase4-config", str(phase4), "--sensor-config", str(sensor_file),
    ])
    code = run_model_control.main()
    assert code == (0 if mode == "normal" else 1)
    assert session.closed
    assert "release_lease" in session.client.calls
    assert all([g.group_name for g in c.joint_cmd] == ["right_manipulator"] for c in session.client.published)
    assert len(session.client.published) > (1 if mode == "normal" else 0)


def test_group_ownership_conflict_is_rejected():
    session = fake_session()
    left = AuroraRobotArm(session, sides=("left",))
    duplicate = AuroraRobotArm(session, sides=("left",))
    left.connect()
    with pytest.raises(RobotError, match="already belongs"):
        duplicate.connect()
    duplicate.close()
    assert session.client.running
    left.close()


def test_unusable_registration_success_object_does_not_enable():
    session = fake_session()
    robot = AuroraRobotArm(session, sides=("left",))
    robot.connect()
    original = session.client.register_lease
    def expired(resources):
        result = original(resources)
        result.remaining_duration = timedelta(0)
        return result
    session.client.register_lease = expired
    with pytest.raises(RobotError, match="register_lease: lease unusable"):
        robot.enable()
    assert not robot.enabled and not session.client.published
    robot.close()


def test_large_scheduling_dt_cannot_bypass_backend_limits(rig):
    session, _, ctl, _ = rig
    session.clock.sleep(10)
    sent = ctl.set_joint_positions({"elbow_flexion": 1.0}, dt=10000)
    assert sent["elbow_flexion"] <= 0.8 * session.profile.control_period_s + 1e-10


def test_safety_clipping_is_not_reported_as_full_motion(rig):
    session, _, ctl, service = rig
    session.client.follow_commands = False
    with pytest.raises(SafetyError, match="limited by safety"):
        service.move_joint(side="left", joint="elbow_flexion", angle_deg=3, duration_s=1)
    assert ctl.fault
    assert not ctl.enabled


def test_incomplete_hand_closure_calibration_rejected_before_execute():
    profile = fake_profile(hands=True)
    hand = profile.groups[1]
    bad = replace(hand, joints=(replace(hand.joints[0], open_position=None), *hand.joints[1:]))
    profile = replace(profile, groups=(profile.groups[0], bad, *profile.groups[2:]))
    with pytest.raises(ValueError, match="open/closed calibration"):
        profile.validate(execute=True, simulation=True)
