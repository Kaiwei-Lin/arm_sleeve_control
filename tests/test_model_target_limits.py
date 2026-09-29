"""Sensor target saturation with an injected Aurora session; no real hardware."""
from dataclasses import replace
import math

import pytest

from sleeve_arm.control.safety import SafetyError
from sleeve_arm.domain import MotionIntent
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import fake_session, gr3_fake_profile
from sleeve_arm.runtime.configuration import load_configs, parse_args
from sleeve_arm.runtime.robot_runtime import RobotRuntime


@pytest.fixture
def runtime(monkeypatch):
    # Keep tests independent of edits to the site's tracking-error settings.
    profile = gr3_fake_profile()
    profile = replace(profile, groups=tuple(
        replace(group, max_tracking_error=.2, joints=tuple(
            replace(joint, limits=replace(joint.limits, max_tracking_error=.2))
            for joint in group.joints
        )) for group in profile.groups
    ))
    session = fake_session(profile=profile)
    actual_factory = factory.create_robot
    monkeypatch.setattr(factory, "create_robot",
                        lambda backend, config, **kw: actual_factory(backend, config, session=session, **kw))
    args = parse_args([
        "--robot", "aurora", "--aurora-profile", "configs/robot_aurora.yaml", "--execute",
        "--side", "right", "--source-side", "right", "--duration", "1",
    ])
    runtime = RobotRuntime(args, load_configs(args), clock=session.clock, input_fn=lambda _: "YES")
    runtime.connect()
    try:
        assert runtime.prepare()
        yield runtime
    finally:
        runtime.shutdown()


@pytest.mark.parametrize("joint,field,requested,expected", [
    ("upper_arm_rotation", "upper_arm_rotation_rad", 2., 1.8326),
    ("upper_arm_rotation", "upper_arm_rotation_rad", -2., -1.8326),
    ("shoulder_flexion", "shoulder_flexion_rad", 4., 2.9671),
    ("shoulder_flexion", "shoulder_flexion_rad", -4., -2.9671),
    ("shoulder_abduction", "shoulder_abduction_rad", 2., 1.9199),
    ("shoulder_abduction", "shoulder_abduction_rad", -1., -.2618),
    ("elbow_flexion", "elbow_flexion", 3., 2.2689),
    ("elbow_flexion", "elbow_flexion", -1., -.087266),
    ("upper_arm_rotation", "upper_arm_rotation_rad", .5, .5),
])
def test_model_targets_saturate_and_reach_bounds_without_skipping_slew_limits(runtime, joint, field, requested, expected):
    robot = runtime.robot
    session = robot.session
    group = robot.groups[0]
    entry = next(j for j in group.joints if j.name == joint)
    before = list(session.fake_client.commands[-1][group.name])
    intent = MotionIntent(timestamp=session.clock.monotonic(), **{field: requested})
    previous = entry.from_sdk(before[entry.index])
    limits = robot.config.joints[joint]
    for _ in range(80):
        session.clock.sleep(.01)
        applied = runtime.apply(intent, .01)
        assert set(applied) == {joint}
        assert limits.min_position <= applied[joint] <= limits.max_position
        assert abs(applied[joint] - previous) <= min(limits.max_position_step, limits.max_velocity * .01) + 1e-10
        command = session.fake_client.commands[-1][group.name]
        group.check_vector(command)
        assert command[entry.index] == pytest.approx(entry.to_sdk(applied[joint]))
        assert [v for i, v in enumerate(command) if i != entry.index] == [v for i, v in enumerate(before) if i != entry.index]
        previous = applied[joint]
    assert previous == pytest.approx(expected)
    assert runtime.preview(intent, .01)[joint] == pytest.approx(expected)
    assert getattr(intent, field) == requested
    assert runtime.controller.fault is None


def test_model_control_resumes_from_saturated_limit_when_input_returns_in_range(runtime):
    session = runtime.robot.session
    for requested, expected in ((2., 1.8326), (.1, .1), (-2., -1.8326), (-.1, -.1)):
        for _ in range(80):
            session.clock.sleep(.01)
            applied = runtime.apply(MotionIntent(session.clock.monotonic(), upper_arm_rotation_rad=requested), .01)
        assert applied["upper_arm_rotation"] == pytest.approx(expected)
    assert runtime.controller.fault is None


def test_saturation_does_not_disable_tracking_error_protection(runtime):
    session = runtime.robot.session
    session.fake_client.follow_commands = False
    session.fake_client._pending_vectors.clear()
    with pytest.raises(SafetyError, match="proposed target tracking error exceeds limit"):
        for _ in range(10):
            session.clock.sleep(.01)
            runtime.apply(MotionIntent(session.clock.monotonic(), upper_arm_rotation_rad=2.), .01)
    assert runtime.controller.fault and not runtime.controller.enabled
    assert session.fake_client.closed
    assert all(abs(command["right_manipulator"][2]) <= .2 for command in session.fake_client.commands)


def test_direct_controller_still_rejects_out_of_range_targets(runtime):
    session = runtime.robot.session
    sent = len(session.fake_client.commands)
    session.clock.sleep(.01)
    with pytest.raises(SafetyError, match="target outside limits"):
        runtime.controller.set_joint_positions({"upper_arm_rotation": 2.}, dt=.01)
    assert len(session.fake_client.commands) == sent
    assert runtime.controller.fault


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_invalid_numbers_are_not_converted_into_boundary_commands(runtime, monkeypatch, value):
    session = runtime.robot.session
    sent = len(session.fake_client.commands)
    monkeypatch.setattr(runtime.mapper, "map", lambda _: {"upper_arm_rotation": value})
    with pytest.raises(SafetyError, match="not finite"):
        runtime.apply(MotionIntent(session.clock.monotonic()), .01)
    assert len(session.fake_client.commands) == sent
