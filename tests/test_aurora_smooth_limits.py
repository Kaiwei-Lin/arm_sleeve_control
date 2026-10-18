"""Finite limit overruns shape commands; invalid feedback still faults. No DDS."""
from dataclasses import replace
import math

import pytest

from sleeve_arm.control.aurora_motion import AuroraMotionService
from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.robot.aurora import AuroraRobotArm
from sleeve_arm.robot.aurora_fake import fake_profile, fake_session, gr3_fake_profile


def make_controller(profile=None, parts=("arm", "hand")):
    session = fake_session(profile=profile or gr3_fake_profile())
    robot = AuroraRobotArm(session, sides=("right",), parts=parts)
    controller = SafeArmController(robot, robot.config, clock=session.clock)
    return session, robot, controller


def test_startup_outside_bounds_and_fast_feedback_recover_without_command_jumps():
    session, robot, controller = make_controller()
    client = session.fake_client
    client.groups["right_manipulator"]["position"][1] = -2.1
    client.groups["right_manipulator"]["position"][6] = 1.1  # unmapped wrist slot
    client.groups["right_manipulator"]["velocity"] = [20.] * 7
    client.groups["right_hand"]["position"][0] = 0.
    client.groups["right_hand"]["velocity"] = [2.] * 6
    try:
        controller.connect()
        controller.enable()
        previous = {name: list(client.groups[name]["position"]) for name in robot._targets}
        for _ in range(250):
            session.clock.sleep(.01)
            applied = controller.set_joint_positions(
                {"shoulder_abduction": 100., "thumb_bend": -100., "index_flexion": 100.},
                dt=10., require_exact=True,
            )
            command = client.commands[-1]
            for group in robot.groups:
                for joint in group.joints:
                    change = abs(command[group.name][joint.index] - previous[group.name][joint.index])
                    assert change <= min(joint.limits.max_position_step, joint.limits.max_velocity * .01) + 1e-10
                group.check_vector(command[group.name], enforce_limits=False)
            assert abs(command["right_manipulator"][6] - previous["right_manipulator"][6]) <= .0628 + 1e-10
            previous = command
        assert applied == pytest.approx({"shoulder_abduction": 1.9199, "thumb_bend": .12, "index_flexion": 1.78})
        assert previous["right_manipulator"][6] == pytest.approx(1.0472)
        assert not robot.fault and not controller.fault and controller.enabled
    finally:
        controller.shutdown()


def test_tracking_envelope_waits_then_resumes_and_recovers_gradually():
    profile = gr3_fake_profile()
    profile = replace(profile, groups=tuple(
        replace(g, max_tracking_error=.04) if g.part == "hand" else g for g in profile.groups))
    session, robot, controller = make_controller(profile, ("hand",))
    client = session.fake_client
    client.follow_commands = False
    try:
        controller.connect()
        controller.enable()
        before = .17
        for _ in range(30):
            session.clock.sleep(.01)
            applied = controller.set_joint_position("index_flexion", 1.5, .01)
            assert abs(applied - before) <= .008 + 1e-10
            assert applied <= .21 + 1e-10
            before = applied
        assert applied == pytest.approx(.21)
        assert controller.enabled and not client.closed
        client.groups["right_hand"]["position"][1] = .21
        session.clock.sleep(.01)
        assert controller.set_joint_position("index_flexion", 1.5, .01) == pytest.approx(.218)
        # A feedback discontinuity cannot make the command jump to a new envelope.
        client.groups["right_hand"]["position"][1] = .16
        session.clock.sleep(.01)
        assert controller.set_joint_position("index_flexion", 1.5, .01) == pytest.approx(.210)
        assert not controller.fault and not robot.fault
    finally:
        controller.shutdown()


def test_direct_backend_commands_also_slew_and_zero_elapsed_time_holds():
    session, robot, controller = make_controller(parts=("hand",))
    try:
        controller.connect()
        controller.enable()
        robot.set_joint_position("index_flexion", 100.)
        assert session.fake_client.commands[-1]["right_hand"][1] == pytest.approx(.17)
        session.clock.sleep(.01)
        robot.set_joint_position("index_flexion", 100.)
        assert session.fake_client.commands[-1]["right_hand"][1] == pytest.approx(.178)
        assert not robot.fault
    finally:
        controller.shutdown()


def test_short_trajectory_is_stretched_and_outside_goal_clamped():
    session, robot, controller = make_controller(parts=("arm",))
    try:
        controller.connect()
        controller.enable()
        result = AuroraMotionService(controller).move_joint(
            side="right", joint="shoulder_abduction", angle_deg=180, duration_s=.01)
        assert result.submitted and result.arrived
        assert result.targets_rad["shoulder_abduction"] == pytest.approx(1.9199)
        assert result.elapsed_s >= 1.5 * 1.9199 / 7.75
        previous = 0.
        for command in session.fake_client.commands:
            value = -command["right_manipulator"][1]
            assert 0 <= value <= 1.9199
            assert abs(value - previous) <= .0775 + 1e-10
            previous = value
    finally:
        controller.shutdown()


def test_stalled_trajectory_reports_pending_without_latching_a_limit_fault():
    session, robot, controller = make_controller(parts=("hand",))
    session.fake_client.follow_commands = False
    try:
        controller.connect()
        controller.enable()
        result = AuroraMotionService(controller).move_joints({"index_flexion": 1.5})
        assert not result.submitted and not result.arrived
        assert result.elapsed_s < 5
        assert controller.enabled and not controller.fault and not robot.fault
        assert session.fake_client.commands[-1]["right_hand"][1] == pytest.approx(.47)
    finally:
        controller.shutdown()


def test_calibration_roundoff_does_not_leave_a_completed_target_pending():
    profile = fake_profile()
    profile = replace(profile, smooth_limits=True, groups=tuple(
        replace(group, joints=tuple(replace(joint, zero=.7) for joint in group.joints))
        for group in profile.groups))
    session, robot, controller = make_controller(profile, ("arm",))
    try:
        controller.connect()
        controller.enable()
        result = AuroraMotionService(controller).move_joints({"shoulder_flexion": .02})
        assert result.submitted and result.arrived
        assert result.elapsed_s < profile.arrival_timeout_s
        assert controller._last_targets["shoulder_flexion"] == pytest.approx(.02)
    finally:
        controller.shutdown()


@pytest.mark.parametrize("value", [math.nan, math.inf])
def test_smoothing_does_not_accept_nonfinite_feedback(value):
    session, robot, controller = make_controller(parts=("hand",))
    session.fake_client.groups["right_hand"]["position"][0] = value
    with pytest.raises(ValueError, match="finite"):
        controller.connect()
    assert session.fake_client.closed and not session.fake_client.commands
