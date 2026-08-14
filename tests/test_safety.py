from __future__ import annotations

from dataclasses import replace

import pytest

from sleeve_arm.config import DEFAULT_CONFIG_PATH, RobotConfig, load_robot_config
from sleeve_arm.control import FeedbackError, SafeArmController, SafetyError
from sleeve_arm.control.safety import clamp_position, limit_position_change
from sleeve_arm.robot import FakeRobotArm, RobotError


@pytest.fixture
def config() -> RobotConfig:
    loaded = load_robot_config(DEFAULT_CONFIG_PATH)
    safety = replace(loaded.safety, stable_feedback_interval_s=0.0)
    return replace(loaded, safety=safety)


def test_joint_mapping_is_loaded_from_config(config: RobotConfig) -> None:
    assert {
        name: (joint.motor_id, joint.can_id)
        for name, joint in config.joints.items()
    } == {
        "shoulder_flexion": (22, 2),
        "shoulder_abduction": (23, 2),
        "elbow_flexion": (25, 2),
        "upper_arm_rotation": (24, 2),
    }
    assert all(joint.direction_status == "NEEDS_HARDWARE_VALIDATION" for joint in config.joints.values())


def test_position_clamp(config: RobotConfig) -> None:
    joint = replace(config.joints["elbow_flexion"], min_position=-0.2, max_position=0.3)
    assert clamp_position(joint, -1.0) == pytest.approx(-0.2)
    assert clamp_position(joint, 1.0) == pytest.approx(0.3)
    assert clamp_position(joint, 0.1) == pytest.approx(0.1)


def test_step_and_velocity_limits_use_the_stricter_limit(config: RobotConfig) -> None:
    joint = replace(
        config.joints["elbow_flexion"],
        max_position_step=0.1,
        max_velocity=0.2,
    )
    assert limit_position_change(joint, current=0.0, target=1.0, dt=0.25) == pytest.approx(0.05)
    assert limit_position_change(joint, current=0.0, target=-1.0, dt=1.0) == pytest.approx(-0.1)


def test_velocity_limit_requires_dt(config: RobotConfig) -> None:
    joint = replace(config.joints["elbow_flexion"], max_velocity=0.2)
    with pytest.raises(SafetyError, match="positive dt"):
        limit_position_change(joint, current=0.0, target=0.1)


def test_measured_velocity_limit_is_checked(config: RobotConfig) -> None:
    from sleeve_arm.control.safety import validate_feedback

    joint = replace(config.joints["elbow_flexion"], max_velocity=0.2)
    robot = FakeRobotArm(config)
    robot.connect()
    state = replace(robot.read_joint_state("elbow_flexion"), velocity=0.3)

    with pytest.raises(FeedbackError, match="velocity exceeds"):
        validate_feedback(state, joint)
    robot.close()


def test_feedback_error_disables_robot(config: RobotConfig) -> None:
    robot = FakeRobotArm(config)
    controller = SafeArmController(robot, config)
    controller.connect()
    controller.enable()
    robot.set_joint_error("elbow_flexion", 7)

    with pytest.raises(SafetyError, match="motion stopped"):
        controller.set_joint_position("elbow_flexion", 0.01)

    assert not controller.enabled
    assert not robot.enabled
    assert robot.disable_count == 1
    assert robot.closed
    assert not controller.connected


def test_unavailable_configured_current_is_rejected(config: RobotConfig) -> None:
    joint = replace(config.joints["elbow_flexion"], max_current=1.0)
    current_limited = replace(config, joints={**config.joints, "elbow_flexion": joint})
    robot = FakeRobotArm(current_limited)
    controller = SafeArmController(robot, current_limited)
    controller.connect()
    controller.enable()
    state = robot.read_joint_state("elbow_flexion")
    unavailable = replace(state, current=None)

    from sleeve_arm.control.safety import validate_feedback

    with pytest.raises(FeedbackError, match="current feedback is unavailable"):
        validate_feedback(unavailable, joint)
    controller.shutdown()


def test_shutdown_disables_then_closes(config: RobotConfig) -> None:
    robot = FakeRobotArm(config)
    controller = SafeArmController(robot, config)
    controller.connect()
    controller.enable()

    controller.shutdown()

    assert robot.disable_count == 1
    assert robot.closed
    assert not robot.connected
    assert not controller.connected
    assert robot.events[-2:] == ["disable", "close"]


def test_failed_startup_feedback_closes_backend(config: RobotConfig) -> None:
    robot = FakeRobotArm(config)
    robot.set_joint_error("elbow_flexion", 9)
    controller = SafeArmController(robot, config)

    with pytest.raises(FeedbackError, match="motor error code 9"):
        controller.connect()

    assert robot.closed
    assert not robot.connected
    assert not controller.connected


def test_keyboard_interrupt_during_startup_still_closes(config: RobotConfig) -> None:
    class InterruptingRobot(FakeRobotArm):
        def read_joint_state(self, joint_name: str):
            raise KeyboardInterrupt

    robot = InterruptingRobot(config)
    controller = SafeArmController(robot, config)

    with pytest.raises(KeyboardInterrupt):
        controller.connect()

    assert robot.closed
    assert not robot.connected
    assert robot.events[-2:] == ["disable", "close"]


def test_unstable_startup_feedback_is_rejected(config: RobotConfig) -> None:
    class MovingRobot(FakeRobotArm):
        reads = 0

        def read_joint_state(self, joint_name: str):
            state = super().read_joint_state(joint_name)
            if joint_name == "elbow_flexion":
                self.reads += 1
                return replace(state, position=0.1 * self.reads)
            return state

    elbow = replace(config.joints["elbow_flexion"], max_position_step=0.01)
    stability_config = replace(config, joints={**config.joints, "elbow_flexion": elbow})
    robot = MovingRobot(stability_config)
    controller = SafeArmController(robot, stability_config)

    with pytest.raises(SafetyError, match="startup feedback is not stable"):
        controller.connect()

    assert robot.closed
    assert not controller.connected


def test_keyboard_interrupt_during_motion_disables_and_closes(config: RobotConfig) -> None:
    class InterruptingCommandRobot(FakeRobotArm):
        def set_joint_positions(self, targets):
            raise KeyboardInterrupt

    robot = InterruptingCommandRobot(config)
    controller = SafeArmController(robot, config)
    controller.connect()
    controller.enable()

    with pytest.raises(SafetyError, match="motion stopped"):
        controller.set_joint_position("elbow_flexion", 0.01, dt=0.05)

    assert robot.events[-2:] == ["disable", "close"]
    assert not robot.connected
    assert not controller.connected


def test_shutdown_still_closes_when_disable_is_interrupted(config: RobotConfig) -> None:
    class InterruptingDisableRobot(FakeRobotArm):
        def disable(self) -> None:
            self.events.append("disable-interrupted")
            raise KeyboardInterrupt

    robot = InterruptingDisableRobot(config)
    controller = SafeArmController(robot, config)
    controller.connect()

    with pytest.raises(RobotError, match="Servo Off failed"):
        controller.shutdown()

    assert robot.closed
    assert not controller.connected
    assert robot.events[-1] == "close"
