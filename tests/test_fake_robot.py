from __future__ import annotations

from dataclasses import replace

import pytest

from sleeve_arm.config import DEFAULT_CONFIG_PATH, load_robot_config
from sleeve_arm.control import SafeArmController
from sleeve_arm.domain import JointCommand
from sleeve_arm.robot import FakeRobotArm, RobotError


@pytest.fixture
def controller() -> SafeArmController:
    config = load_robot_config(DEFAULT_CONFIG_PATH)
    config = replace(
        config,
        safety=replace(config.safety, stable_feedback_interval_s=0.0),
    )
    return SafeArmController(FakeRobotArm(config), config)


def test_fake_robot_requires_enable(controller: SafeArmController) -> None:
    controller.connect()
    robot = controller.robot
    with pytest.raises(RobotError, match="not enabled"):
        robot.set_joint_position("elbow_flexion", 0.01)
    controller.shutdown()


def test_fake_robot_three_joint_flow(controller: SafeArmController) -> None:
    controller.connect()
    controller.enable()
    command = JointCommand(0.01, -0.01, 0.005)

    sent = controller.set_joint_positions(command.as_dict(), dt=0.1)
    states = controller.read_joint_states()

    assert {name: states[name].position for name in sent} == pytest.approx(sent)
    assert sent == pytest.approx(command.as_dict())
    assert states["upper_arm_rotation"].position == 0.0
    controller.shutdown()


def test_unknown_joint_is_rejected(controller: SafeArmController) -> None:
    controller.connect()
    controller.enable()
    with pytest.raises(Exception, match="unknown joint"):
        controller.set_joint_position("motor_25", 0.01)
    controller.shutdown()


def test_invalid_semantic_input_is_rejected_before_dispatch(controller: SafeArmController) -> None:
    controller.connect()
    controller.enable()

    with pytest.raises(Exception, match="unknown joint"):
        controller.set_joint_position("motor_25", 0.01)

    # Invalid semantic input is rejected before hardware dispatch, so the
    # existing enabled session remains available; runtime feedback/SDK errors
    # are the paths that force emergency Servo Off.
    assert controller.enabled
    controller.shutdown()
