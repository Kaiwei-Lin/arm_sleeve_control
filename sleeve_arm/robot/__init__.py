from sleeve_arm.robot.base import RobotArm, RobotError
from sleeve_arm.robot.dymotor import DyMotorArm
from sleeve_arm.robot.fake_robot import FakeRobotArm

__all__ = ["RobotArm", "RobotError", "DyMotorArm", "FakeRobotArm"]


# This class imports no optional Fourier SDK; loading is deferred to connect.
from sleeve_arm.robot.aurora import AuroraRobotArm
__all__.append("AuroraRobotArm")
