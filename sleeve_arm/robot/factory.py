"""One creation boundary; optional Aurora SDK is loaded only on real connect."""
from __future__ import annotations

from sleeve_arm.config import load_robot_config
from sleeve_arm.robot.base import RobotArm


def create_robot(backend: str, config=None, *, profile=None, side=None, sides=None,
                 parts=("arm",), execute=False, operator_confirmed=False,
                 library_path=None, diagnostics=False, session=None) -> RobotArm:
    if backend in ("fake", "dymotor"):
        from sleeve_arm.robot import DyMotorArm, FakeRobotArm
        config = config or load_robot_config()
        return FakeRobotArm(config) if backend == "fake" else DyMotorArm(config, library_path, diagnostics=diagnostics)
    if backend not in ("aurora", "aurora-fake"):
        raise ValueError(f"unknown robot backend: {backend}")
    from sleeve_arm.robot.aurora import AuroraRobotArm
    from sleeve_arm.robot.aurora_profile import AuroraRobotProfile, load_aurora_profile
    from sleeve_arm.robot.aurora_session import AuroraSession, SystemClock
    if side is not None and sides is not None:
        raise ValueError("use side or sides, not both")
    selected = tuple(sides) if sides is not None else ((side,) if side is not None else ())
    if session is None:
        if backend == "aurora-fake":
            from sleeve_arm.robot.aurora_fake import fake_session
            session = fake_session(execute=execute, clock=SystemClock())
        else:
            if profile is None:
                raise ValueError("Aurora requires an explicit profile")
            profile = profile if isinstance(profile, AuroraRobotProfile) else load_aurora_profile(profile)
            session = AuroraSession.real(profile, execute=execute, operator_confirmed=operator_confirmed)
    return AuroraRobotArm(session, sides=selected, parts=parts)
