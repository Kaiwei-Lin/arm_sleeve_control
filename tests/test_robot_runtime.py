from types import SimpleNamespace

import pytest

from sleeve_arm.domain import MotionIntent
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import FakeClock, fake_session, gr3_fake_profile
from sleeve_arm.runtime.configuration import parse_args, load_configs
from sleeve_arm.runtime.robot_runtime import RobotRuntime


@pytest.mark.parametrize("backend", ["fake", "aurora-fake"])
def test_runtime_selects_mapper_and_apply_preview_share_semantic_targets(monkeypatch, backend):
    argv = ["--robot", backend, "--sleeve", "fake", "--imus", "fake"]
    if backend == "aurora-fake":
        argv += ["--side", "right", "--source-side", "right"]
    args = parse_args(argv)
    configs = load_configs(args)
    clock = FakeClock()
    real_factory = factory.create_robot
    session = fake_session(clock=clock, profile=gr3_fake_profile())

    def create(name, config, **kwargs):
        if name == "aurora-fake":
            kwargs["session"] = session
        return real_factory(name, config, **kwargs)

    monkeypatch.setattr(factory, "create_robot", create)
    runtime = RobotRuntime(args, configs, clock=clock)
    runtime.connect()
    try:
        assert runtime.prepare()
        intent = MotionIntent(timestamp=100, shoulder_flexion_rad=.1, shoulder_abduction_rad=.2,
                              elbow_flexion=.4, upper_arm_rotation_rad=.3)
        seen = []
        monkeypatch.setattr(runtime.controller, "set_joint_positions", lambda targets, dt: seen.append((targets, dt)) or targets)
        monkeypatch.setattr(runtime.controller, "preview_positions", lambda targets, dt: seen.append((targets, dt)) or targets)
        expected = dict(shoulder_flexion=.1, shoulder_abduction=.2, elbow_flexion=.4,
                        upper_arm_rotation=.3 if backend == "aurora-fake" else 0.)
        assert runtime.apply(intent, .01) == expected
        assert runtime.preview(intent, .01) == expected
        assert seen == [(expected, .01), (expected, .01)]
    finally:
        runtime.shutdown()


@pytest.mark.parametrize("answer", ["", "yes", "YES ", "NO", "YES"])
def test_runtime_owns_confirmation_and_fsm_preparation_with_injected_robot(monkeypatch, answer):
    args = parse_args(["--robot", "aurora", "--aurora-profile", "configs/robot_aurora.yaml", "--execute",
                       "--side", "right", "--source-side", "right", "--duration", "1", "--prepare-aurora-fsm"])
    configs = load_configs(args)
    clock = FakeClock()
    session = fake_session(clock=clock, profile=gr3_fake_profile())
    real_factory = factory.create_robot
    monkeypatch.setattr(factory, "create_robot", lambda backend, config, **kw: real_factory(backend, config, session=session, **kw))
    runtime = RobotRuntime(args, configs, clock=clock, input_fn=lambda _: answer)
    runtime.connect()
    try:
        assert runtime.prepare() is (answer == "YES")
        assert ("set_fsm_state" in session.client.calls) is (answer == "YES")
        assert bool(session.client.commands) is (answer == "YES")
    finally:
        runtime.shutdown()
