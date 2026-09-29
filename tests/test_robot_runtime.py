from types import SimpleNamespace
from dataclasses import replace
from pathlib import Path
import json
import os
import subprocess
import sys

import pytest

from sleeve_arm.domain import MotionIntent
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import FakeClock, fake_session, gr3_fake_profile
from sleeve_arm.robot.aurora_profile import load_aurora_profile, GR3_ARM_MAX_VELOCITY
from sleeve_arm.runtime.configuration import parse_args, load_configs
from sleeve_arm.runtime.robot_runtime import RobotRuntime, PrintOnlyRuntime, create_robot_runtime


@pytest.fixture
def launch_control(tmp_path, monkeypatch):
    """Capture the launcher's final argv; neither stub can open real hardware."""
    root = Path(__file__).resolve().parents[1]
    python = tmp_path / "python"
    python.write_text(
        f"#!{sys.executable}\n"
        "import json, os, sys\n"
        "if sys.argv[1] == '-c':\n"
        "    print(__file__ if 'sys.executable' in sys.argv[2] else '')\n"
        "else:\n"
        "    print(json.dumps(dict(argv=sys.argv[1:], cwd=os.getcwd(), "
        "sudo=os.environ.get('CONTROL_TEST_SUDO') == '1')))\n"
    )
    python.chmod(0o755)
    sudo = tmp_path / "sudo"
    sudo.write_text('#!/bin/sh\nexport CONTROL_TEST_SUDO=1\nexec "$@"\n')
    sudo.chmod(0o755)
    monkeypatch.setenv("PATH", str(tmp_path) + os.pathsep + os.environ["PATH"])
    monkeypatch.delenv("CONTROL_TEST_SUDO", raising=False)

    def launch(*argv):
        result = subprocess.run(["bash", str(root / "control.sh"), *argv], cwd=tmp_path,
                                check=True, capture_output=True, text=True)
        captured = json.loads(result.stdout)
        assert captured["cwd"] == str(root)
        assert captured["argv"][:2] == ["-u", "tools/run_model_control.py"]
        return captured["argv"][2:], captured["sudo"]

    return launch


def test_control_script_runtime_uses_official_speed_for_all_controlled_joints(monkeypatch, capsys, launch_control):
    argv, used_sudo = launch_control()
    assert used_sudo
    args = parse_args(argv)
    profile = load_aurora_profile(args.aurora_profile)
    session = fake_session(profile=replace(profile, simulated=True, allow_missing_velocity_cmd=False))
    actual_factory = factory.create_robot
    monkeypatch.setattr(factory, "create_robot",
                        lambda backend, config, **kwargs: actual_factory(backend, config, session=session, **kwargs))
    runtime = RobotRuntime(args, load_configs(args), clock=session.clock, input_fn=lambda _: "YES")
    assert runtime.configs.sleeve_mode == "none"
    runtime.connect()
    try:
        assert runtime.prepare() and runtime.stream_between_samples
        period = 1. / runtime.configs.phase3.control_hz
        assert period == profile.control_period_s == .01
        intent = MotionIntent(timestamp=session.clock.monotonic(), shoulder_flexion_rad=.5,
                              shoulder_abduction_rad=.5, elbow_flexion=.5, upper_arm_rotation_rad=.5)
        previous = {name: 0. for name in runtime.robot.config.joints}
        for _ in range(3):
            session.clock.sleep(period)
            applied = runtime.apply(intent, period)
            for name, value in applied.items():
                assert value - previous[name] == pytest.approx(GR3_ARM_MAX_VELOCITY[name] * period)
            previous = applied
        assert all(set(command) == {"right_manipulator"} for command in session.fake_client.commands)
        assert "Aurora streaming limits" in capsys.readouterr().out
    finally:
        runtime.shutdown()


def test_control_print_only_forwards_options_without_sudo_or_execute(launch_control):
    argv, used_sudo = launch_control("--print-only", "--duration", "3", "--sleeve", "fake", "--imus", "fake")
    args = parse_args(argv)
    assert args.print_only and not args.execute and not used_sudo
    assert args.duration == 3 and args.sleeve == args.imus == "fake"
    assert args.side == args.source_side == "right"
    assert args.shoulder_predictor == "dual_imu" and args.imu_debug


@pytest.mark.parametrize("options,expected", [((), "none"), (("--sleeve", "real"), "real")])
@pytest.mark.parametrize("mode", [(), ("--print-only",)])
def test_control_skips_sleeve_unless_explicitly_requested(launch_control, options, expected, mode):
    from sleeve_arm.runtime.sensors import create_sensor_runtime

    argv, used_sudo = launch_control(*mode, *options)
    args = parse_args(argv)
    configs = load_configs(args)
    sensors = create_sensor_runtime(args, configs)
    assert used_sudo is (not mode) and args.execute is (not mode)
    assert sensors.sleeve_mode == configs.sleeve_mode == expected


def test_control_imu_only_preserves_elbow_pose_after_confirmation(monkeypatch, launch_control):
    argv, _ = launch_control()
    args = parse_args(argv)
    profile = load_aurora_profile(args.aurora_profile)
    session = fake_session(profile=replace(profile, simulated=True, allow_missing_velocity_cmd=False))
    group = profile.selected_groups(("right",))[0]
    elbow = next(j for j in group.joints if j.name == "elbow_flexion")
    session.fake_client.groups[group.name]["position"][elbow.index] = elbow.to_sdk(.4)
    actual_factory = factory.create_robot
    monkeypatch.setattr(factory, "create_robot",
                        lambda backend, config, **kwargs: actual_factory(backend, config, session=session, **kwargs))

    def confirm(_):
        # The pose may change during calibration or the confirmation prompt.
        session.fake_client.groups[group.name]["position"][elbow.index] = elbow.to_sdk(.7)
        session.clock.sleep(.01)
        return "YES"

    runtime = RobotRuntime(args, load_configs(args), clock=session.clock, input_fn=confirm)
    runtime.connect()
    try:
        assert runtime.prepare()
        assert runtime.startup["elbow_flexion"] == pytest.approx(.7)
        intent = MotionIntent(timestamp=session.clock.monotonic(), shoulder_flexion_rad=.1,
                              shoulder_abduction_rad=.1, upper_arm_rotation_rad=.1)
        for _ in range(4):
            session.clock.sleep(.01)
            applied = runtime.apply(intent, .01)
            assert "elbow_flexion" not in applied
        commands = session.fake_client.commands
        assert len(commands) == 5
        assert all(command[group.name][elbow.index] == pytest.approx(elbow.to_sdk(.7)) for command in commands)
        shoulder = next(j for j in group.joints if j.name == "shoulder_flexion")
        assert commands[-1][group.name][shoulder.index] != commands[0][group.name][shoulder.index]
    finally:
        runtime.shutdown()


def test_control_help_does_not_require_sudo(launch_control):
    argv, used_sudo = launch_control("--help")
    assert not used_sudo
    with pytest.raises(SystemExit) as exc:
        parse_args(argv)
    assert exc.value.code == 0


@pytest.mark.parametrize("flags", [("--execute",), ("--prepare-aurora-fsm",)])
def test_print_only_rejects_motion_flags(flags):
    with pytest.raises(SystemExit) as exc:
        parse_args(["--print-only", *flags])
    assert exc.value.code == 2


@pytest.mark.parametrize("backend", ["dymotor", "fake", "aurora", "aurora-fake"])
def test_print_only_does_not_load_robot_config_or_create_backend(monkeypatch, tmp_path, backend):
    def forbidden(*args, **kwargs):
        pytest.fail("print-only attempted robot initialization")

    monkeypatch.setattr(factory, "create_robot", forbidden)
    args = parse_args(["--print-only", "--robot", backend, "--side", "right", "--source-side", "right",
                       "--robot-config", str(tmp_path / "missing.yaml"),
                       "--aurora-profile", str(tmp_path / "missing-aurora.yaml")])
    configs = load_configs(args)
    assert not configs.offline_preview and configs.robot is None
    runtime = create_robot_runtime(args, configs)
    assert isinstance(runtime, PrintOnlyRuntime)
    runtime.connect()
    try:
        assert runtime.prepare()
        assert runtime.apply(MotionIntent(timestamp=1., shoulder_flexion_rad=.5, elbow_flexion=.2), .01) == {
            "shoulder_flexion": .5, "elbow_flexion": .2,
        }
        runtime.check_health()
        assert runtime.feedback() == {} and not runtime.motion_enabled
    finally:
        runtime.shutdown()


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
