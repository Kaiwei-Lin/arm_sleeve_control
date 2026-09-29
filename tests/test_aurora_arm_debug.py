"""Interactive arm diagnostics with injected SDK/clock; never connect hardware."""
from dataclasses import replace
import math
from pathlib import Path
import subprocess
import sys

import pytest

from tools import aurora_arm_debug as tool


@pytest.mark.parametrize("action,value,side,joint,angle", [
    ("1", "30", "right", "shoulder_flexion", 30),
    ("2", "30", "right", "shoulder_flexion", -30),
    ("1", " ３０．５ ", "right", "shoulder_flexion", 30.5),
    ("3", "20", "left", "shoulder_abduction", 20),
    ("4", "5", "right", "shoulder_abduction", -5),
    ("5", "30", "right", "elbow_flexion", 30),
    ("6", "10", "right", "elbow_flexion", -10),
    ("1", "0", "right", "shoulder_flexion", 0),
])
def test_menu_command_meaning(action, value, side, joint, angle):
    assert tool.make_command(side, action, value) == tool.ArmCommand(side, joint, angle)


@pytest.mark.parametrize("text", [
    "-30", "nan", "inf", "181", "30度", "30 20", "三十", "", "9" * 400,
])
def test_ambiguous_or_invalid_input_is_rejected(text):
    with pytest.raises(ValueError):
        tool.parse_angle(text)


def injected_session(monkeypatch, *, profile=None):
    session = tool.fake_session(profile=profile or tool.gr3_fake_profile())
    monkeypatch.setattr(tool, "fake_session", lambda **kwargs: session)
    return session


def test_repl_moves_both_directions_reuses_session_and_preserves_other_slots(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    original = list(session.fake_client.groups["right_manipulator"]["position"])
    lines = iter(["1", "30", "8", "2", "30", "0"])

    def read(_):
        session.clock.sleep(30.)  # Operator idle time exceeds the feedback timeout.
        return next(lines)

    assert tool.main(["--backend", "fake", "--simulate"], input_fn=read) == 0
    client = session.fake_client
    assert client.calls.count("get_instance") == 1
    assert client.close_count == 1
    assert client.commands
    assert any(command["right_manipulator"][0] == pytest.approx(-math.pi / 6) for command in client.commands)
    assert client.commands[-1]["right_manipulator"][0] == pytest.approx(math.pi / 6)
    for command in client.commands:
        assert set(command) == {"right_manipulator"}
        assert command["right_manipulator"][1:] == original[1:]
    output = capsys.readouterr().out
    assert output.count('"arrived": true') == 2
    assert '"delivery_confirmed": false' in output
    assert "右臂动作菜单" in output
    assert "3. 侧摆（向外抬）" in output
    assert "set_fsm_state" not in client.calls


def test_current_reference_accumulates_against_feedback(monkeypatch):
    session = injected_session(monkeypatch)
    lines = iter(["1", "5", "1", "5", "0"])
    assert tool.main(["--backend", "fake", "--simulate", "--reference", "current"],
                     input_fn=lambda _: next(lines)) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(-math.radians(10))


@pytest.mark.parametrize("side,action,angle,index,sign", [
    ("right", "5", 30, 3, -1),
    ("right", "3", 20, 1, -1),
    ("left", "3", 20, 1, 1),
])
def test_group_and_joint_routing(monkeypatch, side, action, angle, index, sign):
    session = injected_session(monkeypatch)
    assert tool.main(["--backend", "fake", "--simulate", "--side", side,
                      "--action", action, "--angle-deg", str(angle)]) == 0
    group = f"{side}_manipulator"
    assert set(session.fake_client.commands[-1]) == {group}
    assert session.fake_client.commands[-1][group][index] == pytest.approx(sign * math.radians(angle))


@pytest.mark.parametrize("action,angle", [
    ("3", "150"),
    ("5", "150"),
])
def test_rejects_limits_without_sending(monkeypatch, action, angle):
    session = injected_session(monkeypatch)
    assert tool.main(["--backend", "fake", "--simulate", "--action", action, "--angle-deg", angle]) == 1
    assert not session.fake_client.commands
    assert session.fake_client.closed


def test_unverified_real_profile_rejected_before_factory(monkeypatch):
    profile = replace(tool.gr3_fake_profile(), verified=False, simulated=False)
    monkeypatch.setattr(tool, "load_aurora_profile", lambda _: profile)
    def forbidden(*args, **kwargs):
        pytest.fail("must reject before any real factory/SDK call")
    monkeypatch.setattr(tool.factory, "create_robot", forbidden)
    assert tool.main(["--execute", "--action", "1", "--angle-deg", "5"]) == 1


def test_default_preview_never_loads_sdk_or_opens_hardware():
    root = Path(__file__).resolve().parents[1]
    result = subprocess.run([sys.executable, "-c", '''
import importlib.abc
import runpy
import sys
class Block(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {'fourier_aurora_client', 'serial', 'flexarm'}:
            raise AssertionError('unexpected import: ' + fullname)
sys.meta_path.insert(0, Block())
sys.argv = ['tools/aurora_arm_debug.py', '--action', '1', '--angle-deg', '30']
runpy.run_path(sys.argv[0], run_name='__main__')
'''], cwd=root, capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '"status": "NO MOTION"' in result.stdout
    assert '"current_sdk_position": null' in result.stdout


@pytest.mark.parametrize("damage", ["fsm", "unstable", "sdk", "tracking"])
def test_sdk_fsm_and_feedback_failures_are_not_reported_as_success(monkeypatch, capsys, damage):
    session = injected_session(monkeypatch)
    client = session.fake_client
    if damage == "fsm":
        client.fsm = 10
    elif damage == "unstable":
        client.stand_pose[3] = 100
    elif damage == "sdk":
        client.failures["set_group_cmd"] = RuntimeError("DDS publish failed")
    else:
        client.follow_commands = False
    assert tool.main(["--backend", "fake", "--simulate", "--action", "1", "--angle-deg", "30"]) == 1
    assert client.closed
    assert '"arrived": true' not in capsys.readouterr().out
    if damage in ("fsm", "unstable"):
        assert not client.commands
    assert "set_fsm_state" not in client.calls


@pytest.mark.parametrize("interrupt,expected", [(EOFError, 0), (KeyboardInterrupt, 130)])
def test_input_interrupt_always_closes_session(monkeypatch, interrupt, expected):
    session = injected_session(monkeypatch)
    def read(_):
        raise interrupt
    assert tool.main(["--backend", "fake", "--simulate"], input_fn=read) == expected
    assert session.fake_client.closed
    assert not session.fake_client.commands


def injected_execution_session(monkeypatch):
    session = tool.fake_session(profile=tool.gr3_fake_profile())
    monkeypatch.setattr(tool, "load_aurora_profile", lambda _: session.profile)
    # Only the transport is simulated. Exercise the --execute CLI path.
    monkeypatch.setattr(type(session.profile), "validate", lambda *args, **kwargs: None)
    real_factory = tool.factory.create_robot
    monkeypatch.setattr(tool.factory, "create_robot",
                        lambda *args, **kwargs: real_factory("aurora-fake", session=session, side="right"))
    return session


def test_real_single_action_executes_without_confirmation(monkeypatch):
    session = injected_execution_session(monkeypatch)

    def unexpected_input(prompt):
        pytest.fail(f"single action must not prompt: {prompt}")

    assert tool.main(["--execute", "--action", "1", "--angle-deg", "5"], input_fn=unexpected_input) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(-math.radians(5))
    assert session.fake_client.closed


def test_real_menu_executes_repeated_actions_without_confirmation(monkeypatch, capsys):
    session = injected_execution_session(monkeypatch)
    lines = iter(["1", "5", "2", "5", "0"])

    def read(prompt):
        assert "YES" not in prompt
        return next(lines)

    assert tool.main(["--execute"], input_fn=read) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(math.radians(5))
    assert capsys.readouterr().out.count('"arrived": true') == 2
    assert session.fake_client.closed


def test_preparation_delay_preserves_previewed_relative_target(monkeypatch, capsys):
    session = injected_execution_session(monkeypatch)
    prepare = session.prepare_control_mode

    def delayed_prepare(**kwargs):
        prepare(**kwargs)
        session.clock.sleep(20.)
        # Actual arm position changes after the initial 0°→5° preview.
        session.fake_client.groups["right_manipulator"]["position"][0] = -math.radians(3)

    monkeypatch.setattr(session, "prepare_control_mode", delayed_prepare)
    assert tool.main(["--execute", "--reference", "current", "--action", "1", "--angle-deg", "5"]) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(-math.radians(5))
    assert '"arrived": true' in capsys.readouterr().out
    assert session.fake_client.closed


@pytest.mark.parametrize("max_step", [0.03, 0.001])
@pytest.mark.parametrize("action,angle", [("1", 0), ("1", 0.1), ("1", 5), ("1", 30), ("2", 5)])
def test_motion_uses_maximum_allowed_steps_until_exact_target(monkeypatch, action, angle, max_step):
    profile = tool.gr3_fake_profile()
    profile = replace(profile, groups=tuple(
        replace(group, joints=tuple(
            replace(joint, limits=replace(joint.limits, max_position_step=max_step))
            for joint in group.joints
        )) for group in profile.groups
    ))
    session = injected_session(monkeypatch, profile=profile)
    client = session.fake_client
    publish = client.set_group_cmd
    times = []

    def record(*args, **kwargs):
        times.append(session.clock.monotonic())
        return publish(*args, **kwargs)

    monkeypatch.setattr(client, "set_group_cmd", record)
    assert tool.main(["--backend", "fake", "--simulate", "--action", action,
                      "--angle-deg", str(angle)]) == 0
    group = profile.selected_groups(("right",))[0]
    joint = next(j for j in group.joints if j.name == "shoulder_flexion")
    target = math.radians(angle) * (1 if action == "1" else -1)
    period = profile.control_period_s
    max_change = min(joint.limits.max_velocity * period, max_step)
    previous = 0.0
    for command in client.commands:
        value = joint.from_sdk(command[group.name][joint.index])
        assert abs(value - previous) == pytest.approx(min(max_change, abs(target - previous)), abs=1e-10)
        previous = value
    assert previous == pytest.approx(target)
    expected_ticks = max(1, math.ceil(abs(target) / max_change))
    assert len(client.commands) == expected_ticks
    assert times[-1] - times[0] + period == pytest.approx(expected_ticks * period)
    assert client.closed


@pytest.mark.parametrize("delay,expected", [(0.001, 0), (0.02, 1)])
def test_fast_motion_handles_small_jitter_and_stops_on_scheduling_delay(monkeypatch, capsys, delay, expected):
    session = injected_session(monkeypatch)
    original_sleep = session.clock.sleep

    def delayed_sleep(seconds):
        original_sleep(seconds + (delay if session.fake_client.commands else 0.0))

    monkeypatch.setattr(session.clock, "sleep", delayed_sleep)
    assert tool.main(["--backend", "fake", "--simulate", "--action", "1", "--angle-deg", "5"]) == expected
    assert session.fake_client.closed
    output = capsys.readouterr()
    if expected == 0:
        assert '"arrived": true' in output.out
    else:
        assert "scheduling delay" in output.err
        assert len(session.fake_client.commands) == 1


def test_fast_motion_still_requires_arrival_feedback(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    session.fake_client.follow_commands = False
    started = session.clock.monotonic()
    # Outside arrival tolerance, but below the tracking-error limit.
    assert tool.main(["--backend", "fake", "--simulate", "--action", "1", "--angle-deg", "2"]) == 1
    assert session.clock.monotonic() - started < session.profile.arrival_timeout_s + 1
    output = capsys.readouterr()
    assert "arrival timed out" in output.err
    assert '"arrived": true' not in output.out
    assert session.fake_client.closed


def test_close_failure_is_reported_as_failure(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    session.fake_client.failures["close"] = RuntimeError("close failed")
    assert tool.main(["--backend", "fake", "--action", "8"]) == 1
    assert "清理失败" in capsys.readouterr().err


def test_interrupt_during_motion_stops_and_closes(monkeypatch):
    session = injected_session(monkeypatch)
    original_sleep = session.clock.sleep

    def sleep(seconds):
        if session.fake_client.commands:
            raise KeyboardInterrupt
        original_sleep(seconds)

    monkeypatch.setattr(session.clock, "sleep", sleep)
    assert tool.main(["--backend", "fake", "--simulate", "--action", "1", "--angle-deg", "5"]) == 130
    assert session.fake_client.closed


def test_invalid_menu_input_and_cancelled_angle_never_send(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    lines = iter(["9", "右臂向前30度", "1", "", "0"])
    prompts = []

    def read(prompt):
        prompts.append(prompt)
        return next(lines)

    assert tool.main(["--backend", "fake", "--simulate"], input_fn=read) == 0
    assert sum("请输入角度" in prompt for prompt in prompts) == 1
    assert not session.fake_client.commands
    assert "已取消，返回菜单" in capsys.readouterr().out


def test_angle_retries_then_executes_selected_direction(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    lines = iter(["２", "nan", "-5", "181", "30度", "１２．５", "0"])
    assert tool.main(["--backend", "fake", "--simulate"], input_fn=lambda _: next(lines)) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(math.radians(12.5))
    output = capsys.readouterr().out
    assert output.count("输入错误") == 4
    assert output.count('"arrived": true') == 1


@pytest.mark.parametrize("interrupt,expected", [(EOFError, 0), (KeyboardInterrupt, 130)])
def test_interrupt_at_angle_prompt_closes_without_sending(monkeypatch, interrupt, expected):
    session = injected_session(monkeypatch)

    def read(prompt):
        if "动作编号" in prompt:
            return "1"
        raise interrupt

    assert tool.main(["--backend", "fake", "--simulate"], input_fn=read) == expected
    assert not session.fake_client.commands
    assert session.fake_client.closed


@pytest.mark.parametrize("options", [
    ["--action", "1"], ["--angle-deg", "30"], ["--action", "8", "--angle-deg", "30"],
    ["--action", "1", "--angle-deg", "nan"], ["--action", "1", "--angle-deg", "-1"],
    ["--duration", "8"],
])
def test_invalid_single_action_arguments_fail_before_connect(monkeypatch, options):
    monkeypatch.setattr(tool.factory, "create_robot", lambda *a, **k: pytest.fail("unexpected connection"))
    with pytest.raises(SystemExit) as exc:
        tool.main(options)
    assert exc.value.code == 2
