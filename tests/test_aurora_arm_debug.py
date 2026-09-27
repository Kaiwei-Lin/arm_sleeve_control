"""Interactive arm diagnostics with injected SDK/clock; never connect hardware."""
import math
from pathlib import Path
import subprocess
import sys

import pytest

from tools import aurora_arm_debug as tool


@pytest.mark.parametrize("text,side,joint,angle", [
    ("右臂向上抬30度", "right", "shoulder_flexion", 30),
    ("右臂向后30度", "right", "shoulder_flexion", -30),
    (" 右手臂 向前 ３０．５ 度。", "right", "shoulder_flexion", 30.5),
    ("左臂侧举20°", "left", "shoulder_abduction", 20),
    ("右臂向内收5度", "right", "shoulder_abduction", -5),
    ("右臂屈肘30度", "right", "elbow_flexion", 30),
    ("右臂伸肘10度", "right", "elbow_flexion", -10),
])
def test_chinese_command_meaning(text, side, joint, angle):
    assert tool.parse_command(text) == tool.ArmCommand(side, joint, angle)


@pytest.mark.parametrize("text", [
    "右臂向后-30度", "右臂向后nan度", "右臂向前181度", "右臂向前30",
    "右臂向后30度然后左臂向前30度", "左腿向前30度", "右臂向上三十度",
    "右臂回零", "", "右臂向前" + "9" * 400 + "度",
])
def test_ambiguous_or_invalid_input_is_rejected(text):
    with pytest.raises(ValueError):
        tool.parse_command(text)


def injected_session(monkeypatch):
    session = tool.fake_session(profile=tool.gr3_fake_profile())
    monkeypatch.setattr(tool, "fake_session", lambda **kwargs: session)
    return session


def test_repl_moves_both_directions_reuses_session_and_preserves_other_slots(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    original = list(session.fake_client.groups["right_manipulator"]["position"])
    lines = iter(["右臂向上抬30度", "状态", "右臂向后30度", "退出"])

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
    assert "set_fsm_state" not in client.calls


def test_current_reference_accumulates_against_feedback(monkeypatch):
    session = injected_session(monkeypatch)
    lines = iter(["右臂向前5度", "右臂向前5度", "退出"])
    assert tool.main(["--backend", "fake", "--simulate", "--reference", "current"],
                     input_fn=lambda _: next(lines)) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(-math.radians(10))


@pytest.mark.parametrize("side,name,index,sign", [
    ("right", "右臂屈肘30度", 3, -1),
    ("left", "左臂向外20度", 1, 1),
])
def test_group_and_joint_routing(monkeypatch, side, name, index, sign):
    session = injected_session(monkeypatch)
    assert tool.main(["--backend", "fake", "--simulate", "--side", side, "--command", name]) == 0
    group = f"{side}_manipulator"
    assert set(session.fake_client.commands[-1]) == {group}
    angle = 30 if index == 3 else 20
    assert session.fake_client.commands[-1][group][index] == pytest.approx(sign * math.radians(angle))


@pytest.mark.parametrize("command,extra", [
    ("左臂向前5度", []),
    ("右臂向外150度", []),
    ("右臂屈肘150度", []),
    ("右臂向前30度", ["--duration", "0.1"]),
])
def test_rejects_bad_side_limits_and_duration_without_sending(monkeypatch, command, extra):
    session = injected_session(monkeypatch)
    assert tool.main(["--backend", "fake", "--simulate", "--command", command, *extra]) == 1
    assert not session.fake_client.commands
    assert session.fake_client.closed


def test_unverified_real_profile_rejected_before_factory(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("must reject before any real factory/SDK call")
    monkeypatch.setattr(tool.factory, "create_robot", forbidden)
    assert tool.main(["--execute", "--command", "右臂向前5度"]) == 1


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
sys.argv = ['tools/aurora_arm_debug.py', '--command', '右臂向上抬30度']
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
    assert tool.main(["--backend", "fake", "--simulate", "--command", "右臂向前30度"]) == 1
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


@pytest.mark.parametrize("answer", ["no", "yes", "YES "])
def test_real_confirmation_cancellation_with_injected_fake_transport(monkeypatch, answer):
    session = tool.fake_session(profile=tool.gr3_fake_profile())
    monkeypatch.setattr(tool, "load_aurora_profile", lambda _: session.profile)
    # Only the transport is simulated. Exercise the real CLI confirmation path.
    monkeypatch.setattr(type(session.profile), "validate", lambda *args, **kwargs: None)
    real_factory = tool.factory.create_robot
    monkeypatch.setattr(tool.factory, "create_robot",
                        lambda *args, **kwargs: real_factory("aurora-fake", session=session, side="right"))
    assert tool.main(["--execute", "--command", "右臂向前5度"], input_fn=lambda _: answer) == 0
    assert not session.fake_client.commands
    assert session.fake_client.closed


def test_confirmation_delay_preserves_reviewed_relative_target(monkeypatch, capsys):
    session = tool.fake_session(profile=tool.gr3_fake_profile())
    monkeypatch.setattr(tool, "load_aurora_profile", lambda _: session.profile)
    monkeypatch.setattr(type(session.profile), "validate", lambda *args, **kwargs: None)
    real_factory = tool.factory.create_robot
    monkeypatch.setattr(tool.factory, "create_robot",
                        lambda *args, **kwargs: real_factory("aurora-fake", session=session, side="right"))

    def confirm(_):
        session.clock.sleep(20.)
        # Actual arm position changes while the user reviews the initial 0°→5° preview.
        session.fake_client.groups["right_manipulator"]["position"][0] = -math.radians(3)
        return "YES"

    assert tool.main(["--execute", "--reference", "current", "--command", "右臂向前5度"],
                     input_fn=confirm) == 0
    assert session.fake_client.commands[-1]["right_manipulator"][0] == pytest.approx(-math.radians(5))
    assert '"arrived": true' in capsys.readouterr().out
    assert session.fake_client.closed


def test_close_failure_is_reported_as_failure(monkeypatch, capsys):
    session = injected_session(monkeypatch)
    session.fake_client.failures["close"] = RuntimeError("close failed")
    assert tool.main(["--backend", "fake", "--command", "状态"]) == 1
    assert "清理失败" in capsys.readouterr().err


def test_interrupt_during_motion_stops_and_closes(monkeypatch):
    session = injected_session(monkeypatch)
    original_sleep = session.clock.sleep

    def sleep(seconds):
        if session.fake_client.commands:
            raise KeyboardInterrupt
        original_sleep(seconds)

    monkeypatch.setattr(session.clock, "sleep", sleep)
    assert tool.main(["--backend", "fake", "--simulate", "--command", "右臂向前5度"]) == 130
    assert session.fake_client.closed
