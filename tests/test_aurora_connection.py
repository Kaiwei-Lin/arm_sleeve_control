"""Exercise the connection example with an injected SDK; never connect hardware."""
import math
import os
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from sleeve_arm.robot.aurora_fake import FakeClock
from tools import test_aurora_connection as tool


@pytest.fixture
def connection(monkeypatch):
    client = Mock(spec=["get_fsm_state", "get_stand_pose", "get_group_state",
                        "set_fsm_state", "set_group_cmd", "close"])
    client.get_fsm_state.return_value = 11
    client.get_stand_pose.return_value = [0.0, 0.0, 0.0, 0.0]
    positions = {
        "left_manipulator": [0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7],
        "right_manipulator": [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7],
    }
    client.get_group_state.side_effect = lambda group, key: positions[group]
    factory = Mock(return_value=client)

    class DDSMatchTimeout(Exception):
        pass

    class SDK:
        _init_publishers = Mock()

        @classmethod
        def get_instance(cls, **kwargs):
            cls._init_publishers(client)
            return factory(**kwargs)

    monkeypatch.setitem(sys.modules, "fourier_aurora_client",
                        SimpleNamespace(AuroraClient=SDK))
    monkeypatch.setitem(sys.modules, "fourier_aurora_client.exceptions",
                        SimpleNamespace(DDSMatchTimeout=DDSMatchTimeout))
    monkeypatch.setattr(tool.metadata, "version", lambda _: "0.1.8")
    clock = FakeClock()
    monkeypatch.setattr(tool, "time", clock)
    prompts = Mock(return_value="")
    monkeypatch.setattr("builtins.input", prompts)
    monkeypatch.setenv("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT")
    monkeypatch.setenv("FOURIERDDS_ROS_COMPATIBLE", "true")
    monkeypatch.setenv("FOURIERDDS_USE_DISCOVERY_SERVER", "true")
    return SimpleNamespace(client=client, factory=factory, clock=clock,
                           positions=positions, prompts=prompts, sdk=SDK,
                           publisher_init=SDK._init_publishers, timeout=DDSMatchTimeout)


def test_only_arms_lift_smoothly_after_prompts(connection):
    def confirm(_):
        connection.client.set_group_cmd.assert_not_called()
        if connection.prompts.call_count == 1:
            connection.client.set_fsm_state.assert_not_called()
        else:
            connection.client.set_fsm_state.assert_called_once_with(11)
        return ""

    connection.prompts.side_effect = confirm
    assert tool.main([]) == 0
    connection.factory.assert_called_once_with(domain_id=123, robot_name="gr3", is_ros_compatible=False)
    assert os.environ["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"
    assert os.environ["FOURIERDDS_ROS_COMPATIBLE"] == "false"
    assert os.environ["FOURIERDDS_USE_DISCOVERY_SERVER"] == "false"
    connection.client.set_fsm_state.assert_called_once_with(11)
    connection.client.get_stand_pose.assert_not_called()
    assert connection.prompts.call_count == 2
    commands = [call.kwargs["position_cmd"] for call in connection.client.set_group_cmd.call_args_list]
    assert commands[0] == connection.positions
    for group, initial in connection.positions.items():
        assert commands[-1][group][0] == pytest.approx(initial[0] - math.radians(30))
        previous = initial[0]
        for command in commands:
            assert set(command) == set(connection.positions)
            assert command[group][1:] == initial[1:]
            assert 0 <= previous - command[group][0] <= math.radians(15) * 0.01
            previous = command[group][0]
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("fsm", [0, 2, 3, 9, 10, None])
def test_mode_timeout_reports_actual_fsm_and_never_sends_arm_commands(connection, fsm):
    connection.client.get_fsm_state.return_value = fsm
    started = connection.clock.monotonic()
    with pytest.raises(RuntimeError, match=f"等待 UpperBodyUserCmd 超时：需要 FSM 11，当前 FSM={fsm}"):
        tool.main([])
    assert connection.clock.monotonic() - started < 6.2
    assert connection.prompts.call_count == 1
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("pose", [None, [], [0, 0, 0, 0], [0, 0, 0, 100],
                                  [0, 0, 0, float("nan")]])
def test_suspended_mode_does_not_require_pdstand_feedback(connection, pose):
    connection.client.get_stand_pose.return_value = pose
    assert tool.main([]) == 0
    assert connection.client.set_group_cmd.call_count == 301
    connection.client.get_stand_pose.assert_not_called()
    connection.client.close.assert_called_once_with()


def test_waits_for_actual_fsm_transition_before_arm_prompt(connection):
    connection.client.get_fsm_state.side_effect = [0, 0] + [11] * 302

    def confirm(_):
        if connection.prompts.call_count == 2:
            assert connection.client.get_fsm_state.call_count == 3
            connection.client.set_group_cmd.assert_not_called()
        return ""

    connection.prompts.side_effect = confirm
    assert tool.main([]) == 0
    connection.client.set_fsm_state.assert_called_once_with(11)
    assert connection.client.set_group_cmd.call_count == 301


@pytest.mark.parametrize("position", [None, [], [0] * 6, [0] * 8,
                                         [float("nan")] * 7, [float("inf")] * 7,
                                         [-2.9, 0, 0, 0, 0, 0, 0], [3.0, 0, 0, 0, 0, 0, 0]])
def test_invalid_feedback_or_target_never_sends_arm_commands(connection, position):
    connection.positions["right_manipulator"] = position
    with pytest.raises(RuntimeError):
        tool.main([])
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


def test_mode_loss_during_motion_stops_commands(connection):
    connection.client.set_group_cmd.side_effect = lambda **_: setattr(
        connection.client.get_fsm_state, "return_value", 3)
    with pytest.raises(RuntimeError, match="离开 UpperBodyUserCmd.*当前 FSM=3"):
        tool.main([])
    assert connection.client.set_group_cmd.call_count == 1
    connection.client.close.assert_called_once_with()


def test_mode_loss_while_waiting_for_arm_confirmation_never_sends(connection):
    def confirm(_):
        if connection.prompts.call_count == 2:
            connection.client.get_fsm_state.return_value = 2
        return ""

    connection.prompts.side_effect = confirm
    with pytest.raises(RuntimeError, match="离开 UpperBodyUserCmd.*当前 FSM=2"):
        tool.main([])
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("position", [None, [0] * 6, [float("nan")] * 7])
def test_invalid_feedback_during_motion_stops_commands(connection, position):
    def lose_feedback(**_):
        connection.positions["right_manipulator"] = position

    connection.client.set_group_cmd.side_effect = lose_feedback
    with pytest.raises(RuntimeError, match="right_manipulator 未收到有效"):
        tool.main([])
    assert connection.client.set_group_cmd.call_count == 1
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("failure", [RuntimeError("DDS send failed"), KeyboardInterrupt()])
def test_send_failure_or_interrupt_closes_client(connection, failure):
    connection.client.set_group_cmd.side_effect = failure
    with pytest.raises(type(failure)):
        tool.main([])
    connection.client.close.assert_called_once_with()


def test_failed_initialization_does_not_prompt_or_send(connection):
    connection.factory.return_value = None
    with pytest.raises(RuntimeError, match="初始化失败"):
        tool.main([])
    connection.prompts.assert_not_called()
    connection.client.set_fsm_state.assert_not_called()
    connection.client.set_group_cmd.assert_not_called()


def test_connect_only_reads_feedback_and_never_prompts_or_commands(connection, capsys):
    connection.client.get_fsm_state.return_value = 0
    connection.client.get_stand_pose.return_value = [0.0] * 4
    assert tool.main(["--connect-only"]) == 0
    assert "连接成功，当前 FSM：0" in capsys.readouterr().out
    assert connection.client.get_group_state.call_count == 2
    connection.prompts.assert_not_called()
    connection.client.set_fsm_state.assert_not_called()
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


def test_connect_only_invalid_feedback_fails_and_closes(connection):
    connection.positions["right_manipulator"] = None
    with pytest.raises(RuntimeError, match="未收到有效"):
        tool.main(["--connect-only"])
    connection.prompts.assert_not_called()
    connection.client.set_fsm_state.assert_not_called()
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("names,accepted", [
    (["velocity_cmd"], True),
    (["velocity_cmd", "robot_control_group_cmd"], False),
    (["robot_control_group_cmd"], False),
    (["not_velocity_cmd"], False),
    ([], False),
])
def test_connection_uses_exact_velocity_exception_and_restores_sdk(connection, names, accepted):
    connection.client._publisher_list = [
        SimpleNamespace(is_matched=False, _topic=SimpleNamespace(get_name=lambda n=n: n))
        for n in names
    ]
    connection.client._publisher_list.append(SimpleNamespace(is_matched=True))
    connection.publisher_init.side_effect = connection.timeout("publisher timeout")
    if accepted:
        assert tool.main(["--connect-only"]) == 0
        connection.client.close.assert_called_once_with()
    else:
        with pytest.raises(connection.timeout):
            tool.main(["--connect-only"])
        connection.factory.assert_not_called()
    assert connection.sdk._init_publishers is connection.publisher_init
    connection.prompts.assert_not_called()
    connection.client.set_fsm_state.assert_not_called()
    connection.client.set_group_cmd.assert_not_called()


def test_wrong_sdk_version_rejected_before_initialization(connection, monkeypatch):
    monkeypatch.setattr(tool.metadata, "version", lambda _: "0.2.1")
    with pytest.raises(RuntimeError, match="当前为 0.2.1"):
        tool.main(["--connect-only"])
    connection.publisher_init.assert_not_called()
    connection.factory.assert_not_called()
    connection.prompts.assert_not_called()
