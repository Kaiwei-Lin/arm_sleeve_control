"""Exercise the connection example with an injected SDK; never connect hardware."""
import math
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
    client.get_fsm_state.return_value = 2
    client.get_stand_pose.return_value = [0.0, 0.0, 0.0, 101.0]
    positions = {
        "left_manipulator": [0.1, 0.2, 0.3, -0.4, 0.5, 0.6, 0.7],
        "right_manipulator": [-0.1, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7],
    }
    client.get_group_state.side_effect = lambda group, key: positions[group]
    factory = Mock(return_value=client)
    monkeypatch.setitem(sys.modules, "fourier_aurora_client",
                        SimpleNamespace(AuroraClient=SimpleNamespace(get_instance=factory)))
    clock = FakeClock()
    monkeypatch.setattr(tool, "time", clock)
    prompts = Mock(return_value="")
    monkeypatch.setattr("builtins.input", prompts)
    monkeypatch.setenv("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT")
    return SimpleNamespace(client=client, factory=factory, clock=clock,
                           positions=positions, prompts=prompts)


def test_only_arms_lift_smoothly_after_prompts(connection):
    def confirm(_):
        connection.client.set_group_cmd.assert_not_called()
        if connection.prompts.call_count == 1:
            connection.client.set_fsm_state.assert_not_called()
        else:
            connection.client.set_fsm_state.assert_called_once_with(2)
        return ""

    connection.prompts.side_effect = confirm
    assert tool.main() == 0
    connection.factory.assert_called_once_with(domain_id=123, robot_name="gr3")
    assert tool.os.environ["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"
    connection.client.set_fsm_state.assert_called_once_with(2)
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


@pytest.mark.parametrize("fsm,pose", [
    (3, [0, 0, 0, 101]), (2, [0, 0, 0, 100]), (2, None),
    (2, []), (2, [0, 0, 0, float("nan")]), (2, [0, 0, 0, float("inf")]),
])
def test_standing_timeout_never_sends_arm_commands(connection, fsm, pose):
    connection.client.get_fsm_state.return_value = fsm
    connection.client.get_stand_pose.return_value = pose
    started = connection.clock.monotonic()
    with pytest.raises(RuntimeError, match="等待 PdStand 超时"):
        tool.main()
    assert connection.clock.monotonic() - started < 6.2
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("position", [None, [], [0] * 6, [0] * 8,
                                         [float("nan")] * 7, [float("inf")] * 7,
                                         [-2.9, 0, 0, 0, 0, 0, 0], [3.0, 0, 0, 0, 0, 0, 0]])
def test_invalid_feedback_or_target_never_sends_arm_commands(connection, position):
    connection.positions["right_manipulator"] = position
    with pytest.raises(RuntimeError):
        tool.main()
    connection.client.set_group_cmd.assert_not_called()
    connection.client.close.assert_called_once_with()


def test_standing_loss_during_motion_stops_commands(connection):
    connection.client.set_group_cmd.side_effect = lambda **_: setattr(
        connection.client.get_fsm_state, "return_value", 3)
    with pytest.raises(RuntimeError, match="离开稳定站立状态"):
        tool.main()
    assert connection.client.set_group_cmd.call_count == 1
    connection.client.close.assert_called_once_with()


@pytest.mark.parametrize("failure", [RuntimeError("DDS send failed"), KeyboardInterrupt()])
def test_send_failure_or_interrupt_closes_client(connection, failure):
    connection.client.set_group_cmd.side_effect = failure
    with pytest.raises(type(failure)):
        tool.main()
    connection.client.close.assert_called_once_with()


def test_failed_initialization_does_not_prompt_or_send(connection):
    connection.factory.return_value = None
    with pytest.raises(RuntimeError, match="初始化失败"):
        tool.main()
    connection.prompts.assert_not_called()
    connection.client.set_fsm_state.assert_not_called()
    connection.client.set_group_cmd.assert_not_called()
