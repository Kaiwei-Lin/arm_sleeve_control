from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "native/dymotor_bridge/dymotor_bridge.c").read_text()


def _function_body(name: str, next_name: str) -> str:
    return SOURCE.split(f"{name}(", 1)[1].split(f"{next_name}(", 1)[0]


def test_open_and_never_enabled_shutdown_have_no_servo_or_state_machine_writes() -> None:
    open_body = _function_body("int arm_open", "static int read_joint_feedback")
    disable_body = _function_body("int arm_disable", "int arm_set_joint_positions")
    close_body = _function_body("void arm_close", "const char *arm_last_error")

    assert "robot_StateMachine" not in open_body
    assert "servo_off_all" not in open_body
    assert "!g_arm.enabled" in disable_body
    assert "release_session(g_arm.enabled)" in close_body
