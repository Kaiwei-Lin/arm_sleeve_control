from pathlib import Path


SOURCE = (Path(__file__).parents[1] / "native/dymotor_bridge/dymotor_bridge.c").read_text()


def _function_body(name: str, next_name: str) -> str:
    return SOURCE.split(f"{name}(", 1)[1].split(f"{next_name}(", 1)[0]


def test_bridge_uses_vendor_startup_order_before_first_pvct_read() -> None:
    startup = _function_body("static void run_vendor_startup_pipeline", "static int validate_open_config")
    open_body = _function_body("int arm_open", "static int read_joint_feedback")
    expected = (
        "robot_StateMachine(g_arm.ctx, 0x80)",
        "robot_StateMachine(g_arm.ctx, 1)",
        "robot_motor_setHeartbeat",
        "robot_motor_set_control_mode",
        "robot_motor_set_pos",
        "robot_motor_set_big_pose",
        "robot_motor_set_control_world",
        "wait_one_second",
    )

    assert [startup.index(call) for call in expected] == sorted(startup.index(call) for call in expected)
    assert "CTRL_SERVO_ON" in startup
    assert "read_joint_feedback" not in startup
    assert open_body.index("robot_set_fast_mode") < open_body.index("robot_create_motor")
    assert open_body.index("robot_create_motor") < open_body.index("run_vendor_startup_pipeline")
