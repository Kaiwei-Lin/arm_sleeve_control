import math
from types import SimpleNamespace

import pytest

from sleeve_arm.domain import MotionIntent
from sleeve_arm.runtime.telemetry import Telemetry


@pytest.mark.parametrize("action,label", [
    ("Forward", "前抬"), ("Backward", "后伸"), ("Lateral", "侧抬"),
    ("Rest", "静止"), ("Transition", "过渡"), ("Unknown", "未知"),
])
def test_print_only_displays_signed_degrees_and_no_robot_feedback(capsys, action, label):
    intent = MotionIntent(timestamp=1., model_action=action, confidence=.95,
                          shoulder_flexion_rad=math.radians(-30), shoulder_abduction_rad=math.radians(45),
                          elbow_flexion=math.radians(90), upper_arm_rotation_rad=math.radians(-10))
    Telemetry(print_only=True, imu_debug=True).report(
        sample=None, intent=intent, targets={}, feedback={},
        stats=dict(state="STALE", age=.3, invalid=1, stale=2, rotation_invalid=0),
        sensor_stats=None, diagnostics={"shoulder_frames": [
            SimpleNamespace(quat_w=1., quat_x=0., quat_y=0., quat_z=0.),
            SimpleNamespace(quat_w=1., quat_x=0., quat_y=0., quat_z=0.),
        ]},
    )
    output = capsys.readouterr().out
    for expected in (f"动作={label}({action})", "肩前屈/后伸=-30.0°", "肩外展=+45.0°",
                     "肘屈曲=+90.0°", "上臂旋转=-10.0°", "置信度=0.950", "state=STALE",
                     "shoulder_arm_q=(1.0, 0.0, 0.0, 0.0)"):
        assert expected in output
    assert "positions_rad=" not in output and "tracking_rad=" not in output
