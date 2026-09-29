"""Command slew and measured overspeed are separate limits; no hardware calls."""
from dataclasses import asdict, replace
import math
from pathlib import Path

import pytest
import yaml

from sleeve_arm.robot.aurora_fake import fake_profile, fake_session, gr3_fake_profile
from sleeve_arm.robot.aurora_profile import GR3_SEMANTIC_INDICES, load_aurora_profile
from tools import aurora_arm_debug as tool


@pytest.mark.parametrize("filename", ["robot_aurora.yaml", "aurora_196_shoulder.yaml"])
def test_normal_profiles_match_official_position_and_velocity_limits(filename):
    path = Path(__file__).resolve().parents[1] / "configs" / filename
    profile = load_aurora_profile(path)
    profile.validate(execute=True)
    # Independent copy of the official SDK-coordinate ranges, including wrists.
    positions = {
        "left": ((-2.9671, 2.9671), (-0.2618, 1.9199), (-1.8326, 1.8326),
                 (-2.2689, .087266), (-1.8326, 1.8326), (-.87266, 1.309), (-1.0472, 1.2217)),
        "right": ((-2.9671, 2.9671), (-1.9199, .2618), (-1.8326, 1.8326),
                  (-2.2689, .087266), (-1.8326, 1.8326), (-.87266, 1.309), (-1.2217, 1.0472)),
    }
    velocities = (7.75, 7.75, 6.28, 6.28, 6.28, 9.2153, 9.2153)
    for group in profile.groups:
        if group.part != "arm":
            continue
        assert group.sdk_position_limits == positions[group.side]
        for joint in group.joints:
            limits = joint.limits
            semantic_bounds = sorted(joint.from_sdk(v) for v in positions[group.side][joint.index])
            assert [limits.min_position, limits.max_position] == pytest.approx(semantic_bounds)
            assert limits.max_velocity == limits.max_feedback_velocity == velocities[joint.index]
            assert limits.max_position_step == pytest.approx(limits.max_velocity * profile.control_period_s)


def slow_command_profile():
    """Keep the original 0.3 rad/s regression independent of site speed settings."""
    profile = gr3_fake_profile()
    return replace(profile, groups=tuple(
        replace(group, joints=tuple(
            replace(joint, limits=replace(joint.limits, max_velocity=0.3,
                                         max_position_step=0.01, max_feedback_velocity=None))
            for joint in group.joints
        )) for group in profile.groups
    ))


def with_feedback_limit(profile, value):
    return replace(profile, groups=tuple(
        replace(group, joints=tuple(
            replace(joint, limits=replace(joint.limits, max_feedback_velocity=value))
            for joint in group.joints
        )) for group in profile.groups
    ))


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("name,expected", [
    ("shoulder_flexion", 7.75), ("shoulder_abduction", 7.75),
    ("upper_arm_rotation", 6.28), ("elbow_flexion", 6.28), ("wrist_yaw", 6.28),
    ("wrist_pitch", 9.2153), ("wrist_roll", 9.2153),
])
def test_gr3_feedback_limits_match_official_joint_order(side, name, expected):
    profile = slow_command_profile()
    group = profile.selected_groups((side,))[0]
    index = GR3_SEMANTIC_INDICES[name]
    low, high = group.sdk_position_limits[index]
    source = group.joints[0]
    joint = replace(source, name=name, index=index, sign=1, zero=0.,
                    kind="wrist" if name.startswith("wrist_") else "arm",
                    limits=replace(source.limits, name=name, min_position=low, max_position=high))
    profile = replace(profile, groups=(replace(group, joints=(joint,)),))
    profile.validate(execute=True, simulation=True)
    limits = profile.control_config((side,)).joints[name]
    assert limits.max_velocity == 0.3
    assert limits.max_feedback_velocity == expected


def test_unknown_robot_keeps_legacy_feedback_limit():
    profile = fake_profile()
    config = profile.control_config(("left",))
    assert all(joint.max_feedback_velocity == joint.max_velocity == 3. for joint in config.joints.values())


def test_explicit_feedback_limit_roundtrips_and_applies_to_both_arm_keys(tmp_path):
    profile = with_feedback_limit(slow_command_profile(), 0.6)
    path = tmp_path / "profile.yaml"
    path.write_text(yaml.safe_dump(asdict(profile)))
    loaded = load_aurora_profile(path)
    loaded.validate(execute=True, simulation=True)
    config = loaded.control_config(("left", "right"))
    assert config.joints["left.shoulder_flexion"].max_feedback_velocity == 0.6
    assert config.joints["right.shoulder_flexion"].max_feedback_velocity == 0.6
    assert all(joint.max_velocity == 0.3 for joint in config.joints.values())


@pytest.mark.parametrize("value", [0., -1., math.nan, math.inf, True, "7.75", 0.2, 8.])
def test_invalid_or_inconsistent_feedback_limits_are_rejected(value):
    with pytest.raises(ValueError, match="velocity"):
        with_feedback_limit(slow_command_profile(), value).validate()


def test_command_speed_cannot_exceed_gr3_hardware_limit():
    profile = gr3_fake_profile()
    group = profile.groups[0]
    joint = group.joints[0]
    joint = replace(joint, limits=replace(joint.limits, max_velocity=8.))
    profile = replace(profile, groups=(replace(group, joints=(joint, *group.joints[1:])), *profile.groups[1:]))
    with pytest.raises(ValueError, match="max_velocity exceeds feedback velocity limit"):
        profile.validate()


@pytest.mark.parametrize("measured,override,expected", [
    (0.31, None, 0), (-0.31, None, 0),
    (7.75, None, 0), (-7.75, None, 0),
    (8., None, 1), (-8., None, 1),
    (0.5, 0.6, 0), (0.7, 0.6, 1),
])
def test_fast_motion_uses_feedback_limit_without_raising_command_speed(monkeypatch, capsys, measured, override, expected):
    profile = with_feedback_limit(slow_command_profile(), override)
    session = fake_session(profile=profile)
    monkeypatch.setattr(tool, "fake_session", lambda **kwargs: session)
    client = session.fake_client
    publish = client.set_group_cmd

    def publish_with_measured_velocity(*args, **kwargs):
        result = publish(*args, **kwargs)
        # Right shoulder flexion has sign=-1. Real feedback can exceed the
        # command slew rate even though every submitted position step is bounded.
        client.groups["right_manipulator"]["velocity"][0] = -measured
        return result

    monkeypatch.setattr(client, "set_group_cmd", publish_with_measured_velocity)
    assert tool.main(["--backend", "fake", "--simulate", "--action", "1", "--angle-deg", "5"]) == expected
    assert client.closed
    previous = 0.
    for command in client.commands:
        value = command["right_manipulator"][0]
        assert abs(value - previous) <= 0.3 * profile.control_period_s + 1e-10
        previous = value
    assert abs(client.commands[0]["right_manipulator"][0]) == pytest.approx(0.003)
    output = capsys.readouterr()
    assert '"velocity_limits_rad_s"' in output.out
    if expected == 0:
        assert previous == pytest.approx(-math.radians(5))
        assert '"arrived": true' in output.out
        assert '"feedback_velocity_rad_s"' in output.out
    else:
        assert len(client.commands) == 1
        assert '"arrived": true' not in output.out
        assert "feedback velocity exceeds max_feedback_velocity" in output.err
        assert f"measured_rad_s={measured:.9g}" in output.err
        assert f"limit_rad_s={override if override is not None else 7.75}" in output.err
        assert "command_max_velocity_rad_s=0.3" in output.err


@pytest.mark.parametrize("velocity", [None, math.nan, math.inf])
def test_separate_limit_still_rejects_missing_or_nonfinite_feedback(monkeypatch, velocity):
    session = fake_session(profile=gr3_fake_profile())
    monkeypatch.setattr(tool, "fake_session", lambda **kwargs: session)
    session.fake_client.groups["right_manipulator"]["velocity"] = (
        [] if velocity is None else [velocity, 0., 0., 0., 0., 0., 0.]
    )
    assert tool.main(["--backend", "fake", "--simulate", "--action", "1", "--angle-deg", "5"]) == 1
    assert not session.fake_client.commands
    assert session.fake_client.closed
