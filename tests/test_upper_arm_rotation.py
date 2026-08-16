from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pytest
import yaml

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_phase3_config, load_robot_config, load_sensor_config
from sleeve_arm.control import ArmMapper, SafeArmController
from sleeve_arm.domain import ImuFrame, MotionIntent
from sleeve_arm.estimation.upper_arm_rotation import (
    RelativeTwistEstimator,
    TwistEstimator,
    UpperArmRotationEstimator,
    _x_twist_degrees,
    average_quaternions,
    normalize_quaternion,
)
from sleeve_arm.robot import FakeRobotArm


def x_rotation(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return math.cos(half), math.sin(half), 0.0, 0.0


def imu(timestamp: float, quaternion: tuple[float, float, float, float]) -> ImuFrame:
    return ImuFrame(timestamp, 0.0, 0.0, 9.81, 0.0, 0.0, 0.0, *quaternion)


def test_quaternion_normalization_scale_sign_and_average() -> None:
    q = np.asarray(x_rotation(30.0))
    assert normalize_quaternion(2.0 * q) == pytest.approx(q)
    averaged = average_quaternions((q, -q))
    assert abs(float(np.dot(averaged, q))) == pytest.approx(1.0)


@pytest.mark.parametrize("angle", (30.0, -30.0))
def test_x_twist_degrees(angle: float) -> None:
    assert _x_twist_degrees(x_rotation(angle)) == pytest.approx(angle)


def test_unwrap_crosses_179_to_minus_179_continuously() -> None:
    estimator = TwistEstimator(ema_alpha=1.0)
    estimator.calibrate(((1.0, 0.0, 0.0, 0.0),))
    assert estimator.update(x_rotation(179.0)).unwrapped_deg == pytest.approx(179.0)
    assert estimator.update(x_rotation(-179.0)).unwrapped_deg == pytest.approx(181.0)


def test_ema_matches_reference_demo() -> None:
    estimator = TwistEstimator(ema_alpha=0.35)
    estimator.calibrate(((1.0, 0.0, 0.0, 0.0),))
    estimator.update(x_rotation(0.0))
    assert estimator.update(x_rotation(10.0)).filtered_deg == pytest.approx(3.5)


def test_relative_estimator_uses_inverse_reference_times_upper() -> None:
    estimator = RelativeTwistEstimator(ema_alpha=1.0)
    identity = (1.0, 0.0, 0.0, 0.0)
    estimator.calibrate(((identity, identity), (identity, identity)))
    result = estimator.update(x_rotation(30.0), x_rotation(10.0))
    assert result.filtered_deg == pytest.approx(20.0)


def test_calibration_zero_and_difference_formula() -> None:
    estimator = UpperArmRotationEstimator(ema_alpha=1.0, max_sync_ms=20.0)
    identity = imu(1.0, (1.0, 0.0, 0.0, 0.0))
    negative_identity = imu(1.001, (-1.0, 0.0, 0.0, 0.0))
    estimator.calibrate(((identity, identity), (negative_identity, negative_identity)))
    zero = estimator.update(identity, identity)
    assert zero.difference_deg == pytest.approx(0.0)

    result = estimator.update(imu(2.0, x_rotation(30.0)), imu(2.005, x_rotation(10.0)))
    assert result.world_filtered_deg == pytest.approx(30.0)
    assert result.relative_filtered_deg == pytest.approx(20.0)
    assert result.difference_deg == pytest.approx(
        result.world_filtered_deg - result.relative_filtered_deg
    )
    assert result.difference_deg == pytest.approx(10.0)
    assert result.sync_gap_ms == pytest.approx(5.0)


def test_sync_gap_is_rejected() -> None:
    estimator = UpperArmRotationEstimator(max_sync_ms=20.0)
    identity = imu(1.0, (1.0, 0.0, 0.0, 0.0))
    estimator.calibrate(((identity, identity), (identity, identity)))
    with pytest.raises(ValueError, match="sync gap"):
        estimator.update(identity, replace(identity, timestamp=1.021))
    assert estimator.sync_rejected_count == 1


def write_sensor_config(tmp_path, *, enabled: bool, imu3_enabled: bool, imu4_enabled: bool):
    raw = yaml.safe_load(DEFAULT_SENSOR_CONFIG_PATH.read_text(encoding="utf-8"))
    raw["upper_arm_rotation"]["enabled"] = enabled
    raw["sensors"]["imu3"]["enabled"] = imu3_enabled
    raw["sensors"]["imu4"]["enabled"] = imu4_enabled
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_optional_rotation_disabled_needs_no_rotation_imus(tmp_path) -> None:
    config = load_sensor_config(write_sensor_config(
        tmp_path, enabled=False, imu3_enabled=False, imu4_enabled=False
    ))
    assert not config.upper_arm_rotation.enabled
    assert config.upper_arm_rotation.max_sync_ms is None
    assert (config.shoulder_imu.arm_imu, config.shoulder_imu.chest_imu) == ("imu1", "imu2")


def test_enabled_rotation_accepts_two_enabled_role_sources(tmp_path) -> None:
    config = load_sensor_config(write_sensor_config(
        tmp_path, enabled=True, imu3_enabled=True, imu4_enabled=True
    ))
    assert config.upper_arm_rotation.upper_imu == "imu3"
    assert config.upper_arm_rotation.reference_imu == "imu4"
    assert config.imu3.enabled and config.imu4.enabled


@pytest.mark.parametrize(("imu3_enabled", "imu4_enabled"), ((False, True), (True, False)))
def test_enabled_rotation_requires_both_role_sources(
    tmp_path, imu3_enabled: bool, imu4_enabled: bool
) -> None:
    with pytest.raises(ValueError, match="requires both upper_imu and reference_imu"):
        load_sensor_config(write_sensor_config(
            tmp_path,
            enabled=True,
            imu3_enabled=imu3_enabled,
            imu4_enabled=imu4_enabled,
        ))


def test_rotation_config_rejects_invalid_ema(tmp_path) -> None:
    path = write_sensor_config(tmp_path, enabled=False, imu3_enabled=False, imu4_enabled=False)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw["upper_arm_rotation"]["ema_alpha"] = 0.0
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="ema_alpha"):
        load_sensor_config(path)


def test_shoulder_and_rotation_roles_cannot_overlap(tmp_path) -> None:
    path = write_sensor_config(tmp_path, enabled=True, imu3_enabled=True, imu4_enabled=True)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw["upper_arm_rotation"]["upper_imu"] = "imu1"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="disjoint"):
        load_sensor_config(path)


@pytest.mark.parametrize("name", ("imu1", "imu2"))
def test_sensor_config_allows_disabled_shoulder_source_for_flex_predictor(
    tmp_path,
    name: str,
) -> None:
    path = write_sensor_config(tmp_path, enabled=False, imu3_enabled=False, imu4_enabled=False)
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    raw["sensors"][name]["enabled"] = False
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    config = load_sensor_config(path)
    assert not getattr(config, name).enabled


def test_dual_imu_rotation_reaches_fake_robot_through_mapper_and_safety() -> None:
    estimator = UpperArmRotationEstimator(ema_alpha=1.0)
    identity = imu(1.0, (1.0, 0.0, 0.0, 0.0))
    estimator.calibrate(((identity, identity), (identity, identity)))
    rotation = estimator.update(imu(2.0, x_rotation(30.0)), imu(2.0, x_rotation(10.0)))

    robot_config = load_robot_config()
    robot = FakeRobotArm(robot_config)
    controller = SafeArmController(robot, robot_config)
    controller.connect()
    startup = {name: state.position for name, state in controller.read_joint_states().items()}
    mapper = ArmMapper(load_phase3_config().elbow, startup)
    intent = MotionIntent(
        timestamp=2.0,
        elbow_flexion=0.0,
        upper_arm_rotation_rad=math.radians(rotation.difference_deg),
    )
    controller.enable()
    safe = controller.set_joint_positions(mapper.map(intent), dt=0.01)
    controller.shutdown()

    assert safe["upper_arm_rotation"] == pytest.approx(math.radians(10.0))
    assert robot.events == ["connect", "enable", "disable", "close"]
