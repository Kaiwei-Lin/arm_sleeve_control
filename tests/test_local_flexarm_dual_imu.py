import math

import numpy as np
import pytest

from sleeve_arm.estimation.flexarm import DualImuArmEstimator


def quaternion(axis, angle_deg):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    half = math.radians(angle_deg) / 2.0
    return np.r_[math.cos(half), axis * math.sin(half)]


def multiply(left, right):
    w1, x1, y1, z1 = left
    w2, x2, y2, z2 = right
    result = np.asarray((
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ))
    return result / np.linalg.norm(result)


def yxy_quaternion(estimator, alpha_deg, beta_deg, gamma_deg):
    up = -estimator._down
    return multiply(
        multiply(quaternion(up, alpha_deg), quaternion(estimator._forward, beta_deg)),
        quaternion(up, gamma_deg),
    )


def test_calibration_cancels_world_pose_and_arm_sensor_mounting():
    estimator = DualImuArmEstimator()
    chest = quaternion((0, 0, 1), 37)
    mounting = quaternion((0, 0, 1), 23)
    estimator.calibrate([chest, -chest], [multiply(chest, mounting), -multiply(chest, mounting)])

    cases = (
        (quaternion((1, 0, 0), 90), "Forward", 90),
        (quaternion((1, 0, 0), -45), "Backward", 45),
        (quaternion((0, 1, 0), -70), "Lateral", 70),
    )
    for motion, direction, magnitude in cases:
        estimate = estimator.estimate(chest, multiply(multiply(chest, motion), mounting))
        assert estimate.direction == direction
        assert estimate.magnitude_deg == pytest.approx(magnitude, abs=0.2)
        assert estimate.angle_deg == estimate.magnitude_deg


def test_rest_and_diagonal_transition_are_reported():
    estimator = DualImuArmEstimator()
    estimator.calibrate((1, 0, 0, 0), (1, 0, 0, 0))
    assert estimator.update((1, 0, 0, 0), (1, 0, 0, 0)).direction == "Rest"

    target = np.asarray((1.0, 1.0, 0.0)) / math.sqrt(2.0)
    axis = np.cross((0.0, 0.0, -1.0), target)
    estimate = estimator.estimate((1, 0, 0, 0), quaternion(axis, 90))
    assert estimate.direction == "Transition"
    assert estimate.magnitude_deg == pytest.approx(90, abs=0.2)


def test_estimate_requires_calibration_and_valid_samples():
    estimator = DualImuArmEstimator()
    with pytest.raises(RuntimeError, match="calibrate"):
        estimator.estimate((1, 0, 0, 0), (1, 0, 0, 0))
    with pytest.raises(ValueError, match="counts"):
        estimator.calibrate([(1, 0, 0, 0)], [(1, 0, 0, 0), (1, 0, 0, 0)])


def test_forward_calibration_learns_direction_and_opposite_is_backward():
    estimator = DualImuArmEstimator(
        down_axis=(-1, 0, 0),
        forward_axis=(0, 0, 1),
        lateral_axis=(0, 1, 0),
    )
    identity = (1, 0, 0, 0)
    estimator.calibrate(identity, identity)
    forward_pose = quaternion((0, 0, 1), -60)
    result = estimator.calibrate_forward([identity] * 5, [forward_pose] * 5)

    assert result.forward_axis == pytest.approx((0, 1, 0), abs=1e-6)
    assert result.sample_count == 5
    assert estimator.estimate(identity, quaternion((0, 0, 1), -60)).direction == "Forward"
    assert estimator.estimate(identity, quaternion((0, 0, 1), 35)).direction == "Backward"


def test_identity_returns_zero_continuous_shoulder_angles():
    estimator = DualImuArmEstimator()
    identity = (1, 0, 0, 0)
    estimator.calibrate(identity, identity)

    estimate = estimator.estimate(identity, identity)

    assert estimate.forward_backward_deg == pytest.approx(0.0)
    assert estimate.lateral_deg == pytest.approx(0.0)
    assert estimate.upper_arm_rotation_deg == pytest.approx(0.0)
    assert np.isfinite((estimate.forward_backward_deg, estimate.lateral_deg,
                        estimate.upper_arm_rotation_deg)).all()


@pytest.mark.parametrize(
    ("motion", "expected_fb", "expected_lateral"),
    (
        (quaternion((1, 0, 0), 45), 45.0, 0.0),
        (quaternion((1, 0, 0), -30), -30.0, 0.0),
        (quaternion((0, 1, 0), -40), 0.0, 40.0),
    ),
)
def test_pure_raise_angles(motion, expected_fb, expected_lateral):
    estimator = DualImuArmEstimator()
    identity = (1, 0, 0, 0)
    estimator.calibrate(identity, identity)

    estimate = estimator.estimate(identity, motion)

    assert estimate.forward_backward_deg == pytest.approx(expected_fb, abs=0.2)
    assert estimate.lateral_deg == pytest.approx(expected_lateral, abs=0.2)


def test_diagonal_raise_has_two_continuous_components_with_preserved_magnitude():
    estimator = DualImuArmEstimator()
    identity = (1, 0, 0, 0)
    estimator.calibrate(identity, identity)
    target = np.asarray((1.0, 1.0, -1.0))
    target /= np.linalg.norm(target)
    axis = np.cross(estimator._down, target)
    angle = math.degrees(math.acos(float(np.dot(estimator._down, target))))

    estimate = estimator.estimate(identity, quaternion(axis, angle))

    assert estimate.forward_backward_deg > 0.0
    assert estimate.lateral_deg > 0.0
    assert math.hypot(estimate.forward_backward_deg, estimate.lateral_deg) == pytest.approx(
        estimate.magnitude_deg, abs=1e-6
    )


def test_yxy_third_rotation_is_upper_arm_rotation_without_changing_raise_angles():
    estimator = DualImuArmEstimator()
    identity = (1, 0, 0, 0)
    estimator.calibrate(identity, identity)
    without_rotation = estimator.estimate(identity, yxy_quaternion(estimator, -45, 45, 0))
    with_rotation = estimator.estimate(identity, yxy_quaternion(estimator, -45, 45, 30))

    assert with_rotation.upper_arm_rotation_deg == pytest.approx(30.0, abs=1e-6)
    assert with_rotation.forward_backward_deg == pytest.approx(
        without_rotation.forward_backward_deg, abs=1e-6
    )
    assert with_rotation.lateral_deg == pytest.approx(without_rotation.lateral_deg, abs=1e-6)


def test_yxy_singularity_stays_finite_and_anatomical_basis_is_right_handed():
    estimator = DualImuArmEstimator()
    identity = (1, 0, 0, 0)
    estimator.calibrate(identity, identity)
    estimate = estimator.estimate(identity, yxy_quaternion(estimator, 25, 1e-9, -10))
    basis = np.column_stack((estimator._forward, -estimator._down, estimator._lateral))

    assert np.isfinite((estimate.forward_backward_deg, estimate.lateral_deg,
                        estimate.upper_arm_rotation_deg)).all()
    assert basis.T @ basis == pytest.approx(np.eye(3), abs=1e-9)
    assert np.linalg.det(basis) == pytest.approx(1.0, abs=1e-9)
