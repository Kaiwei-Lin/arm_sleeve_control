from __future__ import annotations

import math
from dataclasses import replace

import pytest

from sleeve_arm.config import load_phase3_config, load_robot_config
from sleeve_arm.control import ArmMapper, SafeArmController, SensorWatchdog
from sleeve_arm.domain import MotionIntent, SensorSample, SleeveFrame
from sleeve_arm.predictor import RuleBasedPredictor
from sleeve_arm.robot import FakeRobotArm
from sleeve_arm.sources import FakeSleeveSource
from sleeve_arm.sync import SensorSynchronizer


def elbow_config(**changes):
    config = load_phase3_config().elbow
    defaults = {
        "input_min": 10.0,
        "input_max": 20.0,
        "angle_min_deg": 0.0,
        "angle_max_deg": 100.0,
        "filter": replace(config.filter, type="none", alpha=None),
    }
    defaults.update(changes)
    return replace(config, **defaults)


def test_uncalibrated_real_input_is_rejected() -> None:
    uncalibrated = replace(load_phase3_config().elbow, input_min=None, input_max=None)
    with pytest.raises(ValueError, match="calibrated input_min"):
        RuleBasedPredictor(uncalibrated)


@pytest.mark.parametrize(("raw", "normalized", "angle_deg"), ((10.0, 0.0, 0.0), (20.0, 1.0, 100.0), (15.0, 0.5, 50.0), (0.0, 0.0, 0.0), (30.0, 1.0, 100.0)))
def test_normalization_and_absolute_angle_mapping(raw: float, normalized: float, angle_deg: float) -> None:
    predictor = RuleBasedPredictor(elbow_config(deadzone=0.0))
    intent = predictor.predict(SensorSample(raw, SleeveFrame(raw, (0.0, raw))))
    assert predictor.last_normalized == pytest.approx(normalized)
    assert intent.elbow_flexion == pytest.approx(math.radians(angle_deg))
    assert intent.shoulder_flexion_rad is None
    assert intent.shoulder_abduction_rad is None


def test_invert_deadzone_and_ema() -> None:
    config = elbow_config(
        invert_input=True,
        deadzone=0.1,
        filter=replace(load_phase3_config().elbow.filter, type="ema", alpha=0.5),
    )
    predictor = RuleBasedPredictor(config)
    first = predictor.predict(SensorSample(1.0, SleeveFrame(1.0, (0.0, 10.0))))
    second = predictor.predict(SensorSample(2.0, SleeveFrame(2.0, (0.0, 14.5))))
    assert first.elbow_flexion == pytest.approx(math.radians(100.0))
    assert predictor.last_normalized == 0.5  # inverted 0.55 falls inside center deadzone
    assert second.elbow_flexion == pytest.approx(math.radians(75.0))


def test_missing_channel_is_rejected() -> None:
    predictor = RuleBasedPredictor(elbow_config())
    with pytest.raises(ValueError, match="CH2 is missing"):
        predictor.predict(SensorSample(1.0, SleeveFrame(1.0, (10.0,))))
    with pytest.raises(ValueError):
        SleeveFrame(1.0, (0.0, math.nan))
    with pytest.raises(ValueError):
        SleeveFrame(1.0, (0.0, math.inf))


def test_mapper_holds_shoulders_and_passes_absolute_elbow_angle() -> None:
    startup = {"shoulder_flexion": 0.2, "shoulder_abduction": -0.3, "elbow_flexion": 1.0}
    mapper = ArmMapper(elbow_config(deadzone=0.0), startup)
    low = mapper.map(MotionIntent(1.0, elbow_flexion=0.0))
    middle = mapper.map(MotionIntent(1.0, elbow_flexion=0.5))
    high = mapper.map(MotionIntent(1.0, elbow_flexion=1.0))
    assert low["elbow_flexion"] == pytest.approx(0.0)
    assert middle["elbow_flexion"] == pytest.approx(0.5)
    assert high["elbow_flexion"] == pytest.approx(1.0)
    assert high["shoulder_flexion"] == startup["shoulder_flexion"]
    assert high["shoulder_abduction"] == startup["shoulder_abduction"]


def test_watchdog_fresh_stale_hard_and_invalid_timestamp() -> None:
    watchdog = SensorWatchdog(200, 1000)
    assert not watchdog.is_stale(10.0, 10.1)
    assert watchdog.is_stale(10.0, 10.3)
    assert not watchdog.is_hard_timeout(10.0, 10.3)
    assert watchdog.is_hard_timeout(10.0, 11.1)
    with pytest.raises(ValueError):
        watchdog.age(11.0, 10.0)


def test_fake_end_to_end_uses_safety_and_only_moves_elbow() -> None:
    robot_config = load_robot_config()
    robot = FakeRobotArm(robot_config)
    controller = SafeArmController(robot, robot_config)
    controller.connect()
    startup = {name: state.position for name, state in controller.read_joint_states().items()}
    config = elbow_config(input_min=0.0, input_max=1.0, angle_min_deg=0.0, angle_max_deg=90.0, deadzone=0.0)
    predictor = RuleBasedPredictor(config)
    mapper = ArmMapper(config, startup)
    source = FakeSleeveSource()
    source.start()
    frame = source.latest()
    assert frame is not None
    sample = SensorSynchronizer().synchronize(frame)

    controller.enable()
    safe = controller.set_joint_positions(mapper.map(predictor.predict(sample)), dt=0.05)
    states = controller.read_joint_states()
    controller.shutdown()
    source.close()

    assert states["shoulder_flexion"].position == startup["shoulder_flexion"]
    assert states["shoulder_abduction"].position == startup["shoulder_abduction"]
    assert safe["elbow_flexion"] == pytest.approx(math.radians(90))
    assert robot.events == ["connect", "enable", "disable", "close"]
