from dataclasses import replace
from types import SimpleNamespace

import pytest

from sleeve_arm.domain import ImuFrame, MotionIntent
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import FakeClock, fake_session, gr3_fake_profile
from sleeve_arm.runtime import configuration, intent_pipeline, sensors as sensor_module
from sleeve_arm.runtime.configuration import load_configs, parse_args
from sleeve_arm.runtime.model_control import ModelControlApp
from sleeve_arm.runtime.robot_runtime import PrintOnlyRuntime
from sleeve_arm.runtime.sensors import SensorRuntime
from sleeve_arm.runtime.telemetry import Telemetry
from tools import run_model_control


def frame(timestamp):
    return ImuFrame(timestamp, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0)


def test_print_only_uses_real_imu_pipeline_without_sleeve_or_elbow_predictor(monkeypatch, capsys):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    monkeypatch.setattr("time.sleep", clock.sleep)
    monkeypatch.setattr("builtins.input", lambda _: "")

    def forbidden(*args, **kwargs):
        pytest.fail("IMU-only mode attempted to initialize Sleeve, elbow or robot hardware")

    monkeypatch.setattr(sensor_module, "create_sleeve_source", forbidden)
    monkeypatch.setattr(sensor_module, "FakeSleeveSource", forbidden)
    monkeypatch.setattr(intent_pipeline, "RuleBasedPredictor", forbidden)
    monkeypatch.setattr(factory, "create_robot", forbidden)
    assert run_model_control.main([
        "--print-only", "--imus", "fake", "--calibration-seconds", ".1", "--duration", "2.05",
    ]) == 0
    output = capsys.readouterr().out
    assert "IMU ONLY" in output
    assert output.count("[PRINT ONLY]") == 2
    assert "动作=静止(Rest)" in output
    assert "肩前屈/后伸=+0.0°" in output and "肩外展=+0.0°" in output
    assert "肘屈曲=未提供" in output and "上臂旋转=+0.0°" in output


@pytest.mark.parametrize("options,expected", [
    ([], "real"), (["--print-only"], "none"),
    (["--print-only", "--sleeve", "real"], "real"),
    (["--print-only", "--sleeve", "fake"], "fake"),
    (["--print-only", "--sleeve", "none"], "none"),
])
def test_sleeve_defaults_and_explicit_selection(options, expected):
    configs = load_configs(parse_args(["--robot", "fake", *options]))
    assert configs.sleeve_mode == expected


def test_flex_model_print_only_still_requires_sleeve(monkeypatch, tmp_path):
    phase4 = configuration.load_phase4_config()
    phase4 = replace(phase4, flex_model=replace(phase4.flex_model, model_dir=tmp_path))
    monkeypatch.setattr(configuration, "load_phase4_config", lambda _: phase4)
    args = parse_args(["--print-only", "--shoulder-predictor", "flexarm_estimator"])
    assert load_configs(args).sleeve_mode == "real"
    args.sleeve = "none"
    with pytest.raises(ValueError, match="flexarm_estimator needs Sleeve data"):
        load_configs(args)


def test_sleeveless_dymotor_control_still_requires_sleeve():
    with pytest.raises(SystemExit) as exc:
        parse_args(["--sleeve", "none", "--execute"])
    assert exc.value.code == 2


@pytest.mark.parametrize("sensor_flags", [["--sleeve", "fake"], ["--imus", "fake"]])
def test_real_aurora_still_rejects_fake_sensor_inputs(sensor_flags):
    with pytest.raises(SystemExit) as exc:
        parse_args(["--robot", "aurora", "--execute", "--duration", "1",
                    "--aurora-profile", "configs/robot_aurora.yaml", "--side", "right",
                    "--source-side", "right", *sensor_flags])
    assert exc.value.code == 2


@pytest.mark.parametrize("sleeve", ["auto", "none"])
def test_aurora_imu_control_runs_without_sleeve(monkeypatch, capsys, sleeve):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    monkeypatch.setattr("time.sleep", clock.sleep)
    monkeypatch.setattr("builtins.input", lambda _: "YES")

    def forbidden(*args, **kwargs):
        pytest.fail("Aurora IMU-only control attempted to initialize Sleeve or elbow prediction")

    monkeypatch.setattr(sensor_module, "create_sleeve_source", forbidden)
    monkeypatch.setattr(sensor_module, "FakeSleeveSource", forbidden)
    monkeypatch.setattr(intent_pipeline, "RuleBasedPredictor", forbidden)
    profile = gr3_fake_profile()
    session = fake_session(clock=clock, profile=profile)
    actual_factory = factory.create_robot
    monkeypatch.setattr(factory, "create_robot",
                        lambda backend, config, **kwargs: actual_factory(backend, config, session=session, **kwargs))
    create_sensors = run_model_control.create_sensor_runtime

    def injected_sensors(args, configs):
        sensors = create_sensors(args, configs)
        assert sensors.sleeve_mode == "none"
        # Exercise the execute CLI with simulated IMUs; never open a serial port.
        sensors.imu_mode = "fake"
        return sensors

    monkeypatch.setattr(run_model_control, "create_sensor_runtime", injected_sensors)
    assert run_model_control.main([
        "--robot", "aurora", "--execute", "--aurora-profile", "configs/robot_aurora.yaml",
        "--side", "right", "--source-side", "right", "--sleeve", sleeve,
        "--calibration-seconds", ".1", "--duration", "1.1",
    ]) == 0
    output = capsys.readouterr().out
    assert "IMU ONLY" in output and "imu_fps=" in output and "sleeve_fps=" not in output
    assert len(session.fake_client.commands) > 50
    assert session.fake_client.closed


def test_imu_only_samples_use_oldest_actual_imu_timestamp_and_keep_sleeve_absent():
    configs = load_configs(parse_args(["--print-only"]))
    sensor_config = replace(configs.sensors, upper_arm_rotation=replace(configs.sensors.upper_arm_rotation, enabled=False))
    sensors = SensorRuntime(sensor_config, sleeve="none")
    first, second = sensors.shoulder_names
    latest = {first: None, second: frame(100.01)}
    sensors.imu_sources = {name: SimpleNamespace(latest=lambda name=name: latest[name]) for name in latest}
    assert sensors.latest() is None
    latest[first] = frame(100.)
    sample = sensors.latest()
    assert sample.sleeve is None and sample.timestamp == 100.
    assert (sample.imu1, sample.imu2) == (latest[first], latest[second])
    latest[second] = frame(100.015)
    assert sensors.latest().timestamp == 100.
    latest[first] = frame(100.012)
    assert sensors.latest().timestamp == 100.012


@pytest.mark.parametrize("failed_side", [0, 1])
def test_one_disconnected_imu_cannot_refresh_watchdog(monkeypatch, capsys, failed_side):
    clock = FakeClock()
    configs = load_configs(parse_args(["--print-only"]))
    sensor_config = replace(configs.sensors, upper_arm_rotation=replace(configs.sensors.upper_arm_rotation, enabled=False))
    sensors = SensorRuntime(sensor_config, sleeve="none")
    closed = []
    sensors.imu_sources = {
        name: SimpleNamespace(
            latest=lambda index=index: frame(100. if index == failed_side else clock.now),
            close=lambda name=name: closed.append(name),
        ) for index, name in enumerate(sensors.shoulder_names)
    }
    monkeypatch.setattr(sensors, "start", lambda: None)
    app = ModelControlApp(
        sensors=sensors,
        intents=SimpleNamespace(
            prepare=lambda _: None,
            predict=lambda sample: MotionIntent(sample.timestamp, shoulder_flexion_rad=0.),
        ),
        robot=PrintOnlyRuntime(), phase3=replace(configs.phase3, sensor_timeout_ms=20, hard_timeout_ms=50),
        phase4=configs.phase4, duration=1., clock=clock, telemetry=Telemetry(print_only=True),
    )
    assert app.run() == 1
    assert "IMU hard timeout" in app.fault
    assert app.predictions == 0 and app.stale > 0
    assert set(closed) == set(sensors.shoulder_names)
    assert "WARNING: IMU stale" in capsys.readouterr().out
