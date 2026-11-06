"""Sleeve dropout must not gate IMU or glove control; no hardware is opened."""
from dataclasses import replace
from types import SimpleNamespace

import pytest

from sleeve_arm.domain import ImuFrame, MotionIntent, SleeveFrame
from sleeve_arm.predictor import RuleBasedPredictor
from sleeve_arm.robot.aurora_fake import FakeClock
from sleeve_arm.runtime.configuration import load_configs, parse_args
from sleeve_arm.config import GloveConfig
from sleeve_arm.runtime.intent_pipeline import create_intent_pipeline
from sleeve_arm.runtime.model_control import ModelControlApp
from sleeve_arm.runtime.sensors import SensorRuntime
from sleeve_arm.runtime.glove_control import create_glove_runtime


def imu(timestamp):
    return ImuFrame(timestamp, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0)


@pytest.mark.parametrize("gap", [.25, 1.25])
@pytest.mark.parametrize("missing", [False, True])
@pytest.mark.parametrize("backend", ["dual_imu", "flexarm_estimator"])
def test_sleeve_dropout_keeps_independent_targets_streaming(monkeypatch, capsys, gap, missing, backend):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    args = parse_args(["--robot", "aurora-fake", "--side", "right", "--source-side", "right",
                       "--sleeve", "real"])
    configs = replace(load_configs(args), shoulder_backend=backend)
    configs = replace(configs, phase3=replace(configs.phase3, sensor_timeout_ms=200, hard_timeout_ms=1000),
                      elbow=replace(configs.elbow, sleeve_channel=2, input_min=1, input_max=438))
    sensors = SensorRuntime(configs.sensors, shoulder_backend=backend)
    sensors.start = lambda: None
    sensors.close = lambda: None
    first = clock.now
    reads = 0

    def sleeve():
        nonlocal reads
        reads += 1
        if clock.now - first >= gap:
            return SleeveFrame(first + gap, (0., 300., .3, .4, .5), 2)
        if missing and reads > 1:
            return None
        return SleeveFrame(first, (0., 100., .3, .4, .5), 1)

    sensors.source = SimpleNamespace(latest=sleeve, stats=SimpleNamespace(estimated_fps=1 / gap))
    sensors.imu_sources = {
        name: SimpleNamespace(latest=lambda: imu(clock.now), stats=SimpleNamespace(estimated_fps=100.))
        for name in sensors.required_imus
    }
    pipeline = create_intent_pipeline(args, configs)
    pipeline.sensors = sensors
    pipeline.prepare = lambda _: None
    shoulder_keys, elbow_keys, rotation_keys = [], [], []

    def shoulder(sample):
        stamp = sample.imu1.timestamp if backend == "dual_imu" else sample.sleeve.timestamp
        shoulder_keys.append(stamp)
        return MotionIntent(stamp, shoulder_flexion_rad=stamp - first,
                            shoulder_abduction_rad=0.)

    elbow_predictor = RuleBasedPredictor(configs.elbow)

    def elbow(sample):
        elbow_keys.append(sample.sleeve.sequence_id)
        return elbow_predictor.predict(sample)

    def rotation(a, b):
        rotation_keys.append((a.timestamp, b.timestamp))
        return SimpleNamespace(difference_deg=(a.timestamp - first) * 10)

    pipeline.shoulder = SimpleNamespace(predict=shoulder)
    pipeline.elbow = SimpleNamespace(predict=elbow)
    pipeline.rotation = SimpleNamespace(update=rotation, sync_rejected_count=0)
    pipeline.last_rotation_rad = 0.
    commands = []

    def apply(intent, dt, *, hand_targets):
        commands.append((clock.now - first, intent, hand_targets))
        return {"shoulder_flexion": intent.shoulder_flexion_rad}

    robot = SimpleNamespace(
        offline_preview=False, is_aurora=True, fail_fast=True, stream_between_samples=True,
        startup={"shoulder_flexion": 0.}, connect=lambda: None, prepare=lambda: True,
        check_health=lambda: None, apply=apply, feedback=lambda: {}, shutdown=lambda: None,
    )
    glove = SimpleNamespace(validate_for_control=lambda: None, describe_startup=lambda _: None,
                            start=lambda: None, wait_ready=lambda **_: None, close=lambda: None,
                            targets=lambda: {"index_flexion": clock.now - first}, report=lambda _: None)
    app = ModelControlApp(
        sensors=sensors, intents=pipeline, robot=robot, glove=glove,
        phase3=configs.phase3, phase4=configs.phase4, duration=gap + .06,
        clock=clock, telemetry=SimpleNamespace(report=lambda **_: None),
    )
    assert app.run() == 0 and app.fault is None
    during = [(t, intent, hand) for t, intent, hand in commands if .2 < t < gap]
    assert during and len(commands) == app.cycles
    assert elbow_keys == [1, 2]  # EMA advances once per Sleeve frame, not once per control tick.
    assert len(shoulder_keys) == len(set(shoulder_keys))
    assert len(rotation_keys) == len(set(rotation_keys))
    assert len(rotation_keys) > 20
    for t, intent, hand in during:
        assert intent.elbow_flexion == commands[0][1].elbow_flexion
        assert intent.upper_arm_rotation_rad == pytest.approx(t * 10 * 3.141592653589793 / 180)
        assert hand["index_flexion"] == pytest.approx(t)
        assert intent.shoulder_flexion_rad == pytest.approx(t if backend == "dual_imu" else 0.)
    assert commands[-1][1].elbow_flexion != commands[0][1].elbow_flexion
    assert capsys.readouterr().out.count("WARNING: Sleeve stale") == 1


def test_fresh_sleeve_cannot_mask_stale_imu(monkeypatch):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    args = parse_args(["--robot", "aurora-fake", "--side", "right", "--source-side", "right",
                       "--sleeve", "real"])
    configs = load_configs(args)
    configs = replace(configs, phase3=replace(configs.phase3, sensor_timeout_ms=200, hard_timeout_ms=1000))
    pipeline = create_intent_pipeline(args, configs)
    pipeline.sensors = SimpleNamespace(imu_error=None)
    from sleeve_arm.domain import SensorSample
    sample = SensorSample(clock.now, SleeveFrame(clock.now, (1., 2.)), imu(clock.now - 1), imu(clock.now))
    with pytest.raises(ValueError, match="shoulder IMU pair is stale"):
        pipeline.predict(sample)


@pytest.mark.parametrize("sleeve_mode", ["none", "real"])
@pytest.mark.parametrize("missing", [False, True])
@pytest.mark.parametrize("glove_backend", ["bend5", "bend6"])
def test_glove_dropout_does_not_interrupt_imu_commands(monkeypatch, capsys, sleeve_mode, missing, glove_backend):
    clock = FakeClock()
    monkeypatch.setattr("time.monotonic", clock.monotonic)
    args = parse_args(["--robot", "aurora-fake", "--side", "right", "--source-side", "right",
                       "--sleeve", sleeve_mode, "--glove", "fake"])
    configs = load_configs(args)
    configs = replace(configs, phase3=replace(configs.phase3, sensor_timeout_ms=200, hard_timeout_ms=1000),
                      sensors=replace(configs.sensors, glove=GloveConfig(backend=glove_backend),
                       upper_arm_rotation=replace(configs.sensors.upper_arm_rotation, enabled=False)))
    sensors = SensorRuntime(configs.sensors, sleeve=sleeve_mode)
    sensors.start = sensors.close = lambda: None
    sensors.imu_sources = {
        name: SimpleNamespace(latest=lambda: imu(clock.now), stats=SimpleNamespace(estimated_fps=100.))
        for name in sensors.required_imus
    }
    sensors.source = SimpleNamespace(latest=lambda: SleeveFrame(100., (0., 100., .3, .4, .5)),
                                     stats=SimpleNamespace(estimated_fps=0.))
    pipeline = create_intent_pipeline(args, configs)
    assert pipeline.independent_updates
    pipeline.sensors = sensors
    pipeline.prepare = lambda _: None
    pipeline.shoulder = SimpleNamespace(predict=lambda sample: MotionIntent(
        sample.imu1.timestamp, shoulder_flexion_rad=sample.imu1.timestamp - 100.))
    pipeline.elbow = RuleBasedPredictor(configs.elbow) if sleeve_mode == "real" else None
    glove = create_glove_runtime(args, configs)
    glove.clock = clock
    glove._prepared_source.clock = clock
    start = glove.start

    def start_glove():
        start()
        glove.mode = "real"  # Feed packets without opening a serial port.
        glove.source._store_packet([0.] * 11)

    original_latest = glove.latest
    recovered = False

    def latest():
        nonlocal recovered
        if clock.now >= 101.25 and not recovered:
            glove.source._store_packet([1.] * 9 + [0., 0.])
            recovered = True
        if missing and 100.1 < clock.now < 101.25:
            return None
        return original_latest()

    monkeypatch.setattr(glove, "start", start_glove)
    monkeypatch.setattr(glove, "latest", latest)
    commands = []

    def apply(intent, dt, *, hand_targets):
        commands.append((clock.now, intent, hand_targets))
        return {}

    robot = SimpleNamespace(
        offline_preview=False, is_aurora=True, fail_fast=True, stream_between_samples=True,
        startup={}, connect=lambda: None, prepare=lambda: True, check_health=lambda: None,
        apply=apply, feedback=lambda: {}, shutdown=lambda: None,
    )
    app = ModelControlApp(sensors=sensors, intents=pipeline, glove=glove, robot=robot,
                          phase3=configs.phase3, phase4=configs.phase4, duration=1.31,
                          clock=clock, telemetry=SimpleNamespace(report=lambda **_: None))
    assert app.run() == 0 and app.fault is None
    assert len(commands) == app.cycles > 100
    for stamp, intent, hand in commands:
        assert intent.shoulder_flexion_rad == pytest.approx(stamp - 100.)
        if stamp < 101.25:
            assert hand == commands[0][2]
    assert commands[-1][2] != commands[0][2]
    assert not glove.stale and glove.source is None
    assert capsys.readouterr().out.count("reusing last hand targets") == 1
