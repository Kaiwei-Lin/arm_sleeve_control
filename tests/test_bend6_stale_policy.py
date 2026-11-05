"""Retain last Bend6 input on silence without masking acquisition or robot faults."""
from dataclasses import replace

import pytest
import yaml

from sleeve_arm.config import GloveConfig, load_sensor_config, validate_glove_config
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import fake_session, gr3_fake_profile
from sleeve_arm.runtime.robot_runtime import RobotRuntime, PrintOnlyRuntime
from sleeve_arm.sources.bend6 import Bend6Filter
from test_bend6_glove import EXAMPLE, packet, runtime


def held_runtime(tmp_path, **kwargs):
    glove = runtime(tmp_path, **kwargs)
    glove.config = replace(glove.config, stale_policy="hold_last")
    return glove


def test_policy_loads_from_yaml_and_defaults_remain_strict(tmp_path):
    assert GloveConfig().stale_policy == "error"
    assert load_sensor_config().glove.backend == "bend5"
    assert load_sensor_config().glove.stale_policy == "error"
    data = yaml.safe_load(EXAMPLE.read_text())
    data["sensors"]["glove"]["stale_policy"] = "hold_last"
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(data))
    assert load_sensor_config(path).glove.stale_policy == "hold_last"
    with pytest.raises(ValueError, match="requires backend=bend6"):
        validate_glove_config(GloveConfig(stale_policy="hold_last"))


@pytest.mark.parametrize("value", [None, True, 1, {}, [], "hold", "ignore"])
def test_invalid_policies_rejected_before_serial(value, monkeypatch):
    import serial
    monkeypatch.setattr(serial, "Serial", lambda **_: pytest.fail("opened hardware"))
    with pytest.raises(ValueError, match="stale_policy"):
        validate_glove_config(GloveConfig(backend="bend6", stale_policy=value))


@pytest.mark.parametrize("filtered", [False, True])
def test_silence_holds_exact_reading_and_targets_without_refreshing_or_refiltering(tmp_path, filtered):
    glove = held_runtime(tmp_path)
    try:
        glove.source.filter = Bend6Filter(dict(enabled=filtered, alpha=.5))
        glove.source._feed_bytes(packet([.2] * 9 + [90., -90.]))
        glove.wait_ready()
        reading, targets = glove.reading, glove.targets()
        for seconds in (.252, 10., 3600.):
            glove.clock.sleep(seconds)
            for _ in range(3):
                assert glove.latest() is reading
                assert glove.targets() == targets
                assert glove.preview_targets() == targets
                assert glove.source.diagnostics()["valid_age_s"] == pytest.approx(
                    glove.clock.monotonic() - reading.timestamp)
        glove.source._feed_bytes(b"bad;\xff;1,2;")
        assert glove.reading is reading and glove.targets() == targets
        logs = []
        glove.report(logs.append)
        assert "input_state=holding_last" in logs[0] and "stale_policy=hold_last" in logs[0]
        assert "age=3610.252s" in logs[0]
        # Malformed packets reset filter state; a new complete record resumes processing.
        glove.source._feed_bytes(packet([.8] * 9 + [91., -91.]))
        assert glove.reading.sequence == reading.sequence + 1
        assert glove.reading.timestamp > reading.timestamp
        assert glove.targets() != targets
        logs.clear()
        glove.report(logs.append)
        assert "input_state=fresh" in logs[0]
    finally:
        glove.close()


def test_raw_preview_can_hold_but_raw_control_still_rejected(tmp_path):
    glove = held_runtime(tmp_path, calibrated=False)
    try:
        glove.source._feed_bytes(packet())
        glove.wait_ready(require_control=False)
        glove.clock.sleep(10.)
        assert glove.preview_targets() is None
        with pytest.raises(ValueError, match="raw data is read-only"):
            glove.targets()
    finally:
        glove.close()


@pytest.mark.parametrize("timestamp", [float("nan"), float("inf"), 200.])
def test_hold_does_not_accept_nonfinite_or_future_timestamps(tmp_path, timestamp):
    glove = held_runtime(tmp_path)
    try:
        glove.source._feed_bytes(packet([.2] * 11))
        glove.source._reading = replace(glove.reading, timestamp=timestamp)
        with pytest.raises(RuntimeError, match="watchdog"):
            glove.targets()
        with pytest.raises(RuntimeError, match="watchdog"):
            glove._targets_from_reading(glove.reading)
    finally:
        glove.close()


@pytest.mark.parametrize("old_frame", [False, True])
def test_startup_still_requires_a_fresh_frame(tmp_path, old_frame):
    glove = held_runtime(tmp_path)
    glove.config = replace(glove.config, startup_timeout_s=.03)
    try:
        if old_frame:
            glove.source._feed_bytes(packet([.2] * 11))
            glove.clock.sleep(.3)
        with pytest.raises(RuntimeError, match="readiness timeout"):
            glove.wait_ready()
    finally:
        glove.close()


def test_startup_waits_for_new_frame_after_an_initial_old_one(tmp_path, monkeypatch):
    glove = held_runtime(tmp_path)
    try:
        glove.source._feed_bytes(packet([.2] * 11))
        old = glove.reading
        glove.clock.sleep(.3)
        sleep = glove.clock.sleep

        def tick(seconds):
            sleep(seconds)
            glove.source._feed_bytes(packet([.3] * 11))

        monkeypatch.setattr(glove.clock, "sleep", tick)
        glove.wait_ready()
        assert glove.reading.sequence > old.sequence and glove.reading.timestamp > old.timestamp
    finally:
        glove.close()


def test_debug_fake_robot_keeps_last_target_through_gap_and_resumes(tmp_path, monkeypatch, capsys):
    from tools import debug_glove
    session = fake_session(profile=gr3_fake_profile())
    clock = session.clock
    glove = held_runtime(tmp_path, clock=clock)
    source, serial, thread = glove.source, glove.source._serial, glove.source._thread
    monkeypatch.setattr(debug_glove, "time", clock)
    monkeypatch.setattr(debug_glove, "GloveRuntime", lambda *a, **kw: glove)
    monkeypatch.setattr(glove, "start", lambda: source._feed_bytes(packet([.2] * 11)))
    original_factory = factory.create_robot

    def fake_robot(backend, config, **kwargs):
        assert backend == "aurora-fake"
        return original_factory(backend, config, session=session, **kwargs)

    monkeypatch.setattr(factory, "create_robot", fake_robot)
    submitted = []

    def robot_runtime(*args, **kwargs):
        robot = RobotRuntime(*args, **kwargs, clock=clock)
        apply = robot.apply_hand

        def record(targets, dt, **options):
            submitted.append((clock.monotonic(), dict(targets), glove.reading.timestamp))
            return apply(targets, dt, **options)

        robot.apply_hand = record
        return robot

    monkeypatch.setattr(debug_glove, "RobotRuntime", robot_runtime)
    sleep = clock.sleep
    resumed = False

    def tick(seconds):
        nonlocal resumed
        sleep(seconds)
        if clock.monotonic() >= 100.8 and not resumed:
            source._feed_bytes(packet([.8] * 11))
            resumed = True

    monkeypatch.setattr(clock, "sleep", tick)
    assert debug_glove.main(["--robot", "aurora-fake", "--duration", "1.2"]) == 0
    held = [row for row in submitted if .25 < row[0] - row[2]]
    assert held and all(row[1] == submitted[0][1] for row in held if row[2] == submitted[0][2])
    assert submitted[-1][1] != submitted[0][1] and submitted[-1][2] > submitted[0][2]
    commands = session.fake_client.commands
    assert all(set(command) == {"right_hand"} for command in commands)
    for before, after in zip(commands, commands[1:]):
        assert max(abs(a - b) for a, b in zip(before["right_hand"], after["right_hand"])) <= .03 + 1e-9
    output = capsys.readouterr()
    assert "input_state=holding_last" in output.out and not output.err
    assert session.fake_client.close_count == 1 and serial.closed.is_set() and not thread.is_alive()


@pytest.mark.parametrize("preview", [False, True])
def test_combined_control_keeps_running_on_held_glove_and_recovers(tmp_path, monkeypatch, preview):
    from test_model_control_app import make_app
    app, events = make_app(duration=1.2)
    if preview:
        app.robot = PrintOnlyRuntime()
        app.glove_control_required = False
    app.glove = glove = held_runtime(tmp_path, clock=app.clock, calibrated=not preview)
    source = glove.source
    monkeypatch.setattr(glove, "start", lambda: source._feed_bytes(packet([.2] * 11)))
    latest = source.latest
    resumed = False

    def read():
        nonlocal resumed
        if app.clock.monotonic() >= 100.8 and not resumed:
            source._feed_bytes(packet([.8] * 11))
            resumed = True
        return latest()

    monkeypatch.setattr(source, "latest", read)
    submitted = []
    app.robot.apply = lambda intent, dt, **kw: submitted.append(
        (app.clock.monotonic(), kw["hand_targets"], glove.reading.timestamp)) or {}
    assert app.run() == 0 and app.fault is None and resumed
    assert any(now - timestamp > .25 for now, _, timestamp in submitted)
    assert submitted[-1][2] > submitted[0][2]
    if preview:
        assert all(targets is None for _, targets, _ in submitted)
    else:
        assert submitted[-1][1] != submitted[0][1]
        assert all(targets == submitted[0][1] for _, targets, timestamp in submitted if timestamp < 100.8)
    assert glove.source is None and "sensor_close" in events


@pytest.mark.parametrize("fault", ["reader", "mapping", "nonfinite", "status", "robot", "imu"])
def test_hold_does_not_mask_other_faults_or_skip_cleanup(tmp_path, monkeypatch, fault):
    from test_model_control_app import make_app
    app, events = make_app(duration=2.)
    app.robot.fail_fast = True
    app.glove = glove = held_runtime(tmp_path, clock=app.clock)
    source, serial, thread = glove.source, glove.source._serial, glove.source._thread
    monkeypatch.setattr(glove, "start", lambda: source._feed_bytes(packet([.2] * 11)))
    health = app.robot.check_health
    sensors = app.sensors.latest
    cached_sample = None

    def sensor_read():
        nonlocal cached_sample
        if cached_sample is None or app.clock.monotonic() < 100.5 or fault != "imu":
            cached_sample = sensors()
        return cached_sample

    monkeypatch.setattr(app.sensors, "latest", sensor_read)

    def check_health():
        health()
        if app.clock.monotonic() >= 100.5:
            if fault == "reader":
                source._error = OSError("serial disconnected")
            elif fault == "mapping":
                glove.config.bend6_mapping["thumb_bend"] = "reserved_10"
            elif fault == "nonfinite":
                glove.config = replace(glove.config, closed_pose_rad=(float("nan"),) * 6)
            elif fault == "status":
                source._reading = replace(source._reading, status="invalid")
            elif fault == "robot":
                raise RuntimeError("robot feedback stale")

    monkeypatch.setattr(app.robot, "check_health", check_health)
    submitted = []
    app.robot.apply = lambda intent, dt, **kw: submitted.append(app.clock.monotonic()) or {}
    assert app.run() == 1 and app.fault
    assert any(now > 100.25 for now in submitted)  # A glove gap alone did not stop control.
    if fault != "imu":
        assert all(now < 100.5 for now in submitted)
    else:
        assert "sensor watchdog" in app.fault or "hard timeout" in app.fault
    assert glove.source is None and events[-2:] == ["robot_shutdown", "sensor_close"]
    assert serial.closed.is_set() and not thread.is_alive()
