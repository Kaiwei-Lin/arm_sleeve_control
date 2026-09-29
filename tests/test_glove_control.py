"""Bend5 wire data through the real calibration code into a fake Aurora session."""
from dataclasses import replace
import json
from pathlib import Path
import shutil
import subprocess
import sys
import threading

import numpy as np
import pytest
import yaml

from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH, load_sensor_config
from sleeve_arm.control.safety import FeedbackError, validate_feedback
from sleeve_arm.domain import MotionIntent
from sleeve_arm.domain.joint import JointState
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import FakeClock, fake_session, gr3_fake_profile
from sleeve_arm.runtime.configuration import load_configs, parse_args
from sleeve_arm.runtime.glove_control import GloveRuntime, HAND_JOINTS
from sleeve_arm.runtime.robot_runtime import RobotRuntime
from sleeve_arm.sources.glove import Bend5GloveSource


def make_glove(clock=None):
    glove = GloveRuntime(load_sensor_config().glove, mode="fake", clock=clock or FakeClock())
    glove.start()  # fake mode never opens serial
    glove.mode = "real"  # feed actual ASCII records without generating synthetic packets
    return glove


def test_bend5_wire_order_maps_each_finger_to_aurora_and_repeated_reads_expire():
    glove = make_glove()
    try:
        # [pinky, ring, middle, index, thumb] -> [thumb, index, middle, ring, pinky, thumb].
        for packet_index, joint_indices in ((0, (4,)), (1, (3,)), (2, (2,)), (3, (1,)), (4, (0, 5))):
            packet = [0.] * 11
            packet[packet_index] = 1.
            text = ",".join(map(str, packet))
            glove.source._parse_record(text)
            targets = glove.targets()
            expected = list(glove.config.open_pose_rad)
            for index in joint_indices:
                expected[index] = glove.config.closed_pose_rad[index]
            assert list(targets) == list(HAND_JOINTS)
            assert list(targets.values()) == pytest.approx(expected)
        assert len(glove.reading.raw) == 11
        timestamp = glove.reading.timestamp
        glove.clock.sleep(glove.config.stale_timeout_s + .001)
        assert glove.latest().timestamp == timestamp
        with pytest.raises(RuntimeError, match="watchdog"):
            glove.targets()
        with pytest.raises(ValueError, match="non-finite"):
            glove.source._store_packet(np.asarray([float("inf")] * 11))
        assert glove.latest().timestamp == timestamp
    finally:
        glove.close()


def test_calibration_json_and_semicolon_tokens_are_reused(tmp_path):
    glove = make_glove()
    calibration = tmp_path / "bend5.json"
    calibration.write_text(json.dumps({
        "calibration_mode": "per_finger", "finger_order": ["pinky", "ring", "middle", "index", "thumb"],
        "open_raw": [100.] * 5, "max_delta": [100.] * 5,
        "direction": [1.] * 5, "deadzone": [0.] * 5,
    }))
    try:
        glove.source = Bend5GloveSource(port="FAKE", calibration_path=calibration,
                                        filter_alpha=1., clock=glove.clock)
        for value in [100., 100., 100., 200., 100., 6., 7., 8., 9., 10.]:
            glove.source._parse_record(str(value))
        assert glove.latest() is None  # ten records do not form an eleven-value packet
        glove.source._parse_record("11")
        assert glove.source.calibration_mode_active == "per_finger"
        assert glove.reading.curls == {"thumb": 0., "index": 1., "middle": 0., "ring": 0., "pinky": 0.}
        assert glove.targets()["index_flexion"] == 1.4
    finally:
        glove.close()


def test_startup_zeroing_does_not_expose_uncalibrated_targets():
    glove = make_glove()
    try:
        glove.source = Bend5GloveSource(port="FAKE", zero_frames=2, filter_alpha=1.,
                                        per_finger_max_delta=100., deadzone_value=0., clock=glove.clock)
        glove.source._parse_record(",".join(["100"] * 11))
        with pytest.raises(RuntimeError, match="not ready"):
            glove.targets()
        glove.source._parse_record(",".join(["100"] * 11))
        assert list(glove.targets().values()) == pytest.approx(glove.config.open_pose_rad)
        glove.source._parse_record("100,100,100,200,100,0,0,0,0,0,0")
        assert glove.targets()["index_flexion"] == pytest.approx(1.4)
    finally:
        glove.close()


@pytest.mark.parametrize("parts,groups", [(("arm", "hand"), {"right_manipulator", "right_hand"}),
                                          (("hand",), {"right_hand"})])
@pytest.mark.parametrize("hand_velocity", [0., 2., -2.])
def test_shared_session_batch_limits_and_single_hand_isolation(monkeypatch, parts, groups, hand_velocity):
    args = parse_args(["--robot", "aurora-fake", "--side", "right", "--source-side", "right",
                       "--glove", "fake", "--imus", "fake"])
    configs = load_configs(args)
    session = fake_session(profile=gr3_fake_profile())
    session.fake_client.groups["right_hand"]["velocity"] = [hand_velocity] * 6
    actual_factory = factory.create_robot
    monkeypatch.setattr(factory, "create_robot", lambda backend, config, **kw:
                        actual_factory(backend, config, session=session, **kw))
    runtime = RobotRuntime(args, configs, clock=session.clock, parts=parts)
    glove = make_glove(session.clock)
    try:
        runtime.connect()
        assert not session.fake_client.commands  # connect is read-only
        assert runtime.prepare()
        assert session.fake_client.calls.count("get_instance") == 1
        last = session.fake_client.commands[-1]["right_hand"]
        for _ in range(5):
            session.clock.sleep(.01)
            glove.source._parse_record("1,1,1,1,1,0,0,0,0,0,0")
            if "arm" in parts:
                runtime.apply(MotionIntent(session.clock.monotonic(), shoulder_flexion_rad=.3), .01,
                              hand_targets=glove.targets())
            else:
                runtime.apply_hand(glove.targets(), .01)
            command = session.fake_client.commands[-1]
            assert set(command) == groups
            assert max(abs(a - b) for a, b in zip(last, command["right_hand"])) <= .008 + 1e-10
            last = command["right_hand"]
        count = len(session.fake_client.commands)
        session.clock.sleep(.3)
        with pytest.raises(RuntimeError, match="watchdog"):
            runtime.apply_hand(glove.targets(), .01)
        assert len(session.fake_client.commands) == count
    finally:
        runtime.shutdown()
        glove.close()
    assert session.fake_client.close_count == 1


def test_hand_velocity_policy_preserves_feedback_checks_and_can_be_reenabled():
    profile = gr3_fake_profile()
    for side in ("left", "right"):
        assert all(j.smooth_limits
                   for j in profile.control_config((side,)).joints.values())
        limits = profile.control_config((side,), ("hand",)).joints
        assert all(j.smooth_limits for j in limits.values())
        joint = limits["thumb_bend"]
        state = JointState(name=joint.name, position=.5, velocity=2.,
                           current=None, torque=None, state=None, bus=None, error=None)
        validate_feedback(state, joint)
        with pytest.raises(FeedbackError, match="velocity exceeds"):
            validate_feedback(state, replace(joint, smooth_limits=False))
        for velocity in (None, float("nan"), float("inf")):
            with pytest.raises(FeedbackError, match="velocity"):
                validate_feedback(replace(state, velocity=velocity), joint)
        validate_feedback(replace(state, position=joint.max_position + .1), joint)
        validate_feedback(state, joint, expected_position=1.)
    for value in (None, 0, "false"):
        with pytest.raises(ValueError, match="smooth_limits must be boolean"):
            replace(profile, smooth_limits=value).validate()


def test_app_stops_both_outputs_and_closes_glove_on_dropout(monkeypatch):
    from test_model_control_app import make_app

    app, events = make_app(duration=1.)
    glove = make_glove(app.clock)
    app.glove = glove
    monkeypatch.setattr(glove, "start", lambda: glove.source._parse_record("0,0,0,0,0,0,0,0,0,0,0"))
    applied = []
    app.robot.apply = lambda intent, dt, **kwargs: applied.append((app.clock.monotonic(), kwargs)) or {}
    assert app.run() == 1
    assert "glove watchdog" in app.fault
    assert applied and all(when <= 100.25 + 1e-9 for when, _ in applied)
    assert all(set(kwargs["hand_targets"]) == set(HAND_JOINTS) for _, kwargs in applied)
    assert glove.source is None and events[-2:] == ["robot_shutdown", "sensor_close"]


def test_glove_config_paths_legacy_defaults_and_cli_guards(tmp_path):
    raw = yaml.safe_load(DEFAULT_SENSOR_CONFIG_PATH.read_text(encoding="utf-8"))
    raw["sensors"]["glove"].update(enabled=True, calibration_path="cal.json")
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    config = load_sensor_config(path).glove
    assert not hasattr(config, "project_path")
    assert config.calibration_path == tmp_path / "cal.json"
    with pytest.raises(ValueError, match="requires --robot"):
        load_configs(parse_args(["--sensor-config", str(path)]))
    assert load_configs(parse_args(["--sensor-config", str(path), "--glove", "none"])).glove_mode == "none"
    del raw["sensors"]["glove"]
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    assert not load_sensor_config(path).glove.enabled
    with pytest.raises(SystemExit):
        parse_args(["--robot", "aurora", "--side", "right", "--source-side", "right",
                    "--aurora-profile", "unused.yaml", "--execute", "--duration", "1", "--glove", "fake"])


def test_debug_preview_never_constructs_robot_and_execute_rejects_fake(monkeypatch, capsys):
    from tools import debug_glove

    monkeypatch.setattr(factory, "create_robot", lambda *a, **kw: pytest.fail("preview connected a robot"))
    assert debug_glove.main(["--glove", "fake", "--duration", ".02"]) == 0
    assert "target_rad=" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        debug_glove.parse_args(["--execute", "--glove", "fake", "--duration", "1"])


def test_local_matrix_calibration_reorders_channels_and_removes_crosstalk(tmp_path):
    matrix = np.eye(5) * 100.
    matrix[1, 0] = 20.  # pinky bending also changes the ring sensor
    path = tmp_path / "matrix.json"
    fingers = ("pinky", "ring", "middle", "index", "thumb")
    path.write_text(json.dumps({
        "finger_order": list(reversed(fingers)), "calibration_mode": "matrix",
        "open_raw": [50.] * 5, "max_delta": [100.] * 5, "deadzone": [0.] * 5,
        "single_action_delta_matrix": {name: matrix[:, i][::-1].tolist() for i, name in enumerate(fingers)},
    }), encoding="utf-8")
    source = Bend5GloveSource("FAKE", calibration_path=path, filter_alpha=1.)
    source._store_packet([150, 70, 50, 50, 50, 0, 0, 0, 0, 0, 0])
    assert source.calibration_mode_active == "matrix"
    assert source.latest().curls == pytest.approx(dict(zip(fingers, [1., 0., 0., 0., 0.])))
    source.filter_alpha = .5
    source._store_packet([50, 50, 50, 50, 50, 0, 0, 0, 0, 0, 0])
    assert source.latest().curls["pinky"] == pytest.approx(.5)


@pytest.mark.parametrize("delimiter", [b",", b";"])
def test_local_serial_reader_handles_split_packets_and_closes(delimiter):
    chunks = [b"0", b"1" + delimiter + delimiter.join([b"0"] * 10) + b";\r\n"]
    release = threading.Event()
    received = threading.Event()

    class Serial:
        in_waiting = 8192
        closed = False

        def read(self, size):
            if chunks:
                return chunks.pop(0)
            received.set()  # _feed_bytes has processed the previous chunk
            release.wait(1.)
            return b""

        def close(self):
            self.closed = True
            release.set()

    port = Serial()
    clock = FakeClock()
    source = Bend5GloveSource("FAKE", zero_on_start=False, filter_alpha=1., clock=clock,
                              serial_factory=lambda **kwargs: port)
    try:
        source.start()
        assert received.wait(2.)
        reading = source.latest()
        assert reading.timestamp == clock.monotonic()
        assert reading.raw == (1.,) + (0.,) * 10
        assert reading.curls["pinky"] == 1.
    finally:
        thread = source._thread
        source.close()
        source.close()
    assert port.closed and not thread.is_alive()


@pytest.mark.parametrize("record", ["0,0,nan,0,0,0,0,0,0,0,0", "0,0,inf,0,0,0,0,0,0,0,0", "garbage"])
def test_invalid_record_does_not_replace_last_good_reading(record):
    source = Bend5GloveSource("FAKE", zero_on_start=False)
    source._feed_bytes(b"0,0,0,0,0,0,0,0,0,0,0;")
    previous = source.latest()
    with pytest.raises(ValueError):
        source._parse_record(record)
    assert source.latest() is previous


def test_glove_and_combined_entry_run_from_an_isolated_project_copy(tmp_path):
    root = Path(__file__).resolve().parents[1]
    isolated = tmp_path / "standalone"
    for directory in ("sleeve_arm", "tools", "configs"):
        shutil.copytree(root / directory, isolated / directory,
                        ignore=shutil.ignore_patterns("__pycache__", "*.before-*"))
    # -I removes PYTHONPATH/user-site/current-directory imports. Reject any
    # attempt to import the other application or a real Aurora SDK explicitly.
    bootstrap = """
import importlib.abc, runpy, sys
class NoExternalApplication(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('electronic_skin', 'fourier_aurora_client'):
            raise AssertionError('unexpected dependency: ' + fullname)
sys.meta_path.insert(0, NoExternalApplication())
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
"""
    for script, options in (
        ("debug_glove.py", ["--glove", "fake", "--robot", "aurora-fake"]),
        ("run_model_control.py", ["--robot", "aurora-fake", "--side", "right", "--source-side", "right",
                                  "--imus", "fake", "--glove", "fake", "--calibration-seconds", ".1"]),
    ):
        result = subprocess.run(
            [sys.executable, "-I", "-X", "utf8", "-c", bootstrap,
             str(isolated / "tools" / script), *options, "--duration", ".05"],
            cwd=isolated, input="\n\n\n\n", capture_output=True, text=True, encoding="utf-8", timeout=20,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert "Bend5" in result.stdout
