"""Two-pose calibration using scripted wire data and a monotonic fake clock."""
from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from sleeve_arm.config import GloveConfig, load_sensor_config
from sleeve_arm.robot.aurora_fake import FakeClock
from sleeve_arm.sources.bend6 import BEND6_CHANNEL_NAMES, BEND6_BETWEEN_CHANNELS, Bend6Calibration, Bend6GloveSource, load_bend6_calibration
from tools import calibrate_bend6_glove as calibrate


ROOT = Path(__file__).resolve().parents[1]
OPEN = [100. + 10 * i for i in range(9)] + [0., 0.]
FIST = [value + (-1 if i % 2 else 1) * (40 + i) for i, value in enumerate(OPEN[:9])] + [90., -99.]


def samples(values, count=20):
    return [dict(sequence=i + 1, timestamp=100. + i * .01, raw=list(values)) for i in range(count)]


def assert_calibrated_poses(calibration):
    for raw, value in ((OPEN, 0.), (FIST, 1.)):
        assert calibration.apply(raw[:9]) == tuple(
            raw[i] if name in BEND6_BETWEEN_CHANNELS else value
            for i, name in enumerate(BEND6_CHANNEL_NAMES))


class ScriptedGlove:
    def __init__(self, clock, *, fist=FIST, fail=None):
        self.clock, self.fist, self.fail = clock, fist, fail
        self.pose = "open"
        self.reader = Bend6GloveSource("SCRIPTED", clock=clock)
        self.started = self.closed = False
        self.last_at = None
        self.calls = 0

    def start(self):
        self.started = True
        self.reader._feed_bytes(b"uncertain startup;")
        if self.fail == "start":
            raise OSError("start failed")

    def latest(self):
        self.calls += 1
        if self.calls > 5 and self.fail == "read":
            raise OSError("disconnected")
        if self.fail == "silent":
            return None
        if self.calls > 5 and self.fail == "stale":
            return self.reader.latest()
        if self.last_at != self.clock.monotonic():
            values = OPEN if self.pose == "open" else self.fist
            self.reader._feed_bytes((",".join(map(str, values)) + ";").encode("ascii"))
            self.last_at = self.clock.monotonic()
        return self.reader.latest()

    def close(self):
        self.reader.close()
        self.closed = True


@pytest.fixture
def config_path(tmp_path):
    data = yaml.safe_load((ROOT / "configs/sensors_bend6.example.yaml").read_text())
    data["sensors"]["glove"].update(port="SCRIPTED", calibration_path="missing-old-calibration.yaml")
    data["sensors"]["glove"]["source_options"]["filter"]["enabled"] = True
    data["sensors"]["glove"]["startup_timeout_s"] = .05
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


def test_medians_sort_endpoints_infer_direction_and_keep_reserved_only_in_report():
    opened, fist = samples(OPEN), samples(FIST)
    # One spike in each pose must not become a calibration endpoint.
    opened[0]["raw"][0] = -10000.
    fist[0]["raw"][0] = 10000.
    config, summaries, quality, issues = calibrate.derive_calibration(opened, fist)
    assert not issues and len(config["channels"]) == 9
    assert set(config["channels"]) == set(BEND6_CHANNEL_NAMES)
    for i, name in enumerate(BEND6_CHANNEL_NAMES):
        if name in BEND6_BETWEEN_CHANNELS:
            assert config["channels"][name] == {"mode": "raw"}
            continue
        assert config["channels"][name] == dict(raw_min=min(OPEN[i], FIST[i]),
                                               raw_max=max(OPEN[i], FIST[i]), invert=FIST[i] < OPEN[i])
    calibration = Bend6Calibration(config)
    assert_calibrated_poses(calibration)
    assert summaries["fist"]["median"][9:] == [90., -99.]


def test_insufficient_span_equal_endpoints_and_noise_are_reported():
    fist = samples(FIST)
    for row in fist:
        row["raw"][0] = OPEN[0]
        row["raw"][5] = OPEN[5] + .5
    opened = samples(OPEN)
    for i, row in enumerate(opened):
        row["raw"][2] += 30 if i % 2 else -30
    _, _, _, issues = calibrate.derive_calibration(opened, fist)
    assert set(issues) == {"little_ip", "index_mcp", "ring_ip"}


@pytest.mark.parametrize("span", [0., .5, 6.])
def test_between_channels_never_fail_pose_quality_or_get_guessed_ranges(span):
    opened, fist = samples(OPEN), samples(FIST)
    for i in (1, 4):
        for j, row in enumerate(opened):
            row["raw"][i] = 204. + (-7. if j % 2 else 7.)
        for j, row in enumerate(fist):
            row["raw"][i] = 204. + span + (-7. if j % 2 else 7.)
    config, _, quality, issues = calibrate.derive_calibration(opened, fist)
    assert not issues
    for name in BEND6_BETWEEN_CHANNELS:
        assert config["channels"][name] == {"mode": "raw"}
        assert quality[name]["policy"] == "raw_only"
    assert Bend6Calibration(config).raw_only_channels == BEND6_BETWEEN_CHANNELS


def test_between_reuses_valid_old_calibration_including_points():
    previous, *_ = calibrate.derive_calibration(samples(OPEN), samples(FIST))
    old = {"raw_min": 20., "raw_max": 100., "invert": True, "points": [[20., 0.], [100., 1.]]}
    for name in BEND6_BETWEEN_CHANNELS:
        previous["channels"][name] = dict(old)
    fist = list(FIST)
    fist[1], fist[4] = OPEN[1], OPEN[4]
    config, _, quality, issues = calibrate.derive_calibration(samples(OPEN), samples(fist), previous=previous)
    assert not issues
    for name in BEND6_BETWEEN_CHANNELS:
        assert config["channels"][name] == old
        assert quality[name]["policy"] == "preserved"
    assert not Bend6Calibration(config).raw_only_channels


@pytest.mark.parametrize("data", [[], samples(OPEN, 4), samples([0.] * 10), samples([float("nan")] * 11)])
def test_invalid_pose_data_is_rejected(data):
    with pytest.raises(ValueError):
        calibrate.derive_calibration(data, samples(FIST))


@pytest.mark.parametrize("extension", ["yaml", "yml", "json"])
def test_two_prompts_generate_loadable_calibration_report_and_sensor_copy(tmp_path, config_path, extension):
    # Recalibration must retain the execution file's explicit poses and shared thumb input.
    data = yaml.safe_load(config_path.read_text())
    glove = data["sensors"]["glove"]
    glove["open_pose_rad"] = [.35, .22, .22, .22, .22, .05]
    glove["closed_pose_rad"] = [1.28, 1.78, 1.78, 1.78, 1.78, 1.68]
    glove["bend6_mapping"]["thumb_swing"] = "thumb_tip_joint"
    config_path.write_text(yaml.safe_dump(data))
    before = config_path.read_bytes()
    clock = FakeClock()
    source = ScriptedGlove(clock)
    prompts, logs = [], []
    output = tmp_path / "nested" / f"measured.{extension}"

    def source_factory(*args, **kwargs):
        assert args[0] == "SCRIPTED" and kwargs["filter"] == {"enabled": False}
        assert "calibration_path" not in kwargs and "calibration" not in kwargs
        return source

    def prompt(text):
        prompts.append(text)
        source.pose = "open" if len(prompts) == 1 else "fist"
        return ""

    assert calibrate.main(["--sensor-config", str(config_path), "--output", str(output),
                           "--duration", ".12", "--settle-seconds", "0", "--min-samples", "20"],
                          source_factory=source_factory, clock=clock, input_fn=prompt, print_fn=logs.append) == 0
    assert len(prompts) == 2 and "伸直" in prompts[0] and "握拳" in prompts[1]
    assert source.started and source.closed and config_path.read_bytes() == before
    config = load_bend6_calibration(output)
    assert_calibrated_poses(Bend6Calibration(config))
    sensors = load_sensor_config(output.with_suffix(".sensors.yaml"))
    assert sensors.glove.calibration_path == output
    assert sensors.glove.open_pose_rad == tuple(glove["open_pose_rad"])
    assert sensors.glove.closed_pose_rad == tuple(glove["closed_pose_rad"])
    assert sensors.glove.bend6_mapping == glove["bend6_mapping"]
    report = json.loads(output.with_suffix(".report.json").read_text())
    assert report["usable"] and not report["robot_motion_verified"]
    assert report["poses"]["fist"][0]["raw"] == FIST
    assert all(len(row["raw"]) == 11 for pose in report["poses"].values() for row in pose)
    assert report["poses"]["open"][-1]["sequence"] < report["poses"]["fist"][0]["sequence"]
    assert any("python tools/debug_glove.py" in line for line in logs)


def test_failed_quality_writes_only_report_not_normalized_config(tmp_path, config_path):
    clock = FakeClock()
    source = ScriptedGlove(clock, fist=OPEN)
    output = tmp_path / "rejected.yaml"
    assert calibrate.main(["--sensor-config", str(config_path), "--output", str(output),
                           "--duration", ".1", "--settle-seconds", "0", "--min-samples", "5"],
                          source_factory=lambda *a, **kw: source, clock=clock,
                          input_fn=lambda _: "", print_fn=lambda _: None) == 1
    report = json.loads(output.with_suffix(".report.json").read_text())
    assert not report["usable"] and len(report["issues"]) == 7
    assert not output.exists() and not output.with_suffix(".sensors.yaml").exists()
    assert source.closed
    with pytest.raises(ValueError, match="explicit mode"):
        load_bend6_calibration(output.with_suffix(".report.json"))


def test_full_closure_survives_generation_and_overwrite_through_glove_runtime(tmp_path, config_path):
    from sleeve_arm.runtime.glove_control import GloveRuntime, HAND_JOINTS
    from test_bend6_glove import FakeSerial, packet
    data = yaml.safe_load(config_path.read_text())
    opened = [.35, .22, .22, .22, .22, .05]
    closed = [1.28, 1.78, 1.78, 1.78, 1.78, 1.68]
    data["sensors"]["glove"].update(open_pose_rad=opened, closed_pose_rad=closed)
    data["sensors"]["glove"]["bend6_mapping"]["thumb_swing"] = "thumb_tip_joint"
    data["sensors"]["glove"]["source_options"]["filter"]["enabled"] = False
    config_path.write_text(yaml.safe_dump(data))
    output = tmp_path / "measured.yaml"
    for overwrite in (False, True):
        clock = FakeClock()
        source = ScriptedGlove(clock)
        prompts, logs = [], []

        def prompt(text):
            prompts.append(text)
            source.pose = "open" if len(prompts) == 1 else "fist"

        argv = ["--sensor", str(config_path), "--output", str(output),
                "--duration", ".12", "--settle-seconds", "0"]
        assert calibrate.main(argv + (["--overwrite"] if overwrite else []),
                              source_factory=lambda *a, **kw: source, clock=clock,
                              input_fn=prompt, print_fn=logs.append) == 0
        assert any(f"open_pose_rad={opened}" in line and f"closed_pose_rad={closed}" in line for line in logs)
        serial = FakeSerial()
        runtime = GloveRuntime(load_sensor_config(output.with_suffix(".sensors.yaml")).glove,
                               clock=clock, serial_factory=lambda **_: serial)
        try:
            runtime.start()
            runtime.source._feed_bytes(b"sync;")
            for raw, target in ((OPEN, opened), (FIST, closed), (OPEN, opened)):
                runtime.source._feed_bytes(packet(raw))
                runtime.wait_ready()
                assert runtime.targets() == pytest.approx(dict(zip(HAND_JOINTS, target)))
        finally:
            runtime.close()
        assert serial.closed.is_set()


@pytest.mark.parametrize("failure", ["silent", "stale", "read", "start"])
def test_no_files_on_acquisition_failure_and_resources_always_close(tmp_path, config_path, failure):
    clock = FakeClock()
    source = ScriptedGlove(clock, fail=failure)
    output = tmp_path / "bad.yaml"
    assert calibrate.main(["--sensor-config", str(config_path), "--output", str(output),
                           "--duration", ".5", "--settle-seconds", "0", "--min-samples", "5"],
                          source_factory=lambda *a, **kw: source, clock=clock,
                          input_fn=lambda _: "", print_fn=lambda _: None) == 1
    assert source.closed and not output.exists() and not output.with_suffix(".sensors.yaml").exists()


@pytest.mark.parametrize("error", [KeyboardInterrupt, EOFError])
def test_operator_cancel_closes_source_without_output(tmp_path, config_path, error):
    clock = FakeClock()
    source = ScriptedGlove(clock)
    output = tmp_path / "cancelled.yaml"

    def cancel(_):
        raise error

    assert calibrate.main(["--sensor-config", str(config_path), "--output", str(output)],
                          source_factory=lambda *a, **kw: source, clock=clock,
                          input_fn=cancel, print_fn=lambda _: None) == 130
    assert source.closed and not output.exists()


@pytest.mark.parametrize("suffix", [".yaml", ".report.json", ".sensors.yaml"])
def test_existing_output_is_preserved_before_opening_serial(tmp_path, config_path, suffix):
    output = tmp_path / "measured.yaml"
    existing = output.with_suffix(suffix)
    existing.write_text("KEEP USER DATA")
    assert calibrate.main(["--sensor-config", str(config_path), "--output", str(output)],
                          source_factory=lambda *a, **kw: pytest.fail("opened serial"),
                          print_fn=lambda _: None) == 1
    assert existing.read_text() == "KEEP USER DATA"


@pytest.mark.parametrize("same_input", [False, True])
def test_overwrite_retains_execution_settings_and_replaces_calibration(tmp_path, config_path, same_input):
    output = tmp_path / "measured.yaml"
    sensor_path = output.with_suffix(".sensors.yaml")
    output.write_text("glove: {bend6: {calibration: {mode: raw}}}\n")
    output.with_suffix(".report.json").write_text('{"old": true}')
    settings = yaml.safe_load(config_path.read_text())
    glove = settings["sensors"]["glove"]
    glove.update(open_pose_rad=[.35, .22, .22, .22, .22, .05],
                 closed_pose_rad=[1.28, 1.78, 1.78, 1.78, 1.78, 1.68], stale_policy="hold_last")
    glove["bend6_mapping"]["thumb_swing"] = "thumb_tip_joint"
    sensor_path.write_text(yaml.safe_dump(settings))
    raw_input_before = config_path.read_bytes()
    clock = FakeClock()
    source = ScriptedGlove(clock)
    prompts = []

    def prompt(text):
        prompts.append(text)
        source.pose = "open" if len(prompts) == 1 else "fist"

    assert calibrate.main([
        "--sensor", str(sensor_path if same_input else config_path), "--output", str(output),
        "--overwrite", "--duration", ".12", "--settle-seconds", "0",
    ], source_factory=lambda *a, **kw: source, clock=clock, input_fn=prompt,
       print_fn=lambda _: None) == 0
    glove["calibration_path"] = "measured.yaml"
    assert yaml.safe_load(sensor_path.read_text()) == settings
    assert config_path.read_bytes() == raw_input_before
    calibration = Bend6Calibration(load_bend6_calibration(output))
    assert_calibrated_poses(calibration)
    assert json.loads(output.with_suffix(".report.json").read_text())["usable"]
    assert source.closed and len(prompts) == 2
    assert not list(tmp_path.glob(".measured.*"))


@pytest.mark.parametrize("failure", ["quality", "read", "cancel", "write"])
def test_unsuccessful_overwrite_preserves_all_previous_outputs(tmp_path, config_path, monkeypatch, failure):
    output = tmp_path / "measured.yaml"
    sensor_path = output.with_suffix(".sensors.yaml")
    output.write_text("ORIGINAL CALIBRATION")
    output.with_suffix(".report.json").write_text("ORIGINAL REPORT")
    sensor_path.write_bytes(config_path.read_bytes())
    previous = {path: path.read_bytes() for path in (output, output.with_suffix(".report.json"), sensor_path)}
    clock = FakeClock()
    source = ScriptedGlove(clock, fist=OPEN if failure == "quality" else FIST,
                           fail="read" if failure == "read" else None)
    prompts = []

    def prompt(text):
        if failure == "cancel":
            raise KeyboardInterrupt
        prompts.append(text)
        source.pose = "open" if len(prompts) == 1 else "fist"

    replace_file = calibrate.os.replace

    def replace_or_fail(src, dest):
        if dest == output:
            raise OSError("simulated replacement failure")
        replace_file(src, dest)

    if failure == "write":
        monkeypatch.setattr(calibrate.os, "replace", replace_or_fail)
    result = calibrate.main([
        "--sensor", str(config_path), "--output", str(output), "--overwrite",
        "--duration", ".12", "--settle-seconds", "0",
    ], source_factory=lambda *a, **kw: source, clock=clock, input_fn=prompt, print_fn=lambda _: None)
    assert result == (130 if failure == "cancel" else 1)
    assert all(path.read_bytes() == contents for path, contents in previous.items())
    reports = list(tmp_path.glob("measured.failed-*.report.json"))
    assert len(reports) == (1 if failure == "quality" else 0)
    if reports:
        assert not json.loads(reports[0].read_text())["usable"]
    assert source.closed and not list(tmp_path.glob(".measured.*"))


def test_duplicate_cached_frames_never_satisfy_sample_count():
    clock = FakeClock()
    source = ScriptedGlove(clock)
    source.start()
    source.latest()
    cached = source.reader.latest()
    source.latest = lambda: cached
    with pytest.raises(RuntimeError, match="等待新帧超时"):
        calibrate.collect_pose(source, GloveConfig(startup_timeout_s=.02), .1, 5, clock=clock)
    source.close()


def test_too_few_new_frames_are_rejected():
    clock = FakeClock()
    source = ScriptedGlove(clock)
    source.start()
    with pytest.raises(RuntimeError, match="新帧不足"):
        calibrate.collect_pose(source, GloveConfig(), .01, 20, clock=clock)
    source.close()


@pytest.mark.parametrize("changes", [
    {"synthetic": True}, {"calibration_mode": "normalized"}, {"timestamp": float("nan")},
    {"raw": (float("inf"),) * 11},
])
def test_non_raw_or_invalid_readings_cannot_be_calibrated(changes):
    clock = FakeClock()
    source = ScriptedGlove(clock)
    source.start()
    original = source.latest
    source.latest = lambda: replace(original(), **changes)
    with pytest.raises((ValueError, RuntimeError)):
        calibrate.collect_pose(source, GloveConfig(), .1, 5, clock=clock)
    source.close()


def test_cli_imports_without_robot_sdk_or_serial_hardware():
    bootstrap = """
import importlib.abc, runpy, sys
class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if (fullname.split('.')[0] in ('serial', 'fourier_aurora_client', 'electronic_skin')
                or fullname == 'sleeve_arm.robot' or fullname.startswith('sleeve_arm.robot.')):
            raise AssertionError('unexpected robot/hardware dependency: ' + fullname)
sys.meta_path.insert(0, NoHardware())
sys.argv = [sys.argv[1], '--help']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
    result = subprocess.run([sys.executable, "-I", "-c", bootstrap,
                             str(ROOT / "tools/calibrate_bend6_glove.py")],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stderr
    assert "--min-span" in result.stdout


@pytest.mark.parametrize("argv", [
    ["--duration", "nan"], ["--duration", "0"], ["--settle-seconds", "-1"],
    ["--settle-seconds", "inf"], ["--min-samples", "1"], ["--min-span", "0"], ["--output", "test.txt"],
])
def test_cli_invalid_parameters_rejected(argv):
    with pytest.raises(SystemExit):
        calibrate.parse_args(argv)
