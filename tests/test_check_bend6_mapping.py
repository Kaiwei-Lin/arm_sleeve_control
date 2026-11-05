"""Mapping inspection is read-only, uses fresh wire data, and never rewrites calibration."""
import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from sleeve_arm.sources.bend6 import BEND6_CHANNEL_NAMES, Bend6Calibration
from sleeve_arm.robot.aurora_fake import FakeClock
from tools import check_bend6_mapping as check
from test_calibrate_bend6_glove import OPEN, ScriptedGlove, samples


ROOT = Path(__file__).resolve().parents[1]
REFERENCE_MAPPING = yaml.safe_load((ROOT / "configs/sensors_bend6.example.yaml").read_text())["sensors"]["glove"]["bend6_mapping"]


@pytest.mark.parametrize("channel_index", range(11))
def test_each_wire_field_keeps_index_and_named_route_including_reserved(channel_index):
    action = list(OPEN)
    action[channel_index] -= 30.
    mapping = dict(REFERENCE_MAPPING, thumb_bend="index_mcp")
    rows = check.compare_poses(samples(OPEN), samples(action), mapping, Bend6Calibration())
    assert len(rows) == 11
    assert [r["index"] for r in rows] == list(range(11))
    assert [r["delta"] for r in rows] == [-30. if i == channel_index else 0. for i in range(11)]
    assert all(r["normalized_delta"] is None for r in rows)
    assert rows[5]["mapped_joints"] == ["thumb_bend"]
    assert rows[8]["mapped_joints"] == []
    assert rows[9]["mapped_joints"] == rows[10]["mapped_joints"] == []
    assert rows[channel_index]["exceeds_noise"]


def test_medians_resist_spikes_and_normalized_deltas_respect_direction():
    calibration = Bend6Calibration(dict(mode="normalized", channels={
        name: dict(raw_min=OPEN[i] - 50, raw_max=OPEN[i], invert=True)
        for i, name in enumerate(BEND6_CHANNEL_NAMES)
    }))
    action = list(OPEN)
    action[8] -= 25
    before, after = samples(OPEN), samples(action)
    before[0]["raw"][8] = -5000
    after[0]["raw"][8] = 5000
    rows = check.compare_poses(before, after, REFERENCE_MAPPING, calibration)
    assert rows[8]["delta"] == -25
    assert rows[8]["normalized_delta"] == .5
    assert rows[8]["mapped_joints"] == ["thumb_bend"]
    assert rows[7]["normalized_delta"] == 0


@pytest.mark.parametrize("bad", [[], samples(OPEN, 4), samples([0.] * 10), samples([float("nan")] * 11)])
def test_bad_observations_are_rejected(bad):
    with pytest.raises(ValueError):
        check.compare_poses(bad, samples(OPEN), REFERENCE_MAPPING, Bend6Calibration())


@pytest.fixture
def config_path(tmp_path):
    data = yaml.safe_load((ROOT / "configs/sensors_bend6.example.yaml").read_text())
    data["sensors"]["glove"].update(port="SCRIPTED", startup_timeout_s=.05)
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize("mode", ["raw", "normalized"])
def test_two_pose_cli_reports_raw_data_without_sdk_or_configuration_writes(config_path, tmp_path, monkeypatch, mode):
    # No real serial or robot is reachable even if the implementation regresses.
    import serial
    from sleeve_arm.robot import factory
    monkeypatch.setattr(serial, "Serial", lambda **kw: pytest.fail("real serial opened"))
    monkeypatch.setattr(factory, "create_robot", lambda *a, **kw: pytest.fail("robot created"))
    clock = FakeClock()
    action = list(OPEN)
    action[6] += 300  # A physical thumb action is observed on the field labelled index_ip.
    source = ScriptedGlove(clock, fist=action)
    calibration_path = tmp_path / "calibration.yaml"
    calibration = dict(mode=mode)
    if mode == "normalized":
        calibration["channels"] = {
            name: dict(raw_min=0., raw_max=1000.) for name in BEND6_CHANNEL_NAMES
        }
    calibration_path.write_text(yaml.safe_dump({"glove": {"bend6": {"calibration": calibration}}}))
    calibration_before = calibration_path.read_bytes()
    data = yaml.safe_load(config_path.read_text())
    data["sensors"]["glove"]["calibration_path"] = "calibration.yaml"
    config_path.write_text(yaml.safe_dump(data))
    original = config_path.read_bytes()
    output = tmp_path / "thumb.json"
    prompts, logs = [], []

    def prompt(text):
        prompts.append(text)
        source.pose = "open" if len(prompts) == 1 else "fist"

    def create(*a, **kw):
        assert kw["filter"] == {"enabled": False}
        assert "calibration_path" not in kw
        return source

    assert check.main(["--sensor-config", str(config_path), "--joint", "thumb_bend",
                       "--duration", ".12", "--settle-seconds", "0", "--output", str(output)],
                      source_factory=create, clock=clock, input_fn=prompt, print_fn=logs.append) == 0
    assert len(prompts) == 2 and "拇指指尖" in prompts[1]
    assert source.started and source.closed
    assert config_path.read_bytes() == original
    assert calibration_path.read_bytes() == calibration_before
    report = json.loads(output.read_text())
    assert report["mapping"] == REFERENCE_MAPPING and not report["robot_motion_verified"]
    assert report["channels"][6]["mapped_joints"] == ["index_flexion"]
    assert report["channels"][6]["delta"] == 300
    if mode == "normalized":
        assert report["channels"][6]["normalized_delta"] == pytest.approx(.3)
    else:
        assert report["channels"][6]["normalized_delta"] is None
    assert all(len(row["raw"]) == 11 for pose in report["poses"].values() for row in pose)
    assert any("raw[6] index_ip" in line and "delta=+300.00" in line for line in logs)
    assert any("raw[10] reserved_11" in line for line in logs)


@pytest.mark.parametrize("failure", ["silent", "stale", "read", "start", "cancel"])
def test_failure_cannot_produce_report_and_releases_source(config_path, tmp_path, failure):
    clock = FakeClock()
    source = ScriptedGlove(clock, fail=failure)
    output = tmp_path / "failed.json"

    def prompt(_):
        if failure == "cancel":
            raise KeyboardInterrupt

    assert check.main(["--sensor-config", str(config_path), "--joint", "thumb_bend",
                       "--duration", ".5", "--settle-seconds", "0", "--output", str(output)],
                      source_factory=lambda *a, **kw: source, clock=clock,
                      input_fn=prompt, print_fn=lambda _: None) == (130 if failure == "cancel" else 1)
    assert source.closed and not output.exists()


def test_existing_report_and_invalid_calibration_fail_before_serial(config_path, tmp_path):
    output = tmp_path / "keep.json"
    output.write_text("KEEP")
    kwargs = dict(source_factory=lambda *a, **kw: pytest.fail("opened serial"), print_fn=lambda _: None)
    args = ["--sensor-config", str(config_path), "--joint", "thumb_bend"]
    assert check.main([*args, "--output", str(output)], **kwargs) == 1
    assert output.read_text() == "KEEP"
    data = yaml.safe_load(config_path.read_text())
    data["sensors"]["glove"]["calibration_path"] = "missing.yaml"
    config_path.write_text(yaml.safe_dump(data))
    assert check.main(args, **kwargs) == 1


@pytest.mark.parametrize("options", [["--execute"], ["--robot", "aurora"], ["--joint", "bad"],
                                    ["--duration", "nan"], ["--min-samples", "4"]])
def test_cli_rejects_motion_or_invalid_arguments(options):
    with pytest.raises(SystemExit):
        check.parse_args(["--joint", "thumb_bend", *options])


def test_cli_help_runs_from_outside_repository_without_sdk(tmp_path):
    result = subprocess.run([sys.executable, str(ROOT / "tools/check_bend6_mapping.py"), "--help"],
                            cwd=tmp_path, capture_output=True, text=True)
    assert result.returncode == 0 and "--joint" in result.stdout
