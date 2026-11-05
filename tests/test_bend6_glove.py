"""Bend6 wire -> nine-channel reading -> Aurora hand, entirely offline."""
from dataclasses import replace
import json
from pathlib import Path
import shutil
import subprocess
import sys
import threading

import pytest
import yaml

from sleeve_arm.config import BEND6_DEFAULT_MAPPING, GloveConfig, load_sensor_config
from sleeve_arm.robot import factory
from sleeve_arm.robot.aurora_fake import FakeClock, fake_session, gr3_fake_profile
from sleeve_arm.runtime.configuration import load_configs, parse_args
from sleeve_arm.runtime.glove_control import GloveRuntime, HAND_JOINTS
from sleeve_arm.runtime.robot_runtime import RobotRuntime, PrintOnlyRuntime
from sleeve_arm.sources.bend6 import (
    BEND6_CHANNEL_NAMES, BEND6_RAW_CHANNEL_NAMES, Bend6Calibration, Bend6Filter,
    Bend6GloveSource, Bend6Parser, load_bend6_calibration,
)


ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "configs/sensors_bend6.example.yaml"


def packet(values=range(11), separator=",", ending=";"):
    return (separator.join(map(str, values)) + ending).encode("ascii")


def normalized():
    # Synthetic calibration for tests only; never shipped as hardware settings.
    return {"mode": "normalized", "channels": {
        name: {"raw_min": 0., "raw_max": 1., "invert": False} for name in BEND6_CHANNEL_NAMES
    }}


class FakeSerial:
    def __init__(self, chunks=(), error=None):
        self.chunks = list(chunks)
        self.error = error
        self.processed = threading.Event()
        self.closed = threading.Event()
        self.in_waiting = 8192

    def read(self, size):
        if self.chunks:
            return self.chunks.pop(0)
        self.processed.set()
        if self.error:
            raise self.error
        self.closed.wait(.005)
        return b""

    def close(self):
        self.closed.set()

    def write(self, _):
        pytest.fail("glove acquisition must never write to serial")


def runtime(tmp_path, *, calibrated=True, clock=None, mapping=None):
    path = tmp_path / "calibration.yaml"
    path.write_text(yaml.safe_dump({"glove": {"bend6": {"calibration": normalized()}}}))
    serial = FakeSerial()
    config = GloveConfig(backend="bend6", port="FAKE_SERIAL",
                         calibration_path=path if calibrated else None,
                         bend6_mapping=dict(BEND6_DEFAULT_MAPPING if mapping is None else mapping))
    glove = GloveRuntime(config, clock=clock or FakeClock(), serial_factory=lambda **_: serial)
    glove.start()
    # The first terminated record must always be discarded.
    glove.source._feed_bytes(b"partial;")
    return glove


def test_raw_schema_signs_reserved_sequence_and_immutable_reading():
    clock = FakeClock()
    source = Bend6GloveSource("FAKE", clock=clock, side="left")
    values = (-3., 2., -1., 0., 10., 20., 30., 40., 50., -9., 123.)
    source._feed_bytes(b"startup;" + packet(values))
    reading = source.latest()
    assert reading.raw == values and len(BEND6_RAW_CHANNEL_NAMES) == 11
    assert dict(reading.channels) == reading.raw_channels == dict(zip(BEND6_CHANNEL_NAMES, values))
    assert reading.raw[9:] == (-9., 123.)
    assert not reading.calibrated and reading.calibration_mode == "raw"
    assert not reading.filter_enabled and reading.side == "left" and reading.sequence == 1
    assert reading.status == "ok" and reading.timestamp == 100.
    with pytest.raises(TypeError):
        reading.channels["index_ip"] = 10
    clock.sleep(.1)
    assert source.latest() is reading
    source._feed_bytes(packet(values))
    assert source.latest().sequence == 2 and source.latest().timestamp == 100.1


@pytest.mark.parametrize("delimiter,separator", [(";", ","), (";", " \t\n"), ("\n", ";"), ("\n", ", ")])
def test_every_byte_split_and_glued_packets(delimiter, separator):
    first, second = tuple(range(11)), tuple(range(-5, 6))
    data = b"startup" + delimiter.encode() + packet(first, separator, delimiter) + packet(second, separator, delimiter)
    for split in range(len(data) + 1):
        parser = Bend6Parser(delimiter)
        assert list(parser.feed(data[:split])) + list(parser.feed(data[split:])) == [first, second]
    parser = Bend6Parser(delimiter)
    assert [frame for byte in data for frame in parser.feed(bytes([byte]))] == [first, second]


def test_start_at_every_byte_discards_first_record_and_does_not_guess_unframed_alignment():
    frame = packet()
    for split in range(len(frame)):
        parser = Bend6Parser()
        assert list(parser.feed(frame[split:] + packet(range(11, 22)))) == [tuple(range(11, 22))]
    parser = Bend6Parser()
    assert list(parser.feed(b"1;2;3;4;5;6;7;8;9;10;11;" * 4)) == []
    assert list(parser.feed(packet())) == [tuple(range(11))]
    parser.reset()
    assert list(parser.feed(packet())) == []  # Even a full first frame is uncertain.
    assert list(parser.feed(packet())) == [tuple(range(11))]


@pytest.mark.parametrize("bad", [
    b"\xff,1,2,3,4,5,6,7,8,9,10;", b"1,,2,3,4,5,6,7,8,9,10;",
    b",1,2,3,4,5,6,7,8,9,10;", b"0,1,2,3,4,5,6,7,8,9,;",
    b"0,1,2,3,4,5,6,7,8,9,nan;", b"0,1,2,3,4,5,6,7,8,9,Inf;",
    b"0,1,2,3,4,5,6,7,8,9,1e999;", b"1,2,3;", packet(range(12)),
    b"0,1,2,3,4,5,6,7,8,9,1_0;", b"0,1,2,3,4,5,6,7,8,9,x;", b";",
    b"0,1,2;3,4,5,6,7,8,9,10;", b"0,1,2,3,4,5,6,7,8,9,10\x00;",
])
def test_corrupt_record_cannot_pollute_following_packet_or_refresh_timestamp(bad):
    clock = FakeClock()
    source = Bend6GloveSource("FAKE", clock=clock)
    source._feed_bytes(b"startup;" + packet())
    first = source.latest()
    clock.sleep(.1)
    source._feed_bytes(bad)
    assert source.latest() is first
    source._feed_bytes(packet(range(100, 111)))
    assert source.latest().raw == tuple(range(100, 111))
    assert source.latest().sequence == 2 and source.latest().timestamp == clock.monotonic()
    assert source.parser.invalid_records > 0


def test_oversize_record_discards_to_boundary_and_recovers():
    parser = Bend6Parser()
    assert list(parser.feed(b"sync;" + b"1" * 9000 + b";" + packet())) == [tuple(range(11))]
    assert parser.invalid_records == 1


def test_nine_independent_calibrations_invert_clip_and_raw_bypass():
    config = normalized()
    values, expected = [], []
    for i, name in enumerate(BEND6_CHANNEL_NAMES):
        config["channels"][name].update(raw_min=10 * i, raw_max=10 * i + 5, invert=i % 2 == 1)
        amount = (i - 2) / 4
        values.append(10 * i + 5 * amount)
        clipped = min(1., max(0., amount))
        expected.append(1. - clipped if i % 2 else clipped)
    assert Bend6Calibration(config).apply(values) == pytest.approx(expected)
    assert Bend6Calibration().apply(values) == tuple(values)
    config["channels"]["thumb_mcp"].update(raw_min=10, raw_max=-10, invert=False)
    values[7] = 5
    assert Bend6Calibration(config).apply(values)[7] == .25


def test_piecewise_points_and_invert():
    config = normalized()
    config["channels"]["thumb_tip_joint"].update(points=[[0, 0], [2, .5], [8, 1]], invert=True)
    calibration = Bend6Calibration(config)
    for raw, expected in [(-10, 1), (0, 1), (1, .75), (2, .5), (5, .25), (8, 0), (100, 0)]:
        assert calibration.apply([0.] * 8 + [raw])[8] == expected


@pytest.mark.parametrize("changes", [
    {"raw_min": 1, "raw_max": 1}, {"raw_min": None}, {"raw_min": float("nan")},
    {"raw_max": float("inf")}, {"invert": "false"}, {"raw_min": True}, {"other": 1},
    {"points": [[0, 0], [0, 1]]}, {"points": [[1, 0], [0, 1]]},
    {"points": [[0, 0], [1, .7], [2, .6], [3, 1]]}, {"points": [[0, .1], [1, 1]]},
    {"points": [[0, 0], [1, float("nan")]]}, {"points": [[0, 0]]}, {"points": [0, 1]},
    {"points": [[0, 0], [1, 1.1], [2, 1]]}, {"raw_min": -1e308, "raw_max": 1e308},
])
def test_invalid_channel_calibrations_rejected(changes):
    config = normalized()
    config["channels"]["index_ip"].update(changes)
    with pytest.raises(ValueError):
        Bend6Calibration(config)


@pytest.mark.parametrize("config", [
    {"mode": "auto"}, {"mode": "normalized", "channels": {}},
    {"mode": "raw", "channels": {"reserved_10": {}}}, {"mode": "raw", "channels": []},
])
def test_invalid_calibration_structure(config):
    with pytest.raises(ValueError):
        Bend6Calibration(config)


@pytest.mark.parametrize("extension", ["yaml", "yml", "json"])
@pytest.mark.parametrize("wrapper", ["glove", "bend6", "calibration", "bare"])
def test_calibration_file_formats(tmp_path, extension, wrapper):
    data = normalized()
    if wrapper != "bare":
        data = {"calibration": data}
    if wrapper in ("bend6", "glove"):
        data = {"bend6": data}
    if wrapper == "glove":
        data = {"glove": data}
    path = tmp_path / f"cal.{extension}"
    path.write_text(json.dumps(data) if extension == "json" else yaml.safe_dump(data))
    assert load_bend6_calibration(path) == normalized()
    assert Bend6GloveSource("FAKE", calibration_path=path).calibration_loaded


def test_filter_ema_deadband_delta_independence_and_reset():
    f = Bend6Filter({"enabled": True, "alpha": .5, "deadband": .1, "max_delta_per_sample": .4})
    assert f.apply([0.] * 9) == (0.,) * 9
    assert f.apply([.05] * 9) == (0.,) * 9
    assert f.apply([1.] * 9) == pytest.approx([.2] * 9)
    assert f.apply([.4] * 9) == pytest.approx([.3] * 9)
    other = Bend6Filter({"enabled": True})
    assert other.apply([.9] * 9) == (.9,) * 9
    f.reset()
    assert f.apply([.9] * 9) == (.9,) * 9
    assert Bend6Filter().apply([-200.] * 9) == (-200.,) * 9


@pytest.mark.parametrize("config", [
    {"enabled": "false"}, {"type": "none"}, {"alpha": 0}, {"alpha": 2},
    {"alpha": float("nan")}, {"deadband": float("inf")}, {"deadband": -.1},
    {"max_delta_per_sample": 0}, {"max_delta_per_sample": float("nan")}, {"typo": 1},
])
def test_invalid_filter_rejected_even_when_disabled(config):
    with pytest.raises(ValueError):
        Bend6Filter(config)


def test_filter_follows_calibration_preserves_raw_and_resets_on_corruption():
    calibration = normalized()
    for item in calibration["channels"].values():
        item.update(raw_min=10, raw_max=20)
    source = Bend6GloveSource("FAKE", calibration=calibration, filter={"enabled": True, "alpha": .5})
    source._feed_bytes(b"sync;" + packet([10] * 11) + packet([20] * 11))
    assert source.latest().raw == (20.,) * 11
    assert list(source.latest().channels.values()) == [.5] * 9
    source._feed_bytes(b"bad;" + packet([20] * 11))
    assert list(source.latest().channels.values()) == [1.] * 9
    source.close()


def test_source_lifecycle_lazy_open_restart_reset_and_thread_failure():
    clock = FakeClock()
    serials = [FakeSerial([b"partial;" + packet([0] * 11) + packet([10] * 11)]),
               FakeSerial([packet([999] * 11) + packet([20] * 11)]),
               FakeSerial([b"partial;" + packet()], error=OSError("unplugged"))]
    calls = []

    def factory(**kwargs):
        calls.append(kwargs)
        return serials[len(calls) - 1]

    source = Bend6GloveSource("FAKE", clock=clock, serial_factory=factory,
                              filter={"enabled": True, "alpha": .5})
    assert calls == [] and source.latest() is None
    for index in range(3):
        source.start()
        thread = source._thread
        assert serials[index].processed.wait(2.)
        if index == 2:
            assert serials[index].closed.wait(2.)
            with pytest.raises(RuntimeError, match="unplugged"):
                source.latest()
        else:
            assert list(source.latest().channels.values()) == [5. if index == 0 else 20.] * 9
            assert source.latest().sequence == (2 if index == 0 else 1)
            with pytest.raises(RuntimeError, match="already started"):
                source.start()
        source.close()
        source.close()
        assert serials[index].closed.is_set() and not thread.is_alive()
        clock.sleep(1.)
    assert all(call == {"port": "FAKE", "baudrate": 115200, "timeout": .02} for call in calls)


def test_open_failure_releases_source():
    def denied(**_):
        raise PermissionError("denied")
    source = Bend6GloveSource("FAKE", serial_factory=denied)
    with pytest.raises(PermissionError):
        source.start()
    source.close()
    assert source._serial is source._thread is None


def test_complete_wire_to_six_joints_independent_thumb_and_unused_channels(tmp_path):
    mapping = load_sensor_config(EXAMPLE).glove.bend6_mapping
    glove = runtime(tmp_path, mapping=mapping)
    try:
        for channel_index, name in enumerate(BEND6_CHANNEL_NAMES):
            values = [0.] * 11
            values[channel_index] = 1.
            values[9:] = [123, -456]
            glove.source._feed_bytes(packet(values))
            expected = {joint: glove.config.closed_pose_rad[i] if mapping[joint] == name
                        else glove.config.open_pose_rad[i] for i, joint in enumerate(HAND_JOINTS)}
            assert glove.targets() == pytest.approx(expected)
            assert glove.reading.raw == tuple(values)
            assert len(glove.reading.channels) == 9
        before = glove.reading
        glove.clock.sleep(.251)
        assert glove.latest() is before
        with pytest.raises(RuntimeError, match="watchdog"):
            glove.targets()
        glove.source._feed_bytes(packet([.5] * 11))
        assert glove.reading.timestamp > before.timestamp and glove.targets()
    finally:
        glove.close()


def test_custom_mapping_and_poses_still_use_existing_limits(tmp_path):
    mapping = dict(BEND6_DEFAULT_MAPPING, thumb_bend="index_mcp", thumb_swing="index_middle_between")
    glove = runtime(tmp_path, mapping=mapping)
    try:
        values = [.1 * i for i in range(9)] + [90., 91.]
        glove.source._feed_bytes(packet(values))
        for i, (joint, value) in enumerate(glove.targets().items()):
            normalized_value = values[BEND6_CHANNEL_NAMES.index(mapping[joint])]
            assert value == pytest.approx(glove.config.open_pose_rad[i] + normalized_value * (
                glove.config.closed_pose_rad[i] - glove.config.open_pose_rad[i]))
        glove.config.bend6_mapping["thumb_bend"] = "reserved_10"
        with pytest.raises(ValueError, match="reserved"):
            glove.targets()
    finally:
        glove.close()


def test_nonidentity_calibration_file_wire_to_targets_consistency(tmp_path):
    calibration = normalized()
    for settings in calibration["channels"].values():
        settings.update(raw_min=10., raw_max=30.)
    calibration["channels"]["thumb_mcp"]["invert"] = True
    calibration_path = tmp_path / "calibration.json"
    calibration_path.write_text(json.dumps({"glove": {"bend6": {"calibration": calibration}}}))
    data = yaml.safe_load(EXAMPLE.read_text())
    data["sensors"]["glove"].update(port="FAKE", calibration_path="calibration.json")
    sensor_path = tmp_path / "sensors.yaml"
    sensor_path.write_text(yaml.safe_dump(data))
    config = load_sensor_config(sensor_path).glove
    assert config.calibration_path == calibration_path
    serial = FakeSerial([b"partial;" + packet([10, 25, 30, 20, 16, 18, 15, 12, 24, -90, 99])])
    glove = GloveRuntime(config, clock=FakeClock(), serial_factory=lambda **_: serial)
    try:
        glove.start()
        assert serial.processed.wait(2.)
        glove.wait_ready()
        assert glove.reading.raw == (10., 25., 30., 20., 16., 18., 15., 12., 24., -90., 99.)
        assert list(glove.reading.channels.values()) == pytest.approx([0., .75, 1., .5, .3, .4, .25, .9, .7])
        assert list(glove.targets().values()) == pytest.approx([.599, .515, .81, 1.4, .22, 1.13])
    finally:
        glove.close()
    assert serial.closed.is_set()


def test_raw_between_channels_keep_units_bypass_filter_and_do_not_block_six_axes(tmp_path):
    from tools.check_bend6_mapping import compare_poses
    from test_calibrate_bend6_glove import samples
    calibration = normalized()
    for name in ("little_ring_between", "index_middle_between"):
        calibration["channels"][name] = {"mode": "raw"}
    path = tmp_path / "partial.yaml"
    path.write_text(yaml.safe_dump(calibration))
    config = GloveConfig(backend="bend6", port="FAKE", calibration_path=path,
                         source_options={"filter": {"enabled": True, "alpha": .5, "deadband": 0.}})
    serial = FakeSerial()
    glove = GloveRuntime(config, clock=FakeClock(), serial_factory=lambda **_: serial)
    opened = [0., 204., 0., 0., -515., 0., 0., 0., 0., 99., -99.]
    fist = [1., 210., 1., 1., 1200., 1., 1., 1., 1., 99., -99.]
    try:
        glove.start()
        glove.source._feed_bytes(b"sync;" + packet(opened))
        glove.wait_ready()
        glove.source._feed_bytes(packet(fist))
        assert glove.reading.calibration_mode == "partial" and not glove.reading.calibrated
        assert glove.reading.raw == tuple(fist) and len(glove.reading.channels) == 9
        assert glove.reading.channels["little_ring_between"] == 210.
        assert glove.reading.channels["index_middle_between"] == 1200.
        assert glove.reading.channels["index_ip"] == .5
        expected = dict(zip(HAND_JOINTS, ((a + b) / 2 for a, b in zip(config.open_pose_rad, config.closed_pose_rad))))
        assert glove.targets() == pytest.approx(expected)
        assert glove.preview_targets() == pytest.approx(expected)
        logs = []
        glove.report(logs.append)
        assert "raw_only_channels=" in logs[0] and "unavailable" not in "".join(logs)
        rows = compare_poses(samples(opened), samples(fist), config.bend6_mapping, glove.source.calibration)
        assert rows[1]["normalized_delta"] is None and rows[4]["normalized_delta"] is None
    finally:
        glove.close()
    for name in ("little_ring_between", "index_middle_between"):
        invalid = replace(config, bend6_mapping=dict(config.bend6_mapping, index_flexion=name))
        runtime = GloveRuntime(invalid, serial_factory=lambda **_: pytest.fail("opened serial"))
        with pytest.raises(ValueError, match="uncalibrated raw-only"):
            runtime.validate_for_control()


def test_raw_channel_exception_cannot_disable_finger_calibration():
    calibration = normalized()
    calibration["channels"]["index_ip"] = {"mode": "raw"}
    with pytest.raises(ValueError, match="only allowed for between"):
        Bend6Calibration(calibration)


@pytest.fixture(params=[False, True], ids=["recalibrated", "legacy-inverted"])
def linked_thumb_config(tmp_path, request):
    # Exercise both new open=0/fist=1 calibration and the old inverted thumb.
    calibration = normalized()
    for channel in calibration["channels"].values():
        channel.update(raw_min=100., raw_max=200.)
    calibration["channels"]["thumb_tip_joint"]["invert"] = request.param
    (tmp_path / "calibration.yaml").write_text(yaml.safe_dump(calibration))
    data = yaml.safe_load(EXAMPLE.read_text())
    data["sensors"]["glove"].update(
        port="FAKE_SERIAL", calibration_path="calibration.yaml", stale_policy="hold_last",
        open_pose_rad=[1.28 if request.param else .35, .22, .22, .22, .22,
                       1.68 if request.param else .05],
        closed_pose_rad=[.35 if request.param else 1.28, 1.78, 1.78, 1.78, 1.78,
                         .05 if request.param else 1.68],
        bend6_mapping=dict(BEND6_DEFAULT_MAPPING, thumb_swing="thumb_tip_joint"),
    )
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize("closure", [0., .5, 1.])
def test_linked_thumb_preserves_direction_and_reaches_expanded_endpoints(linked_thumb_config, closure):
    defaults = dict(BEND6_DEFAULT_MAPPING)
    config = load_sensor_config(linked_thumb_config).glove
    serial = FakeSerial()
    glove = GloveRuntime(config, clock=FakeClock(), serial_factory=lambda **_: serial)
    try:
        glove.start()
        values = [100. + 100. * closure] * 9 + [-90., 91.]
        glove.source._feed_bytes(b"sync;" + packet(values))
        glove.wait_ready()
        expected = dict(zip(HAND_JOINTS, (
            .35 + .93 * closure, *([.22 + 1.56 * closure] * 4), .05 + 1.63 * closure,
        )))
        assert glove.targets() == pytest.approx(expected)
        inverted = load_bend6_calibration(config.calibration_path)["channels"]["thumb_tip_joint"]["invert"]
        assert glove.reading.channels["thumb_tip_joint"] == pytest.approx(1. - closure if inverted else closure)
        # The old independent swing channel must no longer move either thumb axis.
        for value in (0., 300.):
            values[7] = value
            glove.source._feed_bytes(packet(values))
            assert glove.targets() == pytest.approx(expected)
            assert glove.reading.raw == tuple(values) and len(glove.reading.channels) == 9
        # Loading a shared-input config must not mutate defaults or the independent example.
        assert GloveConfig().closed_pose_rad == (.35, 1.4, 1.4, 1.4, 1.4, 1.25)
        assert BEND6_DEFAULT_MAPPING == defaults
        assert load_sensor_config(EXAMPLE).glove.bend6_mapping["thumb_swing"] == "thumb_mcp"
    finally:
        glove.close()
    assert serial.closed.is_set()


def test_linked_thumb_opens_and_closes_with_previous_bridge_slew(linked_thumb_config, monkeypatch):
    from sleeve_arm.config import load_phase3_config
    from sleeve_arm.runtime.configuration import ModelControlConfigs
    from tools.debug_glove import parse_args as debug_args
    args = debug_args(["--sensor-config", str(linked_thumb_config), "--robot", "aurora-fake"])
    configs = ModelControlConfigs(sensors=load_sensor_config(linked_thumb_config),
                                  phase3=load_phase3_config(), glove_mode="real")
    session = fake_session(profile=gr3_fake_profile())
    session.fake_client.groups["right_hand"]["position"] = [.22] * 4 + [.35, .05]
    actual = factory.create_robot
    monkeypatch.setattr(factory, "create_robot", lambda backend, config, **kw:
                        actual(backend, config, session=session, **kw))
    serial = FakeSerial()
    glove = GloveRuntime(configs.sensors.glove, clock=session.clock, serial_factory=lambda **_: serial)
    robot = RobotRuntime(args, configs, clock=session.clock, parts=("hand",), print_fn=lambda _: None)
    expected = dict(zip(HAND_JOINTS, (1.28, 1.78, 1.78, 1.78, 1.78, 1.68)))
    try:
        glove.start()
        glove.source._feed_bytes(b"sync;" + packet([100.] * 9 + [0., 0.]))
        glove.wait_ready()
        robot.connect()
        assert robot.prepare(map_arm_intents=False)
        glove.source._feed_bytes(packet([200.] * 9 + [0., 0.]))
        reading = glove.reading
        before = session.fake_client.commands[-1]["right_hand"]
        # The longest travel is 1.63 rad: 55 cycles at 0.03 rad/cycle.
        for tick in range(55):
            session.clock.sleep(.01)
            targets = glove.targets()
            assert targets == pytest.approx(expected)
            robot.apply_hand(targets, .01)
            command = session.fake_client.commands[-1]
            assert set(command) == {"right_hand"}
            assert all(0 <= b - a <= .03 + 1e-9 for a, b in zip(before, command["right_hand"]))
            before = command["right_hand"]
            assert before[5] == pytest.approx(min(1.68, .05 + .03 * (tick + 1)))
        assert before == pytest.approx([1.78] * 4 + [1.28, 1.68])
        session.clock.sleep(.01)
        assert robot.feedback() == pytest.approx(expected)
        assert glove.reading is reading  # hold_last does not refresh or alter the received frame.
        glove.source._feed_bytes(packet([100.] * 9 + [0., 0.]))
        for tick in range(55):
            session.clock.sleep(.01)
            robot.apply_hand(glove.targets(), .01)
            command = session.fake_client.commands[-1]["right_hand"]
            assert all(0 <= a - b <= .03 + 1e-9 for a, b in zip(before, command))
            before = command
            assert before[5] == pytest.approx(max(.05, 1.68 - .03 * (tick + 1)))
        assert before == pytest.approx([.22] * 4 + [.35, .05])
    finally:
        glove.close()
        robot.shutdown()
    assert serial.closed.is_set() and session.fake_client.close_count == 1


def test_raw_readiness_preview_and_control_are_separate(tmp_path):
    glove = runtime(tmp_path, calibrated=False)
    try:
        glove.source._feed_bytes(packet())
        glove.wait_ready(require_control=False)
        assert glove.preview_targets() is None
        for call in (glove.targets, glove.wait_ready, glove.validate_for_control):
            with pytest.raises(ValueError, match="raw data is read-only"):
                call()
        messages = []
        glove.report(messages.append)
        assert all(name in messages[0] for name in BEND6_CHANNEL_NAMES)
        assert "unavailable" in messages[-1]
    finally:
        glove.close()


@pytest.mark.parametrize("timestamp", [float("nan"), float("inf"), 101., 0.])
def test_invalid_reading_timestamp_never_controls(tmp_path, timestamp):
    glove = runtime(tmp_path)
    try:
        glove.source._feed_bytes(packet([0] * 11))
        glove.source._reading = replace(glove.reading, timestamp=timestamp)
        with pytest.raises(RuntimeError, match="watchdog"):
            glove.targets()
    finally:
        glove.close()


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -1., 2.])
def test_invalid_normalized_channel_rejected(tmp_path, value):
    glove = runtime(tmp_path)
    try:
        glove.source._feed_bytes(packet([0] * 11))
        glove.source._reading = replace(glove.reading, channels={**glove.reading.channels, "thumb_mcp": value})
        with pytest.raises(ValueError, match="finite normalized"):
            glove.targets()
    finally:
        glove.close()


def test_sensor_config_defaults_mapping_validation_and_template(tmp_path, monkeypatch):
    import serial
    monkeypatch.setattr(serial, "Serial", lambda **_: pytest.fail("configuration opened hardware"))
    assert load_sensor_config().glove.backend == "bend5"
    config = load_sensor_config(EXAMPLE)
    assert config.glove.backend == "bend6" and not config.glove.enabled
    assert GloveConfig().bend6_mapping == BEND6_DEFAULT_MAPPING
    # Explicit YAML mapping takes precedence over a locally customized default.
    assert config.glove.bend6_mapping == yaml.safe_load(EXAMPLE.read_text())["sensors"]["glove"]["bend6_mapping"]
    for name in ("sleeve", "imu1", "imu2", "imu3", "imu4", "synchronization", "shoulder_imu", "upper_arm_rotation"):
        assert getattr(config, name) == getattr(load_sensor_config(), name)
    with pytest.raises(ValueError, match="finite number"):
        Bend6GloveSource("FAKE", calibration_path=ROOT / "configs/bend6_calibration.template.yaml")
    original = yaml.safe_load(EXAMPLE.read_text())
    for bad in ({}, {**BEND6_DEFAULT_MAPPING, "bad_joint": "index_ip"},
                {**BEND6_DEFAULT_MAPPING, "thumb_bend": "reserved_11"},
                {**BEND6_DEFAULT_MAPPING, "thumb_bend": ["index_ip"]}):
        original["sensors"]["glove"]["bend6_mapping"] = bad
        path = tmp_path / "sensors.yaml"
        path.write_text(yaml.safe_dump(original))
        with pytest.raises(ValueError, match="mapping"):
            load_sensor_config(path)
    original["sensors"]["glove"]["bend6_mapping"] = dict(BEND6_DEFAULT_MAPPING)
    for options in ({"zero_on_start": True}, {"side": "both"}, {"filter": {"alpha": float("inf")}},
                    {"synthetic": True}, {"packet_delimiter": ","}):
        original["sensors"]["glove"]["source_options"] = options
        path.write_text(yaml.safe_dump(original))
        with pytest.raises(ValueError):
            load_sensor_config(path)


@pytest.mark.parametrize("entry,options", [
    ("debug_glove", ["--execute"]), ("debug_glove", ["--robot", "aurora-fake"]),
    ("run_model_control", ["--robot", "aurora-fake", "--side", "right", "--source-side", "right", "--imus", "fake"]),
    ("run_model_control", ["--robot", "aurora", "--side", "right", "--source-side", "right", "--execute",
                           "--aurora-profile", str(ROOT / "configs/robot_aurora.yaml")]),
])
def test_raw_execution_rejected_before_any_hardware_connection(tmp_path, monkeypatch, capsys, entry, options):
    from tools import debug_glove, run_model_control
    import serial
    data = yaml.safe_load(EXAMPLE.read_text())
    data["sensors"]["glove"]["port"] = "MUST_NOT_OPEN"
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(data))
    monkeypatch.setattr(serial, "Serial", lambda **_: pytest.fail("raw execution opened serial"))
    monkeypatch.setattr(factory, "create_robot", lambda *a, **kw: pytest.fail("raw execution connected robot"))
    module = debug_glove if entry == "debug_glove" else run_model_control
    assert module.main(["--sensor-config", str(path), "--glove", "real", "--duration", ".05", *options]) == 1
    assert "raw data is read-only" in capsys.readouterr().err


def test_debug_raw_preview_reads_serial_without_creating_robot(tmp_path, monkeypatch, capsys):
    from tools import debug_glove
    glove = runtime(tmp_path, calibrated=False)
    monkeypatch.setattr(debug_glove, "time", glove.clock)
    monkeypatch.setattr(debug_glove, "GloveRuntime", lambda *a, **kw: glove)
    monkeypatch.setattr(glove, "start", lambda: glove.source._feed_bytes(packet()))
    monkeypatch.setattr(factory, "create_robot", lambda *a, **kw: pytest.fail("preview connected robot"))
    assert debug_glove.main(["--duration", ".05"]) == 0
    output = capsys.readouterr().out
    assert "calibration=raw" in output and "target_rad=unavailable" in output
    assert "sequence=" in output and "age=" in output and "raw=(" in output
    assert glove.source is None


def test_combined_raw_print_only_and_stale_watchdog(tmp_path, monkeypatch):
    from test_model_control_app import make_app
    app, events = make_app(duration=.05)
    app.robot = PrintOnlyRuntime()
    app.glove_control_required = False
    app.glove = runtime(tmp_path, calibrated=False, clock=app.clock)
    monkeypatch.setattr(app.glove, "start", lambda: app.glove.source._feed_bytes(packet()))
    assert app.run() == 0 and app.predictions > 0
    assert app.glove.source is None and "sensor_close" in events
    app, events = make_app(duration=.5)
    app.robot = PrintOnlyRuntime()
    app.glove_control_required = False
    app.glove = runtime(tmp_path, calibrated=False, clock=app.clock)
    monkeypatch.setattr(app.glove, "start", lambda: app.glove.source._feed_bytes(packet()))
    assert app.run() == 1 and "glove watchdog" in app.fault


def test_combined_print_only_cli_allows_real_raw_without_control_targets(tmp_path, monkeypatch, capsys):
    from test_model_control_app import FakeSensorRuntime, FakeIntentPipeline
    from sleeve_arm.runtime.model_control import ModelControlApp
    from tools import run_model_control

    clock, events = FakeClock(), []
    data = yaml.safe_load(EXAMPLE.read_text())
    data["sensors"]["glove"]["port"] = "FAKE_SERIAL"
    path = tmp_path / "sensors.yaml"
    path.write_text(yaml.safe_dump(data))
    monkeypatch.setattr(run_model_control, "create_sensor_runtime", lambda *_: FakeSensorRuntime(clock, events))
    monkeypatch.setattr(run_model_control, "create_intent_pipeline", lambda *_: FakeIntentPipeline(events))
    monkeypatch.setattr(run_model_control, "ModelControlApp", lambda **kw: ModelControlApp(clock=clock, **kw))
    monkeypatch.setattr(factory, "create_robot", lambda *a, **kw: pytest.fail("print-only connected robot"))
    serial = FakeSerial([b"sync;" + packet()])
    actual_factory = run_model_control.create_glove_runtime
    # Use the real factory/CLI readiness decision; only inject transport and time.
    from sleeve_arm.runtime import glove_control
    monkeypatch.setattr(glove_control, "GloveRuntime", lambda *a, **kw:
                        GloveRuntime(*a, **kw, clock=clock, serial_factory=lambda **_: serial))
    gloves = []

    def create_glove(args, configs):
        glove = actual_factory(args, configs)
        original_start, original_latest = glove.start, glove.latest

        def start():
            original_start()
            assert serial.processed.wait(2.)

        def latest():
            # Deterministic next wire packet from a scripted producer.
            glove.source._feed_bytes(packet())
            return original_latest()

        glove.start, glove.latest = start, latest
        gloves.append(glove)
        return glove

    monkeypatch.setattr(run_model_control, "create_glove_runtime", create_glove)
    submitted = []
    original_apply = PrintOnlyRuntime.apply

    def apply(self, intent, dt, *, hand_targets=None):
        submitted.append(hand_targets)
        return original_apply(self, intent, dt, hand_targets=hand_targets)

    monkeypatch.setattr(PrintOnlyRuntime, "apply", apply)
    assert run_model_control.main(["--sensor-config", str(path), "--print-only", "--glove", "real",
                                   "--duration", "1.05"]) == 0
    assert submitted and all(target is None for target in submitted)
    assert "calibration=raw" in capsys.readouterr().out
    assert serial.closed.is_set() and gloves[0].source is None


@pytest.mark.parametrize("failure", ["stale", "reader", "mapping", "nonfinite"])
def test_combined_bend6_fault_stops_both_outputs_and_cleans_up(tmp_path, monkeypatch, failure):
    from test_model_control_app import make_app
    app, events = make_app(duration=.5)
    glove = runtime(tmp_path, clock=app.clock)
    app.glove = glove
    monkeypatch.setattr(glove, "start", lambda: glove.source._feed_bytes(packet([0.] * 11)))
    commands = []

    def apply(intent, dt, **kwargs):
        commands.append((app.clock.monotonic(), kwargs))
        if failure == "reader":
            glove.source._error = OSError("injected disconnect")
        elif failure == "mapping":
            glove.config.bend6_mapping["thumb_swing"] = "reserved_10"
        elif failure == "nonfinite":
            glove.config = replace(glove.config, closed_pose_rad=(float("nan"),) * 6)
        return {}

    app.robot.apply = apply
    assert app.run() == 1 and app.fault
    assert commands and all(when <= 100.25 + 1e-9 for when, _ in commands)
    assert all(set(kwargs["hand_targets"]) == set(HAND_JOINTS) for _, kwargs in commands)
    assert glove.source is None and events[-2:] == ["robot_shutdown", "sensor_close"]


def test_silent_raw_source_readiness_times_out_and_releases(tmp_path):
    glove = runtime(tmp_path, calibrated=False)
    glove.config = replace(glove.config, startup_timeout_s=.03)
    serial, thread = glove.source._serial, glove.source._thread
    try:
        with pytest.raises(RuntimeError, match="readiness timeout"):
            glove.wait_ready(require_control=False)
    finally:
        glove.close()
    assert serial.closed.is_set() and not thread.is_alive()


def test_fake_bend6_shared_aurora_session_slew_and_shutdown(monkeypatch):
    from sleeve_arm.domain import MotionIntent
    args = parse_args(["--sensor-config", str(EXAMPLE), "--robot", "aurora-fake", "--side", "right",
                       "--source-side", "right", "--imus", "fake", "--glove", "fake"])
    configs = load_configs(args)
    session = fake_session(profile=gr3_fake_profile())
    actual = factory.create_robot
    monkeypatch.setattr(factory, "create_robot", lambda backend, config, **kw:
                        actual(backend, config, session=session, **kw))
    glove = GloveRuntime(configs.sensors.glove, mode="fake", clock=session.clock)
    robot = RobotRuntime(args, configs, clock=session.clock)
    try:
        glove.validate_for_control()
        glove.start()
        glove.wait_ready()
        assert glove.reading.calibrated and glove.reading.synthetic
        robot.connect()
        robot.prepare()
        previous = session.fake_client.commands[-1]["right_hand"]
        for _ in range(10):
            session.clock.sleep(.01)
            robot.apply(MotionIntent(session.clock.monotonic(), shoulder_flexion_rad=.3), .01,
                        hand_targets=glove.targets())
            command = session.fake_client.commands[-1]
            assert set(command) == {"right_manipulator", "right_hand"}
            assert max(abs(a - b) for a, b in zip(previous, command["right_hand"])) <= .03 + 1e-10
            previous = command["right_hand"]
        assert session.fake_client.calls.count("get_instance") == 1
    finally:
        glove.close()
        robot.shutdown()
    assert session.fake_client.close_count == 1


@pytest.mark.parametrize("side", ["left", "right"])
def test_each_wire_channel_reaches_only_the_configured_sdk_slot(tmp_path, monkeypatch, side):
    # This verifies software routing through the real profile, not physical hand ordering.
    from sleeve_arm.config import load_phase3_config
    from sleeve_arm.runtime.configuration import ModelControlConfigs
    from tools.debug_glove import parse_args as debug_args
    args = debug_args(["--sensor-config", str(EXAMPLE), "--robot", "aurora-fake", "--side", side,
                       "--glove", "fake"])
    configs = ModelControlConfigs(sensors=load_sensor_config(EXAMPLE), phase3=load_phase3_config(),
                                  glove_mode="fake")
    session = fake_session(profile=gr3_fake_profile())
    actual = factory.create_robot
    monkeypatch.setattr(factory, "create_robot", lambda backend, config, **kw:
                        actual(backend, config, session=session, **kw))
    glove = runtime(tmp_path, clock=session.clock, mapping=configs.sensors.glove.bend6_mapping)
    robot = RobotRuntime(args, configs, clock=session.clock, parts=("hand",), print_fn=lambda _: None)
    # Distinct values detect a feedback permutation as well as a command permutation.
    opened = (.4, .5, .6, .7, .8, .9)
    sdk_opened = opened if side == "left" else (.5, .6, .7, .8, .4, .9)
    wire_slots = ((4, None, 3, 2, None, None, 1, 5, 0, None, None) if side == "left" else
                  (3, None, 2, 1, None, None, 0, 5, 4, None, None))
    session.fake_client.groups[f"{side}_hand"]["position"] = list(sdk_opened)
    glove.config = replace(glove.config, open_pose_rad=opened,
                           closed_pose_rad=tuple(value + .004 for value in opened))
    try:
        robot.connect()
        assert robot.feedback() == pytest.approx(dict(zip(HAND_JOINTS, opened)))
        assert robot.prepare(map_arm_intents=False)
        for raw_index, sdk_index in enumerate(wire_slots):
            session.clock.sleep(.01)
            values = [0.] * 11
            values[raw_index] = 1.
            glove.source._feed_bytes(packet(values))
            targets = glove.targets()
            robot.apply_hand(targets, .01)
            expected = list(sdk_opened)
            if sdk_index is not None:
                expected[sdk_index] += .004
            command = session.fake_client.commands[-1]
            assert set(command) == {f"{side}_hand"}
            assert command[f"{side}_hand"] == pytest.approx(expected)
            session.clock.sleep(.01)
            assert robot.feedback() == pytest.approx(targets)
        assert session.fake_client.calls.count("get_instance") == 1
    finally:
        glove.close()
        robot.shutdown()
    assert session.fake_client.close_count == 1


def test_right_hand_slot_fix_keeps_each_joints_bounds_and_left_layout():
    from sleeve_arm.robot.aurora_profile import load_aurora_profile
    profile = load_aurora_profile(ROOT / "configs/robot_aurora.yaml")
    profile.validate(execute=True)
    left, = profile.selected_groups(("left",), ("hand",))
    right, = profile.selected_groups(("right",), ("hand",))
    left_joints = {j.name: j for j in left.joints}
    assert {j.name: j.index for j in left.joints} == dict(zip(HAND_JOINTS, range(6)))
    assert {j.name: j.index for j in right.joints} == dict(zip(HAND_JOINTS, (4, 0, 1, 2, 3, 5)))
    for joint in right.joints:
        previous = left_joints[joint.name]
        assert joint.limits == replace(previous.limits, max_velocity=3., max_position_step=.03)
        assert (joint.sign, joint.zero) == (previous.sign, previous.zero)
        assert right.sdk_position_limits[joint.index] == left.sdk_position_limits[previous.index]
    assert right.sdk_position_limits[4] == (.12, 1.28)
    assert right.sdk_position_limits[:4] == ((.17, 1.78),) * 4
    # A large index angle is valid at slot 0, but the thumb must still obey its own bound.
    right.check_vector([1.70, .20, .20, .20, 1.20, .10])
    with pytest.raises(ValueError, match=r"right_hand\[4\].*outside limits"):
        right.check_vector([.20, .20, .20, .20, 1.70, .10])


def test_entry_points_from_isolated_copy_without_other_application_or_sdk(tmp_path):
    isolated = tmp_path / "standalone"
    for directory in ("sleeve_arm", "tools", "configs"):
        shutil.copytree(ROOT / directory, isolated / directory, ignore=shutil.ignore_patterns("__pycache__"))
    bootstrap = """
import importlib.abc, runpy, sys
class NoHardware(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('electronic_skin', 'electronic_skin_project', 'fourier_aurora_client', 'aurora', 'serial'):
            raise AssertionError('unexpected hardware/dependency: ' + fullname)
sys.meta_path.insert(0, NoHardware())
sys.argv = sys.argv[1:]
runpy.run_path(sys.argv[0], run_name='__main__')
"""
    for script, options in (
        ("debug_glove.py", ["--glove", "fake", "--robot", "aurora-fake"]),
        ("run_model_control.py", ["--robot", "aurora-fake", "--side", "right", "--source-side", "right",
                                  "--imus", "fake", "--glove", "fake", "--calibration-seconds", ".1"]),
        ("run_model_control.py", ["--print-only", "--imus", "fake", "--glove", "fake", "--calibration-seconds", ".1"]),
    ):
        result = subprocess.run(
            [sys.executable, "-I", "-c", bootstrap, str(isolated / "tools" / script), *options,
             "--sensor-config", str(isolated / "configs/sensors_bend6.example.yaml"), "--duration", ".05"],
            cwd=isolated, input="\n\n\n\n", capture_output=True, text=True, timeout=20,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert "Bend6" in result.stdout


@pytest.mark.parametrize("bad,reason", [
    (b"\xff,1,2,3,4,5,6,7,8,9,10;", "non-ASCII"),
    (b"1,,2,3,4,5,6,7,8,9,10;", "empty field"),
    (b"1,2;", "field_count=2"),
    (b"0,1,2,3,4,5,6,7,8,9,nan;", "invalid numeric token"),
    (b"0,1,2,3,4,5,6,7,8,9,1e999;", "non-finite"),
    (b"1" * 9000 + b";", "8192 bytes"),
])
def test_diagnostics_explain_rejection_and_preserve_strict_recovery(bad, reason):
    source = Bend6GloveSource("FAKE", clock=FakeClock())
    source._feed_bytes(b"sync;" + bad)
    stats = source.diagnostics()
    assert stats["bytes_received"] == len(b"sync;" + bad)
    assert stats["invalid"] == 1 and stats["valid"] == 0
    assert reason in stats["last_error"]
    assert len(stats["last_invalid_record"]) <= 160 and len(stats["last_chunk"]) <= 160
    assert stats["valid_age_s"] is None and source.latest() is None
    source._feed_bytes(packet())
    assert source.diagnostics()["valid"] == 1 and source.latest().raw == tuple(range(11))


def test_diagnostics_distinguish_silence_rejection_and_incomplete_record_without_refreshing_data():
    clock = FakeClock()
    source = Bend6GloveSource("FAKE", clock=clock)
    source._feed_bytes(b"sync;" + packet())
    reading, first = source.latest(), source.diagnostics()
    clock.sleep(.3)
    silent = source.diagnostics()
    assert silent["bytes_received"] == first["bytes_received"]
    assert silent["byte_age_s"] == silent["valid_age_s"] == pytest.approx(.3)
    assert silent["valid"] == 1 and silent["invalid"] == 0
    source._feed_bytes(b"bad;")
    rejected = source.diagnostics()
    assert rejected["bytes_received"] > silent["bytes_received"]
    assert rejected["byte_age_s"] == 0 and rejected["valid_age_s"] == pytest.approx(.3)
    assert rejected["valid"] == 1 and rejected["invalid"] == 1
    source._feed_bytes(b"1,2,3\r\n")
    partial = source.diagnostics()
    assert partial["pending_bytes"] == len(b"1,2,3\r\n")
    assert partial["invalid"] == 1 and partial["valid"] == 1
    assert source.latest() is reading


def test_serial_diagnostics_are_reset_on_restart():
    serials = [FakeSerial([b"sync;bad;" + packet()]), FakeSerial()]
    source = Bend6GloveSource("FAKE", serial_factory=lambda **_: serials.pop(0))
    try:
        source.start()
        assert source._serial.processed.wait(2.)
        assert source.diagnostics()["valid"] == source.diagnostics()["invalid"] == 1
        source.close()
        source.start()
        assert source._serial.processed.wait(2.)
        stats = source.diagnostics()
        assert stats["reader_alive"] and stats["bytes_received"] == 0
        assert stats["valid"] == stats["invalid"] == 0
        assert stats["last_error"] is stats["valid_age_s"] is stats["byte_age_s"] is None
    finally:
        source.close()


@pytest.mark.parametrize("options", [
    ["--execute"], ["--robot", "aurora-fake"], ["--glove", "fake"],
    ["--arm-forward-deg", "45"], ["--prepare-aurora-fsm", "--execute"],
])
def test_diagnostic_mode_rejects_execution_options(options):
    from tools import debug_glove
    with pytest.raises(SystemExit):
        debug_glove.parse_args(["--diagnose-serial", "--duration", "1", *options])


@pytest.mark.parametrize("failure", ["silence", "bad_packets", "reader_error"])
def test_diagnostic_cli_observes_stale_without_robot_or_targets_and_closes(tmp_path, monkeypatch, capsys, failure):
    from tools import debug_glove

    glove = runtime(tmp_path, calibrated=False)
    serial, thread = glove.source._serial, glove.source._thread
    monkeypatch.setattr(debug_glove, "time", glove.clock)
    monkeypatch.setattr(debug_glove, "GloveRuntime", lambda *a, **kw: glove)
    monkeypatch.setattr(glove, "start", lambda: glove.source._feed_bytes(packet()))
    for name in ("targets", "preview_targets", "wait_ready"):
        monkeypatch.setattr(glove, name, lambda *a, **kw: pytest.fail("diagnostics attempted readiness/targets"))
    monkeypatch.setattr(factory, "create_robot", lambda *a, **kw: pytest.fail("diagnostics connected robot"))
    sleep = glove.clock.sleep

    def tick(seconds):
        sleep(seconds)
        if failure == "bad_packets":
            glove.source._feed_bytes(b"\x1b[31m;1,2;")
        elif failure == "reader_error":
            glove.source._error = OSError("disconnected")

    monkeypatch.setattr(glove.clock, "sleep", tick)
    assert debug_glove.main(["--diagnose-serial", "--duration", "1"]) == (1 if failure == "reader_error" else 0)
    output = capsys.readouterr()
    assert "SERIAL DIAGNOSTICS ONLY" in output.out and "bytes_received" in output.out
    assert "\x1b" not in output.out  # Wire control characters must be escaped for the terminal.
    if failure == "reader_error":
        assert "disconnected" in output.err and "reader_error" in output.out
    else:
        assert "STALE" in output.out and not output.err
        assert "field_count=2" in output.out if failure == "bad_packets" else "'invalid': 0" in output.out
    assert glove.source is None and serial.closed.is_set() and not thread.is_alive()


def test_normal_preview_still_faults_on_stale_and_reports_serial_evidence(tmp_path, monkeypatch, capsys):
    from tools import debug_glove
    glove = runtime(tmp_path, calibrated=False)
    monkeypatch.setattr(debug_glove, "time", glove.clock)
    monkeypatch.setattr(debug_glove, "GloveRuntime", lambda *a, **kw: glove)
    monkeypatch.setattr(glove, "start", lambda: glove.source._feed_bytes(packet()))
    assert debug_glove.main(["--duration", "1"]) == 1
    error = capsys.readouterr().err
    assert "watchdog" in error and "bytes_received" in error and "'invalid': 0" in error
    assert glove.source is None
