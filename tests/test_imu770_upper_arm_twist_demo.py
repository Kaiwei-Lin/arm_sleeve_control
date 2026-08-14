from __future__ import annotations

import csv
import struct
import threading
import time
from collections import deque
from math import cos, radians, sin

import numpy as np
import pytest

import tools.demo_upper_arm_twist_imu770 as demo


def _checksum(data: bytes) -> bytes:
    ck1 = ck2 = 0
    for value in data:
        ck1 = (ck1 + value) & 0xFF
        ck2 = (ck2 + ck1) & 0xFF
    return bytes((ck1, ck2))


def _vector_tlv(data_id: int, *values: int) -> bytes:
    payload = struct.pack(f"<{len(values)}i", *values)
    return bytes((data_id, len(payload))) + payload


def _timestamp_tlv(data_id: int, value: int) -> bytes:
    payload = struct.pack("<I", value)
    return bytes((data_id, len(payload))) + payload


def make_imu770_frame(tid: int, *tlvs: bytes) -> bytes:
    message = b"".join(tlvs)
    body = struct.pack("<HB", tid, len(message)) + message
    return b"\x59\x53" + body + _checksum(body)


def axis_angle(axis: tuple[float, float, float], angle_deg: float) -> tuple[float, float, float, float]:
    unit = np.asarray(axis, dtype=float)
    unit /= np.linalg.norm(unit)
    half = radians(angle_deg) / 2.0
    return (cos(half), *(unit * sin(half)))


def sample_at(timestamp_ns: int, tid: int = 1) -> demo.Imu770Sample:
    return demo.Imu770Sample(
        host_timestamp_ns=timestamp_ns,
        tid=tid,
        device_timestamp_us=None,
        dataready_timestamp_us=None,
        accel_mps2=None,
        gyro_dps=None,
        euler_deg=None,
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
    )


def wait_until(predicate: object, timeout_s: float = 1.0) -> None:
    deadline = time.monotonic() + timeout_s
    while not predicate():  # type: ignore[operator]
        if time.monotonic() >= deadline:
            raise AssertionError("condition was not met before timeout")
        time.sleep(0.005)


def test_parser_decodes_quaternion_only_frame() -> None:
    parser = demo.Imu770FrameParser()
    raw = make_imu770_frame(
        7,
        _vector_tlv(0x41, 1_000_000, 0, 0, 0),
        _timestamp_tlv(0x51, 123_456),
        _timestamp_tlv(0x52, 123_450),
    )

    samples = parser.feed(raw, host_timestamp_ns=999)

    assert len(samples) == 1
    assert samples[0].quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert samples[0].device_timestamp_us == 123_456
    assert samples[0].dataready_timestamp_us == 123_450
    assert samples[0].host_timestamp_ns == 999
    assert samples[0].accel_mps2 is None
    assert samples[0].gyro_dps is None


def test_parser_buffers_fragmented_frame_and_decodes_all_fields() -> None:
    parser = demo.Imu770FrameParser()
    raw = make_imu770_frame(
        8,
        _vector_tlv(0x10, 1_000_000, -2_000_000, 3_000_000),
        _vector_tlv(0x20, 4_000_000, 5_000_000, -6_000_000),
        _vector_tlv(0x40, 10_000_000, 20_000_000, 30_000_000),
        _vector_tlv(0x41, 707_107, 707_107, 0, 0),
    )

    assert parser.feed(raw[:5]) == []
    samples = parser.feed(raw[5:], host_timestamp_ns=321)

    assert samples[0].accel_mps2 == (1.0, -2.0, 3.0)
    assert samples[0].gyro_dps == (4.0, 5.0, -6.0)
    assert samples[0].euler_deg == (10.0, 20.0, 30.0)
    assert samples[0].quaternion_wxyz == pytest.approx((0.707107, 0.707107, 0.0, 0.0))


def test_parser_decodes_concatenated_frames_and_skips_unknown_tlv() -> None:
    parser = demo.Imu770FrameParser()
    unknown = bytes((0x99, 3)) + b"abc"
    first = make_imu770_frame(9, unknown, _vector_tlv(0x41, 1_000_000, 0, 0, 0))
    second = make_imu770_frame(10, _vector_tlv(0x41, 0, 1_000_000, 0, 0))

    samples = parser.feed(b"noise" + first + second, host_timestamp_ns=456)

    assert [sample.tid for sample in samples] == [9, 10]
    assert parser.frame_count == 2
    assert parser.valid_frame_count == 2
    assert parser.tid_drop_count == 0


def test_parser_recovers_after_checksum_and_tlv_length_errors() -> None:
    parser = demo.Imu770FrameParser()
    bad_checksum = bytearray(make_imu770_frame(11, _vector_tlv(0x41, 1_000_000, 0, 0, 0)))
    bad_checksum[-1] ^= 0xFF
    bad_length = make_imu770_frame(12, bytes((0x41, 4)) + struct.pack("<i", 1_000_000))
    good = make_imu770_frame(14, _vector_tlv(0x41, 1_000_000, 0, 0, 0))

    samples = parser.feed(bytes(bad_checksum) + bad_length + good)

    assert [sample.tid for sample in samples] == [14]
    assert parser.frame_count == 3
    assert parser.valid_frame_count == 1
    assert parser.checksum_error_count == 1
    assert parser.parser_error_count == 1


def test_parser_counts_tid_gaps_with_60000_to_1_wrap() -> None:
    parser = demo.Imu770FrameParser()
    quaternion = _vector_tlv(0x41, 1_000_000, 0, 0, 0)

    parser.feed(
        make_imu770_frame(59_999, quaternion)
        + make_imu770_frame(1, quaternion)
        + make_imu770_frame(4, quaternion)
    )

    assert parser.tid_drop_count == 3


def test_parser_rejects_zero_norm_quaternion_as_invalid() -> None:
    parser = demo.Imu770FrameParser()
    raw = make_imu770_frame(15, _vector_tlv(0x41, 0, 0, 0, 0))

    assert parser.feed(raw) == []
    assert parser.frame_count == 1
    assert parser.valid_frame_count == 0
    assert parser.invalid_quaternion_count == 1


def test_parser_does_not_report_duplicate_tid_as_full_cycle_drop() -> None:
    parser = demo.Imu770FrameParser()
    quaternion = _vector_tlv(0x41, 1_000_000, 0, 0, 0)

    parser.feed(make_imu770_frame(20, quaternion) + make_imu770_frame(20, quaternion))

    assert parser.tid_drop_count == 0


def test_parser_treats_large_backward_tid_jump_as_device_restart() -> None:
    parser = demo.Imu770FrameParser()
    quaternion = _vector_tlv(0x41, 1_000_000, 0, 0, 0)

    parser.feed(make_imu770_frame(500, quaternion) + make_imu770_frame(1, quaternion))

    assert parser.tid_drop_count == 0
    assert parser.tid_reset_count == 1


@pytest.mark.parametrize(
    "quaternion",
    [(0.0, 0.0, 0.0, 0.0), (float("nan"), 0.0, 0.0, 0.0)],
)
def test_normalize_quaternion_rejects_invalid_input(quaternion: tuple[float, ...]) -> None:
    with pytest.raises(ValueError):
        demo.normalize_quaternion(quaternion)


def test_average_quaternions_aligns_opposite_signs() -> None:
    averaged = demo.average_quaternions([(1.0, 0.0, 0.0, 0.0), (-1.0, 0.0, 0.0, 0.0)])

    assert averaged == pytest.approx((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("angle_deg", [30.0, -45.0])
def test_world_estimator_recovers_signed_x_twist(angle_deg: float) -> None:
    estimator = demo.TwistEstimator(ema_alpha=1.0)
    estimator.calibrate([(1.0, 0.0, 0.0, 0.0)])

    result = estimator.update(axis_angle((1.0, 0.0, 0.0), angle_deg))

    assert result.raw_deg == pytest.approx(angle_deg)
    assert result.unwrapped_deg == pytest.approx(angle_deg)
    assert result.filtered_deg == pytest.approx(angle_deg)


def test_world_estimator_rejects_pure_y_swing_as_x_twist() -> None:
    estimator = demo.TwistEstimator(ema_alpha=1.0)
    estimator.calibrate([(1.0, 0.0, 0.0, 0.0)])

    result = estimator.update(axis_angle((0.0, 1.0, 0.0), 70.0))

    assert result.filtered_deg == pytest.approx(0.0)


def test_world_estimator_unwraps_across_negative_180_degrees() -> None:
    estimator = demo.TwistEstimator(ema_alpha=1.0)
    estimator.calibrate([(1.0, 0.0, 0.0, 0.0)])

    first = estimator.update(axis_angle((1.0, 0.0, 0.0), 179.0))
    second = estimator.update(axis_angle((1.0, 0.0, 0.0), 181.0))

    assert first.unwrapped_deg == pytest.approx(179.0)
    assert second.raw_deg == pytest.approx(-179.0)
    assert second.unwrapped_deg == pytest.approx(181.0)


def test_world_estimator_applies_ema_to_continuous_angle() -> None:
    estimator = demo.TwistEstimator(ema_alpha=0.5)
    estimator.calibrate([(1.0, 0.0, 0.0, 0.0)])

    estimator.update(axis_angle((1.0, 0.0, 0.0), 0.0))
    result = estimator.update(axis_angle((1.0, 0.0, 0.0), 100.0))

    assert result.filtered_deg == pytest.approx(50.0)


def test_relative_estimator_cancels_common_x_rotation() -> None:
    estimator = demo.RelativeTwistEstimator(ema_alpha=1.0)
    identity = (1.0, 0.0, 0.0, 0.0)
    estimator.calibrate([(identity, identity)])
    common_rotation = axis_angle((1.0, 0.0, 0.0), 35.0)

    result = estimator.update(common_rotation, common_rotation)

    assert result.filtered_deg == pytest.approx(0.0)


def test_estimators_require_calibration() -> None:
    with pytest.raises(RuntimeError, match="calibrat"):
        demo.TwistEstimator(ema_alpha=1.0).update((1.0, 0.0, 0.0, 0.0))


@pytest.mark.parametrize("alpha", [0.0, -0.1, 1.1])
def test_estimator_rejects_invalid_ema_alpha(alpha: float) -> None:
    with pytest.raises(ValueError, match="ema_alpha"):
        demo.TwistEstimator(ema_alpha=alpha)


def test_synchronizer_chooses_nearest_pair_and_consumes_samples_once() -> None:
    synchronizer = demo.SampleSynchronizer(max_gap_ns=20_000_000)
    synchronizer.add_upper(sample_at(100_000_000, 1))
    synchronizer.add_upper(sample_at(130_000_000, 2))
    synchronizer.add_forearm(sample_at(125_000_000, 3))

    upper, forearm, gap_ns = synchronizer.pop_pair()  # type: ignore[misc]

    assert upper.host_timestamp_ns == 130_000_000
    assert forearm.host_timestamp_ns == 125_000_000
    assert gap_ns == 5_000_000
    assert synchronizer.pop_pair() is None


def test_synchronizer_rejects_stale_sample_then_recovers() -> None:
    synchronizer = demo.SampleSynchronizer(max_gap_ns=20_000_000)
    synchronizer.add_upper(sample_at(100_000_000, 1))
    synchronizer.add_forearm(sample_at(130_000_000, 2))

    assert synchronizer.pop_pair() is None
    synchronizer.add_upper(sample_at(135_000_000, 3))

    assert synchronizer.pop_pair()[2] == 5_000_000  # type: ignore[index]


def test_synchronizer_bounds_each_input_queue() -> None:
    synchronizer = demo.SampleSynchronizer(max_gap_ns=20_000_000, max_queue=2)
    synchronizer.add_upper(sample_at(1_000_000, 1))
    synchronizer.add_upper(sample_at(2_000_000, 2))
    synchronizer.add_upper(sample_at(3_000_000, 3))
    synchronizer.add_forearm(sample_at(2_000_000, 4))

    upper, _, _ = synchronizer.pop_pair()  # type: ignore[misc]

    assert upper.host_timestamp_ns == 2_000_000
    assert synchronizer.rejected_samples == 1


def test_synchronizer_clear_discards_precalibration_backlog() -> None:
    synchronizer = demo.SampleSynchronizer(max_gap_ns=20_000_000)
    synchronizer.add_upper(sample_at(1_000_000, 1))
    synchronizer.add_forearm(sample_at(1_000_000, 2))

    assert synchronizer.clear() == 2
    assert synchronizer.pop_pair() is None
    assert synchronizer.rejected_samples == 0


class FakeSerial:
    def __init__(self, chunks: list[bytes] | None = None, read_error: BaseException | None = None) -> None:
        self.chunks = deque(chunks or [])
        self.read_error = read_error
        self.is_open = True
        self.closed = threading.Event()

    @property
    def in_waiting(self) -> int:
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size: int) -> bytes:
        if self.read_error is not None:
            error, self.read_error = self.read_error, None
            raise error
        if self.chunks:
            return self.chunks.popleft()
        time.sleep(0.002)
        return b""

    def write(self, data: bytes) -> int:
        raise AssertionError(f"read-only reader attempted to write {data!r}")

    def close(self) -> None:
        self.is_open = False
        self.closed.set()


def test_serial_reader_opens_8n1_and_delivers_real_parsed_sample() -> None:
    raw = make_imu770_frame(21, _vector_tlv(0x41, 1_000_000, 0, 0, 0))
    serial_port = FakeSerial([raw])
    opened_with: dict[str, object] = {}
    samples: list[demo.Imu770Sample] = []

    def factory(**kwargs: object) -> FakeSerial:
        opened_with.update(kwargs)
        return serial_port

    reader = demo.Imu770SerialReader("COM5", 460800, 0.05, samples.append, serial_factory=factory)
    reader.start()
    wait_until(lambda: len(samples) == 1)
    reader.close()

    assert opened_with == {
        "port": "COM5",
        "baudrate": 460800,
        "timeout": 0.05,
        "bytesize": 8,
        "parity": "N",
        "stopbits": 1,
    }
    assert samples[0].tid == 21
    assert reader.stats.valid_frames == 1
    assert serial_port.closed.is_set()


def test_serial_reader_surfaces_read_failure_and_closes_promptly() -> None:
    serial_port = FakeSerial(read_error=OSError("device disconnected"))
    reader = demo.Imu770SerialReader(
        "COM6", 460800, 0.05, lambda sample: None, serial_factory=lambda **kwargs: serial_port
    )

    reader.start()
    wait_until(lambda: reader.error is not None)
    reader.close()

    assert isinstance(reader.error, OSError)
    assert "disconnected" in str(reader.error)
    assert serial_port.closed.is_set()


def test_serial_reader_reports_rejected_quaternion_frames() -> None:
    raw = make_imu770_frame(22, _vector_tlv(0x41, 0, 0, 0, 0))
    serial_port = FakeSerial([raw])
    samples: list[demo.Imu770Sample] = []
    reader = demo.Imu770SerialReader(
        "COM7", 460800, 0.05, samples.append, serial_factory=lambda **kwargs: serial_port
    )

    reader.start()
    wait_until(lambda: reader.stats.frames == 1)
    reader.close()

    assert samples == []
    assert reader.stats.valid_frames == 0
    assert reader.stats.invalid_quaternions == 1


def test_cli_uses_hardware_defaults_and_has_no_simulation_mode() -> None:
    parser = demo.build_argument_parser()

    args = parser.parse_args(["--upper-port", "COM5", "--forearm-port", "COM6"])

    assert args.baudrate == 460800
    assert args.timeout == pytest.approx(0.1)
    assert args.max_sync_ms == pytest.approx(20.0)
    assert args.ema_alpha == pytest.approx(0.35)
    assert "--simulate" not in parser.format_help()


def test_cli_rejects_same_port_for_both_sensors() -> None:
    parser = demo.build_argument_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(["--upper-port", "COM5", "--forearm-port", "com5"])


@pytest.mark.parametrize(
    "option,value",
    [
        ("--baudrate", "0"),
        ("--timeout", "0"),
        ("--calibration-seconds", "-1"),
        ("--print-hz", "0"),
        ("--max-sync-ms", "0"),
        ("--ema-alpha", "1.1"),
    ],
)
def test_cli_rejects_invalid_numeric_options(option: str, value: str) -> None:
    parser = demo.build_argument_parser()

    with pytest.raises(SystemExit):
        parser.parse_args(["--upper-port", "COM5", "--forearm-port", "COM6", option, value])


def test_csv_recorder_writes_raw_and_both_estimator_results(tmp_path: object) -> None:
    path = tmp_path / "twist.csv"  # type: ignore[operator]
    upper = sample_at(100_000_000, 11)
    forearm = sample_at(104_000_000, 12)
    world = demo.TwistResult(10.0, 370.0, 369.0)
    relative = demo.TwistResult(-5.0, -5.0, -4.0)

    with demo.CsvRecorder(path) as recorder:
        recorder.write(upper, forearm, 4_000_000, world, relative)

    with path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert rows[0]["upper_tid"] == "11"
    assert rows[0]["upper_qw"] == "1.0"
    assert rows[0]["world_filtered_deg"] == "369.0"
    assert rows[0]["relative_filtered_deg"] == "-4.0"
    assert rows[0]["sync_gap_ms"] == "4.0"


class SlowDisconnectSerial(FakeSerial):
    def read(self, size: int) -> bytes:
        time.sleep(0.001)
        if self.chunks:
            return self.chunks.popleft()
        raise OSError("test stream ended")


def test_run_calibrates_records_live_pairs_and_closes_both_ports(tmp_path: object, capsys: object) -> None:
    path = tmp_path / "live.csv"  # type: ignore[operator]
    upper_frames = [
        make_imu770_frame(tid, _vector_tlv(0x41, 1_000_000, 0, 0, 0)) for tid in range(1, 81)
    ]
    forearm_frames = [
        make_imu770_frame(tid, _vector_tlv(0x41, 1_000_000, 0, 0, 0)) for tid in range(1, 81)
    ]
    serial_ports = {
        "COM5": SlowDisconnectSerial(upper_frames),
        "COM6": SlowDisconnectSerial(forearm_frames),
    }

    def factory(**kwargs: object) -> SlowDisconnectSerial:
        return serial_ports[str(kwargs["port"])]

    args = demo.build_argument_parser().parse_args(
        [
            "--upper-port",
            "COM5",
            "--forearm-port",
            "COM6",
            "--startup-timeout",
            "1",
            "--calibration-seconds",
            "0.10",
            "--print-hz",
            "100",
            "--csv",
            str(path),
        ]
    )

    result = demo.run(args, serial_factory=factory, input_fn=lambda prompt: "")

    assert result == 1
    assert all(port.closed.is_set() for port in serial_ports.values())
    with path.open(newline="", encoding="utf-8") as csv_file:
        rows = list(csv.DictReader(csv_file))
    assert rows
    captured = capsys.readouterr()  # type: ignore[attr-defined]
    assert "Calibration complete" in captured.out
    assert "sync_rejected=" in captured.out
    assert "checksum=" in captured.out
    assert "malformed=" in captured.out
    assert "invalid_quaternion=" in captured.out
    assert "test stream ended" in captured.err


class PhasedQuaternionSerial(FakeSerial):
    def __init__(self, sensor: str, calibration_started: threading.Event, phase_time: list[float]) -> None:
        super().__init__()
        self.sensor = sensor
        self.calibration_started = calibration_started
        self.phase_time = phase_time
        self.tid = 0

    @property
    def in_waiting(self) -> int:
        return 64

    def read(self, size: int) -> bytes:
        time.sleep(0.0005)
        if not self.calibration_started.is_set():
            angle = 90.0 if self.sensor == "upper" else 0.0
        else:
            elapsed = time.monotonic() - self.phase_time[0]
            if elapsed < 0.20:
                angle = 0.0
            elif elapsed < 0.40:
                angle = 30.0 if self.sensor == "upper" else 0.0
            else:
                raise OSError("phased test complete")
        self.tid = self.tid % 60_000 + 1
        quaternion = axis_angle((1.0, 0.0, 0.0), angle)
        raw_quaternion = tuple(round(value * 1_000_000) for value in quaternion)
        return make_imu770_frame(self.tid, _vector_tlv(0x41, *raw_quaternion))


def test_run_discards_samples_captured_before_calibration_prompt_returns(
    tmp_path: object, capsys: object
) -> None:
    path = tmp_path / "post_prompt.csv"  # type: ignore[operator]
    calibration_started = threading.Event()
    phase_time = [0.0]
    serial_ports = {
        "COM5": PhasedQuaternionSerial("upper", calibration_started, phase_time),
        "COM6": PhasedQuaternionSerial("forearm", calibration_started, phase_time),
    }

    def factory(**kwargs: object) -> PhasedQuaternionSerial:
        return serial_ports[str(kwargs["port"])]

    def confirm_zero_pose(prompt: str) -> str:
        time.sleep(0.10)
        phase_time[0] = time.monotonic()
        calibration_started.set()
        return ""

    args = demo.build_argument_parser().parse_args(
        [
            "--upper-port",
            "COM5",
            "--forearm-port",
            "COM6",
            "--startup-timeout",
            "1",
            "--calibration-seconds",
            "0.10",
            "--print-hz",
            "100",
            "--ema-alpha",
            "1",
            "--csv",
            str(path),
        ]
    )

    result = demo.run(args, serial_factory=factory, input_fn=confirm_zero_pose)

    assert result == 1
    with path.open(newline="", encoding="utf-8") as csv_file:
        world_angles = [float(row["world_filtered_deg"]) for row in csv.DictReader(csv_file)]
    assert any(angle == pytest.approx(30.0, abs=0.2) for angle in world_angles)
    capsys.readouterr()  # type: ignore[attr-defined]
