from __future__ import annotations

import struct
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
