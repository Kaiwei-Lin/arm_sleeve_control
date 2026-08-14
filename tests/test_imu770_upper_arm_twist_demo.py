from __future__ import annotations

import struct

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
