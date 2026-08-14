from __future__ import annotations

import struct

import pytest

from tools.read_wt901pwifi import (
    FRAME_HEADER,
    FRAME_SIZE,
    WT901PStreamParser,
    decode_frame,
)


def make_frame(
    *,
    accel: tuple[int, int, int] = (2048, -4096, 16384),
    gyro: tuple[int, int, int] = (164, -328, 0),
    mag: tuple[int, int, int] = (1024, -512, 256),
    angles: tuple[int, int, int] = (16384, -8192, 4096),
    temperature: int = 2534,
    battery: int = 387,
    rssi: int = -42,
    version: int = 0x1234,
) -> bytes:
    raw = bytearray(FRAME_HEADER)
    raw.extend(b"00001234")
    raw.extend(bytes((26, 8, 14, 12, 34, 56)))
    raw.extend(struct.pack("<H", 789))
    raw.extend(
        struct.pack(
            "<15hH",
            *accel,
            *gyro,
            *mag,
            *angles,
            temperature,
            battery,
            rssi,
            version,
        )
    )
    raw.extend(b"\r\n")
    assert len(raw) == FRAME_SIZE
    return bytes(raw)


def test_decode_frame_applies_vendor_units() -> None:
    frame = decode_frame(make_frame())

    assert frame.device_id == "00001234"
    assert frame.device_time.isoformat(timespec="milliseconds") == "2026-08-14T12:34:56.789"
    assert frame.accel_g == pytest.approx((1.0, -2.0, 8.0))
    assert frame.gyro_dps == pytest.approx((10.009765625, -20.01953125, 0.0))
    assert frame.mag_ut == pytest.approx((100.0, -50.0, 25.0))
    assert frame.euler_deg == pytest.approx((90.0, -45.0, 22.5))
    assert frame.temperature_c == pytest.approx(25.34)
    assert frame.battery_v == pytest.approx(3.87)
    assert frame.rssi_dbm == -42
    assert frame.version == 0x1234


def test_stream_parser_handles_split_and_concatenated_frames() -> None:
    parser = WT901PStreamParser()
    raw = make_frame()

    assert parser.feed(raw[:17]) == []
    frames = parser.feed(raw[17:] + raw)

    assert [item.device_id for item in frames] == ["00001234", "00001234"]


def test_stream_parser_recovers_from_noise_and_bad_terminator() -> None:
    parser = WT901PStreamParser()
    bad = bytearray(make_frame())
    bad[-1] = 0

    frames = parser.feed(b"noise" + bytes(bad) + make_frame())

    assert len(frames) == 1
    assert frames[0].device_id == "00001234"
    assert parser.invalid_frames >= 1
