from __future__ import annotations

import struct

import sleeve_arm.sources.fake_imu as fake_imu_module
import sleeve_arm.sources.fake_sleeve as fake_sleeve_module
from sleeve_arm.sources import FakeImuSource, FakeSleeveSource, Imu770Parser, SerialSleeveSource


def tlv(data_id: int, values: tuple[int, ...]) -> bytes:
    payload = struct.pack(f"<{len(values)}i", *values)
    return bytes((data_id, len(payload))) + payload


def test_sleeve_parser_keeps_all_eleven_fields() -> None:
    values = SerialSleeveSource.parse_record(",".join(str(value) for value in range(11)))
    assert values == tuple(float(value) for value in range(11))


def test_imu770_parser_decodes_verified_protocol() -> None:
    parser = Imu770Parser()
    message = tlv(0x10, (1_000_000, 2_000_000, 3_000_000)) + tlv(0x20, (4_000_000, 5_000_000, 6_000_000)) + tlv(0x41, (1_000_000, 0, 0, 0))
    body = struct.pack("<H", 7) + bytes((len(message),)) + message
    frame = b"\x59\x53" + body + bytes(parser.checksum(body))
    decoded = parser.feed(frame, host_timestamp_ns=123_456_789)
    assert len(decoded) == 1
    assert decoded[0].accel_x == 1.0
    assert decoded[0].gyro_z == 6.0
    assert decoded[0].quat_w == 1.0
    assert decoded[0].host_timestamp_ns == 123_456_789


def test_fake_sources_lifecycle() -> None:
    sources = (FakeSleeveSource(), FakeImuSource())
    for source in sources:
        source.start()
        assert source.latest() is not None
        source.close()
        assert not source.running


def test_fake_drain_preserves_every_generated_sample(monkeypatch) -> None:
    clock = {"value": 10.0}
    monkeypatch.setattr(fake_sleeve_module.time, "monotonic", lambda: clock["value"])
    monkeypatch.setattr(fake_imu_module.time, "monotonic", lambda: clock["value"])
    sleeve = FakeSleeveSource(frequency_hz=100.0)
    imu = FakeImuSource(frequency_hz=100.0)
    sleeve.start()
    imu.start()

    clock["value"] = 10.055
    sleeve_frames = sleeve.drain()
    imu_frames = imu.drain()

    assert [frame.sequence_id for frame in sleeve_frames] == [1, 2, 3, 4, 5, 6]
    assert [frame.sequence_id for frame in imu_frames] == [1, 2, 3, 4, 5, 6]
    assert [frame.host_timestamp_ns for frame in sleeve_frames] == [
        10_000_000_000,
        10_010_000_000,
        10_020_000_000,
        10_030_000_000,
        10_040_000_000,
        10_050_000_000,
    ]
    assert sleeve.drain() == ()
    assert imu.drain() == ()
