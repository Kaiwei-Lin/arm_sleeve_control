from __future__ import annotations

import struct

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
    decoded = parser.feed(frame)
    assert len(decoded) == 1
    assert decoded[0].accel_x == 1.0
    assert decoded[0].gyro_z == 6.0
    assert decoded[0].quat_w == 1.0


def test_fake_sources_lifecycle() -> None:
    sources = (FakeSleeveSource(), FakeImuSource())
    for source in sources:
        source.start()
        assert source.latest() is not None
        source.close()
        assert not source.running
