from __future__ import annotations

from sleeve_arm.domain import ImuFrame, SleeveFrame
from sleeve_arm.sync import SensorSynchronizer


def imu(timestamp: float) -> ImuFrame:
    return ImuFrame(timestamp, 0, 0, 0, 0, 0, 0)


def sleeve(timestamp: float) -> SleeveFrame:
    return SleeveFrame(timestamp, (1, 2, 3))


def test_close_frame_matches_and_nearest_wins() -> None:
    sync = SensorSynchronizer(20, 500, imu1_enabled=True)
    sync.add_imu1(imu(0.990))
    sync.add_imu1(imu(1.003))
    sample = sync.synchronize(sleeve(1.0))
    assert sample.imu1 is not None and sample.imu1.timestamp == 1.003
    assert sync.stats.imu1_matched == 1


def test_outside_delta_is_none_not_stale_data() -> None:
    sync = SensorSynchronizer(20, 500, imu1_enabled=True)
    sync.add_imu1(imu(0.9))
    assert sync.synchronize(sleeve(1.0)).imu1 is None
    assert sync.stats.imu1_missed == 1


def test_disabled_imus_are_optional() -> None:
    sample = SensorSynchronizer().synchronize(sleeve(1.0))
    assert sample.imu1 is None and sample.imu2 is None


def test_old_buffer_is_pruned() -> None:
    sync = SensorSynchronizer(20, 100, imu1_enabled=True)
    sync.add_imu1(imu(1.0))
    sync.add_imu1(imu(1.2))
    assert sync.buffer_sizes() == (1, 0)


def test_repeated_latest_frame_is_not_buffered_twice() -> None:
    sync = SensorSynchronizer(20, 500, imu1_enabled=True)
    frame = imu(1.0)
    sync.add_imu1(frame)
    sync.add_imu1(frame)
    assert sync.buffer_sizes() == (1, 0)
