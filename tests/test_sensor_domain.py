from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from sleeve_arm.domain import ImuFrame, SensorSample, SleeveFrame


def imu(timestamp: float) -> ImuFrame:
    return ImuFrame(timestamp, 0, 0, 9.81, 0, 0, 0)


def test_sleeve_frame_preserves_all_channels_and_is_immutable() -> None:
    frame = SleeveFrame(1.0, range(11))
    assert frame.channels == tuple(float(value) for value in range(11))
    with pytest.raises(FrozenInstanceError):
        frame.channels = ()  # type: ignore[misc]


def test_invalid_timestamp_is_rejected() -> None:
    with pytest.raises(ValueError, match="finite"):
        SleeveFrame(float("nan"), (1,))


@pytest.mark.parametrize("imu1,imu2", [(None, None), (imu(1.0), None), (imu(1.0), imu(1.0))])
def test_optional_imu_combinations(imu1: ImuFrame | None, imu2: ImuFrame | None) -> None:
    sleeve = SleeveFrame(1.0, (1, 2))
    sample = SensorSample(1.0, sleeve, imu1, imu2)
    assert sample.imu1 is imu1
    assert sample.imu2 is imu2
