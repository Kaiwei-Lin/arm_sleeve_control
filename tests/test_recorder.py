from __future__ import annotations

import csv

import yaml

from sleeve_arm.domain import ImuFrame, SensorSample, SleeveFrame
from sleeve_arm.recording import SensorRecorder


def test_recorder_fixed_schema_optional_imu_and_metadata(tmp_path) -> None:
    sleeve = SleeveFrame(1.0, (10, 20, 30))
    imu = ImuFrame(1.0, 1, 2, 3, 4, 5, 6)
    recorder = SensorRecorder(tmp_path, 3, {"imu1": {"enabled": True}, "imu2": {"enabled": False}}, "session")
    recorder.start()
    recorder.write(SensorSample(1.0, sleeve, imu1=imu))
    recorder.close()
    with recorder.csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]["sleeve_ch_2"] == "30.0"
    assert rows[0]["imu1_ax"] == "1.0"
    assert rows[0]["imu2_ax"] == ""
    assert recorder.samples_written == 1
    metadata = yaml.safe_load(recorder.metadata_path.read_text(encoding="utf-8"))
    assert metadata["channel_count"] == 3
    assert metadata["imu2"]["enabled"] is False
