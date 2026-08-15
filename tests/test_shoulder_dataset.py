from __future__ import annotations

import csv
import json
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
import yaml

from sleeve_arm.domain import ImuFrame, SleeveFrame
from sleeve_arm.estimation import (
    quaternion_from_axis_angle,
    quaternion_multiply,
    quaternion_slerp,
)
from sleeve_arm.shoulder_alignment import ShoulderAligner, UniformTimeline
from sleeve_arm.shoulder_collection import (
    ShoulderDatasetCollector,
    _RepetitionEvent,
    _repetition_state_at,
    fake_sources,
)
from sleeve_arm.shoulder_dataset_config import (
    MOTIONS,
    BodyFrameConfig,
    DirectionLabelConfig,
    load_shoulder_dataset_config,
)
from sleeve_arm.shoulder_kinematics import (
    neutral_relative_quaternion,
    shoulder_angles,
    shoulder_quaternion,
)


ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = ROOT / "configs" / "shoulder_dataset.yaml"
IDENTITY = (1.0, 0.0, 0.0, 0.0)
BODY = BodyFrameConfig(
    forward_axis=(1.0, 0.0, 0.0),
    lateral_axis=(0.0, 1.0, 0.0),
    up_axis=(0.0, 0.0, 1.0),
    upper_arm_axis_neutral=(0.0, 0.0, -1.0),
    side="right",
)
LABEL = DirectionLabelConfig(min_amplitude_deg=5.0, max_direction_error_deg=30.0)


def rotation(axis: tuple[float, float, float], degrees: float) -> np.ndarray:
    return quaternion_from_axis_angle(axis, math.radians(degrees))


def action_quaternion(motion: str, degrees: float) -> np.ndarray:
    if motion == "forward":
        return rotation((0.0, 1.0, 0.0), -degrees)
    if motion == "backward":
        return rotation((0.0, 1.0, 0.0), degrees)
    if motion == "lateral":
        return rotation((1.0, 0.0, 0.0), degrees)
    raise ValueError(motion)


def imu(timestamp_ns: int, quaternion=IDENTITY) -> ImuFrame:
    return ImuFrame(
        timestamp_ns / 1e9,
        0.0,
        0.0,
        9.81,
        0.0,
        0.0,
        0.0,
        *quaternion,
        host_timestamp_ns=timestamp_ns,
    )


def sleeve(timestamp_ns: int) -> SleeveFrame:
    return SleeveFrame(
        timestamp_ns / 1e9,
        tuple(float(index) for index in range(11)),
        host_timestamp_ns=timestamp_ns,
    )


def test_neutral_is_identity_and_zero_angles() -> None:
    q_zero = neutral_relative_quaternion(((IDENTITY, IDENTITY), (IDENTITY, IDENTITY)))
    shoulder = shoulder_quaternion(IDENTITY, IDENTITY, q_zero)
    angles = shoulder_angles(shoulder, BODY, LABEL)

    assert shoulder == pytest.approx(IDENTITY)
    assert angles.signed_forward_backward_deg == pytest.approx(0.0)
    assert angles.lateral_deg == pytest.approx(0.0)
    assert angles.amplitude_deg == pytest.approx(0.0)
    assert angles.derived_direction == "neutral"
    assert not angles.direction_valid


def test_whole_body_rotation_cancels() -> None:
    whole_body = rotation((0.2, 0.5, 0.8), 73.0)
    shoulder = shoulder_quaternion(whole_body, whole_body, IDENTITY)
    assert abs(float(np.dot(shoulder, IDENTITY))) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("motion", "degrees", "expected_fb", "expected_lateral"),
    (
        ("forward", 30.0, 30.0, 0.0),
        ("backward", 20.0, -20.0, 0.0),
        ("lateral", 45.0, 0.0, 45.0),
    ),
)
def test_canonical_motion_angles_and_direction(
    motion: str,
    degrees: float,
    expected_fb: float,
    expected_lateral: float,
) -> None:
    quaternion = action_quaternion(motion, degrees)
    angles = shoulder_angles(quaternion, BODY, LABEL)

    assert angles.signed_forward_backward_deg == pytest.approx(expected_fb)
    assert angles.lateral_deg == pytest.approx(expected_lateral)
    assert angles.amplitude_deg == pytest.approx(degrees)
    assert angles.derived_direction == motion
    assert angles.direction_confidence == pytest.approx(1.0)
    assert angles.direction_valid


def test_crossing_neutral_is_continuous() -> None:
    requested = (10.0, 5.0, 2.0, 0.0, -2.0, -5.0, -10.0)
    actual = [
        shoulder_angles(
            action_quaternion("forward" if degrees >= 0 else "backward", abs(degrees)),
            BODY,
            LABEL,
        ).signed_forward_backward_deg
        for degrees in requested
    ]
    assert actual == pytest.approx(requested)


def test_quaternion_sign_does_not_change_orientation_or_angles() -> None:
    quaternion = action_quaternion("forward", 30.0)
    positive = shoulder_angles(quaternion, BODY, LABEL)
    negative = shoulder_angles(-quaternion, BODY, LABEL)
    assert positive == negative


def test_slerp_midpoint_is_45_degrees_and_handles_sign() -> None:
    ninety = rotation((1.0, 0.0, 0.0), 90.0)
    midpoint = quaternion_slerp(IDENTITY, -ninety, 0.5)
    angles = shoulder_angles(midpoint, BODY, LABEL)
    assert angles.lateral_deg == pytest.approx(45.0)
    assert angles.amplitude_deg == pytest.approx(45.0)


def test_aligned_imu_and_shoulder_quaternions_keep_sign_continuity() -> None:
    aligner = ShoulderAligner(1000.0, 20.0, "linear")
    quaternion = action_quaternion("forward", 30.0)
    for timestamp_ns in (0, 10_000_000, 20_000_000):
        aligner.add_flex([sleeve(timestamp_ns)])
        aligner.add_torso([imu(timestamp_ns, IDENTITY)])
    aligner.add_arm([imu(0, quaternion)])
    aligner.add_arm([imu(10_000_000, -quaternion)])
    aligner.add_arm([imu(20_000_000, quaternion)])

    aligned = [aligner.align(timestamp_ns) for timestamp_ns in (0, 10_000_000, 20_000_000)]
    arm_quaternions = [np.asarray(frame.arm.value) for frame in aligned]
    shoulder_quaternions = []
    previous = None
    for frame in aligned:
        current = shoulder_quaternion(frame.torso.value, frame.arm.value, IDENTITY, previous)
        shoulder_quaternions.append(current)
        previous = current

    assert all(
        float(np.dot(left, right)) > 0.0
        for left, right in zip(arm_quaternions, arm_quaternions[1:])
    )
    assert all(
        float(np.dot(left, right)) > 0.0
        for left, right in zip(shoulder_quaternions, shoulder_quaternions[1:])
    )


def test_neutral_mounting_offset_is_removed_on_the_right() -> None:
    torso = rotation((0.0, 0.0, 1.0), 37.0)
    mounting_offset = rotation((1.0, 0.0, 0.0), 23.0)
    expected_shoulder = action_quaternion("forward", 30.0)
    arm = quaternion_multiply(
        quaternion_multiply(torso, expected_shoulder),
        mounting_offset,
    )
    actual = shoulder_quaternion(torso, arm, mounting_offset)
    assert abs(float(np.dot(actual, expected_shoulder))) == pytest.approx(1.0)


def test_small_and_diagonal_actions_are_not_forced_valid() -> None:
    small = shoulder_angles(action_quaternion("forward", 2.0), BODY, LABEL)
    assert small.derived_direction == "neutral"
    assert not small.direction_valid

    diagonal_axis = (-1.0, 1.0, 0.0)
    diagonal = shoulder_angles(rotation(diagonal_axis, 30.0), BODY, LABEL)
    assert diagonal.derived_direction in MOTIONS
    assert diagonal.direction_confidence == pytest.approx(math.sqrt(0.5))
    assert not diagonal.direction_valid


def test_different_source_rates_align_to_configured_output_rate() -> None:
    aligner = ShoulderAligner(1500.0, 20.0, "linear")
    for index in range(37):
        aligner.add_flex([sleeve(round(index * 1e9 / 30.0))])
    for index in range(73):
        aligner.add_torso([imu(round(index * 1e9 / 60.0))])
    for index in range(121):
        aligner.add_arm([imu(round(index * 1e9 / 100.0))])

    start_ns = 100_000_000
    timeline = UniformTimeline(60.0, start_ns)
    targets = timeline.due(start_ns + 1_000_000_000 - 1)
    aligned = [aligner.align(timestamp_ns) for _, timestamp_ns in targets]

    assert len(aligned) == 60
    assert all(frame.frame_valid for frame in aligned)
    assert all(frame.flex.skew_ms is not None and frame.flex.skew_ms <= 20.0 for frame in aligned)


def test_dropout_marks_frame_invalid_instead_of_reusing_old_data() -> None:
    aligner = ShoulderAligner(1000.0, 20.0, "linear")
    aligner.add_flex([sleeve(0), sleeve(100_000_000), sleeve(200_000_000)])
    aligner.add_torso([imu(490_000_000), imu(510_000_000)])
    aligner.add_arm([imu(490_000_000), imu(510_000_000)])

    aligned = aligner.align(500_000_000)

    assert not aligned.flex.valid
    assert aligned.flex.skew_ms == pytest.approx(300.0)
    assert not aligned.frame_valid


def test_repetition_events_label_target_time_not_delayed_processing_time() -> None:
    events = [
        _RepetitionEvent(100, 1, False),
        _RepetitionEvent(200, 1, True),
        _RepetitionEvent(500, 1, False),
        _RepetitionEvent(700, 2, True),
    ]

    assert not _repetition_state_at(events, 199).active
    assert _repetition_state_at(events, 200).active
    assert _repetition_state_at(events, 499).repetition == 1
    assert not _repetition_state_at(events, 500).active
    assert _repetition_state_at(events, 700).repetition == 2


def write_dataset_config(tmp_path: Path, channels: object) -> Path:
    raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    raw["sensor_config"] = str((ROOT / "configs" / "sensors.yaml").resolve())
    raw["flex"]["channels"] = channels
    path = tmp_path / "shoulder.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_flex_channel_mapping_preserves_configured_order(tmp_path: Path) -> None:
    config = load_shoulder_dataset_config(write_dataset_config(tmp_path, [7, 2, 4]))
    assert config.flex.channels == (7, 2, 4)


@pytest.mark.parametrize("channels", ([2, 4], [2, 4, 11], [2, 2, 4], [2, 4, 7.5]))
def test_flex_channel_mapping_requires_three_distinct_legal_indices(
    tmp_path: Path,
    channels: object,
) -> None:
    with pytest.raises(ValueError, match="flex"):
        load_shoulder_dataset_config(write_dataset_config(tmp_path, channels))


def test_fake_session_writes_raw_aligned_training_and_metadata(tmp_path: Path) -> None:
    config = load_shoulder_dataset_config(CONFIG_PATH)
    config = replace(
        config,
        collection=replace(
            config.collection,
            output_hz=60.0,
            flush_interval_s=0.05,
            poll_interval_ms=1.0,
        ),
        calibration=replace(
            config.calibration,
            neutral_duration_s=0.15,
            countdown_s=0,
            startup_timeout_s=1.0,
        ),
    )
    fake = fake_sources("forward", 11)
    collector = ShoulderDatasetCollector(
        config,
        "forward",
        tmp_path,
        sleeve_source=fake[0],
        torso_source=fake[1],
        arm_source=fake[2],
        print_fn=lambda _: None,
        on_repetition_start=fake[3],
    )

    statistics = collector.run(
        duration_s=0.3,
        initial_repetition=7,
        countdown=False,
    )

    expected_files = {
        "metadata.json",
        "raw_flex.csv",
        "raw_torso_imu.csv",
        "raw_upper_arm_imu.csv",
        "aligned.csv",
        "training.csv",
    }
    assert {path.name for path in statistics.session_dir.iterdir()} == expected_files

    with (statistics.session_dir / "raw_flex.csv").open(newline="", encoding="utf-8") as stream:
        raw_flex = list(csv.DictReader(stream))
    assert raw_flex
    assert all(f"channel_{index}" in raw_flex[0] for index in range(11))
    assert raw_flex[0]["host_timestamp_ns"]

    with (statistics.session_dir / "raw_torso_imu.csv").open(newline="", encoding="utf-8") as stream:
        raw_torso = list(csv.DictReader(stream))
    assert raw_torso[0]["source_timestamp"]
    assert raw_torso[0]["sequence_id"]

    with (statistics.session_dir / "aligned.csv").open(newline="", encoding="utf-8") as stream:
        aligned = list(csv.DictReader(stream))
    with (statistics.session_dir / "training.csv").open(newline="", encoding="utf-8") as stream:
        training = list(csv.DictReader(stream))
    assert aligned and training
    valid_timestamps = {
        row["timestamp_ns"] for row in aligned if row["frame_valid"] == "True"
    }
    assert {row["timestamp_ns"] for row in training} == valid_timestamps
    assert statistics.training_frames == statistics.valid_frames == len(training)
    assert all(row["repetition_id"] == "007" for row in aligned)
    assert all(row["protocol_direction"] == "forward" for row in aligned)
    valid_aligned = [row for row in aligned if row["frame_valid"] == "True"]
    assert all(float(row["flex_skew_ms"]) <= 20.0 for row in valid_aligned)
    assert all(float(row["torso_skew_ms"]) <= 20.0 for row in valid_aligned)
    assert all(float(row["arm_skew_ms"]) <= 20.0 for row in valid_aligned)

    quaternions = np.asarray(
        [[float(row[f"shoulder_q{name}"]) for name in "wxyz"] for row in training]
    )
    inputs = np.asarray(
        [[float(row[f"sensor_{index}"]) for index in range(1, 4)] for row in training]
    )
    assert inputs.shape == (len(training), 3)
    assert quaternions.shape == (len(training), 4)
    assert np.linalg.norm(quaternions, axis=1) == pytest.approx(np.ones(len(training)))
    assert all(float(np.dot(left, right)) >= 0.0 for left, right in zip(quaternions, quaternions[1:]))
    forward_angles = [float(row["signed_forward_backward_deg"]) for row in training]
    assert min(forward_angles) >= -1e-9
    assert forward_angles[-1] == pytest.approx(30.0)

    metadata = json.loads((statistics.session_dir / "metadata.json").read_text(encoding="utf-8"))
    assert metadata["task"] == "three_direction_shoulder_quaternion_regression"
    assert metadata["training_input"] == ["sensor_1", "sensor_2", "sensor_3"]
    assert metadata["training_target"] == [
        "shoulder_qw",
        "shoulder_qx",
        "shoulder_qy",
        "shoulder_qz",
    ]
    assert metadata["flex_channel_mapping"] == {
        "sensor_1": "channel_2",
        "sensor_2": "channel_4",
        "sensor_3": "channel_7",
    }
    assert metadata["quaternion_order"] == "wxyz"
    assert metadata["neutral_calibration"]["valid_pair_count"] >= 2


def test_keyboard_interrupt_finalizes_session_safely(tmp_path: Path) -> None:
    config = load_shoulder_dataset_config(CONFIG_PATH)
    config = replace(
        config,
        collection=replace(
            config.collection,
            flush_interval_s=0.02,
            poll_interval_ms=1.0,
        ),
        calibration=replace(
            config.calibration,
            neutral_duration_s=0.1,
            countdown_s=0,
            startup_timeout_s=1.0,
        ),
    )
    fake = fake_sources("lateral", 11)
    collector = ShoulderDatasetCollector(
        config,
        "lateral",
        tmp_path,
        sleeve_source=fake[0],
        torso_source=fake[1],
        arm_source=fake[2],
        print_fn=lambda _: None,
    )

    def interrupt() -> list[str]:
        raise KeyboardInterrupt

    statistics = collector.run(key_reader=interrupt, countdown=False)

    metadata_path = statistics.session_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert metadata["status"] == "interrupted"
    for path in statistics.session_dir.iterdir():
        with path.open("a", encoding="utf-8"):
            pass
