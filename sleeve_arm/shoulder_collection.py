from __future__ import annotations

import csv
import json
import subprocess
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

from sleeve_arm.config import PROJECT_ROOT, SensorEndpointConfig
from sleeve_arm.domain import ImuFrame, SleeveFrame
from sleeve_arm.estimation import normalize_quaternion
from sleeve_arm.shoulder_alignment import AlignedInputs, ShoulderAligner, UniformTimeline
from sleeve_arm.shoulder_dataset_config import MOTIONS, ShoulderDatasetConfig
from sleeve_arm.shoulder_kinematics import (
    neutral_relative_quaternion,
    shoulder_angles,
    shoulder_quaternion,
)
from sleeve_arm.sources import (
    FakeImuSource,
    FakeSleeveSource,
    ImuSource,
    SleeveSource,
    create_imu_source,
    create_sleeve_source,
)


ALIGNED_FIELDS = (
    "frame_index",
    "timestamp_ns",
    "elapsed_s",
    "sensor_1",
    "sensor_2",
    "sensor_3",
    "torso_qw",
    "torso_qx",
    "torso_qy",
    "torso_qz",
    "arm_qw",
    "arm_qx",
    "arm_qy",
    "arm_qz",
    "shoulder_qw",
    "shoulder_qx",
    "shoulder_qy",
    "shoulder_qz",
    "signed_forward_backward_deg",
    "lateral_deg",
    "amplitude_deg",
    "motion",
    "protocol_direction",
    "derived_direction",
    "direction_confidence",
    "direction_valid",
    "repetition_id",
    "flex_skew_ms",
    "torso_skew_ms",
    "arm_skew_ms",
    "flex_valid",
    "torso_valid",
    "arm_valid",
    "frame_valid",
)

TRAINING_FIELDS = (
    "timestamp_ns",
    "sensor_1",
    "sensor_2",
    "sensor_3",
    "shoulder_qw",
    "shoulder_qx",
    "shoulder_qy",
    "shoulder_qz",
    "signed_forward_backward_deg",
    "lateral_deg",
    "amplitude_deg",
    "motion",
    "protocol_direction",
    "derived_direction",
    "direction_valid",
    "repetition_id",
)


@dataclass(frozen=True, slots=True)
class SessionStatistics:
    session_dir: Path
    elapsed_s: float
    raw_flex_frames: int
    raw_torso_frames: int
    raw_arm_frames: int
    aligned_frames: int
    valid_frames: int
    training_frames: int
    valid_percent: float


@dataclass(slots=True)
class _RepetitionState:
    repetition: int
    active: bool


@dataclass(frozen=True, slots=True)
class _RepetitionEvent:
    timestamp_ns: int
    repetition: int
    active: bool


class ShoulderSessionWriter:
    def __init__(self, session_dir: Path, channel_count: int) -> None:
        self.session_dir = session_dir
        self.metadata_path = session_dir / "metadata.json"
        self.raw_flex_count = 0
        self.raw_torso_count = 0
        self.raw_arm_count = 0
        self.aligned_count = 0
        self.valid_count = 0
        self.training_count = 0
        self._streams: list[Any] = []

        self._raw_flex_stream, self.raw_flex = self._csv(
            "raw_flex.csv",
            ("host_timestamp_ns", "source_timestamp", "sequence_id")
            + tuple(f"channel_{index}" for index in range(channel_count)),
        )
        imu_fields = (
            "host_timestamp_ns",
            "source_timestamp",
            "sequence_id",
            "qw",
            "qx",
            "qy",
            "qz",
            "accel_x",
            "accel_y",
            "accel_z",
            "gyro_x",
            "gyro_y",
            "gyro_z",
        )
        self._raw_torso_stream, self.raw_torso = self._csv("raw_torso_imu.csv", imu_fields)
        self._raw_arm_stream, self.raw_arm = self._csv("raw_upper_arm_imu.csv", imu_fields)
        self._aligned_stream, self.aligned = self._csv("aligned.csv", ALIGNED_FIELDS)
        self._training_stream, self.training = self._csv("training.csv", TRAINING_FIELDS)

    def _csv(self, name: str, fields: tuple[str, ...]) -> tuple[Any, csv.DictWriter]:
        stream = (self.session_dir / name).open("w", newline="", encoding="utf-8")
        self._streams.append(stream)
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore", lineterminator="\n")
        writer.writeheader()
        return stream, writer

    def write_flex(self, frames: tuple[SleeveFrame, ...]) -> None:
        for frame in frames:
            row: dict[str, object] = {
                "host_timestamp_ns": frame.host_timestamp_ns,
                "source_timestamp": "",
                "sequence_id": "" if frame.sequence_id is None else frame.sequence_id,
            }
            row.update((f"channel_{index}", value) for index, value in enumerate(frame.channels))
            self.raw_flex.writerow(row)
            self.raw_flex_count += 1

    @staticmethod
    def _imu_row(frame: ImuFrame) -> dict[str, object]:
        return {
            "host_timestamp_ns": frame.host_timestamp_ns,
            "source_timestamp": (
                "" if frame.device_timestamp_us is None else frame.device_timestamp_us
            ),
            "sequence_id": "" if frame.sequence_id is None else frame.sequence_id,
            "qw": "" if frame.quat_w is None else frame.quat_w,
            "qx": "" if frame.quat_x is None else frame.quat_x,
            "qy": "" if frame.quat_y is None else frame.quat_y,
            "qz": "" if frame.quat_z is None else frame.quat_z,
            "accel_x": frame.accel_x,
            "accel_y": frame.accel_y,
            "accel_z": frame.accel_z,
            "gyro_x": frame.gyro_x,
            "gyro_y": frame.gyro_y,
            "gyro_z": frame.gyro_z,
        }

    def write_torso(self, frames: tuple[ImuFrame, ...]) -> None:
        for frame in frames:
            self.raw_torso.writerow(self._imu_row(frame))
            self.raw_torso_count += 1

    def write_arm(self, frames: tuple[ImuFrame, ...]) -> None:
        for frame in frames:
            self.raw_arm.writerow(self._imu_row(frame))
            self.raw_arm_count += 1

    def write_aligned(self, row: dict[str, object]) -> None:
        self.aligned.writerow(row)
        self.aligned_count += 1
        if bool(row["frame_valid"]):
            self.valid_count += 1
            self.training.writerow(row)
            self.training_count += 1

    def flush(self) -> None:
        errors: list[BaseException] = []
        for stream in self._streams:
            try:
                stream.flush()
            except BaseException as exc:
                errors.append(exc)
        if errors:
            raise RuntimeError(f"failed to flush session files: {errors[0]}") from errors[0]

    def close(self) -> None:
        errors: list[BaseException] = []
        for stream in self._streams:
            if not stream.closed:
                try:
                    stream.flush()
                except BaseException as exc:
                    errors.append(exc)
                try:
                    stream.close()
                except BaseException as exc:
                    errors.append(exc)
        if errors:
            raise RuntimeError(f"failed to close session files: {errors[0]}") from errors[0]

    def write_metadata(self, metadata: dict[str, object]) -> None:
        temporary = self.metadata_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.metadata_path)


class ShoulderDatasetCollector:
    def __init__(
        self,
        config: ShoulderDatasetConfig,
        motion: str,
        output_root: str | Path,
        *,
        sleeve_source: SleeveSource | None = None,
        torso_source: ImuSource | None = None,
        arm_source: ImuSource | None = None,
        print_fn: Callable[[str], None] = print,
        on_repetition_start: Callable[[], None] | None = None,
    ) -> None:
        if motion not in MOTIONS:
            raise ValueError(f"motion must be one of: {', '.join(MOTIONS)}")
        self.config = config
        self.motion = motion
        self.output_root = Path(output_root).expanduser().resolve()
        self.print = print_fn
        self.on_repetition_start = on_repetition_start
        self.sleeve_source = sleeve_source
        self.torso_source = torso_source
        self.arm_source = arm_source
        self.aligner = ShoulderAligner(
            config.sync.buffer_duration_ms,
            config.sync.max_skew_ms,
            config.flex.interpolation,
        )
        self._previous_shoulder: np.ndarray | None = None
        self._last_row: dict[str, object] | None = None
        self._sources_started = False

    def run(
        self,
        *,
        duration_s: float | None = None,
        initial_repetition: int = 1,
        key_reader: Callable[[], list[str]] | None = None,
        countdown: bool = True,
    ) -> SessionStatistics:
        if duration_s is not None and duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if initial_repetition <= 0:
            raise ValueError("initial_repetition must be positive")
        git = _git_state()
        session_dir = _create_session_dir(self.output_root, self.motion)
        channel_count = int(self.config.sensors.sleeve.expected_fields or 0)
        metadata = self._metadata(git, session_dir)
        writer = ShoulderSessionWriter(session_dir, channel_count)
        interrupted = False
        failure: BaseException | None = None
        started_ns = time.monotonic_ns()
        statistics: SessionStatistics | None = None

        try:
            writer.write_metadata(metadata)
            self._ensure_sources()
            self._start_sources()
            self._wait_for_sources(writer)
            if countdown:
                self._countdown(writer)
            q_zero, calibration_metadata = self._calibrate(writer)
            metadata["neutral_calibration"] = calibration_metadata
            writer.write_metadata(metadata)
            statistics = self._collect(
                writer,
                q_zero,
                duration_s,
                initial_repetition,
                key_reader,
            )
            metadata["status"] = "completed"
        except KeyboardInterrupt:
            interrupted = True
            metadata["status"] = "interrupted"
        except BaseException as exc:
            failure = exc
            metadata["status"] = "failed"
            metadata["error"] = f"{type(exc).__name__}: {exc}"
        finally:
            finalization_errors: list[BaseException] = []
            self._close_sources()
            try:
                self._drain(writer)
            except BaseException as exc:
                self.print(f"Warning: final source drain failed: {exc}")
                finalization_errors.append(exc)
            elapsed_s = (
                statistics.elapsed_s
                if statistics is not None
                else (time.monotonic_ns() - started_ns) / 1_000_000_000.0
            )
            statistics = self._statistics(writer, session_dir, elapsed_s)
            metadata["completed_wall_time_ns"] = time.time_ns()
            metadata["statistics"] = _statistics_dict(statistics)
            try:
                writer.flush()
            except BaseException as exc:
                finalization_errors.append(exc)
            try:
                writer.close()
            except BaseException as exc:
                finalization_errors.append(exc)
            if finalization_errors and failure is None:
                metadata["status"] = "failed"
                metadata["error"] = (
                    "finalization failed: "
                    + "; ".join(str(error) for error in finalization_errors)
                )
            try:
                writer.write_metadata(metadata)
            except BaseException as exc:
                finalization_errors.append(exc)
            if finalization_errors and failure is None:
                failure = RuntimeError(
                    "session finalization failed: "
                    + "; ".join(str(error) for error in finalization_errors)
                )

        self._print_statistics(statistics)
        if failure is not None:
            raise failure
        if interrupted:
            self.print("Collection interrupted safely by Ctrl+C.")
        return statistics

    def _ensure_sources(self) -> None:
        if self.sleeve_source is None:
            self.sleeve_source = create_sleeve_source(self.config.sensors)
        endpoints = {
            "imu1": self.config.sensors.imu1,
            "imu2": self.config.sensors.imu2,
        }
        if self.torso_source is None:
            self.torso_source = create_imu_source(
                self.config.imu_roles.torso_imu,
                endpoints[self.config.imu_roles.torso_imu],
            )
        if self.arm_source is None:
            self.arm_source = create_imu_source(
                self.config.imu_roles.upper_arm_imu,
                endpoints[self.config.imu_roles.upper_arm_imu],
            )
        if self.torso_source is None or self.arm_source is None:
            raise RuntimeError("both configured IMU sources are required")

    def _start_sources(self) -> None:
        assert self.sleeve_source is not None
        assert self.torso_source is not None
        assert self.arm_source is not None
        started: list[SleeveSource | ImuSource] = []
        try:
            for source in (self.sleeve_source, self.torso_source, self.arm_source):
                source.start()
                started.append(source)
        except BaseException:
            for source in reversed(started):
                source.close()
            raise
        self._sources_started = True

    def _close_sources(self) -> None:
        if not self._sources_started:
            return
        for source in (self.arm_source, self.torso_source, self.sleeve_source):
            if source is not None:
                try:
                    source.close()
                except BaseException as exc:
                    self.print(f"Warning: source close failed: {exc}")
        self._sources_started = False

    def _drain(self, writer: ShoulderSessionWriter) -> None:
        assert self.sleeve_source is not None
        assert self.torso_source is not None
        assert self.arm_source is not None
        errors: list[BaseException] = []
        try:
            flex = self.sleeve_source.drain()
        except BaseException as exc:
            flex = ()
            errors.append(exc)
        try:
            torso = self.torso_source.drain()
        except BaseException as exc:
            torso = ()
            errors.append(exc)
        try:
            arm = self.arm_source.drain()
        except BaseException as exc:
            arm = ()
            errors.append(exc)
        writer.write_flex(flex)
        writer.write_torso(torso)
        writer.write_arm(arm)
        self.aligner.add_flex(flex)
        self.aligner.add_torso(torso)
        self.aligner.add_arm(arm)
        if errors:
            raise RuntimeError(
                "source drain failed: " + "; ".join(str(error) for error in errors)
            ) from errors[0]

    def _wait_for_sources(self, writer: ShoulderSessionWriter) -> None:
        deadline_ns = time.monotonic_ns() + round(
            self.config.calibration.startup_timeout_s * 1_000_000_000
        )
        last_flush_ns = time.monotonic_ns()
        while time.monotonic_ns() < deadline_ns:
            self._drain(writer)
            if (
                self.aligner.flex_buffer.frames
                and self.aligner.torso_buffer.frames
                and self.aligner.arm_buffer.frames
            ):
                return
            now_ns = time.monotonic_ns()
            if now_ns - last_flush_ns >= round(self.config.collection.flush_interval_s * 1e9):
                writer.flush()
                last_flush_ns = now_ns
            time.sleep(self.config.collection.poll_interval_ms / 1000.0)
        raise TimeoutError("timed out waiting for flex, torso IMU, and upper-arm IMU samples")

    def _countdown(self, writer: ShoulderSessionWriter) -> None:
        self.print("Keep torso upright and right arm naturally down.")
        self.print("Neutral calibration begins in:")
        for number in range(self.config.calibration.countdown_s, 0, -1):
            self.print(str(number))
            deadline_ns = time.monotonic_ns() + 1_000_000_000
            while time.monotonic_ns() < deadline_ns:
                self._drain(writer)
                time.sleep(self.config.collection.poll_interval_ms / 1000.0)
            writer.flush()

    def _calibrate(
        self,
        writer: ShoulderSessionWriter,
    ) -> tuple[np.ndarray, dict[str, object]]:
        self.print(
            f"Neutral calibration: hold the pose for {self.config.calibration.neutral_duration_s:.1f} s."
        )
        start_ns = time.monotonic_ns()
        end_ns = start_ns + round(self.config.calibration.neutral_duration_s * 1_000_000_000)
        delay_ns = round(self.config.sync.alignment_delay_ms * 1_000_000)
        timeline = UniformTimeline(self.config.collection.output_hz, start_ns)
        pairs: list[tuple[tuple[float, ...], tuple[float, ...]]] = []
        last_flush_ns = start_ns
        while True:
            now_ns = time.monotonic_ns()
            self._drain(writer)
            available = min(end_ns, now_ns - delay_ns)
            for _, timestamp_ns in timeline.due(available):
                aligned = self.aligner.align(timestamp_ns)
                if aligned.torso.valid and aligned.arm.valid:
                    assert aligned.torso.value is not None and aligned.arm.value is not None
                    pairs.append((aligned.torso.value, aligned.arm.value))
            if now_ns - last_flush_ns >= round(self.config.collection.flush_interval_s * 1e9):
                writer.flush()
                last_flush_ns = now_ns
            if now_ns >= end_ns + delay_ns and timeline.next_timestamp_ns > end_ns:
                break
            time.sleep(self.config.collection.poll_interval_ms / 1000.0)
        if len(pairs) < 2:
            raise RuntimeError(
                f"neutral calibration received only {len(pairs)} valid aligned IMU pairs"
            )
        q_zero = neutral_relative_quaternion(pairs)
        residuals = [
            shoulder_quaternion(torso, arm, q_zero)
            for torso, arm in pairs
        ]
        residual_deg = [
            math_degrees_from_identity(quaternion)
            for quaternion in residuals
        ]
        self.print(
            f"Neutral calibrated from {len(pairs)} pairs; mean residual = "
            f"{float(np.mean(residual_deg)):.3f} deg."
        )
        return q_zero, {
            "pose": "torso upright, facing forward, right arm naturally down, elbow naturally extended",
            "duration_s": self.config.calibration.neutral_duration_s,
            "valid_pair_count": len(pairs),
            "q_zero_wxyz": [float(value) for value in q_zero],
            "mean_residual_deg": float(np.mean(residual_deg)),
            "max_residual_deg": float(np.max(residual_deg)),
            "start_host_timestamp_ns": start_ns,
            "end_host_timestamp_ns": end_ns,
            "method": "Markley average of inverse(q_torso) * q_upper_arm",
        }

    def _collect(
        self,
        writer: ShoulderSessionWriter,
        q_zero: np.ndarray,
        duration_s: float | None,
        initial_repetition: int,
        key_reader: Callable[[], list[str]] | None,
    ) -> SessionStatistics:
        start_ns = time.monotonic_ns()
        delay_ns = round(self.config.sync.alignment_delay_ms * 1_000_000)
        end_ns = None if duration_s is None else start_ns + round(duration_s * 1_000_000_000)
        timeline = UniformTimeline(self.config.collection.output_hz, start_ns)
        state = _RepetitionState(initial_repetition, duration_s is not None)
        repetition_events = [
            _RepetitionEvent(start_ns, state.repetition, state.active)
        ]
        written_index = 0
        stop_at_ns: int | None = None
        last_status_ns = start_ns
        last_flush_ns = start_ns
        status_period_ns = round(1_000_000_000 / self.config.collection.status_hz)
        flush_period_ns = round(self.config.collection.flush_interval_s * 1_000_000_000)
        if state.active:
            self._repetition_started(state.repetition)
        else:
            self.print("Controls: SPACE=start/stop repetition, N=next repetition, Q=quit")

        while True:
            now_ns = time.monotonic_ns()
            self._drain(writer)
            if key_reader is not None and stop_at_ns is None:
                for key in key_reader():
                    key = key.lower()
                    event_ns = time.monotonic_ns()
                    if key == " ":
                        state.active = not state.active
                        repetition_events.append(
                            _RepetitionEvent(event_ns, state.repetition, state.active)
                        )
                        if state.active:
                            self._repetition_started(state.repetition)
                        else:
                            self.print(f"Repetition {state.repetition:03d} stopped.")
                    elif key == "n":
                        state.active = False
                        state.repetition += 1
                        repetition_events.append(
                            _RepetitionEvent(event_ns, state.repetition, False)
                        )
                        self.print(f"Ready for repetition {state.repetition:03d}.")
                    elif key == "q":
                        stop_at_ns = event_ns

            collection_limit = now_ns - delay_ns
            if end_ns is not None:
                collection_limit = min(collection_limit, end_ns)
            if stop_at_ns is not None:
                collection_limit = min(collection_limit, stop_at_ns)
            for _, timestamp_ns in timeline.due(collection_limit):
                aligned = self.aligner.align(timestamp_ns)
                target_state = _repetition_state_at(repetition_events, timestamp_ns)
                if target_state.active:
                    row = self._row(
                        written_index,
                        timestamp_ns,
                        start_ns,
                        target_state.repetition,
                        aligned,
                        q_zero,
                    )
                    writer.write_aligned(row)
                    self._last_row = row
                    written_index += 1

            if now_ns - last_flush_ns >= flush_period_ns:
                writer.flush()
                last_flush_ns = now_ns
            if now_ns - last_status_ns >= status_period_ns:
                self._print_status(writer, state, start_ns, now_ns)
                last_status_ns = now_ns

            auto_complete = (
                end_ns is not None
                and now_ns >= end_ns + delay_ns
                and timeline.next_timestamp_ns > end_ns
            )
            manual_complete = (
                stop_at_ns is not None
                and now_ns >= stop_at_ns + delay_ns
                and timeline.next_timestamp_ns > stop_at_ns
            )
            if manual_complete or auto_complete:
                break
            time.sleep(self.config.collection.poll_interval_ms / 1000.0)

        return self._statistics(
            writer,
            writer.session_dir,
            (time.monotonic_ns() - start_ns) / 1_000_000_000.0,
        )

    def _repetition_started(self, repetition: int) -> None:
        self.print(f"Repetition {repetition:03d} started.")
        if self.on_repetition_start is not None:
            self.on_repetition_start()

    def _row(
        self,
        frame_index: int,
        timestamp_ns: int,
        start_ns: int,
        repetition: int,
        aligned: AlignedInputs,
        q_zero: np.ndarray,
    ) -> dict[str, object]:
        flex_values = aligned.flex.value
        selected = (
            None
            if flex_values is None
            else tuple(flex_values[index] for index in self.config.flex.channels)
        )
        torso = aligned.torso.value
        arm = aligned.arm.value
        shoulder: np.ndarray | None = None
        angles = None
        if torso is not None and arm is not None:
            shoulder = shoulder_quaternion(
                torso,
                arm,
                q_zero,
                self._previous_shoulder,
            )
            self._previous_shoulder = shoulder
            angles = shoulder_angles(shoulder, self.config.body_frame, self.config.label)

        row: dict[str, object] = {
            "frame_index": frame_index,
            "timestamp_ns": timestamp_ns,
            "elapsed_s": (timestamp_ns - start_ns) / 1_000_000_000.0,
            "motion": self.motion,
            "protocol_direction": self.motion,
            "derived_direction": "" if angles is None else angles.derived_direction,
            "direction_confidence": "" if angles is None else angles.direction_confidence,
            "direction_valid": False if angles is None else angles.direction_valid,
            "repetition_id": f"{repetition:03d}",
            "flex_skew_ms": _blank(aligned.flex.skew_ms),
            "torso_skew_ms": _blank(aligned.torso.skew_ms),
            "arm_skew_ms": _blank(aligned.arm.skew_ms),
            "flex_valid": aligned.flex.valid,
            "torso_valid": aligned.torso.valid,
            "arm_valid": aligned.arm.valid,
            "frame_valid": aligned.frame_valid and shoulder is not None,
        }
        _put_values(row, "sensor", selected, one_based=True)
        _put_quaternion(row, "torso", torso)
        _put_quaternion(row, "arm", arm)
        _put_quaternion(row, "shoulder", shoulder)
        row.update(
            {
                "signed_forward_backward_deg": (
                    "" if angles is None else angles.signed_forward_backward_deg
                ),
                "lateral_deg": "" if angles is None else angles.lateral_deg,
                "amplitude_deg": "" if angles is None else angles.amplitude_deg,
            }
        )
        return row

    def _metadata(
        self,
        git: dict[str, object],
        session_dir: Path,
    ) -> dict[str, object]:
        endpoints = {
            "imu1": self.config.sensors.imu1,
            "imu2": self.config.sensors.imu2,
        }
        return {
            "task": "three_direction_shoulder_quaternion_regression",
            "status": "collecting",
            "session_id": session_dir.name,
            "protocol_direction": self.motion,
            "directions": list(MOTIONS),
            "training_input": ["sensor_1", "sensor_2", "sensor_3"],
            "training_target": [
                "shoulder_qw",
                "shoulder_qx",
                "shoulder_qy",
                "shoulder_qz",
            ],
            "flex_channel_mapping": {
                f"sensor_{index + 1}": f"channel_{channel}"
                for index, channel in enumerate(self.config.flex.channels)
            },
            "flex_channel_indexing": "zero_based",
            "quaternion_order": "wxyz",
            "quaternion_direction": "sensor_to_world",
            "quaternion_multiplication": (
                "Hamilton active rotation; left*right applies right then left"
            ),
            "quaternion_convention_evidence": {
                "project_parser": "IMU770 TLV 0x41 is decoded as WXYZ",
                "vendor_manual": "https://download.hipnuc.com/en/products/imu/cum.html",
                "hardware_rotation_check": "required_before_real_collection",
            },
            "shoulder_formula": (
                "q_relative=inverse(q_torso)*q_upper_arm; "
                "q_shoulder=q_relative*inverse(q_zero)"
            ),
            "output_hz": self.config.collection.output_hz,
            "neutral_calibration": {"status": "pending"},
            "body_frame": asdict(self.config.body_frame),
            "sync": {
                **asdict(self.config.sync),
                "clock": "time.monotonic_ns",
                "flex_interpolation": self.config.flex.interpolation,
                "imu_interpolation": "SLERP",
            },
            "label": asdict(self.config.label),
            "derived_information": {
                "robot_control_fields": [
                    "signed_forward_backward_deg",
                    "lateral_deg",
                ],
                "derived_direction_use": "analysis_only_not_robot_control",
                "signed_forward_backward_formula": (
                    "degrees(atan2(dot(u,forward_axis),dot(u,neutral_arm_axis)))"
                ),
                "lateral_formula": (
                    "max(0,degrees(atan2(dot(u,lateral_axis),dot(u,neutral_arm_axis))))"
                ),
                "amplitude_formula": (
                    "degrees(acos(clamp(dot(neutral_arm_axis,u),-1,1)))"
                ),
            },
            "raw_timestamp_fields": {
                "host_timestamp_ns": "time.monotonic_ns at source read arrival",
                "source_timestamp": "IMU device_timestamp_us when TLV 0x51 is present",
                "sequence_id": "sleeve host sequence or IMU TID",
            },
            "repetition_controls": {
                "SPACE": "start_or_stop_current_repetition",
                "N": "advance_to_next_repetition",
                "Q": "finish_session",
            },
            "imu_roles": {
                "torso_imu": self.config.imu_roles.torso_imu,
                "upper_arm_imu": self.config.imu_roles.upper_arm_imu,
                "torso_endpoint": _endpoint_dict(endpoints[self.config.imu_roles.torso_imu]),
                "upper_arm_endpoint": _endpoint_dict(
                    endpoints[self.config.imu_roles.upper_arm_imu]
                ),
            },
            "sensor_config": str(self.config.sensor_config_path),
            "dataset_config": str(self.config.path),
            "created_wall_time": datetime.now().astimezone().isoformat(),
            "created_wall_time_ns": time.time_ns(),
            **git,
        }

    def _statistics(
        self,
        writer: ShoulderSessionWriter,
        session_dir: Path,
        elapsed_s: float,
    ) -> SessionStatistics:
        valid_percent = (
            0.0
            if writer.aligned_count == 0
            else 100.0 * writer.valid_count / writer.aligned_count
        )
        return SessionStatistics(
            session_dir=session_dir,
            elapsed_s=elapsed_s,
            raw_flex_frames=writer.raw_flex_count,
            raw_torso_frames=writer.raw_torso_count,
            raw_arm_frames=writer.raw_arm_count,
            aligned_frames=writer.aligned_count,
            valid_frames=writer.valid_count,
            training_frames=writer.training_count,
            valid_percent=valid_percent,
        )

    def _print_status(
        self,
        writer: ShoulderSessionWriter,
        state: _RepetitionState,
        start_ns: int,
        now_ns: int,
    ) -> None:
        assert self.sleeve_source is not None
        assert self.torso_source is not None
        assert self.arm_source is not None
        row = self._last_row or {}
        q = [row.get(f"shoulder_q{name}", "") for name in "wxyz"]
        q_text = "unavailable" if any(value == "" for value in q) else (
            "[" + ", ".join(f"{float(value):.3f}" for value in q) + "]"
        )
        valid = 0.0 if writer.aligned_count == 0 else 100.0 * writer.valid_count / writer.aligned_count
        self.print(
            "\n".join(
                (
                    f"Session: motion={self.motion}, repetition={state.repetition:03d}, active={state.active}",
                    f"Time: {(now_ns - start_ns) / 1e9:.1f} s",
                    "Rates: "
                    f"flex={self.sleeve_source.stats.estimated_fps:.1f} Hz, "
                    f"torso={self.torso_source.stats.estimated_fps:.1f} Hz, "
                    f"arm={self.arm_source.stats.estimated_fps:.1f} Hz, "
                    f"aligned={writer.aligned_count / max((now_ns - start_ns) / 1e9, 1e-9):.1f} Hz",
                    f"Shoulder: q={q_text}",
                    "Continuous: "
                    f"forward_backward={_status_number(row.get('signed_forward_backward_deg'))} deg, "
                    f"lateral={_status_number(row.get('lateral_deg'))} deg, "
                    f"amplitude={_status_number(row.get('amplitude_deg'))} deg",
                    f"Labels: protocol={self.motion}, derived={row.get('derived_direction', 'unavailable')}",
                    "Sync: "
                    f"flex={_status_number(row.get('flex_skew_ms'))} ms, "
                    f"torso={_status_number(row.get('torso_skew_ms'))} ms, "
                    f"arm={_status_number(row.get('arm_skew_ms'))} ms",
                    f"Valid: {valid:.1f}%",
                )
            )
        )

    def _print_statistics(self, statistics: SessionStatistics) -> None:
        self.print(
            "\n".join(
                (
                    f"Session saved: {statistics.session_dir}",
                    f"Raw frames: flex={statistics.raw_flex_frames}, "
                    f"torso={statistics.raw_torso_frames}, arm={statistics.raw_arm_frames}",
                    f"Aligned={statistics.aligned_frames}, valid={statistics.valid_frames} "
                    f"({statistics.valid_percent:.1f}%), training={statistics.training_frames}",
                )
            )
        )


def fake_sources(
    motion: str,
    channel_count: int,
) -> tuple[FakeSleeveSource, FakeImuSource, FakeImuSource, Callable[[], None]]:
    if motion not in MOTIONS:
        raise ValueError(f"motion must be one of: {', '.join(MOTIONS)}")
    active = {"value": False}

    def arm_quaternion(_: float) -> tuple[float, float, float, float]:
        if not active["value"]:
            return (1.0, 0.0, 0.0, 0.0)
        from sleeve_arm.estimation import quaternion_from_axis_angle

        axis_angle = {
            "forward": ((0.0, 1.0, 0.0), -30.0),
            "lateral": ((1.0, 0.0, 0.0), 45.0),
            "backward": ((0.0, 1.0, 0.0), 20.0),
        }[motion]
        quaternion = quaternion_from_axis_angle(
            axis_angle[0],
            np.radians(axis_angle[1]),
        )
        return tuple(float(value) for value in quaternion)

    def activate() -> None:
        active["value"] = True

    return (
        FakeSleeveSource(channel_count=channel_count, frequency_hz=30.0),
        FakeImuSource(frequency_hz=60.0),
        FakeImuSource(frequency_hz=100.0, quaternion_fn=arm_quaternion),
        activate,
    )


def math_degrees_from_identity(quaternion: np.ndarray) -> float:
    q = normalize_quaternion(quaternion)
    return float(np.degrees(2.0 * np.arccos(np.clip(abs(q[0]), -1.0, 1.0))))


def _put_values(
    row: dict[str, object],
    prefix: str,
    values: tuple[float, ...] | None,
    *,
    one_based: bool = False,
) -> None:
    if values is None:
        values = tuple()
    for index in range(3):
        name_index = index + 1 if one_based else index
        row[f"{prefix}_{name_index}"] = "" if index >= len(values) else values[index]


def _put_quaternion(
    row: dict[str, object],
    prefix: str,
    quaternion: tuple[float, ...] | np.ndarray | None,
) -> None:
    values = tuple() if quaternion is None else quaternion
    for index, component in enumerate("wxyz"):
        row[f"{prefix}_q{component}"] = "" if index >= len(values) else float(values[index])


def _blank(value: object | None) -> object:
    return "" if value is None else value


def _status_number(value: object | None) -> str:
    return "unavailable" if value in (None, "") else f"{float(value):+.2f}"


def _create_session_dir(output_root: Path, motion: str) -> Path:
    output_root.mkdir(parents=True, exist_ok=True)
    base = f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{motion}"
    for suffix in range(1000):
        name = base if suffix == 0 else f"{base}_{suffix:03d}"
        candidate = output_root / name
        try:
            candidate.mkdir()
        except FileExistsError:
            continue
        return candidate
    raise RuntimeError("could not allocate a unique session directory")


def _git_state() -> dict[str, object]:
    def run(*arguments: str) -> str:
        completed = subprocess.run(
            ("git", *arguments),
            cwd=PROJECT_ROOT,
            check=True,
            capture_output=True,
            text=True,
            encoding="utf-8",
        )
        return completed.stdout.strip()

    try:
        return {
            "git_commit": run("rev-parse", "HEAD"),
            "git_branch": run("branch", "--show-current"),
            "git_dirty": bool(run("status", "--porcelain")),
        }
    except (OSError, subprocess.SubprocessError):
        return {"git_commit": None, "git_branch": None, "git_dirty": None}


def _endpoint_dict(endpoint: SensorEndpointConfig) -> dict[str, object]:
    return asdict(endpoint)


def _statistics_dict(statistics: SessionStatistics) -> dict[str, object]:
    result = asdict(statistics)
    result["session_dir"] = str(statistics.session_dir)
    return result


def _repetition_state_at(
    events: list[_RepetitionEvent],
    timestamp_ns: int,
) -> _RepetitionEvent:
    for event in reversed(events):
        if event.timestamp_ns <= timestamp_ns:
            return event
    return events[0]
