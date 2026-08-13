from __future__ import annotations

import math
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from sleeve_arm.domain.joint import JOINT_NAMES


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "configs" / "robot.yaml"
DEFAULT_SENSOR_CONFIG_PATH = PROJECT_ROOT / "configs" / "sensors.yaml"
DEFAULT_PHASE3_CONFIG_PATH = PROJECT_ROOT / "configs" / "phase3.yaml"
DEFAULT_PHASE4_CONFIG_PATH = PROJECT_ROOT / "configs" / "phase4.yaml"


@dataclass(frozen=True)
class NetworkConfig:
    device_id: int
    local_ip: str
    local_port: int
    remote_ip: str
    remote_port: int
    fast_mode: int
    status: str


@dataclass(frozen=True)
class SafetyConfig:
    stable_feedback_samples: int
    stable_feedback_interval_s: float
    monitor_interval_s: float


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_id: int
    can_id: int
    direction: int
    direction_status: str
    zero_position: float | None
    min_position: float | None
    max_position: float | None
    max_velocity: float | None
    max_current: float | None
    max_tracking_error: float | None
    max_position_step: float | None


@dataclass(frozen=True)
class RobotConfig:
    network: NetworkConfig
    safety: SafetyConfig
    joints: dict[str, JointConfig]


@dataclass(frozen=True)
class SensorEndpointConfig:
    enabled: bool
    backend: str
    port: str | None
    baudrate: int | None
    timeout_s: float
    delimiter: str | None = None
    expected_fields: int | None = None


@dataclass(frozen=True)
class SynchronizationConfig:
    max_time_delta_ms: float
    buffer_duration_ms: float


@dataclass(frozen=True)
class RecordingConfig:
    output_dir: Path
    format: str


@dataclass(frozen=True)
class SensorConfig:
    sleeve: SensorEndpointConfig
    imu1: SensorEndpointConfig
    imu2: SensorEndpointConfig
    synchronization: SynchronizationConfig
    recording: RecordingConfig


@dataclass(frozen=True)
class Phase3FilterConfig:
    type: str
    alpha: float | None


@dataclass(frozen=True)
class Phase3ElbowConfig:
    sleeve_channel: int
    input_min: float | None
    input_max: float | None
    invert_input: bool
    deadzone: float
    filter: Phase3FilterConfig
    angle_min_deg: float | None
    angle_max_deg: float | None
    invert_output: bool


@dataclass(frozen=True)
class Phase3Config:
    elbow: Phase3ElbowConfig
    sensor_timeout_ms: float
    hard_timeout_ms: float
    control_hz: float


@dataclass(frozen=True)
class FlexModelConfig:
    model_module: str
    model_class: str
    sleeve_channels: tuple[int, int, int]
    baseline: tuple[float, float, float]
    scale: tuple[float, float, float]
    trial_rest: tuple[float, float, float]
    min_action_confidence: float
    required_consecutive_frames: int
    angle_min_deg: float
    angle_max_deg: float


@dataclass(frozen=True)
class Phase4Config:
    predictor_backend: str
    flex_model: FlexModelConfig | None
    max_consecutive_prediction_errors: int


def _optional_float(data: dict[str, Any], key: str) -> float | None:
    value = data.get(key)
    return None if value is None else float(value)


def _positive_optional(name: str, value: float | None) -> None:
    if value is not None and value <= 0:
        raise ValueError(f"{name} must be positive or null")


def load_robot_config(path: str | Path = DEFAULT_CONFIG_PATH) -> RobotConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"invalid robot config: {config_path}")

    network_raw = raw.get("network")
    safety_raw = raw.get("safety")
    joints_raw = raw.get("joints")
    if not all(isinstance(item, dict) for item in (network_raw, safety_raw, joints_raw)):
        raise ValueError("robot config requires network, safety, and joints mappings")

    network = NetworkConfig(
        device_id=int(network_raw["device_id"]),
        local_ip=str(network_raw["local_ip"]),
        local_port=int(network_raw["local_port"]),
        remote_ip=str(network_raw["remote_ip"]),
        remote_port=int(network_raw["remote_port"]),
        fast_mode=int(network_raw["fast_mode"]),
        status=str(network_raw["status"]),
    )
    if not 0 <= network.device_id <= 0xFFFF:
        raise ValueError("network.device_id must fit uint16")
    for label, port in (("local_port", network.local_port), ("remote_port", network.remote_port)):
        if not 1 <= port <= 65535:
            raise ValueError(f"network.{label} must be in 1..65535")
    if network.fast_mode <= 0:
        raise ValueError("network.fast_mode must be positive")

    safety = SafetyConfig(
        stable_feedback_samples=int(safety_raw["stable_feedback_samples"]),
        stable_feedback_interval_s=float(safety_raw["stable_feedback_interval_s"]),
        monitor_interval_s=float(safety_raw["monitor_interval_s"]),
    )
    if safety.stable_feedback_samples < 2:
        raise ValueError("safety.stable_feedback_samples must be at least 2")
    if safety.stable_feedback_interval_s < 0 or safety.monitor_interval_s <= 0:
        raise ValueError("feedback intervals must be non-negative/positive")

    if set(joints_raw) != set(JOINT_NAMES):
        raise ValueError(f"joints must be exactly: {', '.join(JOINT_NAMES)}")

    joints: dict[str, JointConfig] = {}
    identities: set[tuple[int, int]] = set()
    for name in JOINT_NAMES:
        item = joints_raw[name]
        if not isinstance(item, dict):
            raise ValueError(f"joints.{name} must be a mapping")
        joint = JointConfig(
            name=name,
            motor_id=int(item["motor_id"]),
            can_id=int(item["can_id"]),
            direction=int(item["direction"]),
            direction_status=str(item["direction_status"]),
            zero_position=_optional_float(item, "zero_position"),
            min_position=_optional_float(item, "min_position"),
            max_position=_optional_float(item, "max_position"),
            max_velocity=_optional_float(item, "max_velocity"),
            max_current=_optional_float(item, "max_current"),
            max_tracking_error=_optional_float(item, "max_tracking_error"),
            max_position_step=_optional_float(item, "max_position_step"),
        )
        if not 0 <= joint.motor_id <= 0xFFFF or not 0 <= joint.can_id <= 0xFFFF:
            raise ValueError(f"{name}: motor_id and can_id must fit uint16")
        if joint.direction not in (-1, 1):
            raise ValueError(f"{name}: direction must be -1 or 1")
        if joint.direction_status not in ("NEEDS_HARDWARE_VALIDATION", "VALIDATED"):
            raise ValueError(f"{name}: invalid direction_status")
        if (
            joint.min_position is not None
            and joint.max_position is not None
            and joint.min_position > joint.max_position
        ):
            raise ValueError(f"{name}: min_position exceeds max_position")
        _positive_optional(f"{name}.max_velocity", joint.max_velocity)
        _positive_optional(f"{name}.max_current", joint.max_current)
        _positive_optional(f"{name}.max_tracking_error", joint.max_tracking_error)
        _positive_optional(f"{name}.max_position_step", joint.max_position_step)
        identity = (joint.motor_id, joint.can_id)
        if identity in identities:
            raise ValueError(f"duplicate motor/CAN mapping: {identity}")
        identities.add(identity)
        joints[name] = joint

    return RobotConfig(network=network, safety=safety, joints=joints)


def load_sensor_config(path: str | Path = DEFAULT_SENSOR_CONFIG_PATH) -> SensorConfig:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError(f"invalid sensor config: {config_path}")
    sensors = raw.get("sensors")
    sync = raw.get("synchronization")
    recording = raw.get("recording")
    if not isinstance(sensors, dict) or not isinstance(sync, dict) or not isinstance(recording, dict):
        raise ValueError("sensor config requires sensors, synchronization, and recording mappings")

    def endpoint(name: str) -> SensorEndpointConfig:
        item = sensors.get(name)
        if not isinstance(item, dict):
            raise ValueError(f"sensors.{name} must be a mapping")
        port = item.get("port")
        baudrate = item.get("baudrate")
        result = SensorEndpointConfig(
            enabled=bool(item.get("enabled", False)),
            backend=str(item.get("backend", "")),
            port=None if port is None else str(port),
            baudrate=None if baudrate is None else int(baudrate),
            timeout_s=float(item.get("timeout_s", 1.0)),
            delimiter=None if item.get("delimiter") is None else str(item["delimiter"]),
            expected_fields=None if item.get("expected_fields") is None else int(item["expected_fields"]),
        )
        if not result.backend:
            raise ValueError(f"sensors.{name}.backend is required")
        if result.timeout_s <= 0:
            raise ValueError(f"sensors.{name}.timeout_s must be positive")
        if result.baudrate is not None and result.baudrate <= 0:
            raise ValueError(f"sensors.{name}.baudrate must be positive or null")
        return result

    synchronization = SynchronizationConfig(
        max_time_delta_ms=float(sync["max_time_delta_ms"]),
        buffer_duration_ms=float(sync["buffer_duration_ms"]),
    )
    if synchronization.max_time_delta_ms < 0 or synchronization.buffer_duration_ms <= 0:
        raise ValueError("synchronization deltas must be non-negative/positive")
    if synchronization.max_time_delta_ms > synchronization.buffer_duration_ms:
        raise ValueError("max_time_delta_ms cannot exceed buffer_duration_ms")

    output_dir = Path(str(recording["output_dir"]))
    if not output_dir.is_absolute():
        output_dir = PROJECT_ROOT / output_dir
    recording_config = RecordingConfig(output_dir.resolve(), str(recording["format"]))
    if recording_config.format != "csv":
        raise ValueError("only csv recording is supported")
    return SensorConfig(endpoint("sleeve"), endpoint("imu1"), endpoint("imu2"), synchronization, recording_config)


def load_phase3_config(path: str | Path = DEFAULT_PHASE3_CONFIG_PATH) -> Phase3Config:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    phase3 = raw.get("phase3") if isinstance(raw, dict) else None
    elbow = phase3.get("elbow") if isinstance(phase3, dict) else None
    filter_raw = elbow.get("filter") if isinstance(elbow, dict) else None
    range_raw = elbow.get("angle_range") if isinstance(elbow, dict) else None
    if not all(isinstance(item, dict) for item in (phase3, elbow, filter_raw, range_raw)):
        raise ValueError("phase3 config requires phase3.elbow.filter and angle_range mappings")

    filter_config = Phase3FilterConfig(
        type=str(filter_raw.get("type", "none")),
        alpha=None if filter_raw.get("alpha") is None else float(filter_raw["alpha"]),
    )
    config = Phase3Config(
        elbow=Phase3ElbowConfig(
            sleeve_channel=int(elbow["sleeve_channel"]),
            input_min=_optional_float(elbow, "input_min"),
            input_max=_optional_float(elbow, "input_max"),
            invert_input=bool(elbow.get("invert_input", False)),
            deadzone=float(elbow.get("deadzone", 0.0)),
            filter=filter_config,
            angle_min_deg=_optional_float(range_raw, "min_deg"),
            angle_max_deg=_optional_float(range_raw, "max_deg"),
            invert_output=bool(elbow.get("invert_output", False)),
        ),
        sensor_timeout_ms=float(phase3["sensor_timeout_ms"]),
        hard_timeout_ms=float(phase3["hard_timeout_ms"]),
        control_hz=float(phase3["control_hz"]),
    )
    numeric_values = {
        "deadzone": config.elbow.deadzone,
        "sensor_timeout_ms": config.sensor_timeout_ms,
        "hard_timeout_ms": config.hard_timeout_ms,
        "control_hz": config.control_hz,
    }
    numeric_values.update(
        (name, value)
        for name, value in (
            ("input_min", config.elbow.input_min),
            ("input_max", config.elbow.input_max),
            ("filter.alpha", filter_config.alpha),
            ("angle_range.min_deg", config.elbow.angle_min_deg),
            ("angle_range.max_deg", config.elbow.angle_max_deg),
        )
        if value is not None
    )
    if not all(math.isfinite(value) for value in numeric_values.values()):
        raise ValueError("all phase3 numeric values must be finite")
    if config.elbow.sleeve_channel < 1:
        raise ValueError("phase3.elbow.sleeve_channel uses 1-based numbering and must be >= 1")
    if (config.elbow.input_min is None) != (config.elbow.input_max is None):
        raise ValueError("input_min and input_max must both be set or both be null")
    if config.elbow.input_min is not None and config.elbow.input_min >= config.elbow.input_max:
        raise ValueError("input_min must be less than input_max")
    if not 0 <= config.elbow.deadzone < 0.5:
        raise ValueError("deadzone must be in [0, 0.5)")
    if filter_config.type not in ("none", "ema"):
        raise ValueError("filter.type must be none or ema")
    if filter_config.type == "ema" and (filter_config.alpha is None or not 0 < filter_config.alpha <= 1):
        raise ValueError("EMA filter requires alpha in (0, 1]")
    if (config.elbow.angle_min_deg is None) != (config.elbow.angle_max_deg is None):
        raise ValueError("angle_range min_deg and max_deg must both be set or both be null")
    if config.elbow.angle_min_deg is not None and config.elbow.angle_min_deg >= config.elbow.angle_max_deg:
        raise ValueError("angle_range min_deg must be less than max_deg")
    if config.sensor_timeout_ms <= 0 or config.hard_timeout_ms <= config.sensor_timeout_ms:
        raise ValueError("timeouts must satisfy 0 < sensor_timeout_ms < hard_timeout_ms")
    if config.control_hz <= 0:
        raise ValueError("control_hz must be positive")
    return config


def load_phase4_config(path: str | Path = DEFAULT_PHASE4_CONFIG_PATH) -> Phase4Config:
    config_path = Path(path).expanduser().resolve()
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    predictor = raw.get("predictor") if isinstance(raw, dict) else None
    stability = predictor.get("action_stability") if isinstance(predictor, dict) else None
    angle = predictor.get("angle") if isinstance(predictor, dict) else None
    validation = raw.get("phase4_validation") if isinstance(raw, dict) else None
    if not all(isinstance(item, dict) for item in (predictor, stability, angle, validation)):
        raise ValueError("phase4 config requires predictor stability/angle and phase4_validation")

    def triple(name: str, values: Any) -> tuple[float, float, float]:
        if not isinstance(values, list) or len(values) != 3 or any(value is None for value in values):
            raise ValueError(f"predictor.calibration.{name} requires exactly three non-null values")
        result = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in result):
            raise ValueError(f"predictor.calibration.{name} values must be finite")
        return result  # type: ignore[return-value]

    channels_raw = predictor.get("sleeve_channels")
    if not isinstance(channels_raw, list) or len(channels_raw) != 3:
        raise ValueError("predictor.sleeve_channels requires exactly three channels")
    channels = tuple(int(value) for value in channels_raw)
    if channels != (2, 3, 4):
        raise ValueError("FlexPredictor sleeve_channels must be exactly [2, 3, 4]")
    backend = str(predictor.get("backend", ""))
    flex_model = None
    if backend == "flex_model":
        maximum = angle.get("max_deg")
        if maximum is None:
            raise ValueError("predictor.angle.max_deg must be configured for Phase 4")
        calibration_value = predictor.get("calibration_file")
        if not calibration_value:
            raise ValueError("predictor.calibration_file is required for flex_model")
        calibration_path = Path(str(calibration_value)).expanduser()
        if not calibration_path.is_absolute():
            calibration_path = PROJECT_ROOT / calibration_path
        try:
            with calibration_path.resolve().open("r", encoding="utf-8") as stream:
                calibration = json.load(stream)
        except FileNotFoundError as exc:
            raise ValueError(
                f"Flex calibration file was not found: {calibration_path.resolve()}. "
                "Run: python tools/calibrate_flex_model.py"
            ) from exc
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"could not read Flex calibration file {calibration_path.resolve()}: {exc}") from exc
        if not isinstance(calibration, dict):
            raise ValueError("Flex calibration JSON must contain an object")
        flex_model = FlexModelConfig(
            model_module=str(predictor.get("model_module", "")),
            model_class=str(predictor.get("model_class", "")),
            sleeve_channels=channels,  # type: ignore[arg-type]
            baseline=triple("calibration_baseline", calibration.get("calibration_baseline")),
            scale=triple("calibration_scale", calibration.get("calibration_scale")),
            trial_rest=triple("trial_rest", calibration.get("trial_rest")),
            min_action_confidence=float(predictor.get("min_action_confidence", 0.6)),
            required_consecutive_frames=int(stability.get("required_consecutive_frames", 3)),
            angle_min_deg=float(angle.get("min_deg", 0.0)),
            angle_max_deg=float(maximum),
        )
    config = Phase4Config(
        predictor_backend=backend,
        flex_model=flex_model,
        max_consecutive_prediction_errors=int(validation.get("max_consecutive_prediction_errors", 3)),
    )
    values = []
    if flex_model is not None:
        values.extend((flex_model.min_action_confidence, flex_model.angle_min_deg, flex_model.angle_max_deg))
    if config.predictor_backend not in ("flex_model", "rule_based"):
        raise ValueError("predictor.backend must be flex_model or rule_based")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("phase4 numeric values must be finite")
    if flex_model is not None:
        if not flex_model.model_module or not flex_model.model_class:
            raise ValueError("model_module and model_class are required")
        if not 0 <= flex_model.min_action_confidence <= 1:
            raise ValueError("min_action_confidence must be in [0, 1]")
        if flex_model.required_consecutive_frames < 1:
            raise ValueError("required_consecutive_frames must be at least 1")
        if  flex_model.angle_max_deg < flex_model.angle_min_deg:
            raise ValueError("angle range must satisfy 0 <= min_deg <= max_deg")
    if config.max_consecutive_prediction_errors < 1:
        raise ValueError("max_consecutive_prediction_errors must be at least 1")
    return config
