#!/usr/bin/env python3
"""Audit SDK offline, or explicitly subscribe to 0.1.8 state (no robot writes).

No AuroraClient instance is created. Its 0.1.8 initialization creates writers
and service clients. The public DDSInterface lets this tool create only state
subscriptions.
See docs/aurora_sdk_environment.md for the audited sources and limitations.
"""
from __future__ import annotations

import argparse
import importlib
import importlib.metadata as metadata
import inspect
import json
import math
import os
import platform
import sys
import threading
import time
from typing import Any

PACKAGE = "fourier_aurora_client"
SUPPORTED_VERSION = "0.1.8"
LEGACY_FAMILY = "aurora-python-dds-0.1.8"
METHODS = (
    "get_instance", "config", "configure", "start", "stop", "close",
    "wait_for_endpoints", "get_fsm_state", "get_aurora_state",
    "get_group_state", "get_group_position", "get_control_group_state",
    "set_group_cmd", "publish_control_group_command", "register_lease",
    "get_lease", "release_lease",
)
TOPICS = ("aurora_state", "robot_control_group_state")


class DoctorError(RuntimeError):
    pass


def api_surface(sdk: Any) -> dict:
    """Inspect class attributes; never invoke SDK methods or constructors."""
    client = getattr(sdk, "AuroraClient", None)
    names = set(METHODS)
    if client is not None:
        names.update(name for name in dir(client) if not name.startswith("_"))
    result = {}
    for name in sorted(names):
        value = getattr(client, name, None)
        item = {"exists": callable(value), "signature": None}
        if item["exists"]:
            try:
                item["signature"] = str(inspect.signature(value))
            except (TypeError, ValueError):
                item["signature"] = "unavailable (native binding); see doc"
                item["doc"] = inspect.getdoc(value)
        result[name] = item
    return result


def api_family(surface: dict) -> str:
    def has(*names):
        return all(surface.get(n, {}).get("exists", False) for n in names)
    legacy = has("get_instance", "get_fsm_state", "get_group_state", "set_group_cmd", "close")
    modern = has("configure", "start", "get_aurora_state", "get_control_group_state", "register_lease")
    if legacy and not modern:
        return LEGACY_FAMILY
    if modern and not legacy:
        return "aurora-configure-lease"
    return "unknown-or-mixed"


def offline_report() -> tuple[dict, Any]:
    report = {
        "mode": "offline", "dds_started": False,
        "python": sys.executable, "python_version": sys.version,
        "platform": platform.platform(), "machine": platform.machine(),
        "shell_conda_prefix": os.environ.get("CONDA_PREFIX"),
        "shell_virtual_env": os.environ.get("VIRTUAL_ENV"),
        "python_prefix": sys.prefix, "package": PACKAGE,
    }
    try:
        report["package_version"] = metadata.version(PACKAGE)
    except metadata.PackageNotFoundError as exc:
        raise DoctorError(
            f"{PACKAGE} is not installed in {sys.executable}. "
            "From the project root run: python -m pip install -r requirements-aurora.txt "
            "(activate the intended environment first)."
        ) from exc
    try:
        sdk = importlib.import_module(PACKAGE)
    except (ImportError, OSError, RuntimeError) as exc:
        raise DoctorError(
            f"import {PACKAGE} {report['package_version']} failed: {exc}. "
            "The audited wheel contains Linux x86_64 native libraries; "
            "do not copy .so files or mask this error with LD_LIBRARY_PATH."
        ) from exc
    report["module"] = getattr(sdk, "__file__", None)
    report["AuroraClient"] = str(getattr(sdk, "AuroraClient", None))
    report["api"] = api_surface(sdk)
    report["api_family"] = api_family(report["api"])
    report["connect_supported"] = (
        report["package_version"] == SUPPORTED_VERSION
        and report["api_family"] == LEGACY_FAMILY
    )
    report["import_ok"] = True
    return report, sdk


class StateSamples:
    """Own copies of received samples, with monotonic reception timestamps."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.values: dict[str, dict] = {}

    def _receive(self, topic, decode, data):
        received_at = self.clock()
        try:
            value = decode(data)
            error = None
        except Exception as exc:
            value, error = None, f"{type(exc).__name__}: {exc}"
        with self.lock:
            self.values[topic] = {
                "received_at": received_at, "data": value, "error": error,
                "sample_count": self.values.get(topic, {}).get("sample_count", 0) + 1,
            }

    def aurora(self, data):
        self._receive(TOPICS[0], self._decode_aurora, data)

    def groups(self, data):
        self._receive(TOPICS[1], self._decode_groups, data)

    @staticmethod
    def _decode_aurora(data):
        # Raw IDs only: an observed ID does not establish a safe control FSM.
        return {
            "whole_body_fsm_state": int(data.whole_body_fsm_state()),
            "upper_body_fsm_state": int(data.upper_body_fsm_state()),
            "velocity_command_source": int(data.velocity_command_source()),
            "allow_upper_body_override": bool(data.allow_upper_body_override()),
        }

    @staticmethod
    def _decode_groups(data):
        groups = {}
        for group in data.group_state():
            name = group.group_name()
            if not isinstance(name, str) or not name or name in groups:
                raise ValueError("empty, invalid or duplicate group name")
            vectors = {
                "position": list(group.joint_position()),
                "velocity": list(group.joint_velocity()),
                "effort": list(group.joint_effort()),
            }
            count = len(vectors["position"])
            if not count:
                raise ValueError(f"{name}: empty position vector")
            for key, values in vectors.items():
                if len(values) not in (0, count):
                    raise ValueError(f"{name}.{key}: inconsistent vector length")
                if any(isinstance(v, bool) or not math.isfinite(v) for v in values):
                    raise ValueError(f"{name}.{key}: non-finite or invalid value")
            groups[name] = {"joint_vector_length": count, **vectors}
        if not groups:
            raise ValueError("no control groups in sample")
        return groups

    def snapshot(self, max_age: float) -> dict:
        with self.lock:
            now = self.clock()
            result = {}
            for topic in TOPICS:
                item = self.values.get(topic)
                age = None if item is None else now - item["received_at"]
                result[topic] = {
                    "received": item is not None,
                    "age_s": age,
                    "valid": item is not None and item["error"] is None and 0 <= age <= max_age,
                    "sample_count": 0 if item is None else item["sample_count"],
                    "error": None if item is None else item["error"],
                    "data": None if item is None else item["data"],
                }
            return result


def read_only_connection(sdk, report, *, domain_id, timeout=5.0,
                         max_age=1.0, namespace=None, ros_compatible=None,
                         clock=time.monotonic, sleep=time.sleep,
                         module_loader=importlib.import_module):
    """Bounded subscription-only diagnostic; requires explicit caller opt-in.

    0.1.8 returns a Subscriber or raises; close returns None. It has no
    OperationResult API. Never infer success from object truthiness, or try
    another API family if creation/discovery fails.
    """
    if report["package_version"] != SUPPORTED_VERSION or api_family(api_surface(sdk)) != LEGACY_FAMILY:
        raise DoctorError("read-only connect supports only audited 0.1.8 legacy API; no automatic fallback")
    if isinstance(domain_id, bool) or not isinstance(domain_id, int) or not 0 <= domain_id <= 2**32 - 1:
        raise DoctorError("domain_id must be an explicit uint32 integer")
    for name, value in (("timeout", timeout), ("max_age", max_age)):
        if isinstance(value, bool) or not math.isfinite(value) or not 0 < value <= 60:
            raise DoctorError(f"{name} must be finite and in (0, 60] seconds")
    if platform.system() != "Linux" or platform.machine().lower() not in ("x86_64", "amd64"):
        raise DoctorError("0.1.8 connection is audited only for Linux x86_64")

    # Load audited state types before creating the participant.
    aurora_type = module_loader("fourier_msgs.msg.AuroraState").AuroraStatePubSubType
    groups_type = module_loader("fourier_msgs.msg.MotionControlState").RobotControlGroupStatePubSubType
    qos = sdk.SubscriberQosProfile.best_effort()
    samples = StateSamples(clock)
    transport = sdk.DDSInterface(domain_id=domain_id, namespace=namespace,
                                 is_ros_compatible=ros_compatible)
    try:
        subscribers = {}
        for topic, message_type, callback in (
            (TOPICS[0], aurora_type, samples.aurora),
            (TOPICS[1], groups_type, samples.groups),
        ):
            subscriber = transport.create_subscription(message_type, topic, callback, qos_profile=qos)
            if subscriber is None or not hasattr(subscriber, "is_matched"):
                raise DoctorError(f"create_subscription({topic}) returned no valid Subscriber")
            subscribers[topic] = subscriber
        deadline = clock() + timeout
        while True:
            states = samples.snapshot(max_age)
            matched = {topic: sub.is_matched is True for topic, sub in subscribers.items()}
            valid = all(matched.values()) and all(state["valid"] for state in states.values())
            if valid or any(s["error"] for s in states.values()) or clock() >= deadline:
                break
            sleep(min(0.02, max(0.0, deadline - clock())))
        return {
            "mode": "read-only-connection", "dds_started": True,
            "domain_id": domain_id, "namespace": namespace,
            "ros_compatible_override": ros_compatible,
            "subscribed_topics": list(TOPICS), "matched": matched,
            "feedback_valid": valid, "states": states,
            "robot_type": None, "hardware_type": None, "end_effector_type": None,
            "robot_info_status": "unavailable: 0.1.8 state messages do not contain these fields",
            "freshness_basis": "local monotonic reception time; not robot measurement age",
            "error": None if valid else "missing, stale, malformed or unmatched state feedback",
        }
    finally:
        # No hold, reset, FSM request or lease action. End local reception only.
        transport.close()


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--connect", action="store_true", help="explicitly start DDS state subscriptions only")
    result.add_argument("--domain-id", type=int, help="required with --connect; official examples use 123")
    result.add_argument("--namespace", help="site-confirmed DDS namespace")
    result.add_argument("--ros-compatible", action=argparse.BooleanOptionalAction, default=None,
                        help="override SDK topic naming locally; otherwise SDK reads its environment")
    result.add_argument("--timeout", type=float, default=5.0, help="bounded discovery/sample wait, seconds (0,60]")
    result.add_argument("--max-age", type=float, default=1.0, help="maximum sample reception age, seconds (0,60]")
    return result


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    if args.connect and args.domain_id is None:
        arguments.error("--connect requires an explicit --domain-id")
    if not args.connect and (args.domain_id is not None or args.namespace is not None or args.ros_compatible is not None):
        arguments.error("connection options require --connect; offline mode never creates a session")
    try:
        report, sdk = offline_report()
        if args.connect:
            report["connection"] = read_only_connection(
                sdk, report, domain_id=args.domain_id, timeout=args.timeout,
                max_age=args.max_age, namespace=args.namespace, ros_compatible=args.ros_compatible,
            )
            report["mode"] = "read-only-connection"
            report["dds_started"] = True
        print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
        return 0 if not args.connect or report["connection"]["feedback_valid"] else 1
    except KeyboardInterrupt:
        print("doctor interrupted; local subscriptions closed, no robot action requested", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"Aurora SDK doctor failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
