"""Narrow, pinned 0.1.8 DDS initialization compatibility.

Aligned with electronic_skin_project's validated Aurora backend. This is the
only private SDK compatibility hook; restore it even if initialization fails.
"""
from __future__ import annotations

import importlib
import logging
import os
import threading

from sleeve_arm.robot.base import RobotError

_initialization_lock = threading.RLock()


def configure_dds_environment():
    os.environ["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    os.environ["FOURIERDDS_ROS_COMPATIBLE"] = "false"
    os.environ["FOURIERDDS_USE_DISCOVERY_SERVER"] = "false"


def velocity_cmd_only_mismatch(names):
    """Accept a proven list of unmatched publishers, never a fuzzy error string."""
    return (isinstance(names, (list, tuple)) and len(names) == 1
            and isinstance(names[0], str) and names[0].strip().rstrip("/").rsplit("/", 1)[-1] == "velocity_cmd")


def _unmatched_publishers(client):
    publishers = getattr(client, "_publisher_list", None)
    if not isinstance(publishers, (list, tuple)) or not publishers:
        return None
    unmatched = []
    for publisher in publishers:
        if not hasattr(publisher, "is_matched"):
            return None
        matched = publisher.is_matched
        if callable(matched):
            matched = matched()
        if matched:
            continue
        topic = getattr(publisher, "_topic", None)
        if not callable(getattr(topic, "get_name", None)):
            return None
        name = topic.get_name()
        if not isinstance(name, str) or not name.strip():
            return None
        unmatched.append(name)
    return unmatched


def initialize_client(client_class, profile, *, accepted_velocity_mismatch=lambda: None):
    with _initialization_lock:
        if not profile.allow_missing_velocity_cmd:
            return client_class.get_instance(**profile.connection)
        if profile.sdk_version != "0.1.8":
            raise RobotError("velocity_cmd compatibility requires exact SDK 0.1.8")
        # The caller has already checked the exact installed SDK version.
        exceptions = importlib.import_module("fourier_aurora_client.exceptions")
        timeout = getattr(exceptions, "DDSMatchTimeout", None)
        original = getattr(client_class, "_init_publishers", None)
        if not isinstance(timeout, type) or not issubclass(timeout, Exception) or not callable(original):
            raise RobotError("0.1.8 private publisher API incompatible with velocity_cmd compatibility")

        def initialize_publishers(instance, *args, **kwargs):
            try:
                return original(instance, *args, **kwargs)
            except timeout:
                if not velocity_cmd_only_mismatch(_unmatched_publishers(instance)):
                    raise
                accepted_velocity_mismatch()
                logging.getLogger(__name__).warning(
                    "Only velocity_cmd publisher is unmatched; continuing without velocity control"
                )

        client_class._init_publishers = initialize_publishers
        try:
            return client_class.get_instance(**profile.connection)
        finally:
            client_class._init_publishers = original
