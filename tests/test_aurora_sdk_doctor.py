"""0.1.8-shaped fakes: no DDS, robot, wall-clock waits or motion APIs."""
import importlib.metadata
import json
import math
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from tools import aurora_sdk_doctor as doctor

ROOT = Path(__file__).resolve().parents[1]


def forbidden(*args, **kwargs):
    raise AssertionError("session/client/write API must not be invoked")


class FakeAuroraClient018:
    """Methods/signatures come from the installed 0.1.8 client.py."""
    def __new__(cls, *args, **kwargs):
        forbidden()

    @classmethod
    def get_instance(cls, domain_id: int, participant_qos=None, robot_name=None,
                     namespace=None, is_ros_compatible=None):
        forbidden()

    def get_fsm_state(self) -> int:
        forbidden()

    def get_group_state(self, group_name: str, key: str = "position") -> list[float]:
        forbidden()

    def set_group_cmd(self, position_cmd, velocity_cmd=None, torque_cmd=None):
        forbidden()

    def close(self):
        forbidden()


def sdk_module():
    return SimpleNamespace(AuroraClient=FakeAuroraClient018, DDSInterface=forbidden,
                           __file__="fake/0.1.8/__init__.py")


def install_fake(monkeypatch, sdk=None, version="0.1.8"):
    sdk = sdk or sdk_module()
    monkeypatch.setattr(doctor.metadata, "version", lambda name: version)
    monkeypatch.setattr(doctor.importlib, "import_module", lambda name: sdk)
    return sdk


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        assert 0 <= seconds <= 0.02
        self.now += seconds


def aurora_message():
    return SimpleNamespace(whole_body_fsm_state=lambda: 7, upper_body_fsm_state=lambda: 0,
                           velocity_command_source=lambda: 2, allow_upper_body_override=lambda: False)


def group_message(position=None, velocity=None, name="observed_arm", duplicate=False):
    group = SimpleNamespace(group_name=lambda: name,
                            joint_position=lambda: [0.1, 0.2, 0.3] if position is None else position,
                            joint_velocity=lambda: [] if velocity is None else velocity,
                            joint_effort=lambda: [])
    return SimpleNamespace(group_state=lambda: [group, group] if duplicate else [group])


class ReadOnlyTransport:
    """Only public 0.1.8 subscriber contract; every other access fails."""
    def __init__(self, messages=True, matched=True, failure=None):
        self.messages, self.matched, self.failure = messages, matched, failure
        self.calls = []
        self.closed = 0

    def create_subscription(self, pub_sub_type, topic_name, callback, qos_profile=None):
        self.calls.append(topic_name)
        if topic_name == doctor.TOPICS[1] and self.failure:
            raise self.failure
        if self.messages:
            callback(aurora_message() if topic_name == doctor.TOPICS[0] else group_message())
        return SimpleNamespace(is_matched=self.matched)

    def close(self):
        self.closed += 1
        # Actual 0.1.8 returns None; do not invent an OperationResult.

    def __getattr__(self, name):
        raise AssertionError(f"unexpected DDS interface access: {name}")


def connected_rig(monkeypatch, transport=None):
    transport = transport or ReadOnlyTransport()
    sdk = sdk_module()
    created = []
    def create(**kwargs):
        created.append(kwargs)
        return transport
    sdk.DDSInterface = create
    sdk.SubscriberQosProfile = SimpleNamespace(best_effort=lambda: "audited-best-effort")
    monkeypatch.setattr(doctor.platform, "system", lambda: "Linux")
    monkeypatch.setattr(doctor.platform, "machine", lambda: "x86_64")
    clock = Clock()
    def loader(name):
        assert name in ("fourier_msgs.msg.AuroraState", "fourier_msgs.msg.MotionControlState")
        return SimpleNamespace(AuroraStatePubSubType=object, RobotControlGroupStatePubSubType=object)
    kwargs = dict(domain_id=123, timeout=0.1, clock=clock, sleep=clock.sleep, module_loader=loader)
    return sdk, transport, created, clock, kwargs


def test_missing_package_clear_install_hint(monkeypatch, capsys):
    def missing(name):
        raise importlib.metadata.PackageNotFoundError(name)
    monkeypatch.setattr(doctor.metadata, "version", missing)
    monkeypatch.setattr(doctor.importlib, "import_module", forbidden)
    assert doctor.main([]) == 1
    assert "python -m pip install -r requirements-aurora.txt" in capsys.readouterr().err


def test_import_failure_not_reported_as_success(monkeypatch, capsys):
    install_fake(monkeypatch)
    def broken(name):
        raise ImportError("missing native library")
    monkeypatch.setattr(doctor.importlib, "import_module", broken)
    assert doctor.main([]) == 1
    assert "missing native library" in capsys.readouterr().err


def test_offline_reports_actual_version_and_never_constructs_client(monkeypatch, capsys):
    install_fake(monkeypatch)
    monkeypatch.setattr(doctor, "read_only_connection", forbidden)
    assert doctor.main([]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["package_version"] == "0.1.8"
    assert report["dds_started"] is False
    assert report["api_family"] == doctor.LEGACY_FAMILY
    assert report["api"]["get_group_state"]["exists"]
    assert "group_name" in report["api"]["get_group_state"]["signature"]
    for name in ("configure", "start", "get_group_position", "register_lease"):
        assert report["api"][name] == {"exists": False, "signature": None}


def test_unverified_version_is_reported_without_fallback(monkeypatch):
    sdk = install_fake(monkeypatch, version="0.1.9")
    report, _ = doctor.offline_report()
    assert report["package_version"] == "0.1.9"
    assert report["connect_supported"] is False
    with pytest.raises(doctor.DoctorError, match="no automatic fallback"):
        doctor.read_only_connection(sdk, report, domain_id=123)


def test_native_signature_doc_fallback(monkeypatch):
    def no_signature(value):
        raise ValueError("native")
    monkeypatch.setattr(doctor.inspect, "signature", no_signature)
    item = doctor.api_surface(sdk_module())["get_instance"]
    assert "native binding" in item["signature"]
    assert "doc" in item


def test_mixed_api_refused_before_participant(monkeypatch):
    class Mixed(FakeAuroraClient018):
        configure = start = get_aurora_state = get_control_group_state = register_lease = forbidden
    sdk = sdk_module()
    sdk.AuroraClient = Mixed
    assert doctor.api_family(doctor.api_surface(sdk)) == "unknown-or-mixed"
    with pytest.raises(doctor.DoctorError, match="no automatic fallback"):
        doctor.read_only_connection(sdk, {"package_version": "0.1.8"}, domain_id=123)


def test_help_without_sdk():
    code = """
import sys, runpy
class BlockSdk:
    def find_spec(self, fullname, *args):
        if fullname.startswith(('fourier', 'fastdds')):
            raise AssertionError('SDK import during help')
sys.meta_path.insert(0, BlockSdk())
sys.argv = [sys.argv[1], '--help']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
    result = subprocess.run([sys.executable, "-c", code, str(ROOT / "tools/aurora_sdk_doctor.py")],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert "--connect" in result.stdout


@pytest.mark.parametrize("args", [["--domain-id", "123"], ["--connect"], ["--namespace", "test"]])
def test_modes_strictly_separated_before_import(monkeypatch, args):
    monkeypatch.setattr(doctor, "offline_report", forbidden)
    with pytest.raises(SystemExit) as exc:
        doctor.main(args)
    assert exc.value.code == 2


def test_connect_only_two_subscriptions_no_client_writers_or_leases(monkeypatch):
    sdk, transport, created, _, kwargs = connected_rig(monkeypatch)
    result = doctor.read_only_connection(sdk, {"package_version": "0.1.8"}, **kwargs)
    assert result["feedback_valid"] is True
    assert created == [dict(domain_id=123, namespace=None, is_ros_compatible=None)]
    assert transport.calls == list(doctor.TOPICS)
    assert transport.closed == 1
    assert result["robot_type"] is None
    groups = result["states"][doctor.TOPICS[1]]["data"]
    assert groups["observed_arm"]["joint_vector_length"] == 3
    assert result["states"][doctor.TOPICS[0]]["data"]["whole_body_fsm_state"] == 7


@pytest.mark.parametrize("messages,matched", [(False, True), (True, False), (False, False)])
def test_missing_or_unmatched_feedback_times_out_and_closes(monkeypatch, messages, matched):
    sdk, transport, _, clock, kwargs = connected_rig(monkeypatch, ReadOnlyTransport(messages, matched))
    result = doctor.read_only_connection(sdk, {"package_version": "0.1.8"}, **kwargs)
    assert not result["feedback_valid"]
    assert clock.now == pytest.approx(100.1)
    assert transport.closed == 1


@pytest.mark.parametrize("failure", [RuntimeError("DDS discovery failure"), KeyboardInterrupt()])
def test_partial_setup_exception_and_ctrl_c_close(monkeypatch, failure):
    sdk, transport, _, _, kwargs = connected_rig(monkeypatch, ReadOnlyTransport(failure=failure))
    with pytest.raises(type(failure)):
        doctor.read_only_connection(sdk, {"package_version": "0.1.8"}, **kwargs)
    assert transport.closed == 1


@pytest.mark.parametrize("field,value", [("domain_id", -1), ("domain_id", True), ("domain_id", 2**32),
                                        ("timeout", math.inf), ("timeout", 0), ("max_age", math.nan)])
def test_invalid_options_do_not_start_dds(field, value):
    kwargs = {"domain_id": 123, field: value}
    with pytest.raises(doctor.DoctorError):
        doctor.read_only_connection(sdk_module(), {"package_version": "0.1.8"}, **kwargs)


def test_repeated_reads_do_not_refresh_received_time_or_mutable_sdk_sample():
    clock = Clock()
    samples = doctor.StateSamples(clock)
    positions = [0.5]
    samples.groups(group_message(position=positions))
    positions[0] = 9.0  # SDK may reuse its SWIG message after the callback.
    assert samples.snapshot(1)[doctor.TOPICS[1]]["data"]["observed_arm"]["position"] == [0.5]
    clock.now += 2
    for _ in range(3):
        result = samples.snapshot(1)[doctor.TOPICS[1]]
        assert not result["valid"]
        assert result["age_s"] == 2
        assert result["sample_count"] == 1


@pytest.mark.parametrize("kwargs", [{"position": [math.nan]}, {"position": [math.inf]},
                                  {"position": []}, {"velocity": [1]}, {"duplicate": True}])
def test_malformed_samples_replace_previous_valid_feedback(kwargs):
    samples = doctor.StateSamples(Clock())
    samples.groups(group_message())
    samples.groups(group_message(**kwargs))
    result = samples.snapshot(1)[doctor.TOPICS[1]]
    assert result["error"]
    assert result["valid"] is False
    assert result["data"] is None


def test_cli_connect_is_explicit_and_invalid_feedback_fails(monkeypatch, capsys):
    install_fake(monkeypatch)
    calls = []
    def fake_connect(*args, **kwargs):
        calls.append(kwargs)
        return {"feedback_valid": False}
    monkeypatch.setattr(doctor, "read_only_connection", fake_connect)
    assert doctor.main(["--connect", "--domain-id", "123"]) == 1
    assert calls[0]["domain_id"] == 123
    assert json.loads(capsys.readouterr().out)["mode"] == "read-only-connection"


def test_installed_sdk_offline_with_transport_constructors_blocked(monkeypatch):
    sdk = pytest.importorskip("fourier_aurora_client")
    if importlib.metadata.version("fourier_aurora_client") != "0.1.8":
        pytest.skip("installed package is not the audited 0.1.8")
    import fastdds
    monkeypatch.setattr(fastdds.DomainParticipantFactory, "get_instance", forbidden)
    monkeypatch.setattr(sdk, "DDSInterface", forbidden)
    monkeypatch.setattr(sdk.AuroraClient, "get_instance", forbidden)
    report, _ = doctor.offline_report()
    assert report["import_ok"] and report["dds_started"] is False
