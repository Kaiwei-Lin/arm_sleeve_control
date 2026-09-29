"""Offline SDK 0.1.8 contract: cached list getters and a None-returning setter."""
from dataclasses import replace
from types import SimpleNamespace

from sleeve_arm.control.parameters import JointSafetyParameters
from sleeve_arm.robot.aurora_profile import API_FAMILY, SDK_VERSION, AuroraGroup, AuroraJoint, AuroraRobotProfile
from sleeve_arm.robot.aurora_session import AuroraSession


class FakeClock:
    def __init__(self):
        self.now = 100.0
    def monotonic(self):
        return self.now
    def sleep(self, seconds):
        if seconds < 0:
            raise ValueError('negative sleep')
        self.now += seconds


def fake_profile():
    groups = []
    names = ('shoulder_flexion', 'shoulder_abduction', 'elbow_flexion', 'upper_arm_rotation', 'wrist_pitch', 'wrist_yaw')
    for side in ('left', 'right'):
        joints = tuple(AuroraJoint(
            name=name, index=i, sign=(1 if side == 'left' else (-1 if i % 2 == 0 else 1)),
            zero=0.1 * (i + 1), limits=JointSafetyParameters(
                name, -2.5, 2.5, max_velocity=3., max_position_step=.08, max_tracking_error=.3),
            verified=True, kind='wrist' if name.startswith('wrist') else 'arm',
        ) for i, name in enumerate(names))
        # Synthetic seventh slot has NO semantic joint; preservation is tested.
        groups.append(AuroraGroup(name=f'FAKE_{side}_arm', side=side, count=7,
                                  joints=joints, sdk_position_limits=tuple([(-4., 4.)] * 7),
                                  max_tracking_error=.3, verified=True,
                                  capabilities=('joint_position', 'wrist_orientation')))
    return AuroraRobotProfile(
        API_FAMILY, SDK_VERSION, {'domain_id': 99, 'robot_name': 'SYNTHETIC', 'is_ros_compatible': False},
        tuple(groups), allowed_fsm=(42,), control_hz=50., arrival_tolerance_rad=.002,
        stop_policy='stop_publishing', verified=True, simulated=True,
        verification_note='SYNTHETIC TEST ONLY; NOT HARDWARE PARAMETERS',
        robot_type='FAKE', authority_verified=True,
        smooth_limits=False,  # Legacy strict-mode contract tests; GR3 uses its site profile.
    )


def gr3_fake_profile():
    """Use the real GR3 layout with explicitly simulated calibration/limits."""
    from pathlib import Path
    from sleeve_arm.robot.aurora_profile import load_aurora_profile
    profile = load_aurora_profile(Path(__file__).resolve().parents[2] / 'configs/robot_aurora.yaml')
    groups = tuple(replace(g, verified=True, joints=tuple(replace(j, verified=True) for j in g.joints))
                   for g in profile.groups)
    return replace(profile, groups=groups, verified=True, simulated=True, authority_verified=True,
                   allow_missing_velocity_cmd=False, verification_note='OFFLINE SIMULATION ONLY')


class FakeAuroraClient:
    def __init__(self, profile=None, clock=None):
        self.profile = profile or fake_profile()
        self.clock = clock or FakeClock()
        self.groups = {}
        for group in self.profile.groups:
            vector = [.17] * group.count
            for joint in group.joints:
                initial = min(joint.limits.max_position, max(joint.limits.min_position, 0.))
                vector[joint.index] = joint.to_sdk(initial)
            self.groups[group.name] = {'position': vector, 'velocity': [0.] * group.count, 'effort': []}
        self.fsm = self.profile.allowed_fsm[0] if self.profile.allowed_fsm else 0
        self.upper_fsm = 0
        self.stand_pose = [0.0, 0.0, 0.0, 101.0]
        self.closed = False
        self.close_count = 0
        self.calls = []
        self.commands = []
        self.failures = {}
        self.auto_feedback = True
        self.follow_commands = True
        self._last_update = self.clock.monotonic()
        self._pending = False
        self._pending_vectors = {}

    @classmethod
    def get_instance(cls, domain_id: int, participant_qos=None, robot_name=None,
                     namespace=None, is_ros_compatible=None):
        raise AssertionError('inject a fake SDK factory; do not create a process singleton')

    def _call(self, name):
        self.calls.append(name)
        if name in self.failures:
            raise self.failures[name]
        if self.closed and name != 'close':
            raise RuntimeError('client closed')

    def get_fsm_state(self) -> int:
        self._call('get_fsm_state')
        return self.fsm

    def get_upper_fsm_state(self) -> int:
        self._call('get_upper_fsm_state')
        return self.upper_fsm

    def get_stand_pose(self) -> list[float]:
        self._call('get_stand_pose')
        return self.stand_pose

    def set_fsm_state(self, state: int):
        self._call('set_fsm_state')
        self.fsm = state

    def get_group_state(self, group_name: str, key: str = 'position') -> list[float]:
        self._call('get_group_state')
        now = self.clock.monotonic()
        if self.auto_feedback and (now > self._last_update or self._pending):
            for cache_name, group in self.groups.items():
                if cache_name in self._pending_vectors:
                    group['position'] = self._pending_vectors[cache_name]
                for name, vector in tuple(group.items()):
                    group[name] = list(vector)  # Like actual callback, replace cached lists.
            self._pending_vectors.clear()
            self._last_update, self._pending = now, False
        return self.groups[group_name][key]

    def set_group_cmd(self, position_cmd: dict[str, list[float]],
                      velocity_cmd: dict[str, list[float]] | None = None,
                      torque_cmd: dict[str, list[float]] | None = None):
        self._call('set_group_cmd')
        self.commands.append({k: list(v) for k, v in position_cmd.items()})
        if self.follow_commands:
            for name, vector in position_cmd.items():
                # Model a new state message AFTER submission, not an in-place cache edit.
                self._pending_vectors[name] = list(vector)
            self._pending = True
        return None

    def close(self):
        self._call('close')
        self.closed = True
        self.close_count += 1


def fake_session(*, execute=True, clock=None, profile=None):
    clock = clock or FakeClock()
    profile = profile or fake_profile()
    client = FakeAuroraClient(profile, clock)
    class Factory(FakeAuroraClient):
        @classmethod
        def get_instance(cls, domain_id: int, participant_qos=None, robot_name=None,
                         namespace=None, is_ros_compatible=None):
            client.calls.append('get_instance')
            return client
    session = AuroraSession(profile, sdk=SimpleNamespace(AuroraClient=Factory), clock=clock,
                            execute=execute, operator_confirmed=True, simulation=True)
    # Exposed solely for failure injection before connect, not an SDK API.
    session.fake_client = client
    return session
