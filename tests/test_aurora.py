"""Aurora 0.1.8 adapter, safety and CLI acceptance; no real DDS/session calls.

The previous configure/lease mock tests are replaced by the installed 0.1.8
contract. Motion, limit, freshness, ownership and shutdown coverage is retained.
"""
from dataclasses import asdict, replace
import importlib.metadata
import inspect
import logging
import math
from pathlib import Path
import subprocess
import sys
import threading
from types import SimpleNamespace

import pytest
import yaml

from sleeve_arm.control.controller import SafeArmController
from sleeve_arm.control.mapper import AuroraIntentMapper
from sleeve_arm.control.safety import SafetyError
from sleeve_arm.control.aurora_motion import AuroraMotionService, preview_joint
from sleeve_arm.domain.joint import DYMOTOR_JOINT_NAMES, JOINT_NAMES
from sleeve_arm.domain.motion import MotionIntent
from sleeve_arm.robot.aurora import AuroraRobotArm
from sleeve_arm.robot.aurora_fake import FakeAuroraClient, FakeClock, fake_profile, fake_session
from sleeve_arm.robot.aurora_profile import load_aurora_profile
from sleeve_arm.robot.aurora_session import AuroraSession, SystemClock
from sleeve_arm.robot.base import RobotError
from sleeve_arm.robot.factory import create_robot
from tools import aurora_control

ROOT = Path(__file__).resolve().parents[1]


def ready(*, sides=('left',), session=None):
    session = session or fake_session()
    robot = AuroraRobotArm(session, sides=sides)
    ctl = SafeArmController(robot, robot.config, clock=session.clock)
    ctl.connect(); ctl.enable()
    return session, robot, ctl, AuroraMotionService(ctl)


@pytest.fixture
def rig():
    objects = ready()
    yield objects
    objects[2].shutdown()


def step(rig, targets=None):
    session, _, ctl, _ = rig
    session.clock.sleep(.02)
    return ctl.set_joint_positions(targets or {'elbow_flexion': .002}, dt=.02)


def edit_joint(profile, **kwargs):
    group = profile.groups[0]
    return replace(profile, groups=(replace(group, joints=(replace(group.joints[0], **kwargs), *group.joints[1:])), *profile.groups[1:]))


@pytest.mark.parametrize('tool', ['aurora_control.py', 'aurora_sdk_doctor.py', 'run_model_control.py',
                                 'run_manual_model_control.py', 'record_sensors.py', 'read_imu.py'])
def test_help_never_imports_sdk(tool):
    script = """
import sys, runpy
class BlockSdk:
    def find_spec(self, fullname, *args):
        if fullname.startswith(('fourier_aurora_client', 'fastdds')):
            raise AssertionError('SDK imported during help')
sys.meta_path.insert(0, BlockSdk())
sys.argv = [sys.argv[1], '--help']
runpy.run_path(sys.argv[0], run_name='__main__')
"""
    result = subprocess.run([sys.executable, '-c', script, str(ROOT/'tools'/tool)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


def test_backend_imports_and_factory_fake_without_optional_sdk(monkeypatch):
    def forbidden(name):
        raise AssertionError('SDK import during construction')
    monkeypatch.setattr('sleeve_arm.robot.aurora_session.importlib.import_module', forbidden)
    AuroraSession(fake_profile())
    assert create_robot('fake').config.joints
    assert create_robot('dymotor').config.joints


def test_missing_sdk_is_clear_and_does_not_install(monkeypatch):
    def missing(name):
        raise importlib.metadata.PackageNotFoundError(name)
    monkeypatch.setattr(importlib.metadata, 'version', missing)
    session = AuroraSession(fake_profile())
    with pytest.raises(RobotError, match='SDK missing.*requirements-aurora'):
        session.attach(object(), {'FAKE_left_arm'})
    assert session.closed


@pytest.mark.parametrize('version', ['1.0.1', '0.1.1', '0.1.9'])
def test_wrong_sdk_version_rejected_before_import(monkeypatch, version):
    monkeypatch.setattr(importlib.metadata, 'version', lambda name: version)
    monkeypatch.setattr('sleeve_arm.robot.aurora_session.importlib.import_module', lambda name: pytest.fail('import'))
    session = AuroraSession(fake_profile())
    with pytest.raises(RobotError, match='unsupported SDK version'):
        session.attach(object(), {'FAKE_left_arm'})


def test_only_actual018_signatures_are_called(monkeypatch):
    session = fake_session()
    seen = []
    actual = session.sdk.AuroraClient.get_instance
    def factory(domain_id: int, participant_qos=None, robot_name=None, namespace=None, is_ros_compatible=None):
        seen.append((domain_id, robot_name, namespace, is_ros_compatible))
        return actual(domain_id, participant_qos, robot_name, namespace, is_ros_compatible)
    monkeypatch.setattr(session.sdk.AuroraClient, 'get_instance', staticmethod(factory))
    s, _, ctl, service = ready(session=session)
    service.move_joint(side='left', joint='elbow_flexion', angle_deg=2, duration_s=1)
    ctl.shutdown()
    assert seen == [(99, 'SYNTHETIC', None, False)]
    assert set(s.client.calls) == {'get_instance','get_fsm_state','get_group_state','set_group_cmd','close'}
    for name in ('configure', 'start', 'get_aurora_state', 'get_group_position', 'register_lease', 'release_lease'):
        assert not hasattr(FakeAuroraClient, name)


def test_installed018_method_contract_without_instantiating_client():
    sdk = pytest.importorskip('fourier_aurora_client')
    assert importlib.metadata.version('fourier_aurora_client') == '0.1.8'
    for name in ('get_instance','get_fsm_state','get_group_state','set_group_cmd','close'):
        assert tuple(inspect.signature(getattr(sdk.AuroraClient, name)).parameters) == tuple(
            inspect.signature(getattr(FakeAuroraClient, name)).parameters)


@pytest.mark.parametrize('kind', ['new_api', 'none', 'external', 'exception'])
def test_initialization_refuses_bad_clients_without_commands(monkeypatch, kind):
    s = fake_session()
    if kind == 'new_api':
        monkeypatch.setattr(s.sdk.AuroraClient, 'configure', lambda: pytest.fail('new API call'), raising=False)
    elif kind == 'none':
        monkeypatch.setattr(s.sdk.AuroraClient, 'get_instance', lambda **kwargs: None)
    elif kind == 'external':
        monkeypatch.setattr(s.sdk.AuroraClient, '_instance', object(), raising=False)
    else:
        def fail(**kwargs):
            raise RuntimeError('DDS failure')
        monkeypatch.setattr(s.sdk.AuroraClient, 'get_instance', fail)
    with pytest.raises((RobotError, RuntimeError)):
        AuroraRobotArm(s, sides=('left',)).connect()
    assert not s.fake_client.commands
    assert s.closed


@pytest.mark.parametrize('side', ['left','right'])
@pytest.mark.parametrize('direction,joint,sign', [('forward','shoulder_flexion',1),('backward','shoulder_flexion',-1),
                                                ('outward','shoulder_abduction',1),('inward','shoulder_abduction',-1)])
def test_side_direction_transform_and_inverse(side, direction, joint, sign):
    s, r, ctl, service = ready(sides=(side,))
    try:
        result = service.move_arm(side=side,direction=direction,angle_deg=4,duration_s=1)
        expected = sign*math.radians(4)
        assert result.submitted and result.arrived and not result.delivery_confirmed
        state = ctl.read_joint_states()[joint]
        assert state.position == pytest.approx(expected)
        assert state.current is state.error is state.state is state.bus is state.torque is None
        g, j = r._joints[joint]
        assert all(set(cmd) == {g.name} for cmd in s.client.commands)
        assert s.client.commands[-1][g.name][j.index] == pytest.approx(j.zero+j.sign*expected)
        if side == 'right':
            assert r._joints['shoulder_flexion'][1].sign != r._joints['shoulder_abduction'][1].sign
    finally:
        ctl.shutdown()


def test_relative_current_is_resolved_once(rig):
    _,_,ctl,service=rig
    service.move_joint(side='left',joint='elbow_flexion',angle_deg=5,duration_s=1)
    result=service.move_joint(side='left',joint='elbow_flexion',angle_deg=3,reference='current',duration_s=1)
    assert result.targets_rad['elbow_flexion'] == pytest.approx(math.radians(8))
    assert ctl.read_joint_states()['elbow_flexion'].position == pytest.approx(math.radians(8))
    service.move_joint(side='left',joint='elbow_flexion',angle_deg=2,duration_s=1)
    assert ctl.read_joint_states()['elbow_flexion'].position == pytest.approx(math.radians(2))


def test_dynamic_joint_set_preserves_dymotor_abi(rig):
    from sleeve_arm.robot.dymotor import _ArmOpenConfig, _JOINT_COUNT
    assert JOINT_NAMES is DYMOTOR_JOINT_NAMES and _JOINT_COUNT == 4
    assert dict(_ArmOpenConfig._fields_)['motor_ids']._length_ == 4
    assert len(rig[1].config.joints) == 6
    rig[3].wrist_joints(side='left',angles_deg={'wrist_pitch':2},duration_s=1)
    with pytest.raises(ValueError,match='physical wrist'):
        rig[3].wrist_joints(side='left',angles_deg={'elbow_flexion':2})


def test_partial_update_preserves_other_slots_and_previous_owner_targets(rig):
    s,r,ctl,service=rig
    before=r.read_group_positions('left')
    service.move_joint(side='left',joint='elbow_flexion',angle_deg=3,duration_s=1)
    service.move_joint(side='left',joint='shoulder_flexion',angle_deg=2,duration_s=1)
    final=s.client.commands[-1]['FAKE_left_arm']
    assert len(final)==7 and final[6]==before[6]==.17
    assert final[2]==pytest.approx(.3+math.radians(3))
    assert final[1]==before[1] and final[3:]==before[3:]


def test_group_multi_joint_merge_and_one_position_read_per_group(monkeypatch):
    s,r,ctl,_=ready(sides=('left','right'))
    observed=[]
    original=s.client.get_group_state
    def read(group_name,key='position'):
        observed.append((group_name,key))
        return original(group_name,key)
    monkeypatch.setattr(s.client,'get_group_state',read)
    s.clock.sleep(.02)
    ctl.set_joint_positions({'left.elbow_flexion':.002,'left.shoulder_flexion':.003,'right.elbow_flexion':.004},dt=.02)
    assert observed.count(('FAKE_left_arm','position'))==observed.count(('FAKE_right_arm','position'))==1
    assert len(s.client.commands)==1
    assert set(s.client.commands[0])=={'FAKE_left_arm','FAKE_right_arm'}
    sent=len(s.client.commands)
    s.clock.sleep(.02)
    with pytest.raises(SafetyError,match='outside limits'):
        ctl.set_joint_positions({'left.elbow_flexion':.004,'right.elbow_flexion':100},dt=.02)
    assert len(s.client.commands)==sent
    ctl.shutdown()


def test_cache_reads_do_not_refresh_age(rig):
    s,_,ctl,_=rig
    s.client.auto_feedback=False
    original=ctl.read_joint_states()['elbow_flexion'].received_at
    s.clock.sleep(.1)
    assert ctl.read_joint_states()['elbow_flexion'].received_at==original
    s.clock.sleep(.2)
    with pytest.raises(SafetyError,match='stale'):
        ctl.read_joint_states()
    assert s.closed and not ctl.enabled
    with pytest.raises(SafetyError,match='fault latched'):
        ctl.connect()


def test_first_read_does_not_establish_freshness():
    s=fake_session()
    s.fake_client.auto_feedback=False
    with pytest.raises(RobotError,match='fresh feedback timeout'):
        AuroraRobotArm(s,sides=('left',)).connect()
    assert s.fake_client.closed and not s.fake_client.commands
    assert s.clock.monotonic()<=106


@pytest.mark.parametrize('damage', ['dimension','nan','inf','missing','velocity','tracking','bounds','unmapped_tracking'])
def test_feedback_faults_latch_without_sending(rig,damage):
    s,r,ctl,_=rig
    g=s.client.groups['FAKE_left_arm']
    if damage=='dimension': g['position'].pop()
    if damage=='nan': g['position'][0]=math.nan
    if damage=='inf': g['position'][0]=math.inf
    if damage=='missing': del s.client.groups['FAKE_left_arm']
    if damage=='velocity': g['velocity']=[]
    if damage=='tracking': g['position'][0]+=.4
    if damage=='bounds': g['position'][6]=10
    if damage=='unmapped_tracking': g['position'][6]+=.4
    with pytest.raises(SafetyError): step(rig)
    assert ctl.fault and not ctl.enabled and not s.client.commands


@pytest.mark.parametrize('method',['get_fsm_state','get_group_state','set_group_cmd'])
def test_sdk_exceptions_latch_session_and_block_writes(rig,method):
    s,_,ctl,_=rig
    s.client.failures[method]=RuntimeError('injected SDK error')
    with pytest.raises(SafetyError,match=method): step(rig)
    assert s.fault and ctl.fault and not s.client.commands


def test_logged_sdk_publish_failure_is_not_silently_successful(rig,monkeypatch):
    s,_,ctl,_=rig
    def logged(position_cmd,velocity_cmd=None,torque_cmd=None):
        s._errors.emit(logging.LogRecord('fourier_aurora_client.dds_interface',logging.ERROR,'',0,'write failed',(),None))
        return None
    monkeypatch.setattr(s.client,'set_group_cmd',logged)
    with pytest.raises(SafetyError,match='SDK error log'): step(rig)
    assert s.closed


@pytest.mark.parametrize('result',[False,True,object()])
def test_unexpected_setter_result_never_treated_as_success(rig,monkeypatch,result):
    monkeypatch.setattr(rig[0].client,'set_group_cmd',lambda **kwargs:result)
    with pytest.raises(SafetyError,match='unexpected return'): step(rig)


def test_fsm_is_checked_again_immediately_before_each_submission(rig,monkeypatch):
    s,_,ctl,_=rig
    original=s.client.get_fsm_state
    calls=0
    def fsm():
        nonlocal calls
        calls+=1
        return 42 if calls<3 else 999
    monkeypatch.setattr(s.client,'get_fsm_state',fsm)
    with pytest.raises(SafetyError,match='FSM'): step(rig)
    assert calls>=3 and not s.client.commands


@pytest.mark.parametrize('field,value',[('verified',False),('allowed_fsm',()),('authority_verified',False),
    ('robot_type',None),('control_hz',None),('arrival_tolerance_rad',None),('stop_policy',None),('verification_note','')])
def test_unverified_execution_profile_rejected_before_sdk(field,value):
    profile=replace(fake_profile(),**{field:value})
    s=fake_session(profile=profile)
    with pytest.raises(ValueError): AuroraRobotArm(s,sides=('left',)).connect()
    assert not s.fake_client.calls


@pytest.mark.parametrize('field,value',[('index',None),('sign',None),('zero',None),('verified',False)])
def test_missing_joint_calibration_blocks_execute(field,value):
    p=edit_joint(fake_profile(),**{field:value})
    with pytest.raises(ValueError):
        p.validate(execute=True,simulation=True)


@pytest.mark.parametrize('field,value',[('max_velocity',None),('max_position_step',None),('max_tracking_error',None),
                                       ('min_position',None),('max_position',None),('max_current',1.),('require_motor_error',True)])
def test_missing_or_unsupported_safety_feedback_refused(field,value):
    p=fake_profile()
    p=edit_joint(p,limits=replace(p.groups[0].joints[0].limits,**{field:value}))
    with pytest.raises(ValueError): p.validate(execute=True,simulation=True)


@pytest.mark.parametrize('field,value',[('name',None),('count',None),('verified',False),('sdk_position_limits',None),('max_tracking_error',None)])
def test_unverified_group_execute_refused(field,value):
    p=fake_profile(); p=replace(p,groups=(replace(p.groups[0],**{field:value}),p.groups[1]))
    with pytest.raises(ValueError): p.validate(execute=True,simulation=True)


def test_other_unverified_side_does_not_block_verified_selected_side():
    p=fake_profile();p=replace(p,groups=(p.groups[0],replace(p.groups[1],verified=False)))
    p.validate(execute=True,simulation=True,groups=(p.groups[0],))
    with pytest.raises(ValueError): p.validate(execute=True,simulation=True,groups=(p.groups[1],))


def test_simulation_profile_cannot_authorize_real_execute():
    with pytest.raises(ValueError,match='simulated profile'):
        fake_profile().validate(execute=True)


def test_enable_is_only_application_gate_and_requires_confirmation():
    s=fake_session(execute=False)
    r=AuroraRobotArm(s,sides=('left',));r.connect()
    with pytest.raises(RobotError,match='read-only'):r.enable()
    assert not s.client.commands
    r.close();r.close()
    assert s.client.close_count==1


def test_unconfirmed_execute_is_blocked():
    s=fake_session();s.operator_confirmed=False
    r=AuroraRobotArm(s,sides=('left',));r.connect()
    with pytest.raises(RobotError,match='confirmation'):r.enable()
    assert not s.client.commands
    r.close()


def test_shared_session_refcount_ownership_and_idempotent_close():
    s=fake_session()
    left=AuroraRobotArm(s,sides=('left',));right=AuroraRobotArm(s,sides=('right',))
    left.connect();right.connect()
    duplicate=AuroraRobotArm(s,sides=('left',))
    with pytest.raises(RobotError,match='already belongs'):duplicate.connect()
    duplicate.close();left.close();left.close()
    assert not s.client.closed
    right.read_joint_states();right.close();right.close()
    assert s.client.calls.count('get_instance')==1 and s.client.close_count==1


def test_local_fault_does_not_write_or_close_healthy_other_side():
    s=fake_session()
    left=AuroraRobotArm(s,sides=('left',));right=AuroraRobotArm(s,sides=('right',))
    left.connect();right.connect();left.enable();right.enable()
    s.client.groups['FAKE_left_arm']['position'][0]=math.nan
    with pytest.raises(ValueError):left.read_joint_states()
    left.close()
    s.clock.sleep(.02)
    right.set_joint_position('elbow_flexion',.001)
    assert set(s.client.commands[-1])=={'FAKE_right_arm'}
    right.close()


def test_session_fsm_fault_blocks_both_views():
    s=fake_session();left=AuroraRobotArm(s,sides=('left',));right=AuroraRobotArm(s,sides=('right',))
    left.connect();right.connect();left.enable();right.enable()
    s.client.fsm=999
    with pytest.raises(RobotError,match='FSM'):left.read_joint_states()
    with pytest.raises(RobotError,match='fault'):right.set_joint_position('elbow_flexion',.001)
    assert not s.client.commands
    left.close();right.close()


def test_conflicting_singleton_config_and_closed_reuse_refused(monkeypatch):
    monkeypatch.setattr(AuroraSession,'_process_session',None)
    p=fake_profile()
    s=AuroraSession.real(p)
    assert AuroraSession.real(p) is s
    with pytest.raises(RobotError,match='conflicting'):AuroraSession.real(replace(p,connection={'domain_id':100}))
    s._close_client()
    with pytest.raises(RobotError,match='restart process'):AuroraSession.real(p)


def test_wrong_command_thread_rejected(rig):
    s,r,_,_=rig;s.clock.sleep(.02)
    errors=[]
    def wrong():
        try:r.set_joint_position('elbow_flexion',.001)
        except BaseException as exc:errors.append(exc)
    t=threading.Thread(target=wrong);t.start();t.join()
    assert errors and 'owner' in str(errors[0]) and not s.client.commands


@pytest.mark.parametrize('bad',[math.nan,math.inf,-math.inf])
def test_nonfinite_motion_parameters_rejected(rig,bad):
    for kwargs in ({'angle_deg':bad},{'angle_deg':1,'duration_s':bad}):
        with pytest.raises(ValueError):rig[3].move_joint(side='left',joint='elbow_flexion',**kwargs)
    with pytest.raises(ValueError):rig[3].swing_arm(side='left',joint='elbow_flexion',amplitude_deg=bad,period_s=2,cycles=1)
    assert not rig[0].client.commands


@pytest.mark.parametrize('kwargs',[{'cycles':0},{'cycles':1.5},{'cycles':True},{'cycles':1001},
                                   {'period_s':0},{'period_s':math.nan},{'amplitude_deg':0},{'center':math.inf}])
def test_illegal_swing_parameters_refused(rig,kwargs):
    args=dict(side='left',joint='elbow_flexion',amplitude_deg=2,period_s=2,cycles=1);args.update(kwargs)
    with pytest.raises(ValueError):rig[3].swing_arm(**args)
    assert not rig[0].client.commands


def test_swing_is_bounded_and_visits_both_extrema(rig):
    s,r,_,service=rig
    start=s.clock.monotonic()
    result=service.swing_arm(side='left',joint='elbow_flexion',amplitude_deg=3,period_s=2,cycles=2)
    values=[r._joints['elbow_flexion'][1].from_sdk(c['FAKE_left_arm'][2]) for c in s.client.commands]
    assert min(values)==pytest.approx(-math.radians(3),abs=1e-8)
    assert max(values)==pytest.approx(math.radians(3),abs=1e-8)
    assert result.arrived and s.clock.monotonic()-start==pytest.approx(8.,abs=.1)
    assert values[-1]==pytest.approx(0.)


@pytest.mark.parametrize('case',['direction','joint','side','reference','bounds','too_fast','hand','palm'])
def test_rejected_manual_requests_do_not_send(rig,case):
    service=rig[3]
    with pytest.raises((ValueError,RobotError)):
        if case=='direction':service.move_arm(side='left',direction='up',angle_deg=1)
        if case=='joint':service.move_joint(side='left',joint='unknown',angle_deg=1)
        if case=='side':service.move_joint(side='right',joint='elbow_flexion',angle_deg=1)
        if case=='reference':service.move_joint(side='left',joint='elbow_flexion',angle_deg=1,reference='delta')
        if case=='bounds':service.move_joint(side='left',joint='elbow_flexion',angle_deg=400)
        if case=='too_fast':service.move_joint(side='left',joint='elbow_flexion',angle_deg=100,duration_s=.02)
        if case=='hand':rig[1].set_hand_joint('left','index_flexion',.01)
        if case=='palm':service.wrist_joints(side='left',angles_deg={'palm':1})
    assert not rig[0].client.commands


def test_scheduling_delay_never_expands_per_step_allowance(rig,monkeypatch):
    s,_,ctl,service=rig
    original=s.clock.sleep
    monkeypatch.setattr(s.clock,'sleep',lambda seconds:original(seconds+.2))
    with pytest.raises(SafetyError,match='scheduling delay'):
        service.move_joint(side='left',joint='elbow_flexion',angle_deg=4,duration_s=1)
    assert not s.client.commands and s.closed and ctl.fault


def test_sdk_submission_is_not_arrival_and_wait_is_bounded(rig):
    s,_,_,service=rig;s.client.follow_commands=False
    start=s.clock.monotonic()
    with pytest.raises(SafetyError,match='arrival timed out'):
        service.move_joint(side='left',joint='elbow_flexion',angle_deg=.2,duration_s=1)
    assert s.clock.monotonic()-start<5 and s.closed


@pytest.mark.parametrize('error',[KeyboardInterrupt(),RuntimeError('control thread failure')])
def test_interrupt_and_thread_exception_close_without_extra_commands(rig,monkeypatch,error):
    s,_,ctl,service=rig
    def fail(seconds):raise error
    monkeypatch.setattr(s.clock,'sleep',fail)
    with pytest.raises(type(error)):
        service.move_joint(side='left',joint='elbow_flexion',angle_deg=2)
    assert s.closed and not s.client.commands and ctl.fault


def test_disable_immediately_blocks_and_close_error_is_reported(rig):
    s,r,_,_=rig
    r.disable()
    with pytest.raises(RobotError,match='not enabled'):r.set_joint_position('elbow_flexion',0.)
    s.client.failures['close']=RuntimeError('close failed')
    with pytest.raises(RuntimeError,match='close failed'):r.close()
    r.close()
    assert s.closed and not s.client.commands


def test_large_explicit_dt_does_not_bypass_backend_step_limit(rig):
    s,r,_,_=rig;s.clock.sleep(.02)
    with pytest.raises(RobotError,match='step/velocity'):r.set_joint_position('elbow_flexion',.1)
    assert not s.client.commands


def test_safety_clipping_is_not_claimed_complete(rig):
    s,_,ctl,_=rig;s.clock.sleep(.02)
    with pytest.raises(SafetyError,match='limited by safety'):
        ctl.set_joint_positions({'elbow_flexion':.2},dt=100,require_exact=True)
    assert not s.client.commands


def test_profile_roundtrip_and_unverified_template(tmp_path):
    path=tmp_path/'fake.yaml';path.write_text(yaml.safe_dump(asdict(fake_profile())))
    p=load_aurora_profile(path);p.validate(execute=True,simulation=True)
    raw=yaml.safe_load(path.read_text());raw['verified']='false';path.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError,match='boolean'):load_aurora_profile(path)
    p=load_aurora_profile(ROOT/'configs/robot_aurora.yaml')
    assert p.connection['domain_id'] is None and all(g.name is None and g.count is None for g in p.groups)
    with pytest.raises(ValueError,match='unverified'):p.validate(execute=True)


def test_mapper_routes_only_declared_source_side():
    s,r,ctl,_=ready(sides=('right',))
    mapper=AuroraIntentMapper(r,source_side='right',target_side='right')
    assert mapper.map(MotionIntent(timestamp=0,shoulder_flexion_rad=.02))=={'shoulder_flexion':.02}
    for source,target in [('left','left'),('right','left')]:
        with pytest.raises(ValueError,match='cross-side'):AuroraIntentMapper(r,source_side=source,target_side=target)
    s.clock.sleep(.02);ctl.set_joint_positions(mapper.map(MotionIntent(timestamp=0,elbow_flexion=.001)),dt=.02)
    assert set(s.client.commands[-1])=={'FAKE_right_arm'}
    ctl.shutdown()


def test_preview_is_pure_and_reports_complete_vectors():
    p=fake_profile();client=FakeAuroraClient(p)
    vector=client.groups['FAKE_right_arm']['position']
    result=preview_joint(p,side='right',joint='elbow_flexion',angle_deg=3,current_sdk=vector,fsm=42)
    assert result['sdk_target']==pytest.approx(.3-math.radians(3))
    assert result['complete_group_before'][6]==result['complete_group_after'][6]==.17
    assert not client.commands
    limited=preview_joint(p,side='right',joint='elbow_flexion',angle_deg=400,current_sdk=vector,fsm=42)
    assert limited['requested_semantic_position']!=limited['clamped_target']
    assert any('clamped' in reason for reason in limited['blocking_reasons'])


def test_default_cli_preview_never_loads_sdk(monkeypatch,capsys):
    monkeypatch.setattr(AuroraSession,'_load_sdk',lambda self:pytest.fail('SDK load'))
    assert aurora_control.main(['move','--side','left','--direction','forward','--angle-deg','10'])==0
    output=capsys.readouterr().out
    assert 'NO MOTION' in output and '"group": null' in output


def test_cli_fake_preview_and_bounded_simulation(capsys):
    args=['--backend','fake','joint','--side','right','--joint','elbow_flexion','--angle-deg','3']
    assert aurora_control.main(args)==0
    assert 'submitted' not in capsys.readouterr().out
    assert aurora_control.main(args+['--simulate'])==0
    assert '"arrived": true' in capsys.readouterr().out


def test_cli_unverified_execute_rejected_before_confirmation_and_sdk(monkeypatch):
    monkeypatch.setattr(AuroraSession,'_load_sdk',lambda self:pytest.fail('SDK load'))
    assert aurora_control.main(['joint','--side','left','--joint','elbow_flexion','--angle-deg','2','--execute'],
                              input_fn=lambda prompt:pytest.fail('confirmation before profile validation'))==1


def test_cli_doctor_reuses_sdk_doctor_offline(monkeypatch):
    from tools import aurora_sdk_doctor
    calls=[]
    monkeypatch.setattr(aurora_sdk_doctor,'main',lambda args:calls.append(args) or 0)
    assert aurora_control.main(['doctor'])==0
    assert calls==[[]]


@pytest.mark.parametrize("mode", ["normal", "prediction_error", "sensor_timeout"])
def test_model_entry_aurora_fake_routes_right_and_cleans_up(tmp_path, monkeypatch, mode):
    import yaml
    from tools import run_model_control
    from sleeve_arm.config import DEFAULT_SENSOR_CONFIG_PATH
    import sleeve_arm.robot.factory as factory
    from sleeve_arm.robot.aurora_session import SystemClock
    phase4 = tmp_path / "phase4.yaml"
    phase4.write_text(yaml.safe_dump({
        "predictor": {"backend": "flexarm_estimator", "flexarm_estimator": {
            "model_dir": str(tmp_path), "sleeve_channels": [3, 4, 5],
            "calibration_file": str(tmp_path / "unused.json"), "calibration_seconds": 0.1,
            "angle": {"min_deg": 0, "max_deg": 180}}},
        "phase4_validation": {"max_consecutive_prediction_errors": 3},
    }))
    sensors = yaml.safe_load(DEFAULT_SENSOR_CONFIG_PATH.read_text())
    sensors["upper_arm_rotation"]["enabled"] = False
    for name in ("imu1", "imu2", "imu3", "imu4"):
        sensors["sensors"][name]["enabled"] = False
    sensor_file = tmp_path / "sensors.yaml"
    sensor_file.write_text(yaml.safe_dump(sensors))
    session = fake_session(clock=SystemClock())
    actual_factory = factory.create_robot
    def create(backend, config=None, **kwargs):
        return actual_factory(backend, config, session=session, **kwargs)
    monkeypatch.setattr(factory, "create_robot", create)
    class Predictor:
        calls = 0
        def predict(self, sample):
            self.calls += 1
            if mode == "prediction_error" and self.calls > 1:
                raise ValueError("injected model failure")
            return MotionIntent(timestamp=sample.timestamp, shoulder_flexion_rad=0.03,
                                model_action="Forward", angle_deg=2, confidence=1, inference_ms=0.1)
    monkeypatch.setattr(run_model_control, "prepare_flexarm_predictor", lambda *a, **k: Predictor())
    if mode == "sensor_timeout":
        monkeypatch.setattr("sleeve_arm.control.SensorWatchdog.is_stale", lambda *a: True)
    monkeypatch.setattr(sys, "argv", [
        "run_model_control.py", "--robot", "aurora-fake", "--side", "right", "--source-side", "right",
        "--sleeve", "fake", "--imus", "fake", "--duration", "0.15",
        "--phase4-config", str(phase4), "--sensor-config", str(sensor_file),
    ])
    code = run_model_control.main()
    assert code == (0 if mode == "normal" else 1)
    assert session.closed
    assert "close" in session.client.calls
    assert all(set(c) == {"FAKE_right_arm"} for c in session.client.commands)
    assert len(session.client.commands) > (1 if mode == "normal" else 0)



@pytest.mark.parametrize('answer', ['YES','yes','YES ','','NO'])
def test_real_cli_requires_exact_yes_with_injected_fake_only(tmp_path,monkeypatch,answer):
    import sleeve_arm.robot.factory as factory
    profile=replace(fake_profile(),simulated=False)
    path=tmp_path/'site.yaml';path.write_text(yaml.safe_dump(asdict(profile)))
    session=fake_session(profile=profile)
    session.operator_confirmed=False
    original=factory.create_robot
    def fake_factory(backend,config=None,**kwargs):
        kwargs['session']=session
        return original(backend,config,**kwargs)
    monkeypatch.setattr(factory,'create_robot',fake_factory)
    monkeypatch.setattr(AuroraSession,'real',lambda *a,**k:pytest.fail('real SDK factory called'))
    prompts=[]
    def confirm(prompt):
        prompts.append(prompt)
        return answer
    code=aurora_control.main(['joint','--profile',str(path),'--side','left','--joint','elbow_flexion',
                             '--angle-deg','2','--duration','1','--execute'],input_fn=confirm)
    assert code==0 and prompts==['Type YES to continue: ']
    assert bool(session.client.commands)==(answer=='YES')
    assert session.closed


def test_fresh_snapshot_required_and_final_preserved_slots_checked(rig):
    s,r,ctl,_=rig
    old=ctl.read_joint_states()
    s.clock.sleep(.02)
    ctl.read_joint_states()
    with pytest.raises(RobotError,match='current complete feedback snapshot'):
        r.set_joint_positions_checked({'elbow_flexion':.001},old)
    assert not s.client.commands


def test_final_entire_vector_validated_before_any_sdk_call(rig):
    s,r,ctl,_=rig
    s.clock.sleep(.02)
    states=ctl.read_joint_states()
    r._targets['FAKE_left_arm'][6]=99  # Corrupt an unmapped preserved slot after feedback validation.
    with pytest.raises(ValueError,match='outside limits'):
        r.set_joint_positions_checked({'elbow_flexion':.001},states)
    assert not s.client.commands and r.fault


def test_velocity_and_effort_nonfinite_fail_before_commands(rig):
    s,_,_,_=rig
    s.client.groups['FAKE_left_arm']['velocity'][1]=math.nan
    with pytest.raises(SafetyError,match='finite'):step(rig)
    assert not s.client.commands


def test_new_identical_values_are_fresh_but_reads_are_not(rig):
    s,_,ctl,_=rig
    before=ctl.read_joint_states()['elbow_flexion'].received_at
    s.clock.sleep(.1)
    ctl.read_joint_states()
    s.clock.sleep(.1)
    now=ctl.read_joint_states()['elbow_flexion'].received_at
    assert now>before


def test_unknown_hand_group_does_not_get_inferred():
    s=fake_session()
    with pytest.raises(ValueError,match='unsupported'):
        AuroraRobotArm(s,sides=('left',),parts=('hand',))
    assert not s.fake_client.calls


@pytest.mark.parametrize('kwargs',[{'domain_id':None},{'domain_id':True},{'domain_id':-1},
                                   {'domain_id':2**32},{'domain_id':1,'topic_prefix':'aurora/'}])
def test_invalid_domain_or_new_sdk_connection_fields_refused(kwargs):
    with pytest.raises(ValueError):
        replace(fake_profile(),connection=kwargs).validate(execute=True,simulation=True)


def test_model_arm_side_alias_dry_run_does_not_create_robot_or_sensors(monkeypatch,capsys):
    from tools import run_model_control
    monkeypatch.setattr('sleeve_arm.robot.factory.create_robot',lambda *a,**k:pytest.fail('factory'))
    monkeypatch.setattr(run_model_control,'create_sleeve_source',lambda *a:pytest.fail('sensor'))
    monkeypatch.setattr(sys,'argv',['run_model_control.py','--robot','aurora','--arm-side','right',
                                  '--source-side','right','--aurora-profile',str(ROOT/'configs/robot_aurora.yaml')])
    assert run_model_control.main()==0
    assert 'NO MOTION' in capsys.readouterr().out


def test_operator_delay_can_reacquire_fresh_feedback_before_enable():
    s=fake_session()
    r=AuroraRobotArm(s,sides=('left',))
    ctl=SafeArmController(r,r.config,clock=s.clock)
    ctl.connect()
    s.clock.sleep(10.)  # Human confirmation/calibration, no active command owner.
    ctl.enable()
    step((s,r,ctl,None))
    assert s.client.commands
    ctl.shutdown()


def test_faulted_group_cannot_rejoin_a_live_shared_session():
    s=fake_session()
    left=AuroraRobotArm(s,sides=('left',));right=AuroraRobotArm(s,sides=('right',))
    left.connect();right.connect();left.enable()
    s.client.groups['FAKE_left_arm']['position'][0]=math.nan
    with pytest.raises(ValueError):left.read_joint_states()
    left.close()
    replacement=AuroraRobotArm(s,sides=('left',))
    with pytest.raises(RobotError,match='group fault latched'):replacement.connect()
    right.read_joint_states();right.close()
