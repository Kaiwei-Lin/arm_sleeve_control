#!/usr/bin/env python3
"""Aurora 0.1.8: offline preview by default; real writes need --execute AND YES."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
ROOT = Path(__file__).resolve().parents[1]


def parser():
    common = argparse.ArgumentParser(add_help=False)
    # SUPPRESS allows options either before or after the subcommand.
    common.add_argument('--backend', choices=('aurora', 'fake'), default=argparse.SUPPRESS)
    common.add_argument('--profile', type=Path, default=argparse.SUPPRESS)
    common.add_argument('--execute', action='store_true', default=argparse.SUPPRESS)
    common.add_argument('--simulate', action='store_true', default=argparse.SUPPRESS,
                        help='run a bounded fake trajectory; requires --backend fake')
    common.add_argument('--connect', action='store_true', default=argparse.SUPPRESS,
                        help='explicit read-only DDS preview/doctor; never authorizes motion')
    common.add_argument('--domain-id', type=int, default=argparse.SUPPRESS)
    p = argparse.ArgumentParser(description=__doc__, parents=[common])
    sub = p.add_subparsers(dest='command', required=True)
    sub.add_parser('doctor', parents=[common], help='reuse aurora_sdk_doctor; offline unless --connect')
    for name in ('move', 'joint', 'swing', 'hand-joints'):
        cmd = sub.add_parser(name, parents=[common])
        cmd.add_argument('--side', choices=('left', 'right'), required=True)
        if name == 'move':
            cmd.add_argument('--direction', choices=('forward', 'backward', 'outward', 'inward'), required=True)
        else:
            cmd.add_argument('--joint', required=True)
        if name == 'swing':
            cmd.add_argument('--amplitude-deg', type=float, required=True)
            cmd.add_argument('--period', type=float, required=True)
            cmd.add_argument('--cycles', type=int, required=True)
            cmd.add_argument('--center-rad', type=float)
            cmd.add_argument('--settle-duration', type=float, default=2.)
        else:
            cmd.add_argument('--angle-deg', type=float, required=True)
            cmd.add_argument('--reference', choices=('neutral', 'current'), default='neutral')
            cmd.add_argument('--duration', type=float, default=2.)
    return p


def main(argv=None, *, input_fn=None):
    p = parser()
    args = p.parse_args(argv)
    for key, value in dict(backend='aurora', profile=ROOT/'configs/robot_aurora.yaml', execute=False,
                           simulate=False, connect=False, domain_id=None).items():
        if not hasattr(args, key):
            setattr(args, key, value)
    if args.backend == 'fake' and args.connect:
        p.error('fake never opens a real DDS connection')
    if args.connect and args.execute:
        p.error('--connect is read-only; use --execute separately after preview')
    if args.simulate and (args.backend != 'fake' or args.execute):
        p.error('--simulate requires fake and cannot be combined with --execute')
    if args.execute and args.backend != 'aurora':
        p.error('use --simulate for offline fake; --execute is for the real backend')
    if args.command == 'doctor':
        if args.execute or args.simulate:
            p.error('doctor never executes motion')
        from tools import aurora_sdk_doctor
        options = []
        if args.connect:
            if args.domain_id is None:
                p.error('doctor --connect requires --domain-id')
            options = ['--connect', '--domain-id', str(args.domain_id)]
        return aurora_sdk_doctor.main(options)
    controller = None
    try:
        from sleeve_arm.robot.aurora_profile import load_aurora_profile, finite
        from sleeve_arm.robot.aurora_fake import fake_profile, fake_session
        from sleeve_arm.control.aurora_motion import AuroraMotionService, preview_joint
        from sleeve_arm.control.controller import SafeArmController
        from sleeve_arm.robot.factory import create_robot
        profile = fake_profile() if args.backend == 'fake' else load_aurora_profile(args.profile)
        if args.domain_id is not None and args.domain_id != profile.connection.get('domain_id'):
            raise ValueError('domain override conflicts with profile; edit and verify profile explicitly')
        if args.command == 'hand-joints':
            if args.execute or args.simulate:
                raise ValueError('hand execute unsupported in this CLI: no site-calibrated hand profile')
            print(json.dumps({'status': 'NO MOTION', 'hand': 'unsupported: no verified hand mapping'}))
            return 0
        if args.command == 'move':
            directions = {'forward': ('shoulder_flexion', 1), 'backward': ('shoulder_flexion', -1),
                          'outward': ('shoulder_abduction', 1), 'inward': ('shoulder_abduction', -1)}
            joint, sign = directions[args.direction]
            if finite(args.angle_deg, 'angle_deg') < 0:
                raise ValueError('directional angle must be nonnegative')
            angle = sign * args.angle_deg
        else:
            joint = args.joint
            angle = args.angle_deg if args.command == 'joint' else 0.
        reference = args.reference if args.command != 'swing' else 'current'
        if args.command == 'swing':
            finite(args.amplitude_deg, 'amplitude', positive=True)
            if not 0 < finite(args.period, 'period', positive=True) <= 3600 or not 1 <= args.cycles <= 1000 or args.cycles * args.period > 3600:
                raise ValueError('swing period/cycles must be bounded')
            finite(args.settle_duration, 'settle duration', positive=True)
            if args.center_rad is not None:
                finite(args.center_rad, 'center_rad')
        elif not 0 < finite(args.duration, 'duration', positive=True) <= 3600:
            raise ValueError('duration must be <= 3600s')
        group = profile.selected_groups((args.side,))[0]
        # Reject unverified execution BEFORE importing SDK or opening a session.
        if args.execute or args.simulate:
            profile.validate(execute=True, simulation=args.simulate, groups=(group,))
        before, fsm, diagnostic = None, None, None
        if args.backend == 'fake' or args.execute:
            session = fake_session(execute=args.simulate) if args.backend == 'fake' else None
            robot = create_robot('aurora-fake' if args.backend == 'fake' else 'aurora',
                                 session=session, profile=profile, side=args.side,
                                 execute=args.execute, operator_confirmed=False)
            controller = SafeArmController(robot, robot.config, clock=robot.session.clock)
            controller.connect()
            controller.read_joint_states()
            before = list(robot._snapshot.groups[group.name].position)
            fsm = robot._snapshot.fsm
        elif args.connect:
            from tools import aurora_sdk_doctor as doctor
            report, sdk = doctor.offline_report()
            diagnostic = doctor.read_only_connection(sdk, report, domain_id=profile.connection.get('domain_id'),
                namespace=profile.connection.get('namespace'), ros_compatible=profile.connection.get('is_ros_compatible'))
            data = diagnostic['states']['robot_control_group_state']['data']
            if diagnostic['feedback_valid'] and group.name in (data or {}):
                before = data[group.name]['position']
                fsm = diagnostic['states']['aurora_state']['data']['whole_body_fsm_state']
        preview = preview_joint(profile, side=args.side, joint=joint, angle_deg=angle,
                                reference=reference, current_sdk=before, fsm=fsm)
        if args.command == 'swing':
            center = preview['current_semantic_position'] if args.center_rad is None else args.center_rad
            amplitude = math.radians(args.amplitude_deg)
            preview['swing'] = dict(amplitude_rad=amplitude, period_s=args.period, cycles=args.cycles,
                center_rad=center, endpoints=None if center is None else [center-amplitude, center+amplitude],
                total_trajectory_s=args.period*args.cycles + 2*args.settle_duration)
        preview['simulation'] = args.backend == 'fake'
        if diagnostic is not None:
            preview['read_only_diagnostic'] = diagnostic
        print(json.dumps(preview, indent=2, allow_nan=False))
        if not (args.execute or args.simulate):
            return 0
        if preview['blocking_reasons']:
            raise ValueError('; '.join(preview['blocking_reasons']))
        if args.execute:
            answer = (input if input_fn is None else input_fn)('Type YES to continue: ')
            if answer != 'YES':
                print('Cancelled: NO MOTION')
                return 0
            robot.session.operator_confirmed = True
        controller.enable()
        service = AuroraMotionService(controller)
        if args.command == 'swing':
            result = service.swing_arm(side=args.side, joint=joint, amplitude_deg=args.amplitude_deg,
                period_s=args.period, cycles=args.cycles, center='current' if args.center_rad is None else args.center_rad,
                settle_duration_s=args.settle_duration)
        else:
            result = service.move_joint(side=args.side, joint=joint, angle_deg=angle,
                                        reference=reference, duration_s=args.duration)
        print(json.dumps({'simulation': args.simulate, **asdict(result)}, indent=2))
        return 0
    except KeyboardInterrupt:
        print('Interrupted: writes blocked; physical stop unconfirmed', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        if controller is not None:
            try:
                controller.shutdown()
            except BaseException as exc:
                print(f'ERROR: cleanup failed; physical stop unconfirmed: {exc}', file=sys.stderr)
                return 1


if __name__ == '__main__':
    raise SystemExit(main())
