"""Aurora angular service; all trajectories use the common SafeArmController."""
from sleeve_arm.control.upper_limb import MotionResult, UpperLimbService

AuroraMotionService = UpperLimbService
__all__ = ['AuroraMotionService', 'MotionResult']


def preview_joint(profile, *, side, joint, angle_deg, reference='neutral', current_sdk=None, fsm=None):
    """Pure calculation. Unknown calibration/current values remain None."""
    import math
    from sleeve_arm.robot.aurora_profile import finite
    angle = math.radians(finite(angle_deg, 'angle_deg'))
    if reference not in ('neutral', 'current'):
        raise ValueError('reference must be neutral/current')
    group = profile.selected_groups((side,))[0]
    entry = next((j for j in group.joints if j.name == joint), None)
    if entry is None:
        raise ValueError(f'unsupported joint: {joint}')
    before = None if current_sdk is None else list(current_sdk)
    if before is not None:
        group.check_vector(before)
    current, sdk_current, sdk_target, after = None, None, None, None
    if before is not None and entry.index is not None:
        sdk_current = before[entry.index]
        if entry.sign is not None and entry.zero is not None:
            current = entry.from_sdk(sdk_current)
    requested = angle if reference == 'neutral' else (None if current is None else current + angle)
    clamped = requested
    if requested is not None:
        lo, hi = entry.limits.min_position, entry.limits.max_position
        if lo is not None:
            clamped = max(clamped, lo)
        if hi is not None:
            clamped = min(clamped, hi)
        if entry.index is not None and entry.sign is not None and entry.zero is not None:
            sdk_target = entry.to_sdk(clamped)
            if before is not None:
                after = list(before)
                after[entry.index] = sdk_target
    issues = []
    try:
        profile.validate(execute=True, simulation=profile.simulated, groups=(group,))
    except ValueError as exc:
        issues.append(str(exc))
    if before is None:
        issues.append('current state unavailable: no live connection/snapshot')
    if requested != clamped:
        issues.append('outside limits: clamped value is diagnostic only; execute rejects this request')
    if fsm not in profile.allowed_fsm:
        issues.append('FSM unavailable/not allowed')
    return dict(status='NO MOTION', side=side, group=group.name, joint=joint, index=entry.index,
                reference=reference, current_sdk_position=sdk_current, current_semantic_position=current,
                requested_angle_rad=angle, requested_semantic_position=requested,
                clamped_target=clamped, sdk_target=sdk_target,
                complete_group_before=before, complete_group_after=after, fsm=fsm,
                blocking_reasons=issues)
