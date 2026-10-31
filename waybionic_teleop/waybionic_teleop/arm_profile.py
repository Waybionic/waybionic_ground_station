"""
How one physical arm differs from the arm map in config/arm_drives.yaml.

The map describes every drive the arm is designed with. A profile is a small YAML file that
says which of them this arm actually has on its bus, and holds still each joint that no
fitted drive moves, at the angle it is fixed at, so the model in RViz matches the arm:

    # The shoulder motor is not wired yet; the shoulder is fixed upright.
    fitted_drives: [base_yaw, elbow, wrist_left, wrist_right]
    locked_joints: {joint_2: 0.0, tool_grip: 0.0}

Angles are in radians, in the joint's URDF frame. The drive node publishes a locked joint at
its angle and refuses commands that would move it; teleop treats it as a joint whose limits
are that angle.
"""

from dataclasses import dataclass, field
import math

import yaml

KEYS = {'fitted_drives', 'locked_joints'}


@dataclass(frozen=True)
class ArmProfile:
    # None means every drive in the map is fitted.
    fitted_drives: tuple = None
    locked_joints: dict = field(default_factory=dict)


def load_profile(path):
    """Return the ArmProfile in the YAML file at path, or the default one for no path."""
    if not path:
        return ArmProfile()
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'cannot read the arm profile {path}: {error}') from None
    return profile_from_dict(data if data is not None else {}, path)


def profile_from_dict(data, source='the arm profile'):
    if not isinstance(data, dict):
        raise ValueError(f'{source} must be a mapping')
    unknown = sorted(set(data) - KEYS)
    if unknown:
        raise ValueError(f'{source} has unknown keys {unknown}; expected {sorted(KEYS)}')
    fitted = data.get('fitted_drives')
    if fitted is not None:
        if (not isinstance(fitted, list) or not fitted
                or any(not isinstance(name, str) for name in fitted)
                or len(set(fitted)) != len(fitted)):
            raise ValueError(f'{source}: fitted_drives must list distinct drive names')
        fitted = tuple(fitted)
    locked = data.get('locked_joints') or {}
    if not isinstance(locked, dict) or any(not isinstance(joint, str) for joint in locked):
        raise ValueError(f'{source}: locked_joints must map joint names to angles')
    angles = {}
    for joint, angle in locked.items():
        if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(
                angle):
            raise ValueError(f'{source}: locked joint {joint} needs a finite angle in radians')
        angles[joint] = float(angle)
    return ArmProfile(fitted, angles)


def fit_drive_parameters(params, profile):
    """
    Return the drive node's parameters with only the profile's fitted drives.

    Every joint that only an unfitted drive moves must be locked, or nothing would report
    where it is.
    """
    drives = list(params['drives'])
    if profile.fitted_drives is None:
        fitted = drives
    else:
        unknown = sorted(set(profile.fitted_drives) - set(drives))
        if unknown:
            raise ValueError(f'the arm profile fits drives {unknown} that the arm map lacks')
        fitted = [name for name in drives if name in profile.fitted_drives]

    def joints(names):
        try:
            return {joint for name in names for joint in params[f'{name}.joints']}
        except KeyError as missing:
            raise ValueError(f'missing drive parameter {missing}') from None
    driven = joints(fitted)
    moved = sorted(driven & set(profile.locked_joints))
    if moved:
        raise ValueError(f'locked joints {moved} are moved by fitted drives')
    loose = sorted(joints(drives) - driven - set(profile.locked_joints))
    if loose:
        raise ValueError(f'no fitted drive moves {loose}; lock them in the arm profile')
    return {**params, 'drives': fitted}
