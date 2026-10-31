"""An arm profile says which drives one arm has and holds still the joints it lacks."""

import pytest

from waybionic_teleop.arm_profile import (
    ArmProfile,
    fit_drive_parameters,
    load_profile,
    profile_from_dict,
)


def test_no_profile_fits_every_drive(parameters):
    params = parameters('arm_drives.yaml', 'sim_arm_drives')
    assert load_profile('') == ArmProfile()
    assert fit_drive_parameters(params, ArmProfile()) == params


def test_the_bench_profile_runs_four_drives_and_holds_the_shoulder(parameters, bench_profile):
    params = parameters('arm_drives.yaml', 'sim_arm_drives')
    profile = load_profile(bench_profile)
    assert profile.locked_joints == {'joint_2': 0.0, 'tool_grip': 0.0}
    fitted = fit_drive_parameters(params, profile)
    assert fitted['drives'] == ['base_yaw', 'elbow', 'wrist_left', 'wrist_right']
    assert params['drives'][1] == 'shoulder'


@pytest.mark.parametrize('data, message', [
    ([], 'must be a mapping'),
    ({'fitted': ['base_yaw']}, 'unknown keys'),
    ({'fitted_drives': []}, 'distinct drive names'),
    ({'fitted_drives': ['elbow', 'elbow']}, 'distinct drive names'),
    ({'fitted_drives': 'elbow'}, 'distinct drive names'),
    ({'locked_joints': ['joint_2']}, 'map joint names'),
    ({'locked_joints': {'joint_2': float('nan')}}, 'finite angle'),
    ({'locked_joints': {'joint_2': True}}, 'finite angle'),
    ({'locked_joints': {'joint_2': 'up'}}, 'finite angle'),
])
def test_a_malformed_profile_is_refused(data, message):
    with pytest.raises(ValueError, match=message):
        profile_from_dict(data)


def test_an_unreadable_profile_is_refused(tmp_path):
    with pytest.raises(ValueError, match='cannot read'):
        load_profile(str(tmp_path / 'missing.yaml'))
    broken = tmp_path / 'broken.yaml'
    broken.write_text('fitted_drives: [base_yaw\n', encoding='utf-8')
    with pytest.raises(ValueError, match='cannot read'):
        load_profile(str(broken))


def test_an_empty_profile_file_fits_every_drive(tmp_path):
    empty = tmp_path / 'empty.yaml'
    empty.write_text('', encoding='utf-8')
    assert load_profile(str(empty)) == ArmProfile()


@pytest.mark.parametrize('profile, message', [
    # Nothing would report where the shoulder is.
    (ArmProfile(('base_yaw', 'elbow', 'wrist_left', 'wrist_right', 'tool')), r"\['joint_2'\]"),
    (ArmProfile(('base_yaw', 'elbow', 'gripper')), 'lacks'),
    # A fitted drive would move a joint the model holds still.
    (ArmProfile(None, {'joint_1': 0.0}), 'moved by fitted drives'),
    # One wrist motor moves both wrist joints, so neither can be locked while it is fitted.
    (ArmProfile(('base_yaw', 'shoulder', 'elbow', 'wrist_left', 'tool'), {'joint_4': 0.0}),
     'moved by fitted drives'),
])
def test_a_profile_that_does_not_match_the_map_is_refused(parameters, profile, message):
    with pytest.raises(ValueError, match=message):
        fit_drive_parameters(parameters('arm_drives.yaml', 'sim_arm_drives'), profile)
