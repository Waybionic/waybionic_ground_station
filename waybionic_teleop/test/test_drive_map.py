"""Joint positions to drive encoder counts with the arm's drive map."""

import math

import pytest

from waybionic_teleop import drive_map, mks_can

COUNTS = mks_can.COUNTS_PER_REV


@pytest.fixture
def arm_map(parameters):
    return drive_map.drive_map_from_parameters(
        parameters('arm_drives.yaml', 'sim_arm_drives'), COUNTS)


def test_the_map_covers_every_arm_joint_with_the_cad_gear_ratios(arm_map):
    assert arm_map.joints == ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'tool_grip']
    assert [drive.can_id for drive in arm_map.drives] == [1, 2, 3, 4, 5, 6]
    # 26:1 cycloidal reducers on the base, shoulder and elbow; 3:1 belts on the wrist motors.
    assert [drive.gear_ratio for drive in arm_map.drives] == [26.0, 26.0, 26.0, 3.0, 3.0, 1.0]


def test_positions_round_trip_through_counts(arm_map):
    positions = dict(zip(arm_map.joints, [0.5, -1.2, 0.3, 0.7, -0.4, 0.8]))
    decoded = arm_map.to_positions(arm_map.to_counts(positions))
    assert all(abs(decoded[joint] - positions[joint]) <= 2 * math.pi / COUNTS
               for joint in positions)


def test_wrist_differential_mixes_pitch_and_roll(arm_map):
    zero = dict.fromkeys(arm_map.joints, 0.0)
    # A quarter turn of the wrist is three quarters of a turn of each motor through the belts.
    turn = 3 * COUNTS // 4
    assert arm_map.to_counts({**zero, 'joint_4': math.pi / 2})[3:5] == [turn, turn]
    assert arm_map.to_counts({**zero, 'joint_5': math.pi / 2})[3:5] == [-turn, turn]


def test_gear_ratio_scales_counts_and_speed():
    base = drive_map.DriveMap([drive_map.Drive('base', 1, 50.0, {'joint_1': 1.0})], COUNTS)
    assert base.to_counts({'joint_1': 2 * math.pi}) == [50 * COUNTS]
    assert base.to_rpm({'joint_1': -2 * math.pi}) == pytest.approx([3000.0])


@pytest.mark.parametrize('drives', [
    [drive_map.Drive('a', 1, 1.0, {'j1': 1.0}), drive_map.Drive('b', 1, 1.0, {'j2': 1.0})],
    [drive_map.Drive('a', 1, 1.0, {'j1': 1.0, 'j2': 1.0}),
     drive_map.Drive('b', 2, 1.0, {'j1': 2.0, 'j2': 2.0})],
    [drive_map.Drive('a', 1, 0.0, {'j1': 1.0})],
    [drive_map.Drive('a', 1, 1.0, {'j1': 1.0, 'j2': 1.0})],
])
def test_duplicate_ids_singular_mixes_and_bad_ratios_are_rejected(drives):
    with pytest.raises(ValueError):
        drive_map.DriveMap(drives, COUNTS)
