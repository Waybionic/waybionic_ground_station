"""Joint positions to drive encoder counts with the placeholder drive map."""

import math

import pytest

from waybionic_teleop import drive_map, mks_can

COUNTS = mks_can.COUNTS_PER_REV


@pytest.fixture
def arm_map(parameters):
    return drive_map.drive_map_from_parameters(
        parameters('arm_drives.yaml', 'sim_arm_drives'), COUNTS)


def test_placeholder_map_covers_every_arm_joint(arm_map):
    assert arm_map.joints == ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'tool_grip']
    assert [drive.can_id for drive in arm_map.drives] == [1, 2, 3, 4, 5, 6]


def test_positions_round_trip_through_counts(arm_map):
    positions = dict(zip(arm_map.joints, [0.5, -1.2, 0.3, 0.7, -0.4, 0.8]))
    decoded = arm_map.to_positions(arm_map.to_counts(positions))
    assert all(abs(decoded[joint] - positions[joint]) <= 2 * math.pi / COUNTS
               for joint in positions)


def test_wrist_differential_mixes_pitch_and_roll(arm_map):
    zero = dict.fromkeys(arm_map.joints, 0.0)
    quarter = COUNTS // 4
    assert arm_map.to_counts({**zero, 'joint_4': math.pi / 2})[3:5] == [quarter, quarter]
    assert arm_map.to_counts({**zero, 'joint_5': math.pi / 2})[3:5] == [-quarter, quarter]


def test_gear_ratio_scales_counts_and_speed():
    base = drive_map.DriveMap([drive_map.Drive('base', 1, 50.0, {'joint_1': 1.0})], COUNTS)
    assert base.to_counts({'joint_1': 2 * math.pi}) == [50 * COUNTS]
    assert base.to_rpm({'joint_1': -2 * math.pi}) == pytest.approx([3000.0])


def test_a_setpoint_that_jumped_ahead_is_chased_near_the_commanded_speed(arm_map):
    period = 1.0 / 120.0
    rates = {'joint_1': math.radians(60.0), 'joint_4': 0.5, 'joint_5': 0.2}
    # A 100 ms host stall leaves the base setpoint 6 degrees ahead of its drive.
    ahead = {joint: rates.get(joint, 0.0) * 0.1 for joint in arm_map.joints}
    counts = arm_map.to_counts(dict.fromkeys(arm_map.joints, 0.0))
    free = arm_map.synchronized(ahead, counts, period, 300)
    capped = arm_map.synchronized(ahead, counts, period, 300, rates)
    commanded = arm_map.to_rpm(rates)
    assert commanded[0] == pytest.approx(10.0) and free[0][1] == 120
    # Each wrist drive is capped by its own mix of pitch and roll.
    for (axis, rpm), (free_axis, free_rpm), limit in zip(capped, free, commanded):
        assert axis == free_axis and 1 <= rpm <= 1.5 * limit + 1.0
        assert rpm == free_rpm or rpm >= 1.5 * limit
    with pytest.raises(ValueError):
        arm_map.synchronized(ahead, counts, period, 300, {'joint_1': math.nan})


@pytest.mark.parametrize('drives', [
    [drive_map.Drive('a', 1, 1.0, {'j1': 1.0}), drive_map.Drive('b', 1, 1.0, {'j2': 1.0})],
    [drive_map.Drive('a', 1, 1.0, {'j1': 1.0, 'j2': 1.0}),
     drive_map.Drive('b', 2, 1.0, {'j1': 2.0, 'j2': 2.0})],
    [drive_map.Drive('a', 1, 0.0, {'j1': 1.0})],
    [drive_map.Drive('a', 1, 1.0, {'j1': 1.0, 'j2': 1.0})],
    [drive_map.Drive('a', 1, math.nan, {'j1': 1.0})],
    [drive_map.Drive('a', 1, 1.0, {'j1': math.inf})],
])
def test_duplicate_ids_singular_mixes_and_bad_ratios_are_rejected(drives):
    with pytest.raises(ValueError):
        drive_map.DriveMap(drives, COUNTS)


@pytest.mark.parametrize('positions,counts,period,max_rpm', [
    ({'joint_1': math.nan}, [0], 0.01, 300),
    ({}, [0], 0.01, 300),
    ({'joint_1': 0.0}, [None], 0.01, 300),
    ({'joint_1': 0.0}, [0], 0.0, 300),
    ({'joint_1': 0.0}, [0], 0.01, 0),
])
def test_synchronized_move_rejects_invalid_setpoints(positions, counts, period, max_rpm):
    mapping = drive_map.DriveMap([drive_map.Drive('base', 1, 1.0, {'joint_1': 1.0})], COUNTS)
    with pytest.raises(ValueError):
        mapping.synchronized(positions, counts, period, max_rpm)
