"""The arm's collision boxes keep its links off the table and out of its own base."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
DT = 1.0 / 120.0


def pose(shoulder, elbow, wrist, yaw=0.0):
    """Joint positions in degrees for the pitch joints."""
    return {'joint_1': yaw, 'joint_2': math.radians(shoulder), 'joint_3': math.radians(elbow),
            'joint_4': math.radians(wrist), 'joint_5': 0.0, 'tool_grip': 0.0}


@pytest.fixture
def check():
    return ArmCollision.from_urdf(URDF)


@pytest.mark.parametrize('joints', [
    pose(0, 0, 0), pose(28.6, 80.2, 71.1, 0.1), pose(90, 0, 0), pose(60, 60, 60, 2.0),
    pose(-30, -90, -90)])
def test_working_poses_are_clear(check, joints):
    assert check.hits(joints) == []


def test_folding_the_arm_down_reaches_the_table(check):
    hits = check.hits(pose(90, 90, 0))
    assert {'forearm_link: table', 'wrist_roll_link: table'} <= set(hits)
    assert not any('base_link' in hit or 'shoulder_link' in hit for hit in hits)


def test_folding_back_into_the_base_is_caught_past_todays_joint_limits(check):
    hits = check.hits(pose(-135, -165, 0))
    assert 'forearm_link: base_link' in hits and 'forearm_link: shoulder_link' in hits
    assert not any('table' in hit for hit in hits)


def test_table_height_and_clearance_are_configurable():
    # This pose's tool tip is 175 mm above the bottom of the base.
    joints = pose(28.6, 80.2, 71.1, 0.1)
    assert ArmCollision.from_urdf(URDF, table_z=0.17).hits(joints)
    assert not ArmCollision.from_urdf(URDF, table_z=0.1).hits(joints)
    assert ArmCollision.from_urdf(URDF, table_z=0.1, clearance=0.1).hits(joints)


def test_a_urdf_without_collision_boxes_is_refused():
    with pytest.raises(ValueError, match='no collision box'):
        ArmCollision.from_urdf(URDF.replace('<collision>', '<!--').replace('</collision>', '-->'))
    # Every link the checks rely on needs its box, including the upper arm.
    upper_arm = URDF.index('<link name="upper_arm_link">')
    start = URDF.index('<collision>', upper_arm)
    end = URDF.index('</collision>', start) + len('</collision>')
    with pytest.raises(ValueError, match='upper_arm_link'):
        ArmCollision.from_urdf(URDF[:start] + URDF[end:])


def test_teleop_backs_out_of_a_collision_but_never_goes_deeper(parameters, check):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), check)
    # The forearm already reaches into the table.
    start = pose(90, 75, 0)
    assert 'forearm_link: table' in check.hits(start)
    for button in ('start', 'y'):
        teleop.update(*sample(button), start, DT)
        teleop.update(*sample(), start, DT)
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] == pytest.approx(start['joint_3'])
    assert 'forearm_link: table' in teleop.blocked
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] < start['joint_3'] - 0.1


def test_teleop_stops_before_the_forearm_reaches_the_table(parameters, check):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), check)
    start = pose(60, 30, 60)
    for button in ('start', 'y'):
        teleop.update(*sample(button), start, DT)
        teleop.update(*sample(), start, DT)
    assert teleop.active_group.name == 'upper'
    stopped = False
    for _ in range(round(4.0 / DT)):
        # Left stick up bends the elbow down towards the table.
        teleop.update(*sample(left_y=1.0), dict(teleop.targets), DT)
        if any(hit.endswith(': table') for hit in teleop.blocked):
            stopped = True
            # The published one-period lookahead stops with the refused step.
            assert teleop.command_targets == teleop.targets
            assert not any(teleop.velocities.values())
        assert check.hits(teleop.targets) == []
    assert stopped and teleop.targets['joint_3'] < LIMITS['joint_3'][1] - 0.05
    elbow = teleop.targets['joint_3']
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] < elbow - 0.05


def test_teleop_refuses_a_step_that_presses_one_contact_deeper(parameters, check):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), check)
    # Folded flat onto the table. Lifting the elbow from here eases four of the five contacts
    # by more than it costs, so a total would call it an escape, but the forearm itself is
    # pressed about 3 mm further into the table.
    start = pose(90, 90, 15)
    before, after = check.check(start), check.check(pose(90, 85, 15))
    assert set(after) == set(before) and sum(after.values()) < sum(before.values())
    assert after['forearm_link: table'] > before['forearm_link: table'] + 0.002
    for button in ('start', 'y'):
        teleop.update(*sample(button), start, DT)
        teleop.update(*sample(), start, DT)
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] == start['joint_3']
    assert teleop.blocked == ['forearm_link: table']


def test_the_check_reports_a_depth_for_every_collision_it_names(check):
    depths = check.check(pose(-135, -165, 0))
    assert list(depths) == check.hits(pose(-135, -165, 0))
    assert set(depths) == {'forearm_link: base_link', 'forearm_link: shoulder_link'}
    assert all(depth > 0.0 for depth in depths.values())
    assert check.check(pose(0, 0, 0)) == {}


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]
