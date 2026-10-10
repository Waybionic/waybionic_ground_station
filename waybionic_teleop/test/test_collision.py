"""The arm's collision boxes keep its links off the table and out of its own base."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.collision import _intrusion, _overlap, _rotation, ArmCollision
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters
from waybionic_teleop.xbox_teleop_node import JAW_OPEN_GAP, JAW_SIZE

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
    # Every link the checks rely on needs its box, including the upper arm and the tool.
    for link in ('upper_arm_link', 'tool_link'):
        start = URDF.index('<collision>', URDF.index(f'<link name="{link}">'))
        end = URDF.index('</collision>', start) + len('</collision>')
        with pytest.raises(ValueError, match=link):
            ArmCollision.from_urdf(URDF[:start] + URDF[end:])


@pytest.mark.parametrize('extra, roots', [
    ('<link name="stray_link"/>', 2),
    ('<joint name="loop" type="fixed"><parent link="tool_link"/>'
     '<child link="base_link"/></joint>', 0),
])
def test_a_urdf_without_exactly_one_root_link_is_refused(extra, roots):
    with pytest.raises(ValueError, match=f'one root link, not {roots}'):
        ArmCollision.from_urdf(URDF.replace('</robot>', extra + '</robot>'))


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


def test_a_tool_pointing_down_stops_with_its_jaws_clear_of_the_table(parameters, check):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), check)
    # The tool points straight down in front of the base, its tip 175 mm above the table.
    start = {'joint_1': 0.1, 'joint_2': 0.5, 'joint_3': 1.4, 'joint_4': math.pi - 1.9,
             'joint_5': 0.0, 'tool_grip': 0.0}
    for button in ('start', 'y', 'y'):
        teleop.update(*sample(button), start, DT)
        teleop.update(*sample(), start, DT)
    assert teleop.active_group.name == 'cartesian'
    floor = check.table_z + check.clearance
    stopped = False
    for _ in range(round(8.0 / DT)):
        # Right stick down lowers the tool tip.
        teleop.update(*sample(right_y=-1.0), dict(teleop.targets), DT)
        assert jaw_bottom(check, teleop.targets) >= floor - 1e-12
        if 'tool_link: table' in teleop.blocked:
            stopped = True
            assert jaw_bottom(check, teleop.command_targets) >= floor - 1e-12
    assert stopped and jaw_bottom(check, teleop.targets) < floor + 0.001


def test_teleop_refuses_a_step_that_presses_one_contact_deeper(parameters, check):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), check)
    # Folded flat onto the table. Lifting the elbow from here eases five of the six contacts
    # by more than it costs, so a total would call it an escape, but the forearm itself is
    # pressed about 3 mm further into the table.
    start = pose(90, 90, 5)
    before, after = check.check(start), check.check(pose(90, 85, 5))
    assert set(after) == set(before) and sum(after.values()) < sum(before.values())
    assert after['forearm_link: table'] > before['forearm_link: table'] + 0.002
    for button in ('start', 'y'):
        teleop.update(*sample(button), start, DT)
        teleop.update(*sample(), start, DT)
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] == start['joint_3']
    assert teleop.blocked == ['forearm_link: table']


def test_the_intrusion_is_the_volume_inside_the_grown_obstacle():
    square = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    obstacle = ((0.0, 0.0, 0.0), square, (0.1, 0.1, 0.1))
    # 30 mm into one face of the obstacle grown by 10 mm, and flush with three others.
    side = ((0.13, 0.0, 0.01), square, (0.05, 0.11, 0.1))
    assert _intrusion(side, obstacle, 0.01) == pytest.approx(0.03 * 0.22 * 0.2)
    turn = _rotation((0.0, 0.0, 1.0), 0.3)
    inside = ((0.0, 0.02, 0.0), tuple(zip(*turn)), (0.01, 0.02, 0.03))
    assert _intrusion(inside, obstacle, 0.01) == pytest.approx(8 * 0.01 * 0.02 * 0.03)
    assert _intrusion(((0.5, 0.0, 0.0), square, (0.1, 0.1, 0.1)), obstacle, 0.01) == 0.0


def test_teleop_refuses_to_slide_the_forearm_further_into_the_side_of_the_base(
        parameters, check):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), check)
    # Folded back past today's joint limits, the end of the forearm reaches into the side of
    # the base. Turning the base towards zero slides it further in along that side, while its
    # overlap across the side, the smallest of the separating-axis overlaps, shrinks.
    start, turned = pose(-130, 40, 0, math.radians(15)), pose(-130, 40, 0, math.radians(14.5))
    assert check.hits(start) == check.hits(turned) == ['forearm_link: base_link']
    assert separating_axis_depth(check, turned) < separating_axis_depth(check, start)
    assert (check.check(turned)['forearm_link: base_link']
            > check.check(start)['forearm_link: base_link'])
    teleop.update(*sample('start'), start, DT)
    teleop.update(*sample(), start, DT)
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_x=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_1'] == start['joint_1']
    assert teleop.blocked == ['forearm_link: base_link']
    # Raising the shoulder backs it out.
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_2'] > start['joint_2'] + 0.1
    assert (check.check(teleop.targets).get('forearm_link: base_link', 0.0)
            < check.check(start)['forearm_link: base_link'])


def test_the_check_reports_a_depth_for_every_collision_it_names(check):
    depths = check.check(pose(-135, -165, 0))
    assert list(depths) == check.hits(pose(-135, -165, 0))
    assert set(depths) == {'forearm_link: base_link', 'forearm_link: shoulder_link'}
    assert all(depth > 0.0 for depth in depths.values())
    assert check.check(pose(0, 0, 0)) == {}


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


def separating_axis_depth(check, joints):
    """Return the forearm's smallest overlap with the base across the separating axes."""
    poses = check.poses(joints)
    return _overlap(next(check.world_boxes(poses, 'forearm_link')),
                    next(check.world_boxes(poses, 'base_link')), check.clearance)


def jaw_bottom(check, joints):
    """Return the height of the lowest corner of the RViz jaws, fully open."""
    rotation, origin = check.poses(joints)['tool_link']
    reach = JAW_OPEN_GAP / 2 + JAW_SIZE[0]
    corners = [(x, y, z) for x in (-reach, reach) for y in (-JAW_SIZE[1] / 2, JAW_SIZE[1] / 2)
               for z in (0.0, JAW_SIZE[2])]
    return min(origin[2] + sum(r * c for r, c in zip(rotation[2], corner)) for corner in corners)
