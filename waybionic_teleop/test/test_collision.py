"""The arm's collision shapes keep its links off the table and out of its own base."""

import math
from pathlib import Path
import re

import pytest

from waybionic_teleop.collision import _intrusion, _rotation, ArmCollision
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics, joint_limits
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters
from waybionic_teleop.xbox_teleop_node import JAW_OPEN_GAP, JAW_SIZE

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
COLLISION = re.compile(r'<collision .*?</collision>', re.S)
LIMITS = joint_limits(URDF, ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'])
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


def test_the_elbow_folds_until_the_forearm_almost_touches_the_shoulder(check):
    # The forearm's boxes would stop this near 134 degrees; the CAD fold table allows 147.
    assert check.hits(pose(0, 146, 0)) == [] and check.hits(pose(0, -146, 0)) == []
    assert check.hits(pose(0, 149, 0)) == ['forearm_link: shoulder_link']
    assert check.hits(pose(0, -149, 0)) == ['forearm_link: shoulder_link']


def test_the_fold_limit_follows_the_shoulder_angle():
    # With the shoulder tilted, the forearm meets the shoulder sooner on that side.
    away = ArmCollision.from_urdf(URDF, table_z=-10.0)
    for sign in (1, -1):
        assert 'forearm_link: shoulder_link' not in away.hits(pose(sign * 90, sign * 128, 0))
        assert 'forearm_link: shoulder_link' in away.hits(pose(sign * 90, sign * 133, 0))


@pytest.mark.parametrize('yaw', [0.0, math.radians(45)])
def test_the_forearm_folds_onto_the_base_until_just_before_its_parts_touch(yaw):
    # With the table out of the way, the CAD parts touch at these elbow angles at either yaw.
    away = ArmCollision.from_urdf(URDF, table_z=-10.0)
    for shoulder, touch in ((45, 130.2), (90, 97.2)):
        assert away.hits(pose(shoulder, touch - 1.4, 0, yaw)) == []
        assert 'forearm_link: base_link' in away.hits(pose(shoulder, touch + 0.3, 0, yaw))
    # Folded back to the shoulder's limit, the forearm clears the base with the elbow bent up;
    # bent down, it touches the base at -11 degrees.
    for elbow in (60, 90, -9.5):
        assert away.hits(pose(-135, elbow, 0, yaw)) == []
    assert away.hits(pose(-135, -11.5, 0, yaw)) == ['forearm_link: base_link']


@pytest.mark.parametrize('yaw', [0.0, math.radians(45)])
def test_the_wrist_and_tool_stop_just_before_their_parts_touch_the_base(yaw):
    # With the table out of the way, the CAD parts come within 1 mm of the base at these elbow
    # angles at either yaw: the tool mount, the tool and the wrist's differential housing. The
    # 2 mm clearance stops the elbow up to 1.3 degrees before them.
    away = ArmCollision.from_urdf(URDF, table_z=-10.0)
    for shoulder, wrist, touch, link in ((45, 60, 120.5, 'wrist_roll_link'),
                                         (30, 90, 122.85, 'tool_link'),
                                         (60, 45, 118.95, 'wrist_pitch_link')):
        assert away.hits(pose(shoulder, touch - 1.3, wrist, yaw)) == []
        assert f'{link}: base_link' in away.hits(pose(shoulder, touch + 0.3, wrist, yaw))


@pytest.mark.parametrize('old, new', [
    ('across="joint_2"', 'across="joint_9"'), ('start="-135"', 'start="nan"'),
    ('step="5"', 'step="0"'), ('upper="147.3 ', 'upper="'), ('lower="-122.7', 'lower="inf'),
    # Each table replaces the shapes for its pair, so it must fit the joint tree.
    ('obstacle="shoulder_link"', 'obstacle="upper_arm_link"'),
    ('link="forearm_link" obstacle="base_link"', 'link="upper_arm_link" obstacle="base_link"'),
    ('joint="joint_3" across="joint_2" start="-135" step="5"',
     'joint="joint_1" across="joint_2" start="-135" step="5"'),
    ('across="joint_2" start="-135" step="5"', 'across="joint_3" start="-135" step="5"'),
    ('across="joint_2" start="-135" step="5"', 'across="joint_4" start="-135" step="5"'),
    ('start="-135" step="5"', 'start="-130" step="5"'),
    ('start="-135" step="2.5"', 'start="-135" step="2"'),
    ('upper="147.3 ', 'upper="-130 '),
])
def test_an_unusable_fold_table_is_refused(old, new):
    assert old in URDF
    with pytest.raises(ValueError, match='waybionic_fold'):
        ArmCollision.from_urdf(URDF.replace(old, new))


def test_a_second_fold_table_for_one_pair_is_refused():
    fold = re.search(r'<waybionic_fold .*?/>', URDF, re.S).group(0)
    with pytest.raises(ValueError, match='two waybionic_fold tables for one pair'):
        ArmCollision.from_urdf(URDF.replace('</robot>', fold + '\n</robot>'))


@pytest.mark.parametrize('old, new', [
    ('<cylinder radius="0.10019"', '<cylinder radius="nan"'),
    ('<cylinder radius="0.10019"', '<cylinder radius="0"'),
    ('radius="0.10019" length="0.02"', 'radius="0.10019" length="-0.02"'),
    ('radius="0.10019" length="0.02"', 'radius="0.10019"'),
    ('<box size="0.036 0.014 0.03"/>', '<box size="0.036 0 0.03"/>'),
    ('<box size="0.036 0.014 0.03"/>', '<box size="0.036 inf 0.03"/>'),
    ('<box size="0.036 0.014 0.03"/>', '<box size="0.036 0.014"/>'),
    ('<origin xyz="0 0 0.1275"', '<origin xyz="0 0 nan"'),
    ('<axis xyz="0 0 1"/>', '<axis xyz="0 0 0"/>'),
    ('<mimic joint="joint_5" multiplier="-1"', '<mimic joint="joint_5" multiplier="nan"'),
])
def test_a_shape_or_joint_without_finite_sizes_is_refused(old, new):
    # NaN would make the broad phase skip the shape, so the check would miss its contacts.
    assert old in URDF
    with pytest.raises(ValueError, match='robot_description'):
        ArmCollision.from_urdf(URDF.replace(old, new, 1))


def test_table_height_and_clearance_are_configurable():
    # This pose's tool tip is 175 mm above the bottom of the base.
    joints = pose(28.6, 80.2, 71.1, 0.1)
    assert ArmCollision.from_urdf(URDF, table_z=0.17).hits(joints)
    assert not ArmCollision.from_urdf(URDF, table_z=0.1).hits(joints)
    assert ArmCollision.from_urdf(URDF, table_z=0.1, clearance=0.1).hits(joints)


def test_the_base_and_shoulder_have_their_own_smaller_clearance():
    # The tool is between 2 and 10 mm from the base's lid here.
    joints = pose(30, 121.5, 90, math.radians(45))
    assert ArmCollision.from_urdf(URDF).hits(joints) == []
    assert ArmCollision.from_urdf(URDF, self_clearance=0.01).hits(joints) == [
        'tool_link: base_link']


def test_a_urdf_without_collision_boxes_is_refused():
    with pytest.raises(ValueError, match='no collision box'):
        ArmCollision.from_urdf(re.sub(COLLISION, '', URDF))
    # Every link the checks rely on needs its boxes, including the upper arm and the tool.
    for link in ('upper_arm_link', 'tool_link'):
        start = URDF.index(f'<link name="{link}">')
        end = URDF.index('</link>', start)
        with pytest.raises(ValueError, match=link):
            ArmCollision.from_urdf(
                URDF[:start] + re.sub(COLLISION, '', URDF[start:end]) + URDF[end:])


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
    # by more than it costs, so a total would call it an escape, but the tool itself is
    # pressed more than 1 mm further into the table.
    start = pose(90, 85, 40)
    before, after = check.check(start), check.check(pose(90, 80, 40))
    assert set(after) == set(before) and sum(after.values()) < sum(before.values())
    assert after['tool_link: table'] > before['tool_link: table'] + 0.001
    for button in ('start', 'y'):
        teleop.update(*sample(button), start, DT)
        teleop.update(*sample(), start, DT)
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] == start['joint_3']
    assert teleop.blocked == ['tool_link: table']


def test_the_intrusion_is_the_volume_inside_the_grown_obstacle():
    square = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    obstacle = ((0.0, 0.0, 0.0), square, (0.1, 0.1, 0.1), None)
    # 30 mm into one face of the obstacle grown by 10 mm, and flush with three others.
    side = ((0.13, 0.0, 0.01), square, (0.05, 0.11, 0.1), None)
    assert _intrusion(side, obstacle, 0.01) == pytest.approx(0.03 * 0.22 * 0.2)
    turn = _rotation((0.0, 0.0, 1.0), 0.3)
    inside = ((0.0, 0.02, 0.0), tuple(zip(*turn)), (0.01, 0.02, 0.03), None)
    assert _intrusion(inside, obstacle, 0.01) == pytest.approx(8 * 0.01 * 0.02 * 0.03)
    assert _intrusion(((0.5, 0.0, 0.0), square, (0.1, 0.1, 0.1), None), obstacle, 0.01) == 0.0


@pytest.mark.parametrize('start, contact', [
    (pose(45, 129.5, 0), 'forearm_link: base_link'),
    (pose(45, 121.0, 60), 'wrist_roll_link: base_link'),
])
def test_teleop_never_presses_the_arm_further_into_the_base(parameters, start, contact):
    away = ArmCollision.from_urdf(URDF, table_z=-10.0)
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, ArmKinematics.from_urdf(URDF), away)
    # Past the forearm's table, or with the tool mount in the base's round housing.
    assert away.hits(start) == [contact]
    teleop.update(*sample('start'), start, DT)
    teleop.update(*sample(), start, DT)
    # The base is round, so turning it slides the arm round the base without pressing in.
    for _ in range(round(0.25 / DT)):
        teleop.update(*sample(left_x=1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_1'] > start['joint_1'] + 0.05
    assert away.check(teleop.targets)[contact] == pytest.approx(away.check(start)[contact])
    teleop.update(*sample('y'), dict(teleop.targets), DT)
    teleop.update(*sample(), dict(teleop.targets), DT)
    assert teleop.active_group.name == 'upper'
    before = dict(teleop.targets)
    # Left stick up bends the elbow further in, which is refused.
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] == before['joint_3']
    assert contact in teleop.blocked
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=-1.0), dict(teleop.targets), DT)
    assert teleop.targets['joint_3'] < before['joint_3'] - 0.05
    assert away.check(teleop.targets).get(contact, 0.0) < away.check(before)[contact]


def test_a_cylinder_counts_as_a_prism_turned_to_face_the_other_shape():
    square = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    cylinder = ((0.0, 0.0, 0.0), square, (0.1, 0.1, 0.05), 0.1)
    # 20 mm into the side of the cylinder grown by 10 mm, square on to it, at any turn.
    for angle in (0.0, 0.3, 1.0, 2.5):
        turn = _rotation((0.0, 0.0, 1.0), angle)
        side = ((0.13 * math.cos(angle), 0.13 * math.sin(angle), 0.0), tuple(zip(*turn)),
                (0.04, 0.01, 0.01), None)
        assert _intrusion(side, cylinder, 0.01) == pytest.approx(0.02 * 0.02 * 0.02)
    deeper = ((0.12, 0.0, 0.0), square, (0.04, 0.01, 0.01), None)
    assert _intrusion(deeper, cylinder, 0.01) == pytest.approx(0.03 * 0.02 * 0.02)
    inside = ((0.0, 0.03, 0.0), square, (0.01, 0.02, 0.03), None)
    assert _intrusion(inside, cylinder, 0.01) == pytest.approx(8 * 0.01 * 0.02 * 0.03)
    for clear in ((0.2, 0.0, 0.0), (0.0, 0.0, 0.1)):
        assert _intrusion((clear, square, (0.03, 0.03, 0.03), None), cylinder, 0.01) == 0.0
    # A disc pressed into a box or a cylinder: spinning it about its own axis changes nothing,
    # and pressing it further in adds to the volume.
    box = ((0.0, 0.0, 0.0), square, (0.1, 0.1, 0.05), None)
    for obstacle in (box, cylinder):
        disc = ((0.125, 0.0, 0.0), square, (0.02, 0.02, 0.01), 0.02)
        pressed = _intrusion(disc, obstacle, 0.01)
        assert pressed > 0.0
        for angle in (0.4, 1.3):
            spun = (disc[0], tuple(zip(*_rotation((0.0, 0.0, 1.0), angle))), disc[2], disc[3])
            assert _intrusion(spun, obstacle, 0.01) == pytest.approx(pressed)
        further = ((0.12, 0.0, 0.0), square, disc[2], disc[3])
        assert _intrusion(further, obstacle, 0.01) > pressed


def test_the_check_reports_a_depth_for_every_collision_it_names(check):
    depths = check.check(pose(-135, -165, 0))
    assert list(depths) == check.hits(pose(-135, -165, 0))
    assert set(depths) == {'forearm_link: base_link', 'forearm_link: shoulder_link'}
    assert all(depth > 0.0 for depth in depths.values())
    assert check.check(pose(0, 0, 0)) == {}


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


def jaw_bottom(check, joints):
    """Return the height of the lowest corner of the RViz jaws, fully open."""
    rotation, origin = check.poses(joints)['tool_link']
    reach = JAW_OPEN_GAP / 2 + JAW_SIZE[0]
    corners = [(x, y, z) for x in (-reach, reach) for y in (-JAW_SIZE[1] / 2, JAW_SIZE[1] / 2)
               for z in (0.0, JAW_SIZE[2])]
    return min(origin[2] + sum(r * c for r, c in zip(rotation[2], corner)) for corner in corners)
