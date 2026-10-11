"""Keyhole moves: the tool slides along its own axis and tilts about the incision point."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics, joint_limits
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
# The tool leaning forward and down, about 45 degrees from vertical.
LEANING = {'joint_1': 0.3, 'joint_2': 0.4, 'joint_3': 1.2, 'joint_4': 0.8, 'joint_5': 0.0,
           'tool_grip': 0.0}
# The tool leaning forward and down, its tip about 60 mm above the table.
NEAR_TABLE = {'joint_1': 0.2, 'joint_2': 0.87, 'joint_3': 1.5, 'joint_4': -0.17, 'joint_5': 0.0,
              'tool_grip': 0.0}
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
DT = 1.0 / 120.0


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


def run(teleop, seconds, *pressed, **axes):
    for _ in range(round(seconds / DT)):
        teleop.update(*sample(*pressed, **axes), dict(teleop.targets), DT)


def press(teleop, button):
    teleop.update(*sample(button), dict(teleop.targets), DT)
    teleop.update(*sample(), dict(teleop.targets), DT)


def through(arm, joints, point):
    """Return how far point is from the tool axis, and how deep the tip is past it."""
    tip, _ = arm.forward(joints)
    offset = [a - b for a, b in zip(point, tip)]
    along = sum(a * b for a, b in zip(offset, arm.axis(joints)))
    return math.sqrt(max(sum(a * a for a in offset) - along * along, 0.0)), -along


def shifted(arm, joints, distance):
    """Return joints that move the tool sideways by distance, parallel to its own axis."""
    tip, pitch = arm.forward(joints)
    yaw = joints['joint_1']
    # Square to the tool axis, in the arm's vertical plane.
    side = (math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch))
    position = [a + distance * b for a, b in zip(tip, side)]
    return {**joints, **arm.inverse(position, pitch, joints['joint_5'], joints, LIMITS)}


@pytest.fixture
def arm():
    return ArmKinematics.from_urdf(URDF)


@pytest.fixture
def teleop(parameters, arm):
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, arm)
    for button in ('start', 'y', 'y', 'y'):
        teleop.update(*sample(button), LEANING, DT)
        teleop.update(*sample(), LEANING, DT)
    assert teleop.active_group.name == 'incision' and teleop.enabled
    return teleop


def test_the_incision_point_is_the_tip_when_the_group_takes_over(teleop, arm):
    assert teleop.incision == pytest.approx(arm.forward(LEANING)[0])


def test_insertion_slides_the_tip_along_the_tool_axis(teleop, arm):
    start = dict(teleop.targets)
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    distance, depth = through(arm, teleop.targets, teleop.incision)
    # 50% of 50 mm/s for a second, with the ramps at either end cancelling out.
    assert depth == pytest.approx(0.025, abs=1e-3)
    assert distance < 1e-5
    assert arm.forward(teleop.targets)[1] == pytest.approx(arm.forward(start)[1], abs=1e-9)
    assert teleop.targets['joint_1'] == pytest.approx(start['joint_1'], abs=1e-9)


def test_the_published_setpoint_is_one_period_ahead_through_the_incision(teleop, arm):
    period = teleop.config.period
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(left_y=1.0, right_y=1.0), dict(teleop.targets), DT)
        _, depth = through(arm, teleop.targets, teleop.incision)
        distance, ahead = through(arm, teleop.command_targets, teleop.incision)
        # The drives aim at the next period's pose, so it too passes the incision point.
        assert distance < 1e-9
        assert ahead - depth == pytest.approx(teleop.insert * period, abs=1e-9)
        assert arm.forward(teleop.command_targets)[1] - arm.forward(teleop.targets)[1] == (
            pytest.approx(teleop.tilt * period, abs=1e-9))
    assert teleop.insert > 0 and teleop.tilt < 0


def test_tilting_turns_the_tool_about_the_incision_point(teleop, arm):
    run(teleop, 1.2, left_y=1.0)
    run(teleop, 0.3)
    _, depth = through(arm, teleop.targets, teleop.incision)
    tip, pitch = arm.forward(teleop.targets)
    yaw = teleop.targets['joint_1']
    run(teleop, 1.0, right_y=1.0)
    run(teleop, 0.3)
    distance, tilted_depth = through(arm, teleop.targets, teleop.incision)
    tilted_tip, tilted_pitch = arm.forward(teleop.targets)
    # Pushing up turns the axis towards vertical at 50% of 30 deg/s.
    assert tilted_pitch - pitch == pytest.approx(-math.radians(15.0), abs=math.radians(1.5))
    assert distance < 1e-5 and tilted_depth == pytest.approx(depth, abs=1e-5)
    assert math.dist(tip, tilted_tip) > 0.005
    assert teleop.targets['joint_1'] == pytest.approx(yaw, abs=1e-9)


def test_sideways_stick_and_going_home_do_nothing_here(teleop):
    start = dict(teleop.targets)
    run(teleop, 0.5, left_x=1.0)
    run(teleop, 0.5, 'a')
    assert teleop.targets == pytest.approx(start, abs=1e-9)


def test_roll_turns_the_tool_without_moving_it_off_the_incision(teleop, arm):
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    _, depth = through(arm, teleop.targets, teleop.incision)
    roll, pitch = teleop.targets['joint_5'], arm.forward(teleop.targets)[1]
    run(teleop, 0.5, right_x=1.0)
    run(teleop, 0.3)
    distance, rolled_depth = through(arm, teleop.targets, teleop.incision)
    assert abs(teleop.targets['joint_5'] - roll) > 0.1
    assert distance < 1e-5 and rolled_depth == pytest.approx(depth, abs=1e-5)
    assert arm.forward(teleop.targets)[1] == pytest.approx(pitch, abs=1e-9)


@pytest.mark.parametrize('axes, joint', [
    ({'left_y': -1.0}, 'joint_4'), ({'right_y': 1.0}, 'joint_3'), ({'right_y': -1.0}, 'joint_4')])
def test_moving_into_a_joint_limit_sends_targets_within_the_urdf_limits(
        parameters, arm, axes, joint):
    limits = joint_limits(URDF, arm.joints)
    params = {**parameters('xbox_teleop.yaml', 'xbox_teleop'), 'initial_speed_level': 3}
    teleop = ArmTeleop(config_from_parameters(params), limits, arm)
    for button in ('start', 'y', 'y', 'y'):
        teleop.update(*sample(button), LEANING, DT)
        teleop.update(*sample(), LEANING, DT)
    for _ in range(round(10.0 / DT)):
        teleop.update(*sample(**axes), dict(teleop.targets), DT)
        # The drives refuse any setpoint past a URDF limit, even by solver rounding.
        for targets in (teleop.targets, teleop.command_targets):
            assert all(lower <= targets[name] <= upper for name, (lower, upper) in limits.items())
        if joint in teleop.blocked:
            break
    else:
        pytest.fail(f'{joint} never reached its limit')
    assert teleop.targets[joint] == pytest.approx(limits[joint][1], abs=1e-6)
    assert through(arm, teleop.targets, teleop.incision)[0] < 1e-6


@pytest.mark.parametrize('into, back, contact', [
    ({'left_y': 1.0}, {'left_y': -1.0}, 'tool_link: table'),
    ({'right_y': -1.0}, {'right_y': 1.0}, 'tool_link: table'),
    ({'right_y': 1.0}, {'right_y': -1.0}, 'forearm_link: table'),
], ids=['insert', 'pivot_tool_down', 'pivot_forearm_down'])
def test_an_incision_move_into_the_table_is_refused_but_backing_out_is_not(
        parameters, arm, into, back, contact):
    check = ArmCollision.from_urdf(URDF)
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       LIMITS, arm, check)
    for button in ('start', 'y', 'y', 'y'):
        teleop.update(*sample(button), NEAR_TABLE, DT)
        teleop.update(*sample(), NEAR_TABLE, DT)
    # With the tool 25 mm in, inserting further or pivoting either way reaches the table.
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    first = teleop.incision
    stopped = False
    for _ in range(round(3.0 / DT)):
        before = dict(teleop.targets)
        teleop.update(*sample(**into), dict(teleop.targets), DT)
        assert check.hits(teleop.targets) == []
        if contact in teleop.blocked:
            stopped = True
            # The whole step is undone, its published one-period lookahead included.
            assert teleop.targets == teleop.command_targets == before
            assert not any(teleop.velocities.values())
            assert teleop.insert == teleop.tilt == teleop.roll == 0.0
    # It stopped at the table, not short of it, and kept the incision point and group.
    assert stopped and contact in ArmCollision.from_urdf(
        URDF, clearance=check.clearance + 0.001).hits(teleop.targets)
    assert teleop.active_group.name == 'incision' and teleop.incision == first
    stopped_at = dict(teleop.targets)
    for _ in range(round(0.5 / DT)):
        teleop.update(*sample(**back), dict(teleop.targets), DT)
        assert teleop.blocked == []
    assert max(abs(teleop.targets[joint] - stopped_at[joint]) for joint in arm.joints) > 0.05
    assert through(arm, teleop.targets, first)[0] < 1e-6


def test_re_enabling_away_from_the_incision_point_moves_nothing(teleop, arm):
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    teleop.update(*sample('b'), dict(teleop.targets), DT)
    teleop.update(*sample(), dict(teleop.targets), DT)
    # While disabled, the arm ends up somewhere the axis misses the incision point.
    moved = {**teleop.targets, 'joint_2': teleop.targets['joint_2'] + 0.1}
    teleop.update(*sample('start'), moved, DT)
    teleop.update(*sample(), moved, DT)
    assert teleop.enabled
    run(teleop, 0.5, left_y=1.0)
    assert teleop.targets == pytest.approx(moved, abs=1e-9)
    assert teleop.blocked == ['incision'] and 'press Y' in teleop.note


def test_re_enabling_a_little_off_the_incision_point_moves_the_point_not_the_arm(teleop, arm):
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    first = teleop.incision
    teleop.update(*sample('b'), dict(teleop.targets), DT)
    teleop.update(*sample(), dict(teleop.targets), DT)
    # While disabled, the tool ends up 1.5 mm to the side of the incision point.
    moved = shifted(arm, teleop.targets, 0.0015)
    teleop.update(*sample('start'), moved, DT)
    run(teleop, 0.5)
    assert teleop.enabled and teleop.targets == moved and not teleop.warning
    distance, _ = through(arm, moved, teleop.incision)
    assert distance < 1e-9
    assert math.dist(teleop.incision, first) == pytest.approx(0.0015, abs=1e-9)
    # Inserting then slides along the tool's own axis, with no step sideways onto the old one.
    tip = arm.forward(moved)[0]
    teleop.update(*sample(left_y=1.0), dict(teleop.targets), DT)
    assert 0 < math.dist(arm.forward(teleop.targets)[0], tip) < 1e-4
    assert through(arm, teleop.targets, teleop.incision)[0] < 1e-9


def test_y_keeps_the_incision_group_until_the_tool_is_withdrawn(teleop, arm):
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    first, inserted = teleop.incision, dict(teleop.targets)
    for _ in range(len(teleop.groups)):
        press(teleop, 'y')
    assert teleop.active_group.name == 'incision' and teleop.incision == first
    assert teleop.warning and 'withdraw' in teleop.note
    run(teleop, 0.5, 'a')
    assert teleop.targets == inserted
    # Withdrawing the tip out of the incision clears the warning and frees Y.
    run(teleop, 1.5, left_y=-1.0)
    run(teleop, 0.3)
    assert through(arm, teleop.targets, first)[1] < 0 and not teleop.warning
    press(teleop, 'y')
    assert teleop.active_group.name == 'base'
    # The tool axis still passes the incision point, so choosing the group again keeps it.
    for _ in range(len(teleop.groups) - 1):
        press(teleop, 'y')
    assert teleop.active_group.name == 'incision'
    assert teleop.incision == pytest.approx(first, abs=1e-12)


def test_a_stopped_arm_keeps_the_incision_group_while_the_tool_is_inserted(teleop):
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    press(teleop, 'b')
    press(teleop, 'y')
    assert teleop.active_group.name == 'incision' and 'withdraw' in teleop.note


def test_a_withdrawn_tool_moved_elsewhere_takes_a_new_incision_point(teleop, arm):
    first = teleop.incision
    press(teleop, 'y')
    assert teleop.active_group.name == 'base'
    run(teleop, 0.5, left_x=1.0)
    run(teleop, 0.3)
    for _ in range(len(teleop.groups) - 1):
        press(teleop, 'y')
    assert teleop.active_group.name == 'incision'
    assert math.dist(teleop.incision, first) > 0.02
    assert teleop.incision == pytest.approx(arm.forward(teleop.targets)[0])


def test_choosing_the_group_again_sets_a_new_point_only_once_the_tool_is_withdrawn(teleop, arm):
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    first = teleop.incision
    press(teleop, 'b')
    # While stopped, the arm swings forward: the tool goes deeper in, off the incision point.
    moved = {**teleop.targets, 'joint_2': teleop.targets['joint_2'] + 0.1}
    teleop.update(*sample('start'), moved, DT)
    teleop.update(*sample(), moved, DT)
    run(teleop, 0.1, left_y=1.0)
    assert teleop.warning and 'press Y' in teleop.note
    assert through(arm, moved, first)[1] > 0.02
    # With the tool still inside, choosing the group again keeps the incision point.
    for _ in range(len(teleop.groups)):
        press(teleop, 'y')
    run(teleop, 0.1, left_y=1.0)
    assert teleop.active_group.name == 'incision' and teleop.incision == first
    assert teleop.blocked == ['incision'] and teleop.warning
    assert teleop.targets == pytest.approx(moved, abs=1e-9)
    # The documented way out: withdraw the tool in another group, then choose the group again.
    press(teleop, 'y')
    run(teleop, 0.5, left_y=-1.0)
    run(teleop, 0.3)
    assert through(arm, teleop.targets, first)[1] < 0
    for _ in range(len(teleop.groups) - 1):
        press(teleop, 'y')
    assert teleop.active_group.name == 'incision'
    run(teleop, 1.0, left_y=1.0)
    run(teleop, 0.3)
    assert not teleop.warning and teleop.note == '' and 'incision' not in teleop.blocked
    distance, depth = through(arm, teleop.targets, teleop.incision)
    assert distance < 1e-5 and depth > 0.005


def test_a_new_incision_point_clears_only_the_incision_warning(teleop):
    # Any other warning belongs to whatever raised it, so reseeding must leave it standing.
    teleop.incision = None
    teleop.note, teleop.warning = 'Controller lost; press Start (Menu) to enable', True
    run(teleop, 0.1)
    assert teleop.incision is not None
    assert teleop.warning and teleop.note == 'Controller lost; press Start (Menu) to enable'
