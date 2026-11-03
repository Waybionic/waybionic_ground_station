"""Demo mode: the arm moves around by itself, slowly and smoothly, and comes back to its pose."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters, DEMO_STATES

LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
ARM = tuple(LIMITS)
HOME = dict.fromkeys([*LIMITS, 'tool_grip'], 0.0)
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
DT = 1.0 / 120.0
DEG = math.pi / 180.0


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


def run(teleop, seconds, *pressed, **axes):
    for _ in range(round(seconds / DT)):
        teleop.update(*sample(*pressed, **axes), dict(teleop.targets), DT)


def press(teleop, button):
    teleop.update(*sample(button), dict(teleop.targets), DT)
    teleop.update(*sample(), dict(teleop.targets), DT)


def trace(teleop, seconds, *pressed, **axes):
    """Run the teleop and return the arm's targets after each tick."""
    rows = []
    for _ in range(round(seconds / DT)):
        teleop.update(*sample(*pressed, **axes), dict(teleop.targets), DT)
        rows.append({joint: teleop.targets[joint] for joint in ARM})
    return rows


def speeds(rows):
    """Return each joint's speed (rad/s) between consecutive ticks."""
    return [{joint: (later[joint] - earlier[joint]) / DT for joint in ARM}
            for earlier, later in zip(rows, rows[1:])]


def peak_speed(rows):
    return max(abs(value) for row in speeds(rows) for value in row.values())


def peak_accel(rows):
    velocities = speeds(rows)
    return max(abs(later[joint] - earlier[joint]) / DT
               for earlier, later in zip(velocities, velocities[1:]) for joint in ARM)


def build(params, arm, limits=LIMITS, **options):
    options.setdefault('collision', ArmCollision.from_urdf(URDF, 0.0, 0.01))
    return ArmTeleop(config_from_parameters(params), limits, arm, **options)


def enabled(teleop, measured=HOME):
    teleop.update(*sample('start'), measured, DT)
    teleop.update(*sample(), measured, DT)
    assert teleop.enabled
    return teleop


@pytest.fixture
def arm():
    return ArmKinematics.from_urdf(URDF)


@pytest.fixture
def params(parameters):
    return parameters('xbox_teleop.yaml', 'xbox_teleop')


@pytest.fixture
def teleop(params, arm):
    return enabled(build(params, arm))


def start(teleop):
    ok, message = teleop.set_demo_mode(True)
    assert ok, message


@pytest.fixture(scope='module')
def whole_demo(parameters):
    """Run the demo for its ramp, a whole loop and the return, recording every tick."""
    arm = ArmKinematics.from_urdf(URDF)
    teleop = enabled(build(parameters('xbox_teleop.yaml', 'xbox_teleop'), arm))
    settings = teleop.config.demo
    start(teleop)
    rows, blocked, commands, references = [], [], [], []
    for _ in range(round((settings.ramp + settings.loop) / DT)):
        teleop.update(*sample(), dict(teleop.targets), DT)
        rows.append({joint: teleop.targets[joint] for joint in ARM})
        commands.append({joint: teleop.command_targets[joint] for joint in ARM})
        references.append(teleop.demo.positions())
        blocked.append(list(teleop.blocked))
    running = len(rows)
    teleop.set_demo_mode(False)
    for _ in range(round((settings.ramp + 1.0) / DT)):
        teleop.update(*sample(), dict(teleop.targets), DT)
        rows.append({joint: teleop.targets[joint] for joint in ARM})
    return {'teleop': teleop, 'arm': arm, 'rows': rows, 'running': running, 'blocked': blocked,
            'commands': commands, 'references': references}


def test_the_demo_settings_come_from_the_config(teleop):
    demo = teleop.config.demo
    assert demo.joints == list(ARM)
    assert demo.amplitudes == pytest.approx([30 * DEG, 15 * DEG, 25 * DEG, 20 * DEG, 35 * DEG])
    assert demo.cycles == [3, 5, 4, 6, 2]
    assert (demo.loop, demo.ramp) == (120.0, 12.0)
    assert demo.max_speed == pytest.approx(12 * DEG)


def test_demo_mode_is_off_until_it_is_switched_on(teleop):
    assert teleop.demo_status() == ('off', '')
    assert teleop.demo is None


def test_switching_on_needs_teleop_enabled_first(params, arm):
    teleop = build(params, arm)
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'Press Start' in message
    assert teleop.demo_status() == ('off', '')


def test_demo_mode_says_what_it_is_doing_in_each_state(teleop):
    states = [teleop.demo_status()]
    start(teleop)
    states.append(teleop.demo_status())
    teleop.set_demo_mode(False)
    states.append(teleop.demo_status())
    assert [state for state, _ in states] == ['off', 'running', 'stopping']
    assert set(DEMO_STATES) >= {state for state, _ in states}
    assert not states[0][1] and states[1][1] and states[2][1]


def test_the_arm_swings_slowly_and_smoothly(whole_demo):
    rows = whole_demo['rows']
    limit = whole_demo['teleop'].config.demo.max_speed
    assert peak_speed(rows) <= limit
    # Slow means a few degrees a second, however long it runs.
    assert peak_speed(rows) < 8.0 * DEG
    # Smooth means it speeds up and slows down by a few degrees a second, every second.
    assert peak_accel(rows) < 4.0 * DEG


def test_the_tool_tip_moves_slowly_too(whole_demo):
    arm, rows = whole_demo['arm'], whole_demo['rows']
    tips = [arm.forward(row)[0] for row in rows[::6]]
    fastest = max(math.dist(earlier, later) for earlier, later in zip(tips, tips[1:])) / (6 * DT)
    assert fastest < 0.10


def test_the_arm_follows_the_demo_exactly_and_the_drives_get_the_next_step(whole_demo):
    rows, references, commands = (whole_demo[key] for key in ('rows', 'references', 'commands'))
    for row, reference in zip(rows, references):
        for joint, angle in reference.items():
            assert row[joint] == pytest.approx(angle, abs=1e-9)
    # The lookahead the drives are given is where the next tick will put each joint.
    for command, row, later in zip(commands, rows, rows[1:]):
        for joint in ARM:
            assert command[joint] == pytest.approx(later[joint], abs=2e-5)


def test_the_arm_leaves_its_pose_without_a_jump(whole_demo):
    rows = whole_demo['rows']
    assert max(abs(rows[0][joint]) for joint in ARM) < 1e-6
    # It starts from rest and is still crawling a second later.
    assert peak_speed(rows[:120]) < 0.5 * DEG


def test_every_joint_swings_about_its_start_pose_within_its_amplitude(whole_demo):
    teleop, rows = whole_demo['teleop'], whole_demo['rows'][:whole_demo['running']]
    for joint, amplitude in zip(ARM, teleop.config.demo.amplitudes):
        angles = [row[joint] for row in rows]
        assert max(angles) <= HOME[joint] + amplitude + 1e-9
        assert min(angles) >= HOME[joint] - amplitude - 1e-9
        assert max(angles) - min(angles) > 1.5 * amplitude


def test_the_swing_stays_clear_of_every_joint_limit(whole_demo):
    margin = whole_demo['teleop'].config.demo.margin
    for row in whole_demo['rows']:
        for joint, (lower, upper) in LIMITS.items():
            assert lower + margin <= row[joint] <= upper - margin


def test_the_swing_never_runs_into_the_table_or_the_base(whole_demo):
    assert not any(whole_demo['blocked'])
    collision = ArmCollision.from_urdf(URDF, 0.0, 0.01)
    for row in whole_demo['rows'][::12]:
        assert collision.check({**HOME, **row}) == {}


def test_the_arm_comes_back_to_exactly_where_it_started_and_the_mode_ends(whole_demo):
    teleop, rows = whole_demo['teleop'], whole_demo['rows']
    assert teleop.demo is None and teleop.demo_status()[0] == 'off'
    assert {joint: teleop.targets[joint] for joint in ARM} == {joint: HOME[joint] for joint in ARM}
    assert all(value == 0.0 for value in teleop.velocities.values())
    assert teleop.command_targets == teleop.targets
    assert 'back where it started' in teleop.note and not teleop.warning
    # And it was moving until then, not parked early.
    assert max(abs(rows[-250][joint]) for joint in ARM) > 0.0


def test_the_pose_it_comes_back_to_is_wherever_it_started(params, arm):
    start_pose = {**HOME, 'joint_1': 0.4, 'joint_3': -0.3, 'joint_4': 0.2}
    teleop = enabled(build(params, arm), start_pose)
    start(teleop)
    run(teleop, 30.0)
    assert any(abs(teleop.targets[joint] - start_pose[joint]) > 0.02 for joint in ARM)
    teleop.set_demo_mode(False)
    run(teleop, 13.0)
    assert teleop.demo is None
    assert all(teleop.targets[joint] == start_pose[joint] for joint in ARM)


def test_switching_off_returns_through_a_stopping_state(teleop):
    start(teleop)
    run(teleop, 20.0)
    ok, message = teleop.set_demo_mode(False)
    assert ok and 'stopping' in message
    assert teleop.demo_status()[0] == 'stopping'
    run(teleop, 6.0)
    assert teleop.demo_status()[0] == 'stopping'
    run(teleop, 7.0)
    assert teleop.demo_status()[0] == 'off'


def test_switching_on_again_while_it_is_stopping_carries_on_without_a_jolt(teleop):
    start(teleop)
    rows = trace(teleop, 15.0)
    teleop.set_demo_mode(False)
    rows += trace(teleop, 4.0)
    ok, message = teleop.set_demo_mode(True)
    assert ok and teleop.demo_status()[0] == 'running', message
    rows += trace(teleop, 15.0)
    assert peak_speed(rows) <= teleop.config.demo.max_speed
    assert peak_accel(rows) < 6.0 * DEG


@pytest.mark.parametrize('button', ['b', 'start'])
def test_b_and_start_stop_the_demo_at_once_and_hold_the_arm(teleop, button):
    start(teleop)
    run(teleop, 20.0)
    held = dict(teleop.targets)
    teleop.update(*sample(button), dict(held), DT)
    assert teleop.demo is None and not teleop.enabled
    assert teleop.demo_status() == ('off', '')
    assert all(value == 0.0 for value in teleop.velocities.values())
    run(teleop, 1.0)
    assert teleop.targets == held


def test_y_leaves_demo_mode_and_waits_for_the_sticks(teleop):
    start(teleop)
    run(teleop, 20.0)
    held = dict(teleop.targets)
    group = teleop.group
    teleop.update(*sample('y', left_y=1.0), dict(held), DT)
    assert teleop.demo is None and teleop.group != group
    assert teleop.wait_for_center
    run(teleop, 1.0, left_y=1.0)
    assert teleop.targets == held


def test_going_home_leaves_demo_mode_and_homes_the_arm(teleop):
    start(teleop)
    run(teleop, 25.0)
    assert max(abs(teleop.targets[joint]) for joint in ARM) > 0.02
    run(teleop, 6.0, 'a')
    assert teleop.demo is None
    assert max(abs(teleop.targets[joint]) for joint in ARM) < 1e-3


def test_the_sticks_do_nothing_while_it_runs(params, arm):
    alone, pushed = enabled(build(params, arm)), enabled(build(params, arm))
    start(alone)
    start(pushed)
    quiet = trace(alone, 20.0)
    busy = trace(pushed, 20.0, left_x=1.0, left_y=-1.0, right_x=1.0, right_y=1.0)
    assert busy == quiet


def test_the_tool_still_closes_and_opens_with_the_triggers(teleop):
    start(teleop)
    run(teleop, 1.0, right_trigger=-1.0)
    assert teleop.targets['tool_grip'] > 0.5


def test_the_speed_level_slows_the_demo_but_never_speeds_it_up(params, arm):
    def pace(*presses):
        teleop = enabled(build(params, arm))
        for button in presses:
            press(teleop, button)
        start(teleop)
        return trace(teleop, 40.0)

    normal, up = pace(), pace('dpad_up')
    half, fifth = pace('dpad_down'), pace('dpad_down', 'dpad_down')
    assert up == normal
    assert peak_speed(half) == pytest.approx(peak_speed(normal) / 2.0, rel=0.1)
    assert peak_speed(fifth) == pytest.approx(peak_speed(normal) / 5.0, rel=0.15)


def test_a_change_of_speed_level_does_not_jolt_the_arm(teleop):
    start(teleop)
    rows = trace(teleop, 20.0)
    rows += trace(teleop, DT, 'dpad_down')
    rows += trace(teleop, 20.0)
    assert peak_accel(rows) < 4.0 * DEG


def test_a_locked_joint_stays_exactly_where_it_is(params, arm):
    teleop = enabled(build(params, arm, locked={'joint_2': 0.0}))
    start(teleop)
    rows = trace(teleop, 40.0)
    assert all(row['joint_2'] == 0.0 for row in rows)
    assert max(abs(row['joint_3']) for row in rows) > 0.05


def test_a_joint_switched_off_stays_put_while_the_others_swing(params, arm):
    teleop = build(params, arm)
    assert teleop.set_disabled({'joint_4'}, HOME) is True
    enabled(teleop)
    start(teleop)
    rows = trace(teleop, 40.0)
    assert all(row['joint_4'] == 0.0 for row in rows)
    assert max(abs(row['joint_1']) for row in rows) > 0.02


def test_it_is_refused_when_no_demo_joint_can_move(params, arm):
    wrist = {**params, 'demo.joints': ['joint_4', 'joint_5'],
             'demo.amplitudes_deg': [20.0, 35.0], 'demo.cycles': [6, 2],
             'demo.phases_deg': [70.0, 200.0]}
    teleop = enabled(build(wrist, arm, locked={'joint_4': 0.0, 'joint_5': 0.0}))
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'locked or switched off' in message


def test_a_joint_too_near_a_limit_is_left_out_and_the_others_swing(params, arm):
    near = {**HOME, 'joint_3': 85 * DEG}
    teleop = enabled(build(params, arm, collision=None), near)
    start(teleop)
    rows = trace(teleop, 40.0)
    assert all(row['joint_3'] == near['joint_3'] for row in rows)
    assert max(abs(row['joint_1']) for row in rows) > 0.02


def test_every_joint_too_near_a_limit_refuses_the_demo(params, arm):
    near = {**dict.fromkeys(ARM, 85 * DEG), 'joint_1': 175 * DEG, 'tool_grip': 0.0}
    teleop = enabled(build(params, arm, collision=None), near)
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'room to swing' in message


def test_a_swing_with_room_to_one_side_only_stays_inside_the_limits(params, arm):
    # 30 degrees from a limit with a ten degree margin leaves 20 degrees, not the full 25.
    near = {**HOME, 'joint_3': 60 * DEG}
    teleop = enabled(build(params, arm, collision=None), near)
    start(teleop)
    rows = trace(teleop, 130.0)
    margin = teleop.config.demo.margin
    assert max(row['joint_3'] for row in rows) <= LIMITS['joint_3'][1] - margin + 1e-9
    assert max(row['joint_3'] for row in rows) > 78 * DEG
    # The swing is the same size to either side of where it began.
    assert min(row['joint_3'] for row in rows) < 42 * DEG


def test_chicken_head_and_demo_mode_refuse_each_other(params, arm):
    teleop = enabled(build(params, arm))
    ok, _ = teleop.set_head_mode(True)
    assert ok
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'Chicken head off first' in message
    teleop.set_head_mode(False)
    start(teleop)
    ok, message = teleop.set_head_mode(True)
    assert not ok and 'Demo mode off first' in message
    assert teleop.demo is not None and teleop.head is None


def test_a_tool_through_the_incision_refuses_the_demo(teleop, monkeypatch):
    monkeypatch.setattr(teleop, 'inserted', lambda: True)
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'inserted' in message


class Wall:
    """Stands where the shoulder cannot go further than ``reach`` from where it started."""

    def __init__(self, reach):
        self.reach = reach

    def check(self, joints):
        excess = abs(joints['joint_2']) - self.reach
        return {'wall': excess} if excess > 0.0 else {}


def test_a_swing_that_would_reach_an_obstacle_is_made_smaller_to_fit(params, arm):
    teleop = enabled(build(params, arm, collision=Wall(0.1)))
    start(teleop)
    rows = trace(teleop, 130.0)
    assert 0.05 < max(abs(row['joint_2']) for row in rows) <= 0.1
    assert teleop.demo is not None and not teleop.blocked
    # Every joint got the same smaller share of its swing.
    assert max(abs(row['joint_1']) for row in rows) < 0.4 * 30 * DEG


def test_a_swing_with_no_room_at_all_is_refused_and_names_what_is_in_the_way(params, arm):
    teleop = enabled(build(params, arm, collision=Wall(-1.0)))
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'wall' in message and 'move it away' in message
    assert teleop.demo is None


def test_something_in_the_way_ends_the_demo_and_holds_the_arm(params, arm):
    class Obstacle:
        """Appears at the pose the arm has reached, and stops it going on."""

        def __init__(self):
            self.pose = None

        def check(self, joints):
            if self.pose is None:
                return {}
            moved = any(abs(joints[joint] - angle) > 1e-9 for joint, angle in self.pose.items())
            return {'obstacle': 0.1} if moved else {}

    obstacle = Obstacle()
    teleop = enabled(build(params, arm, collision=obstacle))
    start(teleop)
    run(teleop, 20.0)
    obstacle.pose = dict(teleop.targets)
    run(teleop, 1 * DT)
    assert teleop.demo is None
    state, message = teleop.demo_status()
    assert state == 'off' and 'Demo stopped' in message and 'obstacle' in message
    assert teleop.warning and 'obstacle' in teleop.blocked
    assert teleop.targets == obstacle.pose
    held = dict(teleop.targets)
    run(teleop, 2.0)
    assert teleop.targets == held and teleop.enabled


def test_a_stopped_demo_keeps_its_reason_until_it_is_switched_on_again(params, arm):
    class Obstacle:
        armed = False

        def check(self, joints):
            return {'obstacle': 0.1} if self.armed and joints['joint_1'] != 0.0 else {}

    obstacle = Obstacle()
    teleop = enabled(build(params, arm, collision=obstacle))
    start(teleop)
    obstacle.armed = True
    run(teleop, 1.0)
    assert 'obstacle' in teleop.demo_status()[1]
    obstacle.armed = False
    start(teleop)
    assert teleop.demo_status() == ('running', 'Moving slowly around its start pose; '
                                    'B stops it at once')


def test_a_demo_that_finishes_with_a_stick_held_waits_for_it_to_centre(teleop):
    start(teleop)
    run(teleop, 20.0)
    teleop.set_demo_mode(False)
    run(teleop, 13.5, left_y=1.0)
    assert teleop.demo is None and teleop.wait_for_center
    held = dict(teleop.targets)
    run(teleop, 1.0, left_y=1.0)
    assert teleop.targets == held and 'Center' in teleop.note
    run(teleop, 0.2)
    run(teleop, 0.5, left_y=1.0)
    assert teleop.targets != held


def test_a_step_of_no_time_changes_nothing(teleop):
    start(teleop)
    run(teleop, 5.0)
    before = dict(teleop.targets)
    teleop.update(*sample(), dict(before), 0.0)
    assert teleop.targets == before


def test_a_joint_switched_off_while_it_runs_stops_teleop_and_the_demo(params, arm):
    teleop = enabled(build(params, arm))
    start(teleop)
    run(teleop, 5.0)
    assert teleop.set_disabled({'joint_3'}, dict(teleop.targets))
    assert not teleop.enabled and teleop.demo is None


def test_demo_mode_is_unavailable_without_a_demo_section(params, arm):
    without = {name: value for name, value in params.items() if not name.startswith('demo.')}
    teleop = enabled(build(without, arm))
    assert teleop.config.demo is None
    assert teleop.demo_status()[0] == 'unavailable'
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'demo section' in message


def test_demo_mode_is_unavailable_when_the_arm_lacks_a_demo_joint(params, arm):
    short = {joint: bounds for joint, bounds in LIMITS.items() if joint != 'joint_5'}
    teleop = ArmTeleop(config_from_parameters(params), short, None, None)
    assert teleop.demo_status()[0] == 'unavailable'
    ok, message = teleop.set_demo_mode(True)
    assert not ok and 'fits this arm' in message


@pytest.mark.parametrize('change, message', [
    ({'demo.amplitudes_deg': [30.0, 15.0]}, 'a phase for each swing'),
    ({'demo.joints': []}, 'a phase for each swing'),
    ({'demo.joints': ['joint_1', 'joint_1', 'joint_3', 'joint_4', 'joint_5']}, 'different'),
    ({'demo.joints': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'tool_grip']}, 'not the tool'),
    ({'demo.loop_s': float('inf')}, 'finite'),
    ({'demo.phases_deg': [0.0, 140.0, 250.0, float('nan'), 200.0]}, 'finite'),
    ({'demo.amplitudes_deg': [30.0, -15.0, 25.0, 20.0, 40.0]}, 'negative'),
    ({'demo.margin_deg': -1.0}, 'negative'),
    ({'demo.loop_s': 0.0}, 'positive'),
    ({'demo.ramp_s': -3.0}, 'positive'),
    ({'demo.cycles': [3, 5, 4, 0, 2]}, 'whole numbers'),
    ({'demo.cycles': [3, 5, 4, 6, 2.5]}, 'whole numbers'),
    ({'demo.max_speed_deg_s': 90.0}, 'above max_speed_deg_s'),
    ({'demo.max_speed_deg_s': 5.0}, 'could go at'),
    ({'demo.ramp_s': 2.0}, 'could go at'),
    ({'demo.cycles': [3, 5, 4, 6, 20]}, 'could go at'),
])
def test_a_demo_that_is_badly_set_up_or_too_fast_is_refused(params, change, message):
    with pytest.raises(ValueError, match=message):
        config_from_parameters({**params, **change})


def test_a_whole_number_of_cycles_that_arrives_as_a_float_is_accepted(params):
    config = config_from_parameters({**params, 'demo.cycles': [3.0, 5.0, 4.0, 6.0, 2.0]})
    assert config.demo.cycles == [3, 5, 4, 6, 2]
