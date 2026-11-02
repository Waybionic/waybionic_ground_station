"""Chicken head mode: the tip goes to a head spot and stays there while the tool tilts."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
HOME = dict.fromkeys([*LIMITS, 'tool_grip'], 0.0)
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
DT = 1.0 / 120.0
TIP_TOLERANCE = 0.0005


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


def run(teleop, seconds, *pressed, **axes):
    for _ in range(round(seconds / DT)):
        teleop.update(*sample(*pressed, **axes), dict(teleop.targets), DT)


def press(teleop, button):
    teleop.update(*sample(button), dict(teleop.targets), DT)
    teleop.update(*sample(), dict(teleop.targets), DT)


@pytest.fixture
def arm():
    return ArmKinematics.from_urdf(URDF)


@pytest.fixture
def params(parameters):
    return parameters('xbox_teleop.yaml', 'xbox_teleop')


def build(params, arm, **options):
    options.setdefault('collision', ArmCollision.from_urdf(URDF, 0.0, 0.01))
    return ArmTeleop(config_from_parameters(params), LIMITS, arm, **options)


def enabled(teleop):
    teleop.update(*sample('start'), HOME, DT)
    teleop.update(*sample(), HOME, DT)
    assert teleop.enabled
    return teleop


@pytest.fixture
def teleop(params, arm):
    return enabled(build(params, arm))


def reach_head(teleop, seconds=40.0):
    ok, message = teleop.set_head_mode(True)
    assert ok, message
    for _ in range(round(seconds / DT)):
        teleop.update(*sample(), dict(teleop.targets), DT)
        if teleop.head_status()[0] == 'hold':
            # The sticks are centred as it arrives, as an operator's hands are.
            teleop.update(*sample(), dict(teleop.targets), DT)
            return
    pytest.fail(f'never reached the head position: {teleop.head_status()}')


def tip(arm, teleop):
    return arm.forward(teleop.targets)


def test_the_head_spot_and_sticks_come_from_the_config(teleop):
    head = teleop.config.head
    assert head.position == (0.30, 0.0, 0.55)
    assert math.isclose(head.pitch, math.pi / 2)
    assert (head.tilt_axis, head.roll_axis) == ('left_y', 'right_x')


def test_the_mode_is_off_until_it_is_switched_on(teleop):
    assert teleop.head_status()[0] == 'off'
    assert teleop.head is None


def test_the_tip_goes_to_the_head_spot_and_holds_there(teleop, arm):
    reach_head(teleop)
    position, pitch = tip(arm, teleop)
    assert math.dist(position, teleop.config.head.position) < TIP_TOLERANCE
    assert math.isclose(pitch, teleop.config.head.pitch, abs_tol=math.radians(0.5))
    assert 'left stick' in teleop.head_status()[1] and 'right stick' in teleop.head_status()[1]


def test_the_approach_never_moves_a_joint_faster_than_the_speed_limit(teleop):
    ok, message = teleop.set_head_mode(True)
    assert ok, message
    previous, fastest = dict(teleop.targets), 0.0
    for _ in range(round(40.0 / DT)):
        teleop.update(*sample(), dict(teleop.targets), DT)
        fastest = max(fastest, *(abs(teleop.targets[joint] - previous[joint]) / DT
                                 for joint in LIMITS))
        for joint, (lower, upper) in LIMITS.items():
            assert lower <= teleop.targets[joint] <= upper
        previous = dict(teleop.targets)
        if teleop.head_status()[0] == 'hold':
            break
    assert teleop.head_status()[0] == 'hold'
    assert 0.0 < fastest <= teleop.config.max_speed * 1.0001


def test_the_approach_ignores_the_sticks(teleop, arm):
    ok, _ = teleop.set_head_mode(True)
    assert ok
    for _ in range(round(40.0 / DT)):
        teleop.update(*sample(left_y=1.0, right_x=1.0), dict(teleop.targets), DT)
        if teleop.head_status()[0] == 'hold':
            break
    assert math.dist(tip(arm, teleop)[0], teleop.config.head.position) < TIP_TOLERANCE


def test_a_stick_still_held_on_arrival_waits_for_the_centre(teleop, arm):
    ok, _ = teleop.set_head_mode(True)
    assert ok
    for _ in range(round(40.0 / DT)):
        teleop.update(*sample(left_y=1.0), dict(teleop.targets), DT)
        if teleop.head_status()[0] == 'hold':
            break
    arrived = dict(teleop.targets)
    run(teleop, 1.0, left_y=1.0)
    assert teleop.targets == arrived
    assert 'Center' in teleop.note
    run(teleop, 0.1)
    run(teleop, 1.0, left_y=1.0)
    assert teleop.targets != arrived


def test_the_left_stick_tilts_the_tool_while_the_tip_stays(teleop, arm):
    reach_head(teleop)
    head, level = teleop.config.head.position, tip(arm, teleop)[1]
    run(teleop, 3.0, left_y=1.0)
    position, pitch = tip(arm, teleop)
    assert math.dist(position, head) < TIP_TOLERANCE
    # Stick up tilts the tool up, which lowers the angle from straight up.
    assert level - pitch > math.radians(20)
    run(teleop, 6.0, left_y=-1.0)
    position, pitch = tip(arm, teleop)
    assert math.dist(position, head) < TIP_TOLERANCE
    assert pitch - level > math.radians(20)


def test_the_right_stick_rolls_the_tool_while_the_tip_stays(teleop, arm):
    reach_head(teleop)
    roll = teleop.targets['joint_5']
    run(teleop, 2.0, right_x=1.0)
    assert abs(teleop.targets['joint_5'] - roll) > math.radians(20)
    assert math.dist(tip(arm, teleop)[0], teleop.config.head.position) < TIP_TOLERANCE


def test_tilting_moves_the_shoulder_elbow_and_wrist_around_the_tip(teleop, arm):
    reach_head(teleop)
    before = dict(teleop.targets)
    run(teleop, 3.0, left_y=1.0)
    moved = [joint for joint in ('joint_2', 'joint_3', 'joint_4')
             if abs(teleop.targets[joint] - before[joint]) > math.radians(5)]
    assert len(moved) >= 2
    assert math.dist(tip(arm, teleop)[0], teleop.config.head.position) < TIP_TOLERANCE


@pytest.mark.parametrize('stick', [1.0, -1.0])
def test_tilting_stops_at_a_joint_limit_and_names_it_with_the_tip_in_place(teleop, arm, stick):
    reach_head(teleop)
    run(teleop, 25.0, left_y=stick)
    assert teleop.blocked
    assert all(name in LIMITS for name in teleop.blocked), teleop.blocked
    assert math.dist(tip(arm, teleop)[0], teleop.config.head.position) < TIP_TOLERANCE
    for joint, (lower, upper) in LIMITS.items():
        assert lower - 1e-6 <= teleop.targets[joint] <= upper + 1e-6
    stopped = dict(teleop.targets)
    run(teleop, 1.0, left_y=stick)
    assert max(abs(teleop.targets[joint] - stopped[joint]) for joint in LIMITS) < 1e-6
    assert teleop.blocked
    # Letting go clears it, and the other way is open.
    run(teleop, 0.5)
    assert not teleop.blocked
    run(teleop, 1.0, left_y=-stick)
    assert max(abs(teleop.targets[joint] - stopped[joint]) for joint in LIMITS) > 0.01


def test_a_stick_that_moves_nothing_reports_nothing_when_it_is_not_pushed(teleop):
    reach_head(teleop)
    run(teleop, 25.0, left_y=1.0)
    run(teleop, 1.0)
    assert teleop.blocked == []


def test_centered_sticks_leave_the_arm_alone(teleop):
    reach_head(teleop)
    held = dict(teleop.targets)
    run(teleop, 2.0)
    assert max(abs(teleop.targets[joint] - held[joint]) for joint in LIMITS) < 1e-6


def test_the_tip_does_not_drift_over_a_long_wiggle(teleop, arm):
    reach_head(teleop)
    for _ in range(4):
        run(teleop, 2.0, left_y=1.0, right_x=1.0)
        run(teleop, 2.0, left_y=-1.0, right_x=-1.0)
    assert math.dist(tip(arm, teleop)[0], teleop.config.head.position) < TIP_TOLERANCE / 5


def test_switching_it_off_holds_the_arm_and_gives_the_sticks_back(teleop):
    reach_head(teleop)
    ok, message = teleop.set_head_mode(False)
    assert ok and teleop.head_status()[0] == 'off'
    held = dict(teleop.targets)
    run(teleop, 1.0)
    assert teleop.targets == held
    run(teleop, 1.0, left_x=1.0)
    assert teleop.targets['joint_1'] != held['joint_1']


@pytest.mark.parametrize('button', ['b', 'y', 'a'])
def test_stop_group_and_home_end_the_mode(teleop, button):
    reach_head(teleop)
    press(teleop, button)
    assert teleop.head_status()[0] == 'off'


def test_the_mode_needs_teleop_enabled_first(params, arm):
    teleop = build(params, arm)
    ok, message = teleop.set_head_mode(True)
    assert not ok and 'Start' in message
    assert teleop.head_status()[0] == 'off'


def test_a_joint_that_cannot_move_refuses_the_mode(params, arm):
    teleop = enabled(build(params, arm, locked={'joint_2': 0.0}))
    ok, message = teleop.set_head_mode(True)
    assert not ok and 'joint_2' in message
    assert teleop.head_status()[0] == 'off'


def test_a_head_spot_the_arm_cannot_reach_is_refused(params, arm):
    teleop = enabled(build({**params, 'chicken_head.position': [2.0, 0.0, 0.5]}, arm))
    ok, message = teleop.set_head_mode(True)
    assert not ok and 'reach' in message
    assert teleop.head_status()[0] == 'off'


def test_a_mode_with_no_head_settings_is_unavailable(params, arm):
    flat = {name: value for name, value in params.items() if not name.startswith('chicken_head.')}
    teleop = enabled(build(flat, arm))
    ok, message = teleop.set_head_mode(True)
    assert not ok and 'chicken_head' in message
    assert teleop.head_status()[0] == 'unavailable'


def test_a_mode_without_arm_kinematics_is_unavailable(params):
    teleop = ArmTeleop(config_from_parameters(params), LIMITS)
    assert teleop.head_status()[0] == 'unavailable'


def test_an_approach_that_cannot_make_progress_says_it_is_blocked(params, arm):
    class Wall:
        """Any move from the start pose goes into something."""

        def __init__(self, start):
            self.start = dict(start)

        def check(self, joints):
            return {'wall': 0.1} if any(joints[name] != self.start[name]
                                        for name in LIMITS) else {}

    teleop = enabled(build(params, arm, collision=Wall(HOME)))
    ok, _ = teleop.set_head_mode(True)
    assert ok
    run(teleop, 3.0)
    state, message = teleop.head_status()
    assert state == 'blocked' and 'wall' in message
    assert teleop.targets == HOME or max(abs(teleop.targets[j] - HOME[j]) for j in LIMITS) == 0.0


@pytest.mark.parametrize('change, message', [
    ({'chicken_head.position': [0.3, 0.0]}, 'position'),
    ({'chicken_head.position': [0.3, 0.0, float('nan')]}, 'position'),
    ({'chicken_head.pitch_deg': float('inf')}, 'pitch'),
    ({'chicken_head.axes': ['left_y', 'sideways']}, 'unknown controller inputs'),
    ({'chicken_head.axes': ['left_y']}, 'axes'),
    ({'chicken_head.scales': [1.0, 2.0]}, 'scales'),
])
def test_a_bad_head_config_is_refused(params, change, message):
    with pytest.raises(ValueError, match=message):
        config_from_parameters({**params, **change})
