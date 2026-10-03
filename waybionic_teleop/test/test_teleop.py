"""Controller-to-joint behaviour of the placeholder Xbox mapping."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
HOME = dict.fromkeys([*LIMITS, 'tool_grip'], 0.0)
# Tool pointing straight down in front of the base, where straight cuts start.
DOWN = {**HOME, 'joint_1': 0.1, 'joint_2': 0.5, 'joint_3': 1.4, 'joint_4': math.pi - 1.9}
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
DT = 0.02


def sample(*pressed, **axes):
    return [axes.get(name, 0.0) for name in AXES], [int(name in pressed) for name in BUTTONS]


def run(teleop, seconds, *pressed, **axes):
    for _ in range(round(seconds / DT)):
        teleop.update(*sample(*pressed, **axes), HOME, DT)


def press(teleop, button, pose=HOME):
    teleop.update(*sample(button), pose, DT)
    teleop.update(*sample(), pose, DT)


@pytest.fixture
def params(parameters):
    return parameters('xbox_teleop.yaml', 'xbox_teleop')


@pytest.fixture
def teleop(params):
    return ArmTeleop(config_from_parameters(params), LIMITS)


@pytest.fixture
def arm():
    return ArmKinematics.from_urdf(URDF)


@pytest.fixture
def cartesian(params, arm):
    teleop = ArmTeleop(config_from_parameters(params), LIMITS, arm)
    press(teleop, 'start', DOWN)
    press(teleop, 'y', DOWN)
    press(teleop, 'y', DOWN)
    return teleop


def test_sticks_do_nothing_until_start_enables_from_the_measured_pose(teleop):
    run(teleop, 1.0, left_x=1.0)
    assert not teleop.enabled and teleop.targets == {}
    pose = {**HOME, 'joint_2': 0.4}
    press(teleop, 'start', pose)
    assert teleop.enabled and teleop.targets == pose


def test_enable_waits_for_joint_states(teleop):
    teleop.update(*sample('start'), {'joint_1': 0.0}, DT)
    assert not teleop.enabled and teleop.warning and 'joint_2' in teleop.note


@pytest.mark.parametrize('axes', [{'left_x': 0.8}, {'right_y': -0.5}, {'right_trigger': -0.6}])
def test_enable_refuses_held_or_stuck_inputs(teleop, axes):
    teleop.update(*sample('start', **axes), HOME, DT)
    assert not teleop.enabled and teleop.warning and 'Center' in teleop.note


@pytest.mark.parametrize('button', ['dpad_left', 'dpad_right', 'a'])
def test_enable_refuses_held_tilt_and_home_buttons(params, arm, button):
    teleop = ArmTeleop(config_from_parameters(params), LIMITS, arm)
    press(teleop, 'y', DOWN)
    press(teleop, 'y', DOWN)
    teleop.update(*sample('start', button), DOWN, DT)
    assert not teleop.enabled and teleop.warning and 'release' in teleop.note


def test_base_group_moves_yaw_shoulder_and_elbow(teleop):
    press(teleop, 'start')
    run(teleop, 1.0, left_x=1.0, left_y=-1.0, right_y=0.5)
    targets = teleop.targets
    assert targets['joint_1'] > 0.3 and targets['joint_2'] < -0.3 and targets['joint_3'] > 0.1
    assert targets['joint_4'] == targets['joint_5'] == 0.0


def test_y_switches_to_the_upper_joints_and_leaves_the_base_alone(teleop):
    press(teleop, 'start')
    press(teleop, 'y')
    run(teleop, 1.0, left_x=1.0, left_y=1.0, right_x=-1.0, right_y=1.0)
    targets = teleop.targets
    assert teleop.active_group.name == 'upper'
    assert targets['joint_1'] == 0.0 and targets['joint_2'] > 0.3
    assert targets['joint_3'] > 0.3 and targets['joint_4'] > 0.3 and targets['joint_5'] < -0.3


def test_small_stick_drift_is_ignored(teleop):
    press(teleop, 'start')
    run(teleop, 1.0, left_x=0.1, left_y=-0.12)
    assert teleop.targets == HOME


def test_b_holds_the_measured_pose_and_disables(teleop):
    press(teleop, 'start')
    run(teleop, 0.5, left_x=1.0)
    pose = {**HOME, 'joint_1': 0.2}
    assert teleop.update(*sample('b'), pose, DT)
    assert not teleop.enabled and not teleop.warning and teleop.targets['joint_1'] == 0.2
    assert not teleop.update(*sample(left_x=1.0), pose, DT)


def test_joint_limits_stop_motion_and_are_reported(teleop):
    press(teleop, 'start')
    run(teleop, 6.0, left_y=1.0)
    assert teleop.targets['joint_2'] == pytest.approx(1.5708)
    assert teleop.blocked == ['joint_2']


def test_triggers_close_and_open_the_tool_within_its_range(teleop):
    press(teleop, 'start')
    run(teleop, 2.0, right_trigger=-1.0)
    assert teleop.targets['tool_grip'] == 1.0
    run(teleop, 0.5, left_trigger=-1.0)
    assert teleop.targets['tool_grip'] == pytest.approx(0.5)


def test_dpad_changes_speed_once_per_press(teleop):
    press(teleop, 'start')
    run(teleop, 0.5, 'dpad_up')
    assert teleop.level == 3
    press(teleop, 'dpad_down')
    press(teleop, 'dpad_down')
    assert teleop.level == 1


def test_holding_a_returns_to_the_zero_pose(teleop):
    press(teleop, 'start')
    run(teleop, 1.0, left_x=1.0)
    run(teleop, 5.0, 'a')
    assert teleop.targets['joint_1'] == pytest.approx(0.0, abs=1e-3)


def test_the_cartesian_group_moves_the_tip_along_a_straight_line(arm, cartesian):
    assert cartesian.active_group.name == 'cartesian'
    start, pitch = arm.forward(cartesian.targets)
    path = []
    for _ in range(round(1.0 / DT)):
        cartesian.update(*sample(left_x=1.0), DOWN, DT)
        path.append(arm.forward(cartesian.targets))
    for (x, y, z), tool_pitch in path:
        assert (x, z, tool_pitch) == pytest.approx((start[0], start[2], pitch), abs=1e-9)
    # Half of the 50 mm/s maximum at the initial speed level, after a 0.1 s ramp.
    assert cartesian.linear == pytest.approx((0.0, 0.025, 0.0))
    assert path[-1][0][1] - start[1] == pytest.approx(0.024, abs=1e-6)
    assert all(cartesian.targets[joint] != DOWN[joint] for joint in LIMITS)


def test_holding_lb_keeps_only_the_strongest_direction(arm, cartesian):
    start = arm.forward(cartesian.targets)[0]
    for _ in range(25):
        cartesian.update(*sample('left_bumper', left_x=0.4, left_y=1.0), DOWN, DT)
    end = arm.forward(cartesian.targets)[0]
    assert end[0] > start[0] + 0.005
    assert end[1:] == pytest.approx(start[1:], abs=1e-12)


def test_the_dpad_tilts_the_tool_about_its_tip(arm, cartesian):
    start, pitch = arm.forward(cartesian.targets)
    for _ in range(round(0.5 / DT)):
        cartesian.update(*sample('dpad_right'), DOWN, DT)
        assert arm.forward(cartesian.targets)[0] == pytest.approx(start, abs=1e-9)
    # 15 deg/s at the initial speed level, less the short ramp.
    assert arm.forward(cartesian.targets)[1] - pitch == pytest.approx(-math.radians(7.5),
                                                                      abs=math.radians(0.3))


def test_releasing_a_after_homing_does_not_resume_the_last_move(arm, cartesian):
    for _ in range(round(0.5 / DT)):
        cartesian.update(*sample('dpad_right', left_x=1.0, right_x=1.0), DOWN, DT)
    for _ in range(3):
        cartesian.update(*sample('a'), DOWN, DT)
    held = dict(cartesian.targets)
    cartesian.update(*sample(), DOWN, DT)
    assert cartesian.targets == pytest.approx(held, abs=1e-12)


def test_cartesian_roll_follows_the_acceleration_ramp(params, cartesian):
    step = math.radians(params['max_accel_deg_s2']) * DT
    rates = []
    for _ in range(3):
        cartesian.update(*sample(right_x=1.0), DOWN, DT)
        rates.append(cartesian.roll)
    cartesian.update(*sample(), DOWN, DT)
    assert rates == pytest.approx([step, 2 * step, 3 * step])
    assert cartesian.roll == pytest.approx(2 * step)


def test_without_arm_kinematics_the_cartesian_group_is_skipped(teleop):
    assert [group.name for group in teleop.groups] == ['base', 'upper']
    press(teleop, 'start')
    press(teleop, 'y')
    press(teleop, 'y')
    assert teleop.active_group.name == 'base'


@pytest.mark.parametrize('change', [
    {'base.axes': ['left_x', 'left_y', 'trackpad']},
    {'stop_button': 'turbo'},
    {'base.scales': [1.0]},
    {'tool_limits': [1.0, 0.0]},
    {'cartesian.axes': ['left_y', 'left_x', 'right_y']},
    {'cartesian.mode': 'polar'},
    {'keep_in_min_mm': [0.0, 0.0, 900.0]},
    {'keep_in_max_mm': [500.0, 500.0]},
    {'keep_in_min_mm': [math.nan, 0.0, 0.0]},
])
def test_invalid_mappings_are_rejected(params, change):
    with pytest.raises(ValueError):
        config_from_parameters({**params, **change})


def test_a_keep_in_box_without_the_home_pose_is_rejected(params, arm):
    # The home tool tip is about 759 mm above the base.
    params['keep_in_max_mm'] = [500.0, 500.0, 500.0]
    with pytest.raises(ValueError, match='home'):
        ArmTeleop(config_from_parameters(params), LIMITS, arm)


def test_missing_parameters_are_named(params):
    del params['deadzone']
    with pytest.raises(ValueError, match='deadzone'):
        config_from_parameters(params)
