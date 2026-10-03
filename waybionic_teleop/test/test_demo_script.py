"""The self-running demo steers teleop into the same Cartesian square from whatever it finds."""

import math
from pathlib import Path

import pytest

from waybionic_teleop.demo_script import (
    CARTESIAN, DemoPlayer, LINES, operator_active, TeleopState)
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
LIMITS = {'joint_1': (-math.pi, math.pi), 'joint_2': (-1.5708, 1.5708),
          'joint_3': (-1.5708, 1.5708), 'joint_4': (-1.5708, 1.5708),
          'joint_5': (-1.5708, 1.5708)}
RATE = 120.0
# The teleop node's input timeout and how often it reports its state.
INPUT_TIMEOUT_S = 0.5
REPORT_S = 0.5
# The pause before the first edge, the edge itself, and the pause after it.
FIRST_EDGE_TICKS = round(sum(seconds for seconds, *_ in CARTESIAN[:3]) * RATE)


def play(teleop, kinematics, measured, seconds):
    """Run the player against teleop and perfect drives; return the tip at each square start."""
    player, state = DemoPlayer(RATE), TeleopState()
    last, quiet, tips, caption = None, 0.0, [], None
    for tick in range(round(seconds * RATE)):
        if tick % round(REPORT_S * RATE) == 0:
            state.enabled, state.group = teleop.enabled, teleop.active_group.name
        state.at_home = all(abs(measured[joint]) < 0.01 for joint in kinematics.joints)
        sample = player.step(state)
        # Like the teleop node: the last sample stays in force until the input times out.
        quiet = quiet + 1.0 / RATE if sample is None else 0.0
        last = sample or last
        if quiet > INPUT_TIMEOUT_S:
            if teleop.enabled:
                teleop.disable(measured, 'Controller lost')
            teleop.update((), (), measured, 1.0 / RATE)
        elif last is not None:
            teleop.update(*last, measured, 1.0 / RATE)
        if teleop.targets:
            measured = dict(teleop.targets)
        if player.caption == LINES and caption != LINES:
            tips.append((tick, kinematics.forward(measured)[0], teleop.active_group.name))
        if tips and tick == tips[-1][0] + FIRST_EDGE_TICKS:
            tips[-1] += (kinematics.forward(measured)[0], teleop.level)
        caption = player.caption
    return tips


@pytest.fixture
def make_teleop(parameters):
    def make(**state):
        kinematics = ArmKinematics.from_urdf(URDF)
        teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                           LIMITS, kinematics)
        for name, value in state.items():
            setattr(teleop, name, value)
        return teleop, kinematics
    return make


def test_the_demo_draws_the_same_square_from_a_fresh_start_and_on_every_loop(make_teleop):
    teleop, kinematics = make_teleop()
    measured = dict.fromkeys([*LIMITS, 'tool_grip'], 0.0)
    squares = play(teleop, kinematics, measured, 90.0)
    assert len(squares) >= 2
    for _, start, group, end, level in squares[:2]:
        assert (group, level) == ('cartesian', 2)
        # The first edge pushes left at 25 mm/s for 1.7 s, so the tip moves about 41 mm.
        assert 0.038 < end[1] - start[1] < 0.044
        assert abs(end[0] - start[0]) < 5e-4 and abs(end[2] - start[2]) < 5e-4
    assert squares[1][1] == pytest.approx(squares[0][1], abs=1e-3)


def test_the_demo_recovers_whatever_state_the_operator_left(make_teleop):
    fresh, kinematics = make_teleop()
    reference = play(fresh, kinematics, dict.fromkeys([*LIMITS, 'tool_grip'], 0.0), 40.0)[0]
    # Enabled in the Cartesian group at the slowest speed, with the arm away from home.
    measured = {'joint_1': 0.6, 'joint_2': -0.4, 'joint_3': 0.9, 'joint_4': -0.7,
                'joint_5': 0.3, 'tool_grip': 0.5}
    teleop, _ = make_teleop(level=0, group=2)
    teleop.enable(measured, ())
    assert teleop.enabled and teleop.active_group.name == 'cartesian'
    _, start, group, end, level = play(teleop, kinematics, measured, 45.0)[0]
    assert (group, level) == ('cartesian', 2)
    assert start == pytest.approx(reference[1], abs=1e-3)
    assert end == pytest.approx(reference[3], abs=1e-3)


@pytest.mark.parametrize('axes, buttons, active', [
    ([0.0] * 6, [0] * 21, False),
    ([0.1, -0.1, 0.0, 0.0, 0.0, 0.0], [0] * 21, False),
    ([0.0, 0.5, 0.0, 0.0, 0.0, 0.0], [0] * 21, True),
    ([0.0, 0.0, 0.0, 0.0, 0.0, -0.6], [0] * 21, True),
    ([0.0] * 6, [0] * 6 + [1] + [0] * 14, True),
    ([math.nan] + [0.0] * 5, [0] * 21, False),
])
def test_operator_activity(axes, buttons, active):
    assert operator_active(axes, buttons, 0.15) is active
