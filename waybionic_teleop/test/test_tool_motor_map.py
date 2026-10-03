"""Test four-motor claw conversion with synthetic, non-hardware calibration."""

import math

import pytest

from waybionic_teleop.tool_motor_map import (
    ClawCalibration, MotorCalibration, ToolMotorMap)


@pytest.fixture
def tool_map():
    """Use synthetic values that are not calibrated to the real clamp."""
    return ToolMotorMap((
        ClawCalibration('left', 0.0, 30.0, 0.0, 10.0, (
            MotorCalibration('left_a', 1, 80.0, 100),
            MotorCalibration('left_b', -1, 80.0, -100))),
        ClawCalibration('right', -5.0, 25.0, 5.0, 8.0, (
            MotorCalibration('right_a', -1, 100.0, 20),
            MotorCalibration('right_b', 1, 100.0, -20))),
    ))


def test_each_pair_moves_oppositely_and_claws_are_independent(tool_map):
    """One claw's command must not change the other pair's step targets."""
    targets = tool_map.target_steps({'left': 10.0, 'right': 5.0})
    assert targets == {'left_a': 900, 'left_b': -900,
                       'right_a': 20, 'right_b': -20}
    other = tool_map.target_steps({'left': 10.0, 'right': 15.0})
    assert other['left_a'] == targets['left_a']
    assert other['left_b'] == targets['left_b']
    assert other['right_a'] == -980
    assert other['right_b'] == 980


def test_limits_and_speed_are_enforced(tool_map):
    """Reject travel and speed outside each claw's configured range."""
    assert tool_map.speed_steps_per_second({'left': -10.0, 'right': 4.0}) == {
        'left_a': 800.0, 'left_b': 800.0,
        'right_a': 400.0, 'right_b': 400.0}
    for positions in ({'left': 30.1, 'right': 5.0},
                      {'left': 0.0, 'right': -5.1},
                      {'left': math.nan, 'right': 5.0},
                      {'left': 0.0}):
        with pytest.raises(ValueError):
            tool_map.target_steps(positions)
    with pytest.raises(ValueError):
        tool_map.speed_steps_per_second({'left': 10.1, 'right': 0.0})


def test_feedback_reports_pair_disagreement(tool_map):
    """Decode each motor independently to detect pair mismatch."""
    targets = tool_map.target_steps({'left': 10.0, 'right': 15.0})
    positions, disagreement = tool_map.feedback(targets)
    assert positions == {'left': 10.0, 'right': 15.0}
    assert disagreement == {'left': 0.0, 'right': 0.0}
    targets['left_a'] += 80
    positions, disagreement = tool_map.feedback(targets)
    assert positions['left'] == pytest.approx(10.5)
    assert disagreement['left'] == pytest.approx(1.0)


def test_invalid_pair_calibration_is_rejected():
    """Each claw must have one positive and one negative motor mapping."""
    left = ClawCalibration('left', 0.0, 30.0, 0.0, 10.0, (
        MotorCalibration('a', 1, 80.0, 0), MotorCalibration('b', 1, 80.0, 0)))
    right = ClawCalibration('right', 0.0, 30.0, 0.0, 10.0, (
        MotorCalibration('c', 1, 80.0, 0), MotorCalibration('d', -1, 80.0, 0)))
    with pytest.raises(ValueError, match='opposing motor pair'):
        ToolMotorMap((left, right))
