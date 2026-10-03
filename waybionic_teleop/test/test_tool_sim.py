"""Test provisional four-motor simulation without ROS or hardware."""

import pytest

from waybionic_teleop.tool_sim import SimulatedTool, tool_map_from_parameters


@pytest.fixture
def simulator(parameters):
    """Load simulation-only calibration values from the tool config."""
    values = parameters('tool_drives.yaml', 'sim_tool_motors')
    return SimulatedTool(tool_map_from_parameters(values))


def test_left_claw_moves_only_its_motor_pair(simulator):
    """Independent claw commands leave the other pair at its reference."""
    simulator.command({'left_claw': 0.5, 'right_claw': 0.0})
    simulator.tick(1.0)
    positions, disagreement, counts = simulator.feedback()
    assert positions['left_claw'] == pytest.approx(0.25, abs=0.02)
    assert positions['right_claw'] == 0.0
    assert counts['left_1'] > 0 and counts['left_2'] < 0
    assert counts['right_1'] == counts['right_2'] == 0
    assert disagreement['left_claw'] == 0.0


def test_both_pairs_move_and_stop_at_targets(simulator):
    """Four motors reach their commanded positions without overshoot."""
    simulator.command({'left_claw': 0.5, 'right_claw': 1.0})
    for _ in range(6):
        simulator.tick(1.0)
    positions, disagreement, counts = simulator.feedback()
    expected = {'left_claw': 0.5, 'right_claw': 1.0}
    assert positions == pytest.approx(expected, abs=0.01)
    assert counts == simulator.targets
    assert disagreement == {'left_claw': 0.0, 'right_claw': 0.0}


def test_invalid_command_does_not_change_existing_targets(simulator):
    """Out-of-range targets are rejected before changing either pair."""
    old = dict(simulator.targets)
    with pytest.raises(ValueError):
        simulator.command({'left_claw': 0.5, 'right_claw': 1.1})
    assert simulator.targets == old


def test_pair_disagreement_is_visible_in_feedback(simulator):
    """A motor that falls behind produces a nonzero claw-pair error."""
    simulator.counts['left_1'] = 33
    _, disagreement, _ = simulator.feedback()
    assert disagreement['left_claw'] == pytest.approx(0.5)
