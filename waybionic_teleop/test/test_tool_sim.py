"""Test provisional four-motor simulation without ROS or hardware."""

import pytest

from waybionic_teleop.tool_sim import (
    SimulatedTool, tool_map_from_parameters, ToolSafety)


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


def test_limits_are_reached_without_overshoot(simulator):
    """Both endpoints are exact even after more simulation time passes."""
    simulator.command({'left_claw': 1.0, 'right_claw': 1.0})
    simulator.tick(10.0)
    simulator.tick(10.0)
    positions, disagreement, counts = simulator.feedback()
    assert positions == {'left_claw': 1.0, 'right_claw': 1.0}
    assert counts == simulator.targets
    assert disagreement == {'left_claw': 0.0, 'right_claw': 0.0}
    simulator.command({'left_claw': 0.0, 'right_claw': 0.0})
    simulator.tick(10.0)
    assert simulator.feedback()[0] == {'left_claw': 0.0, 'right_claw': 0.0}


def test_simulated_stall_exposes_pair_error_and_holds_stalled_motor(simulator):
    """A faulted motor stops while its paired motor moves toward the target."""
    simulator.set_stalled_motors(['left_1'])
    simulator.command({'left_claw': 1.0, 'right_claw': 0.5})
    simulator.tick(1.0)
    positions, disagreement, counts = simulator.feedback()
    assert counts['left_1'] == 0
    assert counts['left_2'] < 0
    assert counts['right_1'] > 0 and counts['right_2'] < 0
    assert disagreement['left_claw'] > 0.05
    assert disagreement['right_claw'] == 0.0
    simulator.hold()
    held = simulator.feedback()[2]
    simulator.tick(10.0)
    assert simulator.feedback()[2] == held
    simulator.set_stalled_motors([])
    assert not simulator.stalled_motors


def test_unknown_fault_motor_is_rejected(simulator):
    """Typos cannot silently disable fault injection."""
    with pytest.raises(ValueError, match='unknown simulated motor'):
        simulator.set_stalled_motors(['left_9'])


def test_invalid_command_does_not_change_existing_targets(simulator):
    """Out-of-range targets are rejected before changing either pair."""
    old = dict(simulator.targets)
    with pytest.raises(ValueError):
        simulator.command({'left_claw': 0.5, 'right_claw': 1.1})
    assert simulator.targets == old


def test_pair_disagreement_is_visible_in_feedback(simulator):
    """A motor that falls behind produces a nonzero claw-pair error."""
    simulator.counts['left_1'] = 160
    _, disagreement, _ = simulator.feedback()
    assert disagreement['left_claw'] == pytest.approx(0.5)


def test_stop_holds_both_claws_until_fresh_enable(simulator):
    """Disabling teleop stops an in-flight move and drops stale commands."""
    safety = ToolSafety(simulator, 0.5, 0.0)
    target = {'left_claw': 1.0, 'right_claw': 1.0}
    assert not safety.command(target, 0.0)
    safety.update_enable(True, 0.0)
    assert safety.command(target, 0.0)
    safety.tick(0.1)
    before = simulator.feedback()[2]
    assert any(before.values())
    assert safety.update_enable(False, 0.11)
    assert not safety.command(target, 0.12)
    safety.tick(0.2)
    assert simulator.feedback()[2] == before
    safety.update_enable(True, 0.21)
    assert safety.command(target, 0.22)
    safety.tick(0.3)
    assert simulator.feedback()[2] != before


@pytest.mark.parametrize('heartbeat', ['enable', 'command'])
def test_stale_heartbeat_latches_stop_until_release(simulator, heartbeat):
    """A missing enable or command stream cannot keep the claws moving."""
    safety = ToolSafety(simulator, 0.5, 0.0)
    target = {'left_claw': 1.0, 'right_claw': 1.0}
    safety.update_enable(True, 0.0)
    safety.command(target, 0.0)
    safety.tick(0.1)
    if heartbeat == 'enable':
        safety.command(target, 0.4)
    else:
        safety.update_enable(True, 0.4)
    assert safety.tick(0.51)
    held = simulator.feedback()[2]
    assert not safety.command(target, 0.52)
    safety.update_enable(True, 0.53)
    safety.tick(0.54)
    assert simulator.feedback()[2] == held
    safety.update_enable(False, 0.55)
    safety.update_enable(True, 0.56)
    assert safety.command(target, 0.56)


def test_host_pause_latches_stop(simulator):
    """A long scheduling pause holds motors rather than catching up."""
    safety = ToolSafety(simulator, 0.5, 0.0)
    target = {'left_claw': 1.0, 'right_claw': 1.0}
    safety.update_enable(True, 0.0)
    safety.command(target, 0.0)
    assert safety.tick(0.6)
    assert safety.stop_reason == 'simulation host paused'
    assert simulator.feedback()[2] == simulator.targets
