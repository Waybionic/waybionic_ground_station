"""Policy tests for the emergency-stop interlock; this module has no ROS imports."""

from waybionic_teleop.safety_interlock import EmergencyStopInterlock


def test_optional_status_allows_missing_and_stale_reports():
    interlock = EmergencyStopInterlock(timeout=0.5)

    assert not interlock.is_pressed(1.0)
    assert interlock.display_state(1.0) == 'UNKNOWN'
    assert interlock.block_reason(1.0) == ''

    interlock.update(False, 1.0)
    assert interlock.display_state(1.1) == 'RELEASED'
    assert not interlock.is_pressed(1.6)
    assert interlock.display_state(1.6) == 'STALE'


def test_required_missing_or_stale_status_is_fail_safe_pressed():
    interlock = EmergencyStopInterlock(timeout=0.5, required=True)

    assert interlock.is_pressed(1.0)
    assert interlock.display_state(1.0) == 'PRESSED-FAILSAFE'
    assert 'treated as pressed' in interlock.block_reason(1.0)

    interlock.update(False, 1.0)
    assert not interlock.is_pressed(1.5)
    assert interlock.display_state(1.5) == 'RELEASED'
    assert interlock.is_pressed(1.5001)
    assert interlock.display_state(1.5001) == 'PRESSED-FAILSAFE'


def test_reported_pressed_state_blocks_until_a_fresh_release():
    interlock = EmergencyStopInterlock(timeout=0.5, required=True)
    interlock.update(True, 2.0)

    assert interlock.is_pressed(2.1)
    assert interlock.display_state(2.1) == 'PRESSED'
    assert 'Emergency stop pressed' in interlock.block_reason(2.1)

    interlock.update(False, 2.2)
    assert not interlock.is_pressed(2.3)
    assert interlock.display_state(2.3) == 'RELEASED'
