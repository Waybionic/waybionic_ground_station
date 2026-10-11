"""Bench set-up of the MKS drives: finding them, new CAN IDs and bit rates."""

import pytest

from waybionic_teleop import mks_setup
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo


@pytest.fixture
def bus():
    # Two drives as they ship, one already set up as the elbow.
    return SimulatedBus([SimulatedServo(1), SimulatedServo(3)], 500000)


def test_scan_finds_the_drives_that_answer(bus):
    bus.drives[3].axis = 0x4000
    assert mks_setup.scan(bus, [1, 2, 3]) == {1: 0, 3: 0x4000}


def test_a_drive_takes_a_free_can_id(bus):
    assert 'now answers on 4' in mks_setup.set_id(bus, 1, 4)
    assert mks_setup.scan(bus, [1, 4]) == {4: 0}


def test_a_can_id_in_use_or_without_a_drive_is_refused(bus):
    with pytest.raises(SystemExit, match='taken'):
        mks_setup.set_id(bus, 1, 3)
    with pytest.raises(SystemExit, match='no drive'):
        mks_setup.set_id(bus, 2, 5)


def test_a_drive_switches_its_bit_rate(bus):
    assert '1000000 bit/s' in mks_setup.set_bitrate(bus, 3, 1000000)
    assert bus.drives[3].bitrate == 1000000
