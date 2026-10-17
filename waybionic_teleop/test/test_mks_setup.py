"""Bench set-up of the MKS drives: finding them, new CAN IDs and bit rates."""

import pytest

from waybionic_teleop import mks_can, mks_setup
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo
from waybionic_teleop.slcan_bus import SlcanBus

STOP = mks_can.emergency_stop(mks_can.BROADCAST_ID)
STOP_LINE = b't000%d%s' % (len(STOP), STOP.hex().upper().encode())


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


def test_stop_reaches_every_drive(bus, monkeypatch, capsys):
    for can_id, servo in bus.drives.items():
        for data in (mks_can.set_mode(can_id), mks_can.enable(can_id),
                     mks_can.absolute_axis(can_id, 0x4000, 300, 0)):
            servo.receive(data)
        servo.step(0.05)
    monkeypatch.setattr(mks_setup, 'open_bus', lambda *args: bus)
    mks_setup.main(['--interface', 'virtual', '--channel', 'sim', 'stop'])
    assert all(servo.target is None and servo.rpm == 0.0 for servo in bus.drives.values())
    assert 'The carrier took the emergency stop' in capsys.readouterr().out


@pytest.mark.parametrize('refusals, sent', [(1, True), (5, False)])
def test_stop_goes_out_again_until_the_carrier_takes_it(monkeypatch, capsys, usb_carrier,
                                                        refusals, sent):
    port = usb_carrier(refuse=[STOP_LINE] * refusals)
    monkeypatch.setattr(mks_setup, 'open_bus',
                        lambda *args: SlcanBus('/dev/fake', 500000, open_port=lambda: port))
    monkeypatch.setattr(mks_setup, 'wait_for_carrier', lambda bus: True)
    if sent:
        mks_setup.main(['--channel', '/dev/fake', 'stop'])
        assert 'The carrier took the emergency stop' in capsys.readouterr().out
    else:
        with pytest.raises(SystemExit, match='did not go out'):
            mks_setup.main(['--channel', '/dev/fake', 'stop'])
    assert b''.join(port.writes).count(STOP_LINE) == min(refusals + 1, 5)


def test_stop_fails_when_the_carrier_is_unplugged(monkeypatch, usb_carrier):
    port = usb_carrier()
    monkeypatch.setattr(mks_setup, 'open_bus',
                        lambda *args: SlcanBus('/dev/fake', 500000, open_port=lambda: port))
    # The carrier booted, then its USB link failed before the stop.
    monkeypatch.setattr(mks_setup, 'wait_for_carrier',
                        lambda bus: setattr(port, 'broken', True) or True)
    with pytest.raises(SystemExit, match='did not go out'):
        mks_setup.main(['--channel', '/dev/fake', 'stop'])


def test_a_scan_over_a_lost_carrier_says_so_instead_of_no_drive(usb_carrier):
    port = usb_carrier()
    bus = SlcanBus('/dev/fake', 500000, open_port=lambda: port)
    port.broken = True
    with pytest.raises(SystemExit, match='Lost the carrier'):
        mks_setup.scan(bus, [1])


def test_a_refused_request_goes_out_again(bus):
    send, first, refusals = bus.send, [True], []

    def busy_once(can_id, data):
        # The carrier refuses the first frame, as while its CAN controller is busy.
        if first:
            first.clear()
            refusals.append((can_id, bytes(data)))
            return True
        return send(can_id, data)

    def take_refused():
        taken = refusals[:]
        refusals.clear()
        return taken
    bus.send, bus.take_refused = busy_once, take_refused
    bus.drives[3].axis = 0x4000
    assert mks_setup.scan(bus, [3]) == {3: 0x4000}
