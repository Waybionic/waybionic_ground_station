"""Bench set-up of the MKS drives: finding them, new CAN IDs and bit rates."""

from collections import deque

import pytest

from waybionic_teleop import carrier, mks_can, mks_setup
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


class ErrorReadBus:
    """Supply recorded error replies without a simulation of motor torque."""

    def __init__(self, replies):
        self.replies = deque(replies)
        self.sent = []
        self.closed = False

    def send(self, can_id, data):
        self.sent.append((can_id, data))
        return True

    def step(self, dt):
        pass

    def receive(self):
        return self.replies.popleft() if self.replies else None

    def shutdown(self):
        self.closed = True


def test_read_error_is_signed_and_sends_only_a_read(monkeypatch, capsys):
    # CAN ID 3 + 39h + FFFF3800 has checksum 72h; -51200 is -360 motor degrees.
    bus = ErrorReadBus([
        (0x7F0, bytes(8)),
        (3, bytes.fromhex('39FFFF380072')),
    ])
    monkeypatch.setattr(mks_setup, 'open_bus', lambda *args: bus)
    mks_setup.main(['--interface', 'virtual', '--channel', 'fake', 'read-error', '3'])
    assert bus.sent == [(3, bytes.fromhex('393C'))]
    assert bus.closed
    assert '-360.000000 motor deg (-51200 error counts)' in capsys.readouterr().out


@pytest.mark.parametrize('reply, reason', [
    (bytes.fromhex('39003C'), 'four bytes'),
    (bytes.fromhex('39000000003D'), 'bad frame'),
])
def test_read_error_reports_bad_replies_and_releases_the_bus(monkeypatch, reply, reason):
    bus = ErrorReadBus([(3, reply)])
    monkeypatch.setattr(mks_setup, 'open_bus', lambda *args: bus)
    with pytest.raises(SystemExit, match=reason):
        mks_setup.main(['--interface', 'virtual', '--channel', 'fake', 'read-error', '3'])
    assert bus.closed
    assert bus.sent == [(3, bytes.fromhex('393C'))]


def test_read_error_reports_an_unanswered_request():
    bus = ErrorReadBus([])
    with pytest.raises(SystemExit, match='did not report its position error'):
        mks_setup.read_error(bus, 3)
    assert bus.sent == [(3, bytes.fromhex('393C'))]


@pytest.mark.parametrize('can_id', ['0', '-1', '2032', '2048'])
def test_read_error_rejects_invalid_or_reserved_ids_before_opening_usb(monkeypatch, can_id):
    def unexpected_open(*args):
        pytest.fail('invalid input opened USB')
    monkeypatch.setattr(mks_setup, 'open_bus', unexpected_open)
    with pytest.raises(SystemExit):
        mks_setup.main(['--channel', 'fake', 'read-error', can_id])


def test_a_drive_may_use_the_carriers_status_id_off_the_carrier():
    # Only the carrier's own link reserves that ID; on SocketCAN it is an ordinary CAN ID.
    bus = SimulatedBus([SimulatedServo(1)], 500000)
    assert 'now answers on 2032' in mks_setup.set_id(bus, 1, carrier.STATUS_ID)
    assert mks_setup.scan(bus, [carrier.STATUS_ID]) == {carrier.STATUS_ID: 0}


@pytest.mark.parametrize('old, new', [(1, carrier.STATUS_ID), (carrier.STATUS_ID, 1)])
def test_the_carriers_status_id_is_refused_before_a_drive_is_changed(usb_carrier, old, new):
    port = usb_carrier()
    bus = SlcanBus('/dev/fake', 500000, open_port=lambda: port)
    with pytest.raises(SystemExit, match='reserved for the carrier'):
        mks_setup.set_id(bus, old, new)
    assert not any(line.startswith(b't') for line in b''.join(port.writes).split(b'\r'))


def test_read_error_accepts_the_carriers_status_id_off_the_carrier(monkeypatch, capsys):
    reply = mks_can.frame(carrier.STATUS_ID, mks_can.READ_POSITION_ERROR, bytes([0, 0, 0, 100]))
    bus = ErrorReadBus([(carrier.STATUS_ID, reply)])
    monkeypatch.setattr(mks_setup, 'open_bus', lambda *args: bus)
    mks_setup.main(['--interface', 'socketcan', '--channel', 'can0', 'read-error',
                    str(carrier.STATUS_ID)])
    assert 'position error +0.703125 motor deg (+100 error counts)' in capsys.readouterr().out
