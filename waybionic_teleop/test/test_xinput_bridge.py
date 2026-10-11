"""The Windows bridge must send exactly what game_controller_node would publish."""

import select
import socket

import pytest

from waybionic_teleop import gamepad, xinput_bridge


def pressed(buttons):
    return [gamepad.BUTTONS[index] for index, value in enumerate(buttons) if value]


@pytest.fixture
def sockets():
    """Return the bridge socket, the ground station's socket and another program's socket."""
    opened = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(3)]
    for sock in opened:
        sock.bind(('127.0.0.1', 0))
    opened[0].setblocking(False)
    yield opened
    for sock in opened:
        sock.close()


def rumble_from(sock, bridge, intensity):
    sock.sendto(gamepad.pack_rumble(intensity), bridge.getsockname())
    assert select.select([bridge], [], [], 2.0)[0], 'the rumble request never arrived'


def test_sticks_are_positive_left_and_up():
    pad = xinput_bridge.XInputGamepad(thumb_lx=-32768, thumb_ly=32767, thumb_rx=16384,
                                      thumb_ry=-32768)
    axes, _ = xinput_bridge.to_joy(pad)
    assert axes[:4] == pytest.approx([1.0, 1.0, -0.5, -1.0], abs=1e-4)


def test_triggers_rest_at_zero_and_reach_minus_one():
    axes, _ = xinput_bridge.to_joy(xinput_bridge.XInputGamepad(left_trigger=255))
    assert axes[4:] == [-1.0, 0.0]


def test_buttons_use_game_controller_indices():
    pad = xinput_bridge.XInputGamepad(buttons=0x1000 | 0x8000 | 0x0010 | 0x0200 | 0x0001)
    assert pressed(xinput_bridge.to_joy(pad)[1]) == ['a', 'y', 'start', 'right_bumper',
                                                     'dpad_up']


def test_bridge_output_fits_the_packet():
    pad = xinput_bridge.XInputGamepad(thumb_lx=-32768, right_trigger=128, buttons=0xFFFF)
    axes, buttons = xinput_bridge.to_joy(pad)
    _, _, decoded_axes, decoded_buttons = gamepad.unpack(gamepad.pack(1, axes, buttons))
    assert decoded_axes == pytest.approx(axes)
    assert decoded_buttons == buttons


@pytest.mark.parametrize('intensity, speed', [(0.0, 0), (0.5, 32768), (1.0, 65535), (2.0, 65535)])
def test_rumble_drives_both_motors(intensity, speed):
    motors = xinput_bridge.vibration(intensity)
    assert (motors.left_motor, motors.right_motor) == (speed, speed)


def test_only_the_ground_station_address_can_rumble_the_controller(sockets):
    bridge, station, stranger = sockets
    rumble_from(stranger, bridge, 1.0)
    assert xinput_bridge.read_rumble(bridge, station.getsockname()) is None
    rumble_from(stranger, bridge, 1.0)
    rumble_from(station, bridge, 0.5)
    assert xinput_bridge.read_rumble(bridge, station.getsockname()) == pytest.approx(0.5)


def test_a_flood_of_rumble_requests_is_read_a_few_per_update(sockets):
    bridge, station, _ = sockets
    count = 2 * xinput_bridge.MAX_READS
    for index in range(1, count + 1):
        rumble_from(station, bridge, index / 100)
    for update in range(1, 3):
        intensity = xinput_bridge.read_rumble(bridge, station.getsockname())
        assert intensity == pytest.approx(update * xinput_bridge.MAX_READS / 100)
    assert xinput_bridge.read_rumble(bridge, station.getsockname()) is None


def test_only_the_ground_station_gets_its_ping_back(sockets):
    bridge, station, stranger = sockets
    ping = gamepad.pack_ping(5, 12.5)
    for sock in (stranger, station):
        sock.sendto(ping, bridge.getsockname())
        assert select.select([bridge], [], [], 2.0)[0], 'the ping never arrived'
        assert xinput_bridge.read_rumble(bridge, station.getsockname()) is None
    assert select.select([station], [], [], 2.0)[0], 'the station got no pong'
    assert gamepad.unpack_pong(station.recv(64)) == (5, 12.5)
    assert not select.select([stranger], [], [], 0.2)[0], 'a stranger got a pong'


class WindowsSocket:
    """Raise the errors Windows reports for an oversize datagram and an ICMP reply."""

    def __init__(self, *replies):
        self.replies = list(replies)

    def recvfrom(self, size):
        reply = self.replies.pop(0) if self.replies else BlockingIOError()
        if isinstance(reply, Exception):
            raise reply
        return reply


def test_a_receive_error_does_not_hide_the_replies_behind_it():
    station = ('127.0.0.1', 47300)
    sock = WindowsSocket(OSError(10040, 'datagram larger than the buffer'),
                         ConnectionResetError(10054, 'port unreachable'),
                         (gamepad.pack_rumble(0.5), station))
    assert xinput_bridge.read_rumble(sock, station) == pytest.approx(0.5)
