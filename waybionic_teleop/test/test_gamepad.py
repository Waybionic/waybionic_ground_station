"""The controller packet shared by the Windows bridge and the UDP receiver."""

import struct

import pytest

from waybionic_teleop import gamepad

NEUTRAL = [0.0] * len(gamepad.AXES)


def test_round_trip_keeps_axes_buttons_and_sequence():
    axes = [0.5, -0.25, 1.0, -1.0, 0.0, -0.75]
    buttons = [int(name in ('a', 'start', 'dpad_down')) for name in gamepad.BUTTONS]
    sequence, connected, decoded_axes, decoded_buttons = gamepad.unpack(
        gamepad.pack(2**32 + 7, axes, buttons))
    assert (sequence, connected) == (7, True)
    assert decoded_axes == pytest.approx(axes)
    assert decoded_buttons == buttons


def test_disconnected_flag_survives():
    assert gamepad.unpack(gamepad.pack(1, NEUTRAL, [], connected=False))[1] is False


@pytest.mark.parametrize('packet', [
    b'',
    gamepad.pack(1, NEUTRAL, [])[:-1],
    b'XXXX' + gamepad.pack(1, NEUTRAL, [])[4:],
    gamepad.pack(1, [float('nan')] + NEUTRAL[1:], []),
    gamepad.pack(1, [1.5] + NEUTRAL[1:], []),
])
def test_malformed_packets_are_rejected(packet):
    with pytest.raises(ValueError):
        gamepad.unpack(packet)


def test_unknown_button_bits_and_flags_are_rejected():
    packet = bytearray(gamepad.pack(1, NEUTRAL, []))
    struct.pack_into('<I', packet, len(packet) - 4, 1 << len(gamepad.BUTTONS))
    with pytest.raises(ValueError):
        gamepad.unpack(bytes(packet))
    packet = bytearray(gamepad.pack(1, NEUTRAL, []))
    packet[5] = 0x02
    with pytest.raises(ValueError):
        gamepad.unpack(bytes(packet))


@pytest.mark.parametrize('intensity, expected', [(0.6, 0.6), (0.0, 0.0), (3.0, 1.0), (-1, 0.0)])
def test_rumble_requests_round_trip_within_zero_to_one(intensity, expected):
    assert gamepad.unpack_rumble(gamepad.pack_rumble(intensity)) == pytest.approx(expected)


@pytest.mark.parametrize('packet', [
    b'', gamepad.pack(1, NEUTRAL, []), b'XXXX' + gamepad.pack_rumble(0.5)[4:],
    gamepad.pack_rumble(0.5)[:-4] + struct.pack('<f', float('nan')),
])
def test_anything_else_is_not_a_rumble_request(packet):
    with pytest.raises(ValueError):
        gamepad.unpack_rumble(packet)


def test_a_bridge_answers_a_ping_with_the_same_time():
    ping = gamepad.pack_ping(2**32 + 3, 1234.5)
    assert gamepad.unpack_pong(gamepad.pong(ping)) == (3, 1234.5)
    # A ping is neither a controller packet nor a rumble request, and pongs are not echoed.
    for unpack in (gamepad.unpack, gamepad.unpack_rumble, gamepad.unpack_pong):
        with pytest.raises(ValueError):
            unpack(ping)
    assert gamepad.pong(gamepad.pong(ping)) is None
    assert gamepad.pong(gamepad.pack(1, NEUTRAL, [])) is None


def test_link_quality_counts_loss_jitter_and_the_longest_gap():
    from waybionic_teleop.joy_udp_receiver import LinkQuality
    link = LinkQuality()
    for sequence, now in ((1, 0.000), (2, 0.004), (3, 0.008), (6, 0.020), (7, 0.024)):
        link.arrived(sequence, now)
    loss, jitter, gap = link.take()
    # Packets 4 and 5 never came: 2 of 7.
    assert loss == pytest.approx(2 / 7)
    assert gap == pytest.approx(0.012)
    # Intervals 4, 4, 12, 4 ms vary by 0, 8 and 8 ms.
    assert jitter == pytest.approx(0.016 / 3)
    assert link.take() == (0.0, 0.0, 0.0)
    # A bridge that restarted counts from 1 again; that is not loss.
    link.restart()
    link.arrived(1, 5.0)
    link.arrived(2, 5.004)
    assert link.take()[0] == 0.0
