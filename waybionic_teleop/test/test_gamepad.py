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
