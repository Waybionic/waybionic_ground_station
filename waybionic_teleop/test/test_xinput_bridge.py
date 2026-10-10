"""The Windows bridge must send exactly what game_controller_node would publish."""

import pytest

from waybionic_teleop import gamepad, xinput_bridge


def pressed(buttons):
    return [gamepad.BUTTONS[index] for index, value in enumerate(buttons) if value]


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
