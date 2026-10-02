"""DualSense input must match the teleop layout and fail safe on reconnect."""

from types import SimpleNamespace

import pytest

from waybionic_teleop import gamepad, sdl_bridge


class FakeController:
    """An SDL controller whose input can change during a test."""

    def __init__(self):
        """Start with neutral controls and a connected device."""
        self.axes = {}
        self.buttons = {}
        self.connected = True

    def get_axis(self, index):
        """Return a simulated SDL axis."""
        return self.axes.get(index, 0)

    def get_button(self, index):
        """Return a simulated SDL button."""
        return self.buttons.get(index, False)

    def attached(self):
        """Report whether the simulated device is connected."""
        return self.connected

    def quit(self):  # noqa: A003
        """Match the SDL controller cleanup API."""
        pass


class FakeSDL:
    """Expose the controller constants used by the production bridge."""

    def __init__(self):
        """Assign test constants and start without devices."""
        self.devices = []
        for index, name in enumerate(sdl_bridge.AXIS_NAMES):
            setattr(self, 'CONTROLLER_AXIS_' + name, index)
        for index, name in enumerate(sdl_bridge.BUTTON_NAMES.values()):
            setattr(self, 'CONTROLLER_BUTTON_' + name, index)

    def get_count(self):
        """Return the current SDL device count."""
        return len(self.devices)

    def is_controller(self, index):
        """Mark every test device as an SDL controller."""
        return True

    def Controller(self, index):
        """Open one simulated SDL controller."""
        return self.devices[index]

    def name_forindex(self, index):
        """Name the test controller like a DualSense."""
        return 'DualSense Wireless Controller'


def pressed(buttons):
    """List normalized names of pressed buttons."""
    return [name for name, value in zip(gamepad.BUTTONS, buttons) if value]


def test_dualsense_controls_match_existing_teleop_layout():
    """Keep DualSense axes and buttons compatible with teleop packets."""
    sdl = FakeSDL()
    pad = FakeController()
    pad.axes = {sdl.CONTROLLER_AXIS_LEFTX: -32768,
                sdl.CONTROLLER_AXIS_LEFTY: -32768,
                sdl.CONTROLLER_AXIS_RIGHTX: 16384,
                sdl.CONTROLLER_AXIS_RIGHTY: 32767,
                sdl.CONTROLLER_AXIS_TRIGGERLEFT: 32768}
    pad.buttons = {sdl.CONTROLLER_BUTTON_A: True,  # Cross
                   sdl.CONTROLLER_BUTTON_B: True,  # Circle
                   sdl.CONTROLLER_BUTTON_Y: True,  # Triangle
                   sdl.CONTROLLER_BUTTON_START: True,  # Options
                   sdl.CONTROLLER_BUTTON_DPAD_UP: True}
    axes, buttons = sdl_bridge.to_joy(pad, sdl)
    assert axes == pytest.approx([1.0, 1.0, -0.5, -32767 / 32768, -1.0, 0.0])
    assert pressed(buttons) == ['a', 'b', 'y', 'start', 'dpad_up']
    _, connected, unpacked_axes, unpacked_buttons = gamepad.unpack(
        gamepad.pack(1, axes, buttons))
    assert connected
    assert unpacked_axes == pytest.approx(axes)
    assert unpacked_buttons == buttons


def test_disconnect_sends_neutral_and_reconnect_requires_release():
    """Do not expose controls until a new device has been released."""
    sdl = FakeSDL()
    first = FakeController()
    sdl.devices = [first]
    bridge = sdl_bridge.SDLBridge(
        SimpleNamespace(event=SimpleNamespace(pump=lambda: None)), sdl)
    assert bridge.poll(0.0)[2] is False  # neutral reset
    assert bridge.poll(0.01)[2] is True
    first.buttons[sdl.CONTROLLER_BUTTON_START] = True
    assert 'start' in pressed(bridge.poll(0.02)[1])

    first.connected = False
    axes, buttons, connected, _ = bridge.poll(0.03)
    assert not connected and (axes, buttons) == sdl_bridge.NEUTRAL

    second = FakeController()
    second.buttons[sdl.CONTROLLER_BUTTON_START] = True
    sdl.devices = [second]
    # Held Options cannot enable teleop on reconnect.
    assert bridge.poll(1.04)[2] is False
    second.buttons.clear()
    assert bridge.poll(1.05)[2] is False  # wait for a fresh Options press
    assert bridge.poll(1.06)[2] is True
    second.buttons[sdl.CONTROLLER_BUTTON_START] = True
    assert 'start' in pressed(bridge.poll(1.07)[1])


def test_trigger_or_stick_held_blocks_connection():
    """A held control must block the reset after connection."""
    sdl = FakeSDL()
    pad = FakeController()
    pad.axes[sdl.CONTROLLER_AXIS_TRIGGERLEFT] = 20000
    sdl.devices = [pad]
    bridge = sdl_bridge.SDLBridge(
        SimpleNamespace(event=SimpleNamespace(pump=lambda: None)), sdl)
    assert bridge.poll(0.0)[2] is False
    pad.axes.clear()
    pad.axes[sdl.CONTROLLER_AXIS_LEFTX] = 20000
    assert bridge.poll(0.01)[2] is False
    pad.axes.clear()
    assert bridge.poll(0.02)[2] is False
    assert bridge.poll(0.03)[2] is True
