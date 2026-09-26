"""Xbox controller layout on /joy and the UDP packet that carries it from a host bridge."""

import math
import struct

# Index order published by the joy package's game_controller_node (SDL game controller API).
# Sticks are +1 left/up; triggers rest at 0 and reach -1 when fully pressed.
AXES = ('left_x', 'left_y', 'right_x', 'right_y', 'left_trigger', 'right_trigger')
BUTTONS = ('a', 'b', 'x', 'y', 'back', 'guide', 'start', 'left_stick', 'right_stick',
           'left_bumper', 'right_bumper', 'dpad_up', 'dpad_down', 'dpad_left', 'dpad_right',
           'misc1', 'paddle1', 'paddle2', 'paddle3', 'paddle4', 'touchpad')
AXIS = {name: index for index, name in enumerate(AXES)}
BUTTON = {name: index for index, name in enumerate(BUTTONS)}

MAGIC = b'WBJY'
VERSION = 1
_PACKET = struct.Struct(f'<4sBBxxI{len(AXES)}fI')
PACKET_SIZE = _PACKET.size


def pack(sequence, axes, buttons, connected=True):
    """Encode one controller state; axes are in -1..1 and buttons are 0 or 1."""
    mask = sum(1 << index for index, pressed in enumerate(buttons) if pressed)
    return _PACKET.pack(MAGIC, VERSION, 1 if connected else 0, sequence & 0xFFFFFFFF,
                        *axes, mask)


def unpack(data):
    """Return (sequence, connected, axes, buttons); raise ValueError for a malformed packet."""
    if len(data) != PACKET_SIZE:
        raise ValueError(f'expected {PACKET_SIZE} bytes, got {len(data)}')
    magic, version, flags, sequence, *values = _PACKET.unpack(data)
    axes, mask = values[:-1], values[-1]
    if magic != MAGIC or version != VERSION or flags & ~1:
        raise ValueError('not a version 1 WayBionic controller packet')
    if not all(math.isfinite(value) and -1.0 <= value <= 1.0 for value in axes):
        raise ValueError('axis value outside -1..1')
    if mask >> len(BUTTONS):
        raise ValueError('unknown button bits')
    return sequence, bool(flags), axes, [(mask >> index) & 1 for index in range(len(BUTTONS))]
