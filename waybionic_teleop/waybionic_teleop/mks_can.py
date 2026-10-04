"""
Placeholder CAN frames for MKS SERVO42D/57D drives (CAN user manual V1.0.9 subset).

Electrical has not confirmed the joint drives, so the protocol stays in this module and can
be swapped without touching the teleop or ROS code. Each frame is a standard 11-bit frame
whose ID is the motor ID; the data is a function code, big-endian arguments and a checksum.
"""

COUNTS_PER_REV = 0x4000
MAX_SPEED_RPM = 3000
MAX_AXIS = 0x7FFFFF

READ_ENCODER = 0x31
SET_MODE = 0x82
SET_RESPONSE = 0x8C
SET_HEARTBEAT = 0x98
ENABLE = 0xF3
ABSOLUTE_AXIS = 0xF5

MODE_SR_VFOC = 0x05
RUN_STATUS = {0: 'run failed', 1: 'running', 2: 'run complete', 3: 'stopped at end limit'}


def checksum(can_id, body):
    """Return the manual's CHECKSUM 8bit: the low byte of the ID plus every data byte."""
    return (can_id + sum(body)) & 0xFF


def frame(can_id, code, arguments=b''):
    """Return the data field of one frame: code, arguments, checksum."""
    body = bytes([code]) + bytes(arguments)
    if not 0 <= can_id <= 0x7FF or len(body) > 7:
        raise ValueError(f'cannot build a {code:02X}h frame for CAN ID {can_id}')
    return body + bytes([checksum(can_id, body)])


def parse(can_id, data):
    """Return (code, arguments) of a frame; raise ValueError if it is malformed."""
    data = bytes(data)
    if not 2 <= len(data) <= 8 or data[-1] != checksum(can_id, data[:-1]):
        raise ValueError(f'bad frame {hex_frame(can_id, data)}')
    return data[0], data[1:-1]


def hex_frame(can_id, data):
    """Format a frame like candump, for example 001#F502580200400092."""
    return f'{can_id:03X}#{bytes(data).hex().upper()}'


def absolute_axis(can_id, axis, speed_rpm, acc):
    """Move to an absolute encoder coordinate (F5h); resending updates a running move."""
    if not -MAX_AXIS <= axis <= MAX_AXIS:
        raise ValueError(f'axis {axis} is outside the int24 coordinate range')
    if not 0 <= speed_rpm <= MAX_SPEED_RPM or not 0 <= acc <= 255:
        raise ValueError('speed must be 0-3000 rpm and acc 0-255')
    return frame(can_id, ABSOLUTE_AXIS, speed_rpm.to_bytes(2, 'big') + bytes([acc])
                 + axis.to_bytes(3, 'big', signed=True))


def read_encoder(can_id):
    """Request the cumulative multi-turn encoder value (31h), 0x4000 counts per turn."""
    return frame(can_id, READ_ENCODER)


def encoder_value(arguments):
    """Decode the signed 48-bit value of a 31h reply."""
    if len(arguments) != 6:
        raise ValueError('an encoder reply carries 6 bytes')
    return int.from_bytes(arguments, 'big', signed=True)


def set_mode(can_id, mode=MODE_SR_VFOC):
    """Select the work mode (82h); SR_vFOC is bus control with FOC."""
    return frame(can_id, SET_MODE, [mode])


def set_response(can_id, respond=True, active=True):
    """Choose whether the drive replies (8Ch) and reports finished moves on its own."""
    return frame(can_id, SET_RESPONSE, [int(respond), int(active)])


def set_heartbeat(can_id, milliseconds):
    """Make the drive stop if the host sends nothing for this long (98h, 0 turns it off)."""
    return frame(can_id, SET_HEARTBEAT, milliseconds.to_bytes(4, 'big'))


def enable(can_id, on=True):
    """Lock (on) or release the motor shaft (F3h)."""
    return frame(can_id, ENABLE, [int(on)])
