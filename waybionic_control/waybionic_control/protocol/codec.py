import struct


# Provisional CAN IDs
STATE_BASE_ID = 0x100
CMD_BASE_ID = 0x200


def encode_target_command(position, velocity):
    # Pack 2 floats (8 bytes total)
    return struct.pack('<ff', position, velocity)


def decode_target_command(data):
    if len(data) >= 8:
        return struct.unpack('<ff', data[:8])
    return 0.0, 0.0


def encode_joint_state(position, velocity, health, fault):
    # Pack 2 floats and 2 unsigned bytes (10 bytes total)
    return struct.pack('<ffBB', position, velocity, health, fault)


def decode_joint_state(data):
    if len(data) >= 10:
        return struct.unpack('<ffBB', data[:10])
    return 0.0, 0.0, 0, 0
