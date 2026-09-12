# Copyright 2026 Waybionic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Protocol codec for encoding and decoding provisional Waybionic CAN-FD frames."""

import math
import struct

# Provisional CAN IDs
STATE_BASE_ID = 0x100
CMD_BASE_ID = 0x200


def encode_target_command(position, velocity):
    """
    Encode a target position and velocity into a CAN command frame.

    :param position: Target position.
    :param velocity: Target velocity.
    :return: 8-byte packed payload.
    """
    if not (math.isfinite(position) and math.isfinite(velocity)):
        raise ValueError('Command contains NaN or Inf values')
    return struct.pack('<ff', position, velocity)


def decode_target_command(data):
    """
    Decode a CAN command frame into target position and velocity.

    :param data: Raw byte payload from the CAN frame.
    :return: Tuple containing (position, velocity).
    :raises ValueError: If the payload is less than 8 bytes.
    """
    if len(data) >= 8:
        pos, vel = struct.unpack('<ff', data[:8])
        if not (math.isfinite(pos) and math.isfinite(vel)):
            raise ValueError('Command contains NaN or Inf values')
        return pos, vel
    raise ValueError('Command payload too short')


def encode_joint_state(position, velocity, health, fault):
    """
    Encode actual joint state and health into a CAN-FD state frame.

    :param position: Actual joint position.
    :param velocity: Actual joint velocity.
    :param health: Health status byte (e.g., 1 for OK).
    :param fault: Hardware fault code byte.
    :return: 10-byte packed payload.
    """
    if not (math.isfinite(position) and math.isfinite(velocity)):
        raise ValueError('State contains NaN or Inf values')
    return struct.pack('<ffBB', position, velocity, health, fault)


def decode_joint_state(data):
    """
    Decode a CAN-FD state frame into joint state and health values.

    :param data: Raw byte payload from the CAN frame.
    :return: Tuple containing (position, velocity, health, fault).
    :raises ValueError: If the payload is less than 10 bytes.
    """
    if len(data) >= 10:
        pos, vel, health, fault = struct.unpack('<ffBB', data[:10])
        if not (math.isfinite(pos) and math.isfinite(vel)):
            raise ValueError('State contains NaN or Inf values')
        return pos, vel, health, fault
    raise ValueError('State payload too short')
