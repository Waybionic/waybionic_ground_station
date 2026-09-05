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
    raise ValueError('Command payload too short')


def encode_joint_state(position, velocity, health, fault):
    # Pack 2 floats and 2 unsigned bytes (10 bytes total)
    return struct.pack('<ffBB', position, velocity, health, fault)


def decode_joint_state(data):
    if len(data) >= 10:
        return struct.unpack('<ffBB', data[:10])
    raise ValueError('State payload too short')
