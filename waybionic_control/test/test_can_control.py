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
import time
import unittest
from unittest.mock import MagicMock, patch

import can
from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from sensor_msgs.msg import JointState

from waybionic_control.node.can_host import CanHostNode
from waybionic_control.protocol import codec


class TestCanControlLogic(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.bus_patcher = patch('can.interface.Bus')
        self.mock_bus = self.bus_patcher.start()
        self.node = CanHostNode()

    def tearDown(self):
        self.node.destroy_node()
        self.bus_patcher.stop()

    def test_codec_packing(self):
        cmd_data = codec.encode_target_command(1.5, -0.5)
        self.assertEqual(len(cmd_data), 8)
        pos, vel = codec.decode_target_command(cmd_data)
        self.assertAlmostEqual(pos, 1.5, places=4)
        self.assertAlmostEqual(vel, -0.5, places=4)

        state_data = codec.encode_joint_state(3.14, 0.0, 1, 0xAA)
        self.assertEqual(len(state_data), 10)
        p, v, h, f = codec.decode_joint_state(state_data)
        self.assertEqual(f, 0xAA)

    def test_incomplete_command_rejected(self):
        js = JointState()
        js.name = ['joint_1']
        js.position = []

        self.node.command_callback(js)
        self.node.bus.send.assert_not_called()

        msg_with_pos = JointState()
        msg_with_pos.name = ['joint_1']
        msg_with_pos.position = [1.5]
        self.node.command_callback(msg_with_pos)
        self.node.bus.send.assert_called_once()

    def test_invalid_mappings(self):
        msg = can.Message(
            arbitration_id=0x999, data=b'\x00\x00', is_extended_id=False)
        self.node.bus.recv.side_effect = [msg, None]

        last_seen = dict(self.node.last_seen)

        try:
            self.node.read_bus()
        except Exception as e:
            self.fail(f'Node crashed on invalid mapping: {e}')
        self.assertEqual(self.node.last_seen, last_seen)

    def test_short_payloads(self):
        with self.assertRaises(ValueError):
            codec.decode_target_command(b'\x00' * 7)

        with self.assertRaises(ValueError):
            codec.decode_joint_state(b'\x00' * 9)

    def test_non_finite_values_rejected(self):
        with self.assertRaises(ValueError):
            codec.encode_target_command(float('nan'), 0.0)
        with self.assertRaises(ValueError):
            codec.encode_joint_state(0.0, float('inf'), 1, 0)

        bad_cmd = struct.pack('<ff', float('nan'), 0.0)
        with self.assertRaises(ValueError):
            codec.decode_target_command(bad_cmd)

        bad_state = struct.pack('<ffBB', 0.0, float('-inf'), 1, 0)
        with self.assertRaises(ValueError):
            codec.decode_joint_state(bad_state)

    def test_six_node_configuration_and_stale_detection(self):
        self.assertEqual(len(self.node.last_seen), 6)

        self.node.last_seen[1] = time.time()
        self.node.last_seen[2] = time.time() - 10.0

        self.node.diag_pub.publish = MagicMock()

        self.node.publish_diagnostics()

        self.node.diag_pub.publish.assert_called_once()
        published_msg = self.node.diag_pub.publish.call_args[0][0]
        statuses = {s.hardware_id: s for s in published_msg.status}

        self.assertEqual(statuses['joint_1'].level, DiagnosticStatus.OK)
        self.assertEqual(statuses['joint_1'].message, 'OK')

        self.assertEqual(statuses['joint_2'].level, DiagnosticStatus.ERROR)
        self.assertEqual(statuses['joint_2'].message, 'STALE (No heartbeat)')

    def test_health_zero_reports_error(self):
        self.node.last_seen[3] = time.time()
        self.node.healths[3] = 0
        self.node.faults[3] = 0

        self.node.diag_pub.publish = MagicMock()
        self.node.publish_diagnostics()

        published_msg = self.node.diag_pub.publish.call_args[0][0]
        statuses = {s.hardware_id: s for s in published_msg.status}

        self.assertEqual(statuses['joint_3'].level, DiagnosticStatus.ERROR)
        self.assertEqual(statuses['joint_3'].message, 'UNHEALTHY (health=0, no fault code)')

    def test_health_byte_preserved_from_bus(self):
        state = codec.encode_joint_state(1.0, 0.0, 0, 0)
        msg = can.Message(
            arbitration_id=codec.STATE_BASE_ID + 3,
            data=state,
            is_extended_id=False)
        self.node.bus.recv.side_effect = [msg, None]

        self.node.read_bus()

        self.assertEqual(self.node.healths[3], 0)


if __name__ == '__main__':
    unittest.main()
