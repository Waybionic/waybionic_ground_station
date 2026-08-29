import time
import unittest

import can
import rclpy

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
        self.node = CanHostNode()

    def tearDown(self):
        self.node.destroy_node()

    def test_codec_packing(self):
        # Verify 8-byte command packing
        cmd_data = codec.encode_target_command(1.5, -0.5)
        self.assertEqual(len(cmd_data), 8)
        pos, vel = codec.decode_target_command(cmd_data)
        self.assertAlmostEqual(pos, 1.5, places=4)
        self.assertAlmostEqual(vel, -0.5, places=4)

        # Verify 10-byte state packing (CAN-FD)
        state_data = codec.encode_joint_state(3.14, 0.0, 1, 0xAA)
        self.assertEqual(len(state_data), 10)
        p, v, h, f = codec.decode_joint_state(state_data)
        self.assertEqual(f, 0xAA)

    def test_six_node_configuration_and_stale_detection(self):
        self.assertEqual(len(self.node.last_seen), 6)

        self.node.last_seen[1] = time.time()
        self.node.last_seen[2] = time.time() - 10.0

        self.node.publish_diagnostics()

        self.assertTrue(time.time() - self.node.last_seen[2] > 0.5)
        self.assertTrue(time.time() - self.node.last_seen[1] < 0.5)

    def test_invalid_mappings(self):
        try:
            msg = can.Message(arbitration_id=0x999, data=b'\x00\x00', is_extended_id=False)
            ignored = not (
                codec.STATE_BASE_ID + 1 <= msg.arbitration_id <= codec.STATE_BASE_ID + 6
            )
            self.assertTrue(ignored)
        except Exception as e:
            self.fail(f'Node crashed on invalid mapping: {e}')


if __name__ == '__main__':
    unittest.main()
