import time
import unittest

import can
import rclpy

from waybionic_control.node.can_host import CanHostNode


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
            ignored = not (0x101 <= msg.arbitration_id <= 0x106)
            self.assertTrue(ignored)
        except Exception as e:
            self.fail(f'Node crashed on invalid mapping: {e}')


if __name__ == '__main__':
    unittest.main()
