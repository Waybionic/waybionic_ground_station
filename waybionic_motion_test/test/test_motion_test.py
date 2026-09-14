"""Minimal tests for motion test module."""
import unittest
from types import SimpleNamespace


class TestMotionTest(unittest.TestCase):
    """Test cases for MotionTest."""

    def test_import(self):
        """Test that the module can be imported."""
        try:
            from waybionic_motion_test import motion_test  # noqa: F401
            self.assertTrue(True)
        except ImportError:
            self.fail("Failed to import motion_test")

    def test_joint_names_match_old_arm_model(self):
        """Joint states target the imported old-arm URDF joints."""
        from waybionic_motion_test.motion_test import JOINT_NAMES

        self.assertEqual(JOINT_NAMES, [
            'old_arm_base_yaw_joint',
            'old_arm_shoulder_pitch_joint',
            'old_arm_elbow_pitch_joint',
            'old_arm_wrist_roll_joint',
        ])

    def test_home_does_not_advance_into_sequence(self):
        from std_msgs.msg import String
        from waybionic_motion_test.motion_test import MotionTestNode

        node = MotionTestNode.__new__(MotionTestNode)
        node.home = [0.0, 0.0, 0.0, 0.0]
        node.current = list(node.home)
        node.start = list(node.home)
        node.target = list(node.home)
        node.sequence = [[1.0, 1.0, 1.0, 1.0], [2.0, 2.0, 2.0, 2.0]]
        node.sequence_index = 0
        node.sequence_playback = False
        node.segment_duration = 1.0
        node.segment_elapsed = 0.0
        node.segment_active = False
        node.publish_joint_state = lambda: None
        node.status_publisher = SimpleNamespace(publish=lambda message: None)

        node.handle_command(String(data='HOME'))
        node.segment_elapsed = node.segment_duration
        node.update_trajectory()

        self.assertFalse(node.segment_active)
        self.assertEqual(node.current, node.home)
        self.assertEqual(node.sequence_index, 0)


if __name__ == '__main__':
    unittest.main()
