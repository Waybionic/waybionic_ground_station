"""Minimal tests for motion test module."""
import unittest


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


if __name__ == '__main__':
    unittest.main()
