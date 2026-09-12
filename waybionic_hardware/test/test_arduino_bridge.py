"""Minimal tests for Arduino bridge module."""
import unittest


class TestArduinoBridge(unittest.TestCase):
    """Test cases for ArduinoBridge."""

    def test_import(self):
        """Test that the module can be imported."""
        try:
            from waybionic_hardware import arduino_bridge  # noqa: F401
            self.assertTrue(True)
        except ImportError:
            self.fail("Failed to import arduino_bridge")

    def test_joint_names_match_old_arm_model(self):
        """Joint states target the imported old-arm URDF joints."""
        from waybionic_hardware.arduino_bridge import JOINT_NAMES

        self.assertEqual(JOINT_NAMES, [
            'old_arm_base_yaw_joint',
            'old_arm_shoulder_pitch_joint',
            'old_arm_elbow_pitch_joint',
            'old_arm_wrist_roll_joint',
        ])


if __name__ == '__main__':
    unittest.main()
