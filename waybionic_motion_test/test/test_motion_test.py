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


if __name__ == '__main__':
    unittest.main()
