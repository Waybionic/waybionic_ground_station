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


if __name__ == '__main__':
    unittest.main()
