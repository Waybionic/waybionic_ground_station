"""Minimal tests for Arduino bridge module."""
import unittest
from types import SimpleNamespace


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

    def make_bridge(self):
        from waybionic_hardware.arduino_bridge import ArduinoBridge

        bridge = ArduinoBridge.__new__(ArduinoBridge)
        bridge.serial_port = SimpleNamespace(writes=[])
        bridge.serial_port.write = lambda data: bridge.serial_port.writes.append(data)
        bridge.serial_lock = __import__('threading').Lock()
        bridge.dry_run = False
        bridge.connected = True
        bridge.ready = True
        bridge.faulted = False
        bridge.motion_active = True
        bridge.motion_duration = 0.0
        bridge.sequence_active = True
        bridge.last_error = ''
        bridge.last_status = 'moving'
        bridge.get_logger = lambda: SimpleNamespace(debug=lambda message: None)
        return bridge

    def test_error_latches_fault_and_stops_motion(self):
        bridge = self.make_bridge()

        bridge.process_serial_line('ERROR,OVER_CURRENT')

        self.assertFalse(bridge.connected)
        self.assertFalse(bridge.ready)
        self.assertTrue(bridge.faulted)
        self.assertFalse(bridge.motion_active)
        self.assertFalse(bridge.sequence_active)
        self.assertEqual(bridge.serial_port.writes, [b'HOLD\n'])

    def test_malformed_response_latches_fault_and_stops_motion(self):
        bridge = self.make_bridge()

        bridge.process_serial_line('garbled response')

        self.assertEqual(bridge.last_status, 'malformed-response')
        self.assertTrue(bridge.faulted)
        self.assertFalse(bridge.motion_active)
        self.assertEqual(bridge.serial_port.writes, [b'HOLD\n'])

    def test_read_failure_freezes_motion_and_requests_hold(self):
        bridge = self.make_bridge()

        bridge._latch_fault('serial-read-failed', 'read failed')

        self.assertFalse(bridge.connected)
        self.assertFalse(bridge.motion_active)
        self.assertFalse(bridge.sequence_active)
        self.assertEqual(bridge.serial_port.writes, [b'HOLD\n'])

    def test_response_watchdog_latches_fault(self):
        from waybionic_hardware.arduino_bridge import RESPONSE_TIMEOUT_SECONDS

        bridge = self.make_bridge()
        bridge.last_response_monotonic = __import__('time').monotonic() - (
            RESPONSE_TIMEOUT_SECONDS + 1.0)
        bridge.current = [0.0, 0.0, 0.0, 0.0]
        bridge.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: None))
        bridge.joint_publisher = SimpleNamespace(publish=lambda message: None)

        bridge.publish_estimated_state()

        self.assertTrue(bridge.faulted)
        self.assertEqual(bridge.last_status, 'serial-read-failed')
        self.assertEqual(bridge.serial_port.writes, [b'HOLD\n'])

    def test_fault_is_reported_as_diagnostic_error(self):
        from diagnostic_msgs.msg import DiagnosticStatus

        bridge = self.make_bridge()
        bridge.faulted = True
        bridge.last_status = 'arduino-error'
        bridge.diagnostics_publisher = SimpleNamespace(publish=lambda message: setattr(bridge, 'diagnostic', message))
        bridge.status_publisher = SimpleNamespace(publish=lambda message: setattr(bridge, 'motion_status', message))
        bridge.count_subscribers = lambda topic: 1
        bridge.port = '/dev/fake'
        bridge.baud = 115200
        bridge.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: None))

        bridge.publish_diagnostics()

        self.assertEqual(bridge.diagnostic.status[0].level, DiagnosticStatus.ERROR)
        self.assertEqual(bridge.diagnostic.status[1].message, 'robot_state_publisher connected')
        self.assertEqual(bridge.diagnostic.status[2].level, DiagnosticStatus.WARN)

    def test_shutdown_requests_hold_before_closing_port(self):
        import rclpy
        from unittest.mock import patch
        from waybionic_hardware.arduino_bridge import ArduinoBridge

        class FakePort:
            def __init__(self):
                self.writes = []
                self.closed = False

            def write(self, data):
                self.writes.append(data)

            def close(self):
                self.closed = True

        rclpy.init()
        try:
            with patch.object(ArduinoBridge, 'connect_to_arduino'):
                bridge = ArduinoBridge()
            bridge.serial_port = FakePort()
            bridge.connected = True
            bridge.motion_active = True
            bridge.serial_reader = None

            bridge.destroy_node()

            self.assertEqual(bridge.serial_port.writes, [b'HOLD\n'])
            self.assertTrue(bridge.serial_port.closed)
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
