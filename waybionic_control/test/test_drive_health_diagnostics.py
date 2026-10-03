"""ROS diagnostics integration tests using the simulated drives and fake CAN."""

import math
import unittest
from unittest.mock import MagicMock, patch

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from waybionic_control.node.can_host import CanHostNode
from waybionic_control.node.mock_drives import MockDrivesNode


class TestDriveHealthDiagnostics(unittest.TestCase):
    """Verify per-axis health rows with simulated drive fault injection."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.host_bus = MagicMock()
        self.mock_bus = MagicMock()
        self.bus_patcher = patch(
            'can.interface.Bus', side_effect=[self.host_bus, self.mock_bus])
        self.bus_patcher.start()
        self.host = CanHostNode(parameter_overrides=[
            Parameter('transport', Parameter.Type.STRING, 'udp_multicast')])
        self.mock = MockDrivesNode(parameter_overrides=[
            Parameter('transport', Parameter.Type.STRING, 'udp_multicast'),
            Parameter('simulate_faults', Parameter.Type.BOOL, False),
        ])
        self.host.diag_pub.publish = MagicMock()

    def tearDown(self):
        self.mock.destroy_node()
        self.host.destroy_node()
        self.bus_patcher.stop()

    @staticmethod
    def _statuses(published):
        return {status.hardware_id: status for status in published.status
                if status.hardware_id.startswith('joint_')}

    @staticmethod
    def _values(status):
        return {entry.key: entry.value for entry in status.values}

    def _run_mock_cycle(self, command_frames, wall_time, monotonic_time):
        self.mock.bus.recv.side_effect = list(command_frames) + [None]
        self.mock.timer_callback()
        reply_frames = [call.args[0] for call in self.mock.bus.send.call_args_list]
        self.mock.bus.send.reset_mock()
        self.host.bus.recv.side_effect = reply_frames + [None]
        with patch('waybionic_control.node.can_host.time.time', return_value=wall_time):
            self.host.read_bus()
            self.host.enabled.update(self.mock.enabled)
            with patch(
                    'waybionic_control.node.can_host.time.monotonic',
                    return_value=monotonic_time):
                self.host.publish_diagnostics()
        return self._statuses(self.host.diag_pub.publish.call_args[0][0])

    def test_all_six_axes_publish_health_state_values_and_levels(self):
        for joint_id, fault in (
                (2, 'following_error'),
                (3, 'stalled'),
                (5, 'disabled')):
            self.mock.set_test_health_fault(joint_id, fault)

        commands = JointState()
        commands.name = [f'joint_{joint_id}' for joint_id in range(1, 7)]
        commands.position = [
            0.0,
            math.radians(6.0),
            math.radians(2.0),
            math.radians(1.0),
            math.radians(1.0),
            0.0,
        ]
        commands.velocity = [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
        self.host.command_callback(commands)
        command_frames = [call.args[0] for call in self.host.bus.send.call_args_list]

        self._run_mock_cycle(command_frames, wall_time=100.0, monotonic_time=10.0)
        self.mock.set_test_health_fault(4, 'not_responding')
        statuses = self._run_mock_cycle([], wall_time=102.1, monotonic_time=12.1)

        self.assertEqual(set(statuses), {f'joint_{joint_id}' for joint_id in range(1, 7)})
        expected = {
            'joint_1': ('OK', DiagnosticStatus.OK),
            'joint_2': ('FOLLOWING_ERROR', DiagnosticStatus.ERROR),
            'joint_3': ('STALLED', DiagnosticStatus.ERROR),
            'joint_4': ('NOT_RESPONDING', DiagnosticStatus.STALE),
            'joint_5': ('DISABLED', DiagnosticStatus.ERROR),
            'joint_6': ('OK', DiagnosticStatus.OK),
        }
        for joint_name, (state, level) in expected.items():
            with self.subTest(joint=joint_name):
                values = self._values(statuses[joint_name])
                self.assertEqual(values['state'], state)
                self.assertEqual(statuses[joint_name].level, level)
                joint_id = int(joint_name.split('_')[1])
                self.assertEqual(
                    statuses[joint_name].name,
                    f'can.bus: Joint {joint_id} Health')
                self.assertIn('target_encoder_error_deg', values)
                self.assertIn('reply_age_sec', values)

        healthy_values = self._values(statuses['joint_1'])
        self.assertAlmostEqual(
            float(healthy_values['target_encoder_error_deg']), 0.0, places=2)
        self.assertAlmostEqual(float(healthy_values['reply_age_sec']), 0.0, places=2)

        following_values = self._values(statuses['joint_2'])
        self.assertAlmostEqual(
            float(following_values['target_encoder_error_deg']), 6.0, places=2)
        self.assertAlmostEqual(float(following_values['reply_age_sec']), 0.0, places=2)

        stalled_values = self._values(statuses['joint_3'])
        self.assertAlmostEqual(
            float(stalled_values['target_encoder_error_deg']), 2.0, places=2)
        self.assertAlmostEqual(float(stalled_values['reply_age_sec']), 0.0, places=2)

        stale_values = self._values(statuses['joint_4'])
        self.assertAlmostEqual(
            float(stale_values['target_encoder_error_deg']), 0.5, places=2)
        self.assertAlmostEqual(float(stale_values['reply_age_sec']), 2.1, places=2)

        disabled_values = self._values(statuses['joint_5'])
        self.assertAlmostEqual(
            float(disabled_values['target_encoder_error_deg']), 0.25, places=2)
        self.assertAlmostEqual(float(disabled_values['reply_age_sec']), 0.0, places=2)


if __name__ == '__main__':
    unittest.main()
