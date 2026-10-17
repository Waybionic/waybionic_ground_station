"""Check Arduino-mode launch wiring without opening a serial port."""

import os
import time
import unittest
import uuid

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState


@pytest.mark.launch_test
def generate_test_description():
    """Launch Arduino dry-run mode with GUI and mock diagnostics requested."""
    bringup = get_package_share_directory('waybionic_bringup')
    topic = '/old_arm_arduino_test_' + uuid.uuid4().hex + '/joint_states'
    station = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'old_arm.launch.py')),
        launch_arguments={
            'hardware_mode': 'arduino',
            'arduino_dry_run': 'true',
            'launch_rviz': 'false',
            'use_joint_state_publisher_gui': 'true',
            'use_mock_diagnostics': 'true',
            'joint_states_topic': topic,
            'start_motion_test': 'true',
        }.items(),
    )
    return launch.LaunchDescription([
        station,
        launch_testing.actions.ReadyToTest(),
    ]), {'joint_states_topic': topic}


class TestOldArmArduinoLaunch(unittest.TestCase):
    """Verify Arduino mode owns state output and exposes live bridge diagnostics."""

    def test_gui_is_off_and_bridge_diagnostics_are_live(self, joint_states_topic):
        rclpy.init()
        node = rclpy.create_node('old_arm_arduino_launch_test_' + uuid.uuid4().hex)
        diagnostics = {}
        node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            lambda message: diagnostics.update(
                (status.name, status) for status in message.status),
            10,
        )
        node.create_subscription(JointState, joint_states_topic, lambda _: None, 10)
        try:
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if 'waybionic_arduino_bridge' in diagnostics:
                    break

            self.assertIn('waybionic_arduino_bridge', diagnostics)
            bridge_status = diagnostics['waybionic_arduino_bridge']
            values = {item.key: item.value for item in bridge_status.values}
            self.assertEqual(values['mode'], 'dry-run')
            self.assertIn('waybionic_servo_feedback', diagnostics)
            self.assertEqual(node.count_publishers(joint_states_topic), 1)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestOldArmArduinoShutdown(unittest.TestCase):
    """Require the Arduino dry-run launch to shut down cleanly."""

    def test_exit_codes(self, proc_info):
        """Reject unexpected child process failures."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
