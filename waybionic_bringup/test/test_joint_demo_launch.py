"""Run the ground station in demo mode and wait for every joint to pass its TF check."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy

JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('waybionic_bringup'), 'launch', 'ground_station.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'launch_rviz': 'false',
                'demo_mode': 'true',
                'demo_speed': '360.0',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestJointDemo(unittest.TestCase):

    def test_every_joint_follows_its_sweep(self):
        rclpy.init()
        node = rclpy.create_node('joint_demo_test')
        results = {}

        def record(message):
            for status in message.status:
                results[status.name] = {item.key: item.value for item in status.values}

        node.create_subscription(DiagnosticArray, '/diagnostics', record, 10)
        try:
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if results.get('arm.joint_test', {}).get('result') in ('pass', 'fail'):
                    break
            self.assertEqual(results.get('arm.joint_test', {}).get('result'), 'pass', results)
            self.assertEqual(
                [results[f'arm.{joint}']['result'] for joint in JOINTS], ['pass'] * len(JOINTS))
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
