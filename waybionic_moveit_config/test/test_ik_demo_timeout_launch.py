"""Regression test for bounded IK service discovery."""

import time
import unittest

from launch import LaunchDescription

from launch_ros.actions import Node

import launch_testing.actions

import pytest

import rclpy

from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    """Start only the demo node, deliberately without MoveIt's IK service."""
    demo = Node(
        package='waybionic_moveit_config',
        executable='ik_xyz_demo.py',
        parameters=[{'startup_timeout_seconds': 0.25}],
    )
    return LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestMissingIkService(unittest.TestCase):
    """Ensure missing IK does not leave Replay permanently busy."""

    @classmethod
    def setUpClass(cls):
        """Create a client for the demo's replay service."""
        rclpy.init()
        cls.node = rclpy.create_node('test_missing_ik_service')
        cls.client = cls.node.create_client(Trigger, '/ik_demo/replay')

    @classmethod
    def tearDownClass(cls):
        """Release the test node before launch_testing stops the demo."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_replay(self, timeout_sec=2.0):
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(
            self.node,
            future,
            timeout_sec=timeout_sec,
        )
        self.assertTrue(future.done(), 'replay service did not respond')
        return future.result()

    def test_timeout_releases_busy_state(self):
        """Allow another replay after bounded IK discovery times out."""
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=5.0),
            '/ik_demo/replay did not become available',
        )

        first = self._call_replay()
        self.assertTrue(first.success, first.message)

        busy = self._call_replay()
        self.assertFalse(
            busy.success,
            'replay should be busy during IK discovery',
        )

        time.sleep(0.5)
        retried = self._call_replay()
        self.assertTrue(retried.success, retried.message)
