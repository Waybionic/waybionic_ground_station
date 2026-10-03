"""With autoplay and no controller, the demo drives the arm; touching the controller stops it."""

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
from sensor_msgs.msg import JointState, Joy

from waybionic_teleop import gamepad


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('waybionic_bringup'), 'launch', 'ground_station.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'launch_rviz': 'false',
                'teleop': 'true',
                'joy_source': 'none',
                'autoplay': 'true',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAutoplay(unittest.TestCase):

    def test_demo_plays_and_gives_way_to_the_controller(self):
        rclpy.init()
        node = rclpy.create_node('autoplay_test')
        joints, values = {}, {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda message: joints.update(zip(message.name, message.position)), 10)
        node.create_subscription(
            DiagnosticArray, '/diagnostics', lambda message: values.update(
                (status.name, status.values[0].value) for status in message.status
                if status.values), 10)
        operator = node.create_publisher(Joy, '/joy_operator', 10)

        def spin_until(done, seconds):
            deadline = time.monotonic() + seconds
            while not done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            return done()

        try:
            # The demo enables teleop, lifts the arm in the upper group, then goes Cartesian.
            self.assertTrue(spin_until(
                lambda: values.get('teleop.group') == 'cartesian'
                and values.get('teleop.state') == 'enabled', 45.0),
                f'the demo never reached the Cartesian group: {values}')
            self.assertGreater(joints['joint_2'], 0.5)

            # B on the controller reaches teleop, and the demo stays out of the way.
            buttons = [int(name == 'b') for name in gamepad.BUTTONS]
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                operator.publish(Joy(axes=[0.0] * len(gamepad.AXES), buttons=buttons))
                rclpy.spin_once(node, timeout_sec=0.02)
            self.assertTrue(spin_until(lambda: values.get('teleop.state') == 'disabled', 2.0))
            pose = dict(joints)
            spin_until(lambda: False, 3.0)
            self.assertEqual(values.get('teleop.state'), 'disabled')
            for joint in ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'):
                self.assertAlmostEqual(joints[joint], pose[joint], places=3)
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
