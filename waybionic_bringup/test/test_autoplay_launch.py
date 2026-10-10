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

        def operate(seconds, *buttons, done=lambda: False, **axes):
            """Hold the controller like this until done() or the time is up."""
            message = Joy(axes=[axes.get(name, 0.0) for name in gamepad.AXES],
                          buttons=[int(name in buttons) for name in gamepad.BUTTONS])
            deadline = time.monotonic() + seconds
            while not done() and time.monotonic() < deadline:
                operator.publish(message)
                rclpy.spin_once(node, timeout_sec=0.02)
            return done()

        def disabled():
            return values.get('teleop.state') == 'disabled'

        try:
            # The demo enables teleop, lifts the arm in the upper group, then goes Cartesian.
            self.assertTrue(spin_until(
                lambda: values.get('teleop.group') == 'cartesian'
                and values.get('teleop.state') == 'enabled', 45.0),
                f'the demo never reached the Cartesian group: {values}')
            self.assertGreater(joints['joint_2'], 0.5)

            # Bumping a stick stops the demo and leaves teleop disabled, so the arm stays put.
            self.assertTrue(operate(2.0, done=disabled, left_x=1.0))
            pose = dict(joints)
            operate(2.0, left_x=1.0)
            self.assertTrue(disabled())
            for joint in ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'):
                self.assertAlmostEqual(joints[joint], pose[joint], places=3)

            # Start then hands the arm to the operator.
            operate(0.2, 'start')
            self.assertTrue(operate(2.0, done=lambda: values.get('teleop.state') == 'enabled'))
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
