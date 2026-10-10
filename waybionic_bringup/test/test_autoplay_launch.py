"""With autoplay and no controller, the demo drives the arm; touching the controller stops it."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
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
        joints, values, rows = {}, {}, {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda message: joints.update(zip(message.name, message.position)), 10)

        def on_diagnostics(message):
            for status in message.status:
                rows[status.name] = status
                if status.values:
                    values[status.name] = status.values[0].value
        node.create_subscription(DiagnosticArray, '/diagnostics', on_diagnostics, 10)
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

        def check_drives(gate):
            """Check the drives' command gate, and that no drive faulted or refused a command."""
            self.assertEqual(values.get('arm.command_gate'), gate,
                             rows.get('arm.command_gate'))
            bus = {item.key: item.value for item in rows['can.bus'].values}
            self.assertEqual(bus['rejected_commands'], '0')
            drives = [status for name, status in rows.items() if name.startswith('drive.')]
            self.assertEqual(len(drives), 6)
            for status in drives:
                self.assertEqual(status.level, DiagnosticStatus.OK,
                                 f'{status.name}: {status.message}')

        try:
            # The demo enables teleop, lifts the arm in the upper group, then goes Cartesian.
            self.assertTrue(spin_until(
                lambda: values.get('teleop.group') == 'cartesian'
                and values.get('teleop.state') == 'enabled', 45.0),
                f'the demo never reached the Cartesian group: {values}')
            self.assertGreater(joints['joint_2'], 0.5)
            check_drives('enabled')

            # Bumping a stick stops the demo and leaves teleop disabled, so the arm stays put.
            self.assertTrue(operate(2.0, done=disabled, left_x=1.0))
            pose = dict(joints)
            operate(2.0, left_x=1.0)
            self.assertTrue(disabled())
            check_drives('stopped')
            for joint in ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'):
                self.assertAlmostEqual(joints[joint], pose[joint], places=3)

            # Start then hands the arm to the operator.
            operate(0.2, 'start')
            self.assertTrue(operate(2.0, done=lambda: values.get('teleop.state') == 'enabled'
                                    and values.get('arm.command_gate') == 'enabled'))
            check_drives('enabled')
        finally:
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
