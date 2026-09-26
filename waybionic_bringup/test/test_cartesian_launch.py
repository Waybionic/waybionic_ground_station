"""Cut sideways in the Cartesian group and check the tool tip follows a straight line in TF."""

import math
import os
import socket
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
from rclpy.time import Time
import tf2_ros

from waybionic_teleop import gamepad

PORT = 47392


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
                'joy_source': 'udp',
                'joy_udp_port': str(PORT),
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestCartesian(unittest.TestCase):

    def test_the_tool_tip_cuts_a_straight_line(self):
        rclpy.init()
        node = rclpy.create_node('cartesian_test')
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer, node)
        diagnostics = {}
        node.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda message: diagnostics.update(
                (status.name, status) for status in message.status), 10)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sequence, samples = 0, []

        def tip():
            transform = buffer.lookup_transform('base_link', 'tool_link', Time()).transform
            q, t = transform.rotation, transform.translation
            # z of the tool axis: cos of the tool's angle from straight up.
            return (t.x, t.y, t.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

        def hold(seconds, *pressed, record=False, **axes):
            nonlocal sequence
            packet_axes = [axes.get(name, 0.0) for name in gamepad.AXES]
            buttons = [int(name in pressed) for name in gamepad.BUTTONS]
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                sequence += 1
                sender.sendto(gamepad.pack(sequence, packet_axes, buttons), ('127.0.0.1', PORT))
                rclpy.spin_once(node, timeout_sec=0.02)
                if record:
                    samples.append(tip())

        try:
            deadline = time.monotonic() + 30.0
            # The group row appears once teleop has loaded the robot description.
            while (('teleop.group' not in diagnostics
                    or not buffer.can_transform('base_link', 'tool_link', Time()))
                   and time.monotonic() < deadline):
                rclpy.spin_once(node, timeout_sec=0.1)
            hold(0.2, 'start')
            hold(0.3, 'y')
            # Bend the shoulder, elbow and wrist away from the upright pose.
            hold(1.0, left_x=1.0, left_y=1.0, right_y=1.0)
            hold(0.5)
            hold(0.3, 'y')
            # Diagnostics are reported every 0.5 s.
            hold(0.6)
            self.assertEqual(diagnostics['teleop.state'].values[0].value, 'enabled')
            self.assertEqual(diagnostics['teleop.group'].values[0].value, 'cartesian')
            start, tilt = tip()
            hold(1.5, 'left_bumper', record=True, left_x=1.0)
            hold(0.5, record=True)
            travel = samples[-1][0][1] - start[1]
            worst = max(math.hypot(x - start[0], z - start[2]) for (x, y, z), _ in samples)
            self.assertGreater(travel, 0.025, 'the tool tip did not move sideways')
            self.assertLess(worst, 0.0005, f'the tip left the line by {worst * 1000:.2f} mm')
            self.assertLess(max(abs(value - tilt) for _, value in samples), 0.005)
        finally:
            sender.close()
            listener.unregister()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
