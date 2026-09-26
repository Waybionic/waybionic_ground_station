"""Send controller packets over UDP and check the simulated arm follows through the CAN drives."""

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
from sensor_msgs.msg import JointState

from waybionic_teleop import gamepad

PORT = 47391


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


class TestTeleop(unittest.TestCase):

    def test_controller_moves_the_arm(self):
        rclpy.init()
        node = rclpy.create_node('teleop_test')
        joints, diagnostics = {}, {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda message: joints.update(zip(message.name, message.position)), 10)
        node.create_subscription(
            DiagnosticArray, '/diagnostics',
            lambda message: diagnostics.update(
                (status.name, status) for status in message.status), 10)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sequence = 0

        def hold(seconds, *pressed, **axes):
            nonlocal sequence
            packet_axes = [axes.get(name, 0.0) for name in gamepad.AXES]
            buttons = [int(name in pressed) for name in gamepad.BUTTONS]
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                sequence += 1
                sender.sendto(gamepad.pack(sequence, packet_axes, buttons), ('127.0.0.1', PORT))
                rclpy.spin_once(node, timeout_sec=0.02)

        try:
            deadline = time.monotonic() + 30.0
            while 'tool_grip' not in joints and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertIn('tool_grip', joints, 'the simulated drives never reported positions')
            hold(1.0, left_x=1.0)
            self.assertAlmostEqual(joints['joint_1'], 0.0, places=3, msg='moved while disabled')
            hold(0.2, 'start')
            hold(1.0, left_x=1.0)
            self.assertGreater(joints['joint_1'], 0.2)
            hold(0.3, 'y')
            yaw = joints['joint_1']
            hold(1.0, left_x=1.0, right_y=1.0)
            self.assertAlmostEqual(joints['joint_1'], yaw, places=2)
            self.assertGreater(joints['joint_4'], 0.2)
            hold(0.8, right_trigger=-1.0)
            self.assertGreater(joints['tool_grip'], 0.3)
            hold(0.2, 'b')
            pitch = joints['joint_4']
            hold(0.5, right_y=1.0)
            self.assertAlmostEqual(joints['joint_4'], pitch, places=2, msg='moved after B')
            self.assertEqual(diagnostics['teleop.state'].values[0].value, 'disabled')
            self.assertEqual(diagnostics['can.bus'].level, b'\x00')
            self.assertEqual(diagnostics['drive.wrist_left'].level, b'\x00')
        finally:
            sender.close()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
