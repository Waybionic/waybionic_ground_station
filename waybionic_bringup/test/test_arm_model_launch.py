"""Drive the ground-station arm with joint states and compare TF with SolidWorks."""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
import tf2_ros

# Saved pose of full-arm-smaller.SLDASM in joint space (radians).
CAD_POSE = {'joint_1': -0.155823, 'joint_2': -0.153305, 'joint_3': 0.787757,
            'joint_4': 0.088643, 'joint_5': 0.0}
# Points measured from SolidWorks mates, in base_link (metres).
ELBOW_AXIS_POINT = (-0.00619, 0.12295, 0.378565)
WRIST_CENTER = (0.147391, 0.010149, 0.608506)
ROLL_AXIS = (0.653691, -0.102693, 0.749761)
ZERO_POSE = dict.fromkeys(CAD_POSE, 0.0)


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('waybionic_bringup'), 'launch', 'ground_station.launch.py')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'launch_rviz': 'false',
                'use_joint_state_publisher_gui': 'false',
            }.items()),
        launch_testing.actions.ReadyToTest(),
    ])


def rotate(rotation, vector):
    """Rotate a vector by a geometry_msgs quaternion."""
    q = (rotation.x, rotation.y, rotation.z)
    t = [2.0 * value for value in cross(q, vector)]
    return [v + rotation.w * a + b for v, a, b in zip(vector, t, cross(q, t))]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class TestArmModel(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('arm_model_test')
        cls.buffer = tf2_ros.Buffer()
        cls.listener = tf2_ros.TransformListener(cls.buffer, cls.node)
        cls.publisher = cls.node.create_publisher(JointState, '/joint_states', 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def transform(self, positions, target, source='base_link'):
        """Publish joint positions and return TF computed from that exact message."""
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            message = JointState(name=list(positions), position=list(positions.values()))
            message.header.stamp = self.node.get_clock().now().to_msg()
            self.publisher.publish(message)
            settle = time.monotonic() + 0.2
            while time.monotonic() < settle:
                rclpy.spin_once(self.node, timeout_sec=0.02)
            stamp = Time.from_msg(message.header.stamp)
            if self.buffer.can_transform(source, target, stamp):
                return self.buffer.lookup_transform(source, target, stamp).transform
        self.fail(f'No transform from {source} to {target}')

    def assert_close(self, actual, expected, tolerance):
        for a, e in zip(actual, expected):
            self.assertAlmostEqual(a, e, delta=tolerance)

    def test_cad_pose_matches_solidworks_axes(self):
        elbow = self.transform(CAD_POSE, 'forearm_link').translation
        self.assert_close((elbow.x, elbow.y, elbow.z), ELBOW_AXIS_POINT, 1e-4)
        wrist = self.transform(CAD_POSE, 'wrist_roll_link')
        position = wrist.translation
        self.assert_close((position.x, position.y, position.z), WRIST_CENTER, 1e-4)
        self.assert_close(rotate(wrist.rotation, (0.0, 0.0, 1.0)), ROLL_AXIS, 1e-4)

    def test_zero_pose_is_upright(self):
        tool = self.transform(ZERO_POSE, 'tool_link')
        self.assert_close(rotate(tool.rotation, (0.0, 0.0, 1.0)), (0.0, 0.0, 1.0), 1e-6)
        self.assertGreater(tool.translation.z, 0.7)

    def test_wrist_roll_turns_side_gears_in_opposite_directions(self):
        roll = dict(ZERO_POSE, joint_5=0.5)
        for gear, sign in (('wrist_left_gear_link', -1.0), ('wrist_right_gear_link', 1.0)):
            rotation = self.transform(roll, gear, 'wrist_pitch_link').rotation
            angle = 2.0 * math.atan2(rotation.y, rotation.w)
            self.assertAlmostEqual(rotation.x, 0.0, delta=1e-6)
            self.assertAlmostEqual(rotation.z, 0.0, delta=1e-6)
            self.assertAlmostEqual(angle, sign * 0.5, delta=1e-6)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
