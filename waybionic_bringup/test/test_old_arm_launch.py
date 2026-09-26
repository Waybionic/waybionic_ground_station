"""Check the handoff model and its live JointState-to-TF path without hardware."""

import math
import os
from pathlib import Path
import struct
import subprocess
import time
import unittest
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener


JOINT_NAMES = [
    'old_arm_base_yaw_joint',
    'old_arm_shoulder_pitch_joint',
    'old_arm_elbow_pitch_joint',
    'old_arm_wrist_roll_joint',
]


def expand_model(*mappings):
    """Expand the installed model, including any calibration overrides."""
    source = os.path.join(
        get_package_share_directory('waybionic_description'),
        'urdf', 'waybionic_old_arm.urdf.xacro')
    result = subprocess.run(
        ['xacro', source, *mappings], check=True, capture_output=True, text=True)
    return ET.fromstring(result.stdout)


def expected_wrist_position(positions):
    """Evaluate the handoff's independent forward-kinematics equations."""
    base, shoulder, elbow, _ = positions
    radial = 0.025 + 0.120 * math.cos(shoulder) + 0.050 * math.cos(shoulder + elbow)
    height = 0.070 + 0.120 * math.sin(shoulder) + 0.050 * math.sin(shoulder + elbow)
    return radial * math.cos(base), radial * math.sin(base), height


@pytest.mark.launch_test
def generate_test_description():
    """Launch only the old-arm state publisher on a unique input topic."""
    bringup = get_package_share_directory('waybionic_bringup')
    topic = '/old_arm_test_' + uuid.uuid4().hex + '/joint_states'
    station = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'old_arm.launch.py')),
        launch_arguments={
            'launch_rviz': 'false',
            'use_joint_state_publisher_gui': 'false',
            'joint_states_topic': topic,
            'start_motion_test': 'false',
        }.items(),
    )
    return launch.LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '72'),
        station, launch_testing.actions.ReadyToTest(),
    ]), {'joint_states_topic': topic}


class TestOldArmDescription(unittest.TestCase):
    """Protect handoff calibration and visualization-only model boundaries."""

    def test_joint_contract_and_limits(self):
        """Keep four named joints with handoff software ranges in radians."""
        robot = expand_model()
        joints = [joint for joint in robot.findall('joint') if joint.get('type') != 'fixed']
        self.assertEqual([joint.get('name') for joint in joints], JOINT_NAMES)
        expected_ranges = [(-90.0, 180.0), (12.5, 125.0), (-61.5, 118.5), (-27.5, 242.5)]
        for joint, expected in zip(joints, expected_ranges):
            self.assertEqual(joint.get('type'), 'revolute')
            limits = joint.find('limit')
            for key, degrees in zip(('lower', 'upper'), expected):
                self.assertAlmostEqual(float(limits.get(key)), math.radians(degrees))
            self.assertEqual(float(limits.get('effort')), 0.0)
            self.assertEqual(float(limits.get('velocity')), 0.0)
        self.assertEqual(joints[0].find('axis').get('xyz'), '0 0 1')
        self.assertEqual(joints[1].find('axis').get('xyz'), '0 -1 0')
        self.assertEqual(joints[2].find('axis').get('xyz'), '0 -1 0')
        self.assertEqual(joints[3].find('axis').get('xyz'), '1 0 0')
        self.assertFalse(robot.findall('.//inertial'))
        self.assertFalse(robot.findall('.//collision'))
        self.assertFalse(robot.findall('.//transmission'))
        self.assertFalse(robot.findall('.//ros2_control'))
        for link in robot.findall('link'):
            materials = {visual.find('material').get('name') for visual in link.findall('visual')}
            self.assertLessEqual(len(materials), 1, 'RViz requires separate color groups per link')

    def test_geometry_and_calibration_overrides(self):
        """Allow remeasured lengths and direction signs without changing meshes."""
        robot = expand_model('upper_arm_length:=0.130', 'base_direction:=-1')
        joints = {joint.get('name'): joint for joint in robot.findall('joint')}
        self.assertEqual(
            [float(value) for value in
             joints['old_arm_elbow_pitch_joint'].find('origin').get('xyz').split()],
            [0.130, 0.0, 0.0])
        limits = joints['old_arm_base_yaw_joint'].find('limit')
        self.assertAlmostEqual(float(limits.get('lower')), math.radians(-180.0))
        self.assertAlmostEqual(float(limits.get('upper')), math.radians(90.0))
        for box in robot.findall('.//box'):
            self.assertTrue(all(float(value) > 0 for value in box.get('size').split()))
        for cylinder in robot.findall('.//cylinder'):
            self.assertGreater(float(cylinder.get('radius')), 0)
            self.assertGreater(float(cylinder.get('length')), 0)
        upper_meshes = robot.findall("link[@name='old_arm_upper_arm']/visual/geometry/mesh")
        self.assertEqual(len(upper_meshes), 2)
        for mesh in upper_meshes:
            self.assertAlmostEqual(float(mesh.get('scale').split()[0]), 0.130 / 0.120)

    def test_installed_video_meshes(self):
        """Require the six finite, meter-scale STL resources in the installed package."""
        robot = expand_model()
        directory = Path(get_package_share_directory('waybionic_description'))
        filenames = set()
        for mesh in robot.findall('.//mesh'):
            filename = mesh.get('filename')
            prefix = 'package://waybionic_description/'
            self.assertTrue(filename.startswith(prefix))
            target = directory / filename.removeprefix(prefix)
            content = target.read_bytes()
            triangle_count = struct.unpack_from('<I', content, 80)[0]
            self.assertGreater(triangle_count, 100)
            self.assertEqual(len(content), 84 + triangle_count * 50)
            for triangle in struct.iter_unpack('<12fH', content[84:]):
                self.assertTrue(all(math.isfinite(value) for value in triangle[:12]))
                self.assertTrue(all(abs(value) < 0.25 for value in triangle[3:12]))
            self.assertTrue(all(float(value) > 0 for value in mesh.get('scale').split()))
            filenames.add(target.name)
        self.assertEqual(filenames, {
            'base_plate.stl', 'shoulder_motor_plate.stl', 'shoulder_back_plate.stl',
            'upper_arm_plate.stl', 'wrist_housing.stl', 'wrist_cage.stl',
        })

    def test_distal_servos_remain_visual_only(self):
        """Represent the additional wrist hardware without inventing control joints."""
        robot = expand_model()
        for name in ('old_arm_distal_left_servo_mount', 'old_arm_distal_right_servo_mount'):
            joint = robot.find(f"joint[@name='{name}']")
            self.assertIsNotNone(joint)
            self.assertEqual(joint.get('type'), 'fixed')
            self.assertEqual(joint.find('parent').get('link'), 'old_arm_wrist_roll')


class TestOldArmTransforms(unittest.TestCase):
    """Verify live input reaches the intended model and moves its wrist frame."""

    def test_joint_states_drive_calibrated_transforms(self, joint_states_topic):
        """Match zero, yaw, shoulder, elbow, combined and roll poses to the handoff."""
        rclpy.init()
        node = rclpy.create_node('old_arm_transform_test_' + uuid.uuid4().hex)
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        publisher = node.create_publisher(JointState, joint_states_topic, 10)
        try:
            poses = [
                (0.0, 0.0, 0.0, 0.0),
                (math.pi / 2, 0.0, 0.0, 0.0),
                (0.0, math.pi / 2, 0.0, 0.0),
                (0.0, 0.0, math.pi / 3, 0.0),
                (math.pi / 4, math.pi / 4, -math.pi / 4, 0.0),
                (0.0, 0.0, 0.0, math.pi / 2),
            ]
            for positions in poses:
                expected = expected_wrist_position(positions)
                first_stamp = node.get_clock().now().nanoseconds
                deadline = time.monotonic() + 8.0
                matched = False
                while time.monotonic() < deadline:
                    message = JointState()
                    message.header.stamp = node.get_clock().now().to_msg()
                    message.name = JOINT_NAMES
                    message.position = list(positions)
                    publisher.publish(message)
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if not buffer.can_transform('old_arm_base_link', 'old_arm_tool_frame', Time()):
                        continue
                    transform = buffer.lookup_transform(
                        'old_arm_base_link', 'old_arm_tool_frame', Time())
                    stamp = transform.header.stamp
                    if stamp.sec * 1000000000 + stamp.nanosec < first_stamp:
                        continue
                    translation = transform.transform.translation
                    actual = (translation.x, translation.y, translation.z)
                    if not all(math.isclose(value, target, abs_tol=1e-6)
                               for value, target in zip(actual, expected)):
                        continue
                    if positions[3] != 0.0:
                        rotation = transform.transform.rotation
                        aligned = abs((rotation.x + rotation.w) / math.sqrt(2))
                        if not math.isclose(aligned, 1.0, abs_tol=1e-6):
                            continue
                    matched = True
                    break
                self.assertTrue(matched, f'No matching current transform for pose {positions}')
            self.assertEqual(node.count_publishers(joint_states_topic), 1)
            self.assertGreater(publisher.get_subscription_count(), 0)
        finally:
            listener.unregister()
            node.destroy_node()
            rclpy.shutdown()


@launch_testing.post_shutdown_test()
class TestOldArmShutdown(unittest.TestCase):
    """Require a clean state-publisher shutdown."""

    def test_exit_codes(self, proc_info):
        """Reject unexpected child process failures."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
