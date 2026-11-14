"""Joint sliders asking for a pose that clashes stop the arm where its parts meet."""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from sensor_msgs.msg import JointState

from waybionic_teleop.collision import ArmCollision

# The elbow folded back through the base: a pose the sliders used to allow.
FOLDED = {'joint_1': 0.0, 'joint_2': 0.921, 'joint_3': 2.599, 'joint_4': 1.652,
          'joint_5': -6.283}


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(
        get_package_share_directory('waybionic_bringup'), 'launch', 'ground_station.launch.py')
    return launch.LaunchDescription([
        # The slider window runs without a display.
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'offscreen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={'launch_rviz': 'false', 'use_diagnostics': 'false',
                              'follow_camera': 'false'}.items()),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSliderGuard(unittest.TestCase):

    def test_the_sliders_stop_where_the_parts_meet(self):
        urdf = os.path.join(get_package_share_directory('waybionic_description'), 'urdf',
                            'waybionic_arm.urdf')
        with open(urdf, encoding='utf-8') as source:
            check = ArmCollision.from_urdf(source.read())
        self.assertTrue(check.hits(FOLDED))
        rclpy.init()
        node = rclpy.create_node('slider_guard_test')
        states, requests, accepted = {}, {}, {}
        node.create_subscription(
            JointState, '/joint_states',
            lambda message: states.update(zip(message.name, message.position)), 10)
        node.create_subscription(
            JointState, '/joint_states_requested',
            lambda message: requests.update(zip(message.name, message.position)), 10)
        node.create_subscription(
            JointState, '/joint_state_guard/accepted',
            lambda message: accepted.update(zip(message.name, message.position)), 10)
        sliders = node.create_publisher(JointState, '/joint_states_requested', 10)

        def spin(seconds, until=lambda: False):
            end = time.monotonic() + seconds
            while time.monotonic() < end and not until():
                rclpy.spin_once(node, timeout_sec=0.05)

        try:
            # The slider window publishes, and the guard has the model and passes it on.
            spin(30.0, lambda: 'joint_3' in requests and 'joint_3' in states)
            self.assertIn('joint_3', requests, 'the slider window never published')
            self.assertIn('joint_3', states, 'the guard never passed the sliders on')
            # The real slider window also publishes on this topic at 10 Hz. Retry until
            # the guard acknowledges our pose; a single request can be replaced by a
            # newer GUI message while the guard is checking collisions.
            folded = JointState(name=list(FOLDED), position=list(FOLDED.values()))
            deadline = time.monotonic() + 15.0
            while time.monotonic() < deadline and accepted.get('joint_3', 0.0) <= 1.0:
                sliders.publish(folded)
                spin(0.1)
            self.assertGreater(accepted.get('joint_3', 0.0), 1.0,
                               'the guard did not accept the clashing request')
            # Once the test stops publishing, the GUI should adopt the accepted pose.
            spin(3.0, lambda: 'joint_3' in states and all(
                abs(requests.get(joint, float('inf')) - states.get(joint, 0.0)) < 0.002
                for joint in FOLDED))
            self.assertEqual(check.hits(states), [])
            self.assertGreater(states['joint_3'], 1.0, 'the arm did not move toward the pose')
            # The slider window was sent back to where the arm stopped and now asks for that.
            for joint in FOLDED:
                self.assertAlmostEqual(requests[joint], states[joint], delta=0.002, msg=joint)
        finally:
            node.destroy_node()
            rclpy.shutdown()
