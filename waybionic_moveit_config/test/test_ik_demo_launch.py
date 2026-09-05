"""End-to-end test for the headless XYZ inverse-kinematics demo."""

import math
import os
import time
import unittest

from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray

from ament_index_python.packages import get_package_share_directory

from controller_manager_msgs.srv import ListControllers

from geometry_msgs.msg import PoseStamped

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing.actions

import pytest

import rclpy
from rclpy.qos import qos_profile_action_status_default

from sensor_msgs.msg import JointState

from std_srvs.srv import Trigger


JOINT_NAMES = ('joint1', 'joint2', 'joint3', 'joint4')
TARGET_TOLERANCE_M = 0.005
MOTION_TOLERANCE_RAD = 0.002


@pytest.mark.launch_test
def generate_test_description():
    """Launch the production MoveIt stack without RViz or an automatic run."""
    package_share = get_package_share_directory('waybionic_moveit_config')
    launch_file = os.path.join(package_share, 'launch', 'demo.launch.py')
    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            'use_rviz': 'false',
            'auto_demo': 'false',
        }.items(),
    )

    return LaunchDescription([demo, launch_testing.actions.ReadyToTest()])


class TestIkDemoRuntime(unittest.TestCase):
    """Verify replay targets, IK solutions, and mock-controller execution."""

    @classmethod
    def setUpClass(cls):
        """Create one ROS node and collect all runtime evidence."""
        rclpy.init()
        cls.node = rclpy.create_node('test_ik_demo_runtime')
        cls.targets = []
        cls.joint_states = []
        cls.succeeded_goals = set()

        cls.node.create_subscription(
            JointState,
            '/joint_states',
            cls._on_joint_state,
            100,
        )
        cls.node.create_subscription(
            GoalStatusArray,
            '/arm_controller/follow_joint_trajectory/_action/status',
            cls._on_action_status,
            qos_profile_action_status_default,
        )
        cls.node.create_subscription(
            PoseStamped,
            '/ik_demo/target',
            cls._on_target,
            20,
        )
        cls.replay_client = cls.node.create_client(Trigger, '/ik_demo/replay')
        cls.controllers_client = cls.node.create_client(
            ListControllers,
            '/controller_manager/list_controllers',
        )

    @classmethod
    def tearDownClass(cls):
        """Release the test node before launch_testing stops the stack."""
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_target(cls, message):
        position = message.pose.position
        cls.targets.append(
            (time.monotonic(), (position.x, position.y, position.z))
        )

    @classmethod
    def _on_joint_state(cls, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in JOINT_NAMES):
            cls.joint_states.append((time.monotonic(), positions))

    @classmethod
    def _on_action_status(cls, message):
        for status in message.status_list:
            if status.status == GoalStatus.STATUS_SUCCEEDED:
                cls.succeeded_goals.add(bytes(status.goal_info.goal_id.uuid))

    def _spin_until(self, predicate, timeout_sec, failure_message):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(failure_message)

    def _assert_point_almost_equal(self, actual, expected):
        distance = math.dist(actual, expected)
        self.assertLessEqual(
            distance,
            TARGET_TOLERANCE_M,
            f'target {actual} is {distance:.4f} m from expected {expected}',
        )

    def _assert_motion_between_targets(self, start_index, end_index):
        start_time = self.targets[start_index][0]
        end_time = self.targets[end_index][0]
        samples = [
            positions
            for timestamp, positions in self.joint_states
            if start_time <= timestamp <= end_time
        ]
        self.assertGreaterEqual(
            len(samples),
            2,
            f'not enough joint-state samples for target {start_index}',
        )

        largest_range = max(
            max(sample[name] for sample in samples)
            - min(sample[name] for sample in samples)
            for name in JOINT_NAMES
        )
        self.assertGreater(
            largest_range,
            MOTION_TOLERANCE_RAD,
            f'target {start_index} produced no observable controller motion',
        )

    def _wait_for_active_controllers(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            request = ListControllers.Request()
            future = self.controllers_client.call_async(request)
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            if not future.done():
                break

            response = future.result()
            active = {
                controller.name
                for controller in response.controller
                if controller.state == 'active'
            }
            if {'joint_state_broadcaster', 'arm_controller'} <= active:
                return
            time.sleep(0.05)
        self.fail('mock controllers did not become active')

    def test_replay_runs_xyz_ik_and_controller(self):
        """Run one replay and prove all targets completed on mock hardware."""
        self.assertTrue(
            self.replay_client.wait_for_service(timeout_sec=30.0),
            '/ik_demo/replay did not become available',
        )
        self.assertTrue(
            self.controllers_client.wait_for_service(timeout_sec=30.0),
            '/controller_manager/list_controllers did not become available',
        )
        self._spin_until(
            lambda: bool(self.joint_states),
            30.0,
            '/joint_states did not become available',
        )
        self._wait_for_active_controllers(30.0)

        # Ignore any transient-local samples from before this explicit replay.
        self.targets.clear()
        self.joint_states.clear()
        self.succeeded_goals.clear()

        replay_future = self.replay_client.call_async(Trigger.Request())
        self._spin_until(
            replay_future.done,
            10.0,
            'replay service did not respond',
        )
        response = replay_future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success, response.message)

        self._spin_until(
            lambda: len(self.targets) >= 7 and len(self.succeeded_goals) >= 7,
            45.0,
            'XYZ replay did not finish seven successful controller goals',
        )

        targets = [position for _, position in self.targets[:7]]
        center = targets[1]
        step = 0.04
        expected_targets = [
            (center[0] + step, center[1], center[2]),
            center,
            (center[0], center[1] + step, center[2]),
            center,
            (center[0], center[1], center[2] + step),
            center,
            center,
        ]
        for actual, expected in zip(targets, expected_targets):
            self._assert_point_almost_equal(actual, expected)

        # Each outward X/Y/Z target must create measured joint motion before
        # the following center target is published.
        self._assert_motion_between_targets(0, 1)
        self._assert_motion_between_targets(2, 3)
        self._assert_motion_between_targets(4, 5)
