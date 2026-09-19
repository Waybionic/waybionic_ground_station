"""Blocked-motion regressions for the XYZ inverse-kinematics demo.

Adds collision objects to the MoveIt planning scene and proves that:

* a blocked ready pose is rejected before anything is sent to the controller;
* a blocked X segment is rejected after the ready move, and nothing further
  reaches trajectory execution.

A control run on an empty scene comes first, so a later rejection cannot be
confused with an unreachable target.
"""

import os
import time
import unittest

from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, PoseStamped

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing.actions
import launch_testing.asserts

from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

import pytest

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_action_status_default

from shape_msgs.msg import SolidPrimitive

from std_msgs.msg import String

from std_srvs.srv import Trigger


# Larger than the default 0.04 m so an obstacle can enclose the X+ target
# without touching the 0.10 x 0.115 x 0.12 m wrist box at the ready pose.
TEST_STEP_M = 0.10
CONTROLLER_STATUS_TOPIC = '/arm_controller/follow_joint_trajectory/_action/status'
EXECUTE_STATUS_TOPIC = '/execute_trajectory/_action/status'


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


class TestIkDemoBlocked(unittest.TestCase):
    """Obstacles must make the demo abort before trajectory execution."""

    @classmethod
    def setUpClass(cls):
        """Create one ROS node and subscribe to every signal the tests assert on."""
        rclpy.init()
        cls.node = rclpy.create_node('test_ik_demo_blocked')
        cls.statuses = []
        cls.targets = []
        cls.controller_goals = {}
        cls.execute_goals = {}
        cls.ready_position = None
        cls.goals_before = (set(), set())
        cls.x_target = None

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE

        cls.node.create_subscription(String, '/ik_demo/status', cls._on_status, latched)
        cls.node.create_subscription(PoseStamped, '/ik_demo/target', cls._on_target, latched)
        cls.node.create_subscription(
            GoalStatusArray,
            CONTROLLER_STATUS_TOPIC,
            lambda message: cls._on_action_status(cls.controller_goals, message),
            qos_profile_action_status_default,
        )
        cls.node.create_subscription(
            GoalStatusArray,
            EXECUTE_STATUS_TOPIC,
            lambda message: cls._on_action_status(cls.execute_goals, message),
            qos_profile_action_status_default,
        )

        cls.replay_client = cls.node.create_client(Trigger, '/ik_demo/replay')
        cls.apply_scene_client = cls.node.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )
        cls.get_scene_client = cls.node.create_client(
            GetPlanningScene, '/get_planning_scene'
        )
        cls.param_client = cls.node.create_client(
            SetParameters, '/ik_xyz_demo/set_parameters'
        )

    @classmethod
    def tearDownClass(cls):
        """Release the test node before launch_testing stops the stack."""
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- subscription callbacks -------------------------------------------------

    @classmethod
    def _on_status(cls, message):
        cls.statuses.append((time.monotonic(), message.data))

    @classmethod
    def _on_target(cls, message):
        position = message.pose.position
        cls.targets.append((time.monotonic(), (position.x, position.y, position.z)))

    @classmethod
    def _on_action_status(cls, goals, message):
        for status in message.status_list:
            goals[bytes(status.goal_info.goal_id.uuid)] = status.status

    # --- helpers ----------------------------------------------------------------

    def _spin_until(self, predicate, timeout_sec, failure_message):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail(failure_message)

    def _call(self, client, request, timeout_sec, what):
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), f'{what} unavailable')
        future = client.call_async(request)
        self._spin_until(future.done, timeout_sec, f'{what} did not respond')
        return future.result()

    def _set_step(self, step_m):
        parameter = Parameter()
        parameter.name = 'step_m'
        parameter.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE, double_value=step_m
        )
        request = SetParameters.Request(parameters=[parameter])
        response = self._call(self.param_client, request, 10.0, 'set_parameters')
        self.assertTrue(response.results[0].successful, response.results[0].reason)

    @staticmethod
    def _box(name, center, size, operation):
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = 'world'
        obj.operation = operation
        if operation == CollisionObject.ADD:
            primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size))
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = center
            pose.orientation.w = 1.0
            obj.primitives = [primitive]
            obj.primitive_poses = [pose]
        return obj

    def _scene_object_ids(self):
        request = GetPlanningScene.Request()
        request.components = PlanningSceneComponents(
            components=PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        response = self._call(self.get_scene_client, request, 10.0, 'get_planning_scene')
        return {obj.id for obj in response.scene.world.collision_objects}

    def _apply_scene(self, collision_object):
        scene = PlanningScene(is_diff=True)
        scene.world.collision_objects = [collision_object]
        response = self._call(
            self.apply_scene_client,
            ApplyPlanningScene.Request(scene=scene),
            10.0,
            'apply_planning_scene',
        )
        self.assertTrue(response.success, 'apply_planning_scene refused the update')

        should_exist = collision_object.operation == CollisionObject.ADD
        self._spin_until(
            lambda: (collision_object.id in self._scene_object_ids()) == should_exist,
            10.0,
            f'planning scene did not reflect {collision_object.id}',
        )

    def _replay_and_wait(self, timeout_sec):
        """Trigger one replay and return its terminal status string."""
        self.statuses.clear()
        self.targets.clear()
        # Action status topics are transient-local and still list goals from
        # the previous run, so remember what exists now and count only new ones.
        self._settle(0.5)
        self.goals_before = (set(self.controller_goals), set(self.execute_goals))

        response = self._call(self.replay_client, Trigger.Request(), 10.0, '/ik_demo/replay')
        self.assertTrue(response.success, response.message)

        def finished():
            return any(
                text == 'complete' or text.startswith('aborted')
                for _, text in self.statuses
            )

        self._spin_until(finished, timeout_sec, 'replay did not reach a terminal status')
        return self.statuses[-1][1]

    def _settle(self, seconds):
        """Keep spinning so late action-status updates are observed."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _new_goals(self):
        """Goals (uuid -> status) first seen since the last replay was triggered."""
        before_controller, before_execute = self.goals_before
        controller = {k: v for k, v in self.controller_goals.items() if k not in before_controller}
        execute = {k: v for k, v in self.execute_goals.items() if k not in before_execute}
        return controller, execute

    def _assert_execution_count(self, expected):
        self._settle(2.0)
        controller, execute = self._new_goals()
        self.assertEqual(
            len(execute), expected,
            f'expected {expected} goal(s) on /execute_trajectory, saw {execute}',
        )
        self.assertEqual(
            len(controller), expected,
            f'expected {expected} goal(s) on the arm controller, saw {controller}',
        )
        return controller

    # --- tests (numeric prefixes fix the order; they share the one launch) -----

    def test_1_control_run(self):
        """An empty scene must complete, proving the enlarged targets are reachable."""
        self._set_step(TEST_STEP_M)
        status = self._replay_and_wait(90.0)
        self.assertEqual(status, 'complete', status)

        # Sequence: X+, Center, Y+, Center, Z+, Center, then "Manual IK ready".
        self.assertGreaterEqual(len(self.targets), 6, 'fewer than six targets observed')
        type(self).x_target = self.targets[0][1]
        type(self).ready_position = self.targets[1][1]
        self.assertAlmostEqual(
            self.x_target[0] - self.ready_position[0], TEST_STEP_M, places=3
        )

    def test_2_blocked_ready_pose(self):
        """A cube over the ready pose must be rejected with zero controller goals."""
        self.assertIsNotNone(self.ready_position, 'control run did not record the ready pose')
        obstacle = self._box('blocked_ready', self.ready_position, (0.3, 0.3, 0.3),
                             CollisionObject.ADD)
        self._apply_scene(obstacle)
        try:
            status = self._replay_and_wait(60.0)
            self.assertTrue(status.startswith('aborted at Ready pose:'), status)
            self.assertIn(
                status.split(': ', 1)[1],
                ('planning failed', 'trajectory failed validation'),
                status,
            )
            self._assert_execution_count(0)
        finally:
            self._apply_scene(self._box('blocked_ready', None, None, CollisionObject.REMOVE))

    def test_3_blocked_segment(self):
        """A cube on the X+ target must be rejected after the ready move only."""
        self.assertIsNotNone(self.x_target, 'control run did not record the X+ target')
        # Enclose the X+ wrist origin, offset outward so the cube clears the
        # wrist box at the ready pose (0.10 m back along -X).
        x, y, z = self.x_target
        obstacle = self._box('blocked_x', (x + 0.03, y, z), (0.08, 0.08, 0.08),
                             CollisionObject.ADD)
        self._apply_scene(obstacle)
        try:
            status = self._replay_and_wait(60.0)
            self.assertTrue(status.startswith('aborted at X axis +:'), status)
            self.assertIn(
                status.split(': ', 1)[1],
                ('no IK solution', 'planning failed', 'trajectory failed validation'),
                status,
            )

            # Exactly the ready move reached execution, and it succeeded.
            controller = self._assert_execution_count(1)
            self.assertEqual(list(controller.values()), [GoalStatus.STATUS_SUCCEEDED])
        finally:
            self._apply_scene(self._box('blocked_x', None, None, CollisionObject.REMOVE))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Every process, move_group included, must stop cleanly on SIGINT."""
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2])
