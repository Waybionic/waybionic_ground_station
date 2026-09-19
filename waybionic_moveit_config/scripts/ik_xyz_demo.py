#!/usr/bin/env python3
"""Demonstrate position-only IK along the Cartesian X, Y, and Z axes."""

import copy
import threading
import time

from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK, GetStateValidity
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
READY_POSITION = [0.0, -0.7854, 0.0, 0.7854]


class IkXyzDemo(Node):
    """Drive the mock arm through small Cartesian targets using MoveIt IK."""

    def __init__(self):
        super().__init__("ik_xyz_demo")
        self.declare_parameter("step_m", 0.04)
        # Fraction of the joint_limits.yaml velocity/acceleration limits (0-1].
        self.declare_parameter("velocity_scaling", 0.5)
        self.declare_parameter("pause_seconds", 0.15)
        self.declare_parameter("cycles", 1)
        self.declare_parameter("run_on_start", False)
        self.declare_parameter("startup_timeout_seconds", 15.0)

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self.marker_pub = self.create_publisher(
            MarkerArray, "/ik_demo/markers", marker_qos
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "/ik_demo/target", marker_qos
        )
        # Machine-readable outcome of the last replay: "idle", "running",
        # "complete", or "aborted at <label>: <reason>". Transient-local so a
        # late subscriber (a test, a panel) still sees the latest state.
        self.status_pub = self.create_publisher(String, "/ik_demo/status", marker_qos)
        self._set_status("idle")
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_service(Trigger, "/ik_demo/replay", self._on_replay)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_client = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._joint_state = None
        self._joint_lock = threading.Lock()
        self._demo_lock = threading.Lock()
        self._demo_running = False
        self._demo_requested = threading.Event()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        if self.get_parameter("run_on_start").value:
            self._request_demo()

    def _request_demo(self):
        with self._demo_lock:
            if self._demo_running or self._stop.is_set():
                return False
            self._demo_running = True
            self._demo_requested.set()
            return True

    def _on_replay(self, _request, response):
        response.success = self._request_demo()
        if response.success:
            response.message = "XYZ IK demo accepted"
        else:
            response.message = "XYZ IK demo is already running"
        return response

    def _worker_loop(self):
        while not self._stop.is_set():
            self._demo_requested.wait(timeout=0.2)
            if self._stop.is_set():
                return
            if not self._demo_requested.is_set():
                continue
            self._demo_requested.clear()
            try:
                self._run_demo()
            except Exception as error:  # Keep the replay service alive after a failed run.
                self._abort("unexpected error", str(error))
            finally:
                with self._demo_lock:
                    self._demo_running = False

    def _on_joint_state(self, message):
        with self._joint_lock:
            self._joint_state = copy.deepcopy(message)

    def _wait_for_future(self, future, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not self._stop.is_set() and not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.05)
        return future.result() if future.done() else None

    def _wait_until_available(self, wait_for_endpoint, timeout_sec):
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and not self._stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            try:
                available = wait_for_endpoint(timeout_sec=min(0.2, remaining))
            except Exception:
                if self._stop.is_set() or not rclpy.ok():
                    return False
                raise
            if available:
                return True
        return False

    def _plan_to_joints(self, positions):
        """Ask move_group for a collision-checked plan; return the trajectory or None."""
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        # Empty start_state with is_diff=True means "plan from the current state".
        goal.request.start_state.is_diff = True
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 2.0
        scaling = min(1.0, max(0.01, float(self.get_parameter("velocity_scaling").value)))
        goal.request.max_velocity_scaling_factor = scaling
        goal.request.max_acceleration_scaling_factor = scaling
        constraints = Constraints()
        for name, position in zip(JOINT_NAMES, positions):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(position)
            constraint.tolerance_above = 0.01
            constraint.tolerance_below = 0.01
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True

        send_result = self._wait_for_future(self.move_client.send_goal_async(goal), 5.0)
        if send_result is None or not send_result.accepted:
            self.get_logger().error("move_group rejected the planning goal")
            return None
        result = self._wait_for_future(send_result.get_result_async(), 15.0)
        if result is None:
            self.get_logger().error("move_group did not return a plan in time")
            return None
        if result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Planning failed with MoveIt error code {result.result.error_code.val}"
            )
            return None
        if not result.result.planned_trajectory.joint_trajectory.points:
            self.get_logger().error("move_group returned an empty trajectory")
            return None
        return result.result.planned_trajectory

    def _trajectory_is_valid(self, trajectory):
        """Check every waypoint against the planning scene via /check_state_validity."""
        joint_trajectory = trajectory.joint_trajectory
        for index, point in enumerate(joint_trajectory.points):
            request = GetStateValidity.Request()
            request.group_name = "arm"
            request.robot_state.joint_state.name = list(joint_trajectory.joint_names)
            request.robot_state.joint_state.position = list(point.positions)
            response = self._wait_for_future(
                self.validity_client.call_async(request), 2.0
            )
            if response is None:
                self.get_logger().error(
                    f"State validity check timed out at waypoint {index}"
                )
                return False
            if not response.valid:
                contacts = ", ".join(
                    f"{contact.contact_body_1}<->{contact.contact_body_2}"
                    for contact in response.contacts
                ) or "no contact details"
                self.get_logger().error(
                    f"Trajectory waypoint {index} is invalid ({contacts})"
                )
                return False
        return True

    def _execute_trajectory(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_result = self._wait_for_future(
            self.execute_client.send_goal_async(goal), 5.0
        )
        if send_result is None or not send_result.accepted:
            self.get_logger().error("move_group rejected the trajectory execution")
            return False
        last_point = trajectory.joint_trajectory.points[-1]
        duration = Duration.from_msg(last_point.time_from_start).nanoseconds / 1e9
        result = self._wait_for_future(send_result.get_result_async(), duration + 10.0)
        if result is None or result.result.error_code.val != MoveItErrorCodes.SUCCESS:
            code = "timeout" if result is None else result.result.error_code.val
            self.get_logger().error(f"Trajectory execution failed ({code})")
            return False
        return True

    def _set_status(self, text):
        self.status_pub.publish(String(data=text))

    def _abort(self, label, reason):
        """Log and publish a demo failure; returns None so callers can ``return``."""
        self.get_logger().error(f"XYZ IK demo aborted at {label}: {reason}")
        self._set_status(f"aborted at {label}: {reason}")

    def _plan_and_execute(self, positions, retry_seconds=0.0):
        """Plan, validate every waypoint, then execute the validated trajectory.

        Returns ``None`` on success or a short failure reason. ``retry_seconds``
        re-attempts the whole sequence until the deadline, for the first motion
        when controllers may still be coming up.
        """
        deadline = time.monotonic() + retry_seconds
        reason = "planning failed"
        while rclpy.ok() and not self._stop.is_set():
            trajectory = self._plan_to_joints(positions)
            if trajectory is not None:
                if not self._trajectory_is_valid(trajectory):
                    return "trajectory failed validation"
                if self._execute_trajectory(trajectory):
                    return None
                reason = "execution failed"
            if time.monotonic() >= deadline:
                return reason
            if self._stop.wait(0.5):
                return "stopped"
        return "stopped"

    def _lookup_wrist_pose(self):
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and not self._stop.is_set() and time.monotonic() < deadline:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "world", "wrist", Time(), timeout=Duration(seconds=0.5)
                )
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                return pose
            except TransformException:
                time.sleep(0.1)
        return None

    def _solve_ik(self, target):
        with self._joint_lock:
            joint_state = copy.deepcopy(self._joint_state)
        if joint_state is None:
            self.get_logger().error("No joint state is available for the IK seed")
            return None

        request = GetPositionIK.Request()
        request.ik_request.group_name = "arm"
        # Self colliding goal returns an error code
        request.ik_request.avoid_collisions = True
        request.ik_request.robot_state.joint_state = joint_state
        request.ik_request.pose_stamped = target
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()

        response = self._wait_for_future(self.ik_client.call_async(request), 4.0)
        if response is None:
            self.get_logger().warning("IK request timed out")
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(
                f"IK failed with MoveIt error code {response.error_code.val}"
            )
            return None

        solution = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        if not all(name in solution for name in JOINT_NAMES):
            self.get_logger().error("IK response omitted one or more arm joints")
            return None
        return [solution[name] for name in JOINT_NAMES]

    @staticmethod
    def _point(x, y, z):
        point = Point()
        point.x = x
        point.y = y
        point.z = z
        return point

    def _base_marker(self, marker_id, marker_type, namespace):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _publish_markers(self, origin, target, label):
        markers = []
        axis_length = 0.16
        colors = [
            ("X", axis_length, 0.0, 0.0, 1.0, 0.1, 0.1),
            ("Y", 0.0, axis_length, 0.0, 0.1, 1.0, 0.1),
            ("Z", 0.0, 0.0, axis_length, 0.1, 0.4, 1.0),
        ]
        ox = origin.pose.position.x
        oy = origin.pose.position.y
        oz = origin.pose.position.z

        for index, (name, dx, dy, dz, red, green, blue) in enumerate(colors):
            arrow = self._base_marker(index, Marker.ARROW, "ik_axes")
            arrow.points = [self._point(ox, oy, oz), self._point(ox + dx, oy + dy, oz + dz)]
            arrow.scale.x = 0.008
            arrow.scale.y = 0.018
            arrow.scale.z = 0.025
            arrow.color.r = red
            arrow.color.g = green
            arrow.color.b = blue
            arrow.color.a = 1.0
            markers.append(arrow)

            text = self._base_marker(10 + index, Marker.TEXT_VIEW_FACING, "ik_axes")
            text.pose.position = self._point(ox + dx, oy + dy, oz + dz)
            text.scale.z = 0.035
            text.color.r = red
            text.color.g = green
            text.color.b = blue
            text.color.a = 1.0
            text.text = name
            markers.append(text)

        target_marker = self._base_marker(20, Marker.SPHERE, "ik_target")
        target_marker.pose.position = copy.deepcopy(target.pose.position)
        target_marker.scale.x = 0.035
        target_marker.scale.y = 0.035
        target_marker.scale.z = 0.035
        target_marker.color.r = 1.0
        target_marker.color.g = 0.85
        target_marker.color.b = 0.1
        target_marker.color.a = 1.0
        markers.append(target_marker)

        label_marker = self._base_marker(21, Marker.TEXT_VIEW_FACING, "ik_target")
        label_marker.pose.position = copy.deepcopy(target.pose.position)
        label_marker.pose.position.z += 0.06
        label_marker.scale.z = 0.04
        label_marker.color.r = 1.0
        label_marker.color.g = 1.0
        label_marker.color.b = 1.0
        label_marker.color.a = 1.0
        label_marker.text = label
        markers.append(label_marker)

        self.marker_pub.publish(MarkerArray(markers=markers))
        target.header.stamp = self.get_clock().now().to_msg()
        self.target_pub.publish(target)

    def _offset_pose(self, origin, offset):
        target = copy.deepcopy(origin)
        target.pose.position.x += offset[0]
        target.pose.position.y += offset[1]
        target.pose.position.z += offset[2]
        return target

    def _run_demo(self):
        self._set_status("running")
        self.get_logger().info("Waiting for MoveIt IK, planning, and execution...")
        startup_timeout = self.get_parameter("startup_timeout_seconds").value
        endpoints = [
            (self.ik_client.wait_for_service, "MoveIt IK service"),
            (self.move_client.wait_for_server, "MoveGroup action"),
            (self.execute_client.wait_for_server, "ExecuteTrajectory action"),
            (self.validity_client.wait_for_service, "State validity service"),
        ]
        for wait_for_endpoint, name in endpoints:
            if not self._wait_until_available(wait_for_endpoint, startup_timeout):
                if rclpy.ok() and not self._stop.is_set():
                    self._abort("startup", f"{name} did not become available")
                return

        deadline = time.monotonic() + 10.0
        ready = False
        while rclpy.ok() and not self._stop.is_set():
            with self._joint_lock:
                ready = self._joint_state is not None
            if ready or time.monotonic() >= deadline:
                break
            time.sleep(0.1)
        if not ready:
            self._abort("startup", "joint states did not become available")
            return

        self.get_logger().info("Moving to the ready pose...")
        reason = self._plan_and_execute(READY_POSITION, retry_seconds=5.0)
        if reason:
            self._abort("Ready pose", reason)
            return
        if self._stop.wait(0.15):
            return

        origin = self._lookup_wrist_pose()
        if origin is None:
            self._abort("Ready pose", "could not resolve the wrist pose in the world frame")
            return

        step = self.get_parameter("step_m").value
        pause = max(0.0, float(self.get_parameter("pause_seconds").value))
        sequence = [
            ("X axis +", (step, 0.0, 0.0)),
            ("Center", (0.0, 0.0, 0.0)),
            ("Y axis +", (0.0, step, 0.0)),
            ("Center", (0.0, 0.0, 0.0)),
            ("Z axis +", (0.0, 0.0, step)),
            ("Center", (0.0, 0.0, 0.0)),
        ]

        cycles = max(1, int(self.get_parameter("cycles").value))
        self.get_logger().info(
            "XYZ IK demo started. Red=X, green=Y, blue=Z."
        )
        for _ in range(cycles):
            for label, offset in sequence:
                if not rclpy.ok() or self._stop.is_set():
                    return
                target = self._offset_pose(origin, offset)
                self._publish_markers(origin, target, label)
                self.get_logger().info(
                    f"{label}: x={target.pose.position.x:.3f}, "
                    f"y={target.pose.position.y:.3f}, "
                    f"z={target.pose.position.z:.3f}"
                )
                solution = self._solve_ik(target)
                if solution is None:
                    self._abort(label, "no IK solution")
                    return
                reason = self._plan_and_execute(solution)
                if reason:
                    self._abort(label, reason)
                    return
                if self._stop.wait(pause):
                    return

        self._publish_markers(origin, origin, "Manual IK ready")
        self._set_status("complete")
        self.get_logger().info(
            "Automatic demo complete. Drag the red/green/blue goal handles in "
            "RViz, then click Plan & Execute."
        )

    def destroy_node(self):
        self._stop.set()
        self._demo_requested.set()
        if self._thread.is_alive():
            self._thread.join(timeout=5.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IkXyzDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
