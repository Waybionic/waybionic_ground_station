#!/usr/bin/env python3
"""Demonstrate position-only IK along the Cartesian X, Y, and Z axes."""

import copy
import threading
import time

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4"]
READY_POSITION = [0.0, -0.7854, 0.0, 0.7854]


class IkXyzDemo(Node):
    """Drive the mock arm through small Cartesian targets using MoveIt IK."""

    def __init__(self):
        super().__init__("ik_xyz_demo")
        self.declare_parameter("step_m", 0.04)
        self.declare_parameter("move_seconds", 0.55)
        self.declare_parameter("pause_seconds", 0.15)
        self.declare_parameter("cycles", 1)
        self.declare_parameter("run_on_start", False)

        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        self.marker_pub = self.create_publisher(
            MarkerArray, "/ik_demo/markers", marker_qos
        )
        self.target_pub = self.create_publisher(
            PoseStamped, "/ik_demo/target", marker_qos
        )
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_service(Trigger, "/ik_demo/replay", self._on_replay)

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
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
                self.get_logger().error(f"XYZ IK demo failed: {error}")
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

    def _send_joint_positions(self, positions, seconds, rejection_timeout=0.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.time_from_start = Duration(seconds=seconds).to_msg()
        goal.trajectory.points = [point]

        rejection_deadline = time.monotonic() + rejection_timeout
        send_result = None
        while rclpy.ok() and not self._stop.is_set():
            send_result = self._wait_for_future(
                self.trajectory_client.send_goal_async(goal), 5.0
            )
            if send_result is not None and send_result.accepted:
                break
            if time.monotonic() >= rejection_deadline:
                break
            if self._stop.wait(0.1):
                return False

        if send_result is None or not send_result.accepted:
            self.get_logger().error("The arm controller rejected the trajectory")
            return False

        result = self._wait_for_future(send_result.get_result_async(), seconds + 5.0)
        if result is None or result.result.error_code != 0:
            self.get_logger().error("The arm controller did not complete the motion")
            return False
        return True

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
        request.ik_request.robot_state.joint_state = joint_state
        request.ik_request.pose_stamped = target
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        request.ik_request.avoid_collisions = False

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
        self.get_logger().info("Waiting for MoveIt IK and the arm controller...")
        while rclpy.ok() and not self._stop.is_set():
            if self.ik_client.wait_for_service(timeout_sec=0.5):
                break
        if not rclpy.ok() or self._stop.is_set():
            return
        if not self.trajectory_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("Arm trajectory action did not become available")
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
            self.get_logger().error("Joint states did not become available")
            return

        self.get_logger().info("Moving to the ready pose...")
        move_seconds = max(0.1, float(self.get_parameter("move_seconds").value))
        if not self._send_joint_positions(
            READY_POSITION, max(0.8, move_seconds), rejection_timeout=5.0
        ):
            return
        if self._stop.wait(0.15):
            return

        origin = self._lookup_wrist_pose()
        if origin is None:
            self.get_logger().error("Could not resolve the wrist pose in the world frame")
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
                if solution is not None:
                    self._send_joint_positions(solution, move_seconds)
                if self._stop.wait(pause):
                    return

        self._publish_markers(origin, origin, "Manual IK ready")
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
