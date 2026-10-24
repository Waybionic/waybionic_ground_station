"""
Let the joint sliders move the arm only as far as it can go without hitting itself or the table.

joint_state_publisher_gui publishes the pose its sliders ask for on /joint_states_requested.
This node moves the arm from where it is toward that pose in steps of at most max_step_deg,
with the collision check that teleop uses, and publishes on /joint_states the furthest pose
before any part would touch the table (table_height m above the bottom of the base) or fold into
the base or shoulder. When it stops short of the sliders, or a slider is past a joint limit, it
publishes the pose on /joint_state_guard/accepted too, which the slider window reads back so
that its sliders show where the arm stopped. A model without the arm's collision shapes, such as
the old arm, has nothing to check, so its slider poses pass through unchanged.
"""

import math

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.kinematics import joint_limits

MOVING = ('revolute', 'continuous', 'prismatic')


class JointStateGuard(Node):
    """Pass on slider poses up to where the arm would collide."""

    def __init__(self, **kwargs):
        super().__init__('joint_state_guard', **kwargs)
        self.table_height = float(self.declare_parameter('table_height', 0.0).value)
        self.clearance = float(self.declare_parameter('collision_clearance', 0.0).value)
        self.self_clearance = float(
            self.declare_parameter('self_collision_clearance', 0.0).value)
        self.step = math.radians(float(self.declare_parameter('max_step_deg', 0.5).value))
        if not (math.isfinite(self.table_height) and 0 <= self.clearance < math.inf
                and 0 <= self.self_clearance < math.inf and 0 < self.step < math.inf):
            raise ValueError('table_height must be finite, collision_clearance and '
                             'self_collision_clearance zero or more and finite, and '
                             'max_step_deg positive and finite')
        # None until a description with the arm's collision shapes arrives.
        self.collision = None
        self.unchecked = False
        self.limits = {}
        self.pose = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.load, latched)
        # Only the newest slider pose matters, so a long check never queues old ones.
        self.create_subscription(JointState, 'joint_states_requested', self.on_request, 1)
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.accepted_publisher = self.create_publisher(
            JointState, 'joint_state_guard/accepted', 10)

    def load(self, message):
        try:
            collision = ArmCollision.from_urdf(message.data, self.table_height, self.clearance,
                                               self.self_clearance)
            # robot_state_publisher works out the mimic joints from the joints they follow.
            joints = [joint['name'] for joint in collision.joints.values()
                      if joint['type'] in MOVING and not joint['mimic']]
            limits = joint_limits(message.data, joints)
        except ValueError as error:
            self.collision, self.unchecked = None, True
            self.get_logger().warning(f'Joint sliders are not collision checked: {error}')
            return
        self.collision, self.limits, self.unchecked = collision, limits, False
        # A new description keeps the arm where it was, inside the new limits.
        self.pose = {joint: min(max(self.pose.get(joint, 0.0), low), high)
                     for joint, (low, high) in limits.items()}
        self.get_logger().info('Joint sliders stop where the arm would hit itself or the table')

    def on_request(self, message):
        if self.unchecked:
            self.state_publisher.publish(message)
        if self.collision is None:
            return
        requested = {name: position for name, position in zip(message.name, message.position)
                     if name in self.limits and math.isfinite(position)}
        goal = {joint: min(max(requested.get(joint, self.pose[joint]), low), high)
                for joint, (low, high) in self.limits.items()}
        self.pose, blocked = self.collision.reach(self.pose, goal, self.step)
        state = JointState(name=list(self.pose), position=list(self.pose.values()))
        state.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(state)
        if any(abs(requested[joint] - self.pose[joint]) > 1e-9 for joint in requested):
            self.accepted_publisher.publish(state)
            if blocked:
                self.get_logger().info('Sliders stopped before ' + ', '.join(blocked),
                                       throttle_duration_sec=1.0)


def main():
    rclpy.init()
    node = JointStateGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as error:
        # SIGINT can shut down the context before spin recreates its wait set.
        if rclpy.ok() or 'the given context is not valid' not in str(error):
            raise
    except RuntimeError as error:
        # Jazzy may surface this binding error when SIGINT invalidates a subscription.
        if (rclpy.ok() or not str(error).startswith(
                "Unable to convert call argument '0' to Python object")):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
