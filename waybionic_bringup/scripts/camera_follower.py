#!/usr/bin/env python3
"""Publish a smoothed view_focus frame near the arm's tool so the RViz camera follows it."""

import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros


class CameraFollower(Node):
    """Track a point between the shoulder and the tool, low-pass filtered so the view glides."""

    def __init__(self):
        super().__init__('camera_follower')
        self.fixed_frame = self.declare_parameter('fixed_frame', 'base_link').value
        self.tool_frame = self.declare_parameter('tool_frame', 'tool_link').value
        self.anchor_frame = self.declare_parameter('anchor_frame', 'shoulder_link').value
        self.focus_frame = self.declare_parameter('focus_frame', 'view_focus').value
        self.tool_weight = self.declare_parameter('tool_weight', 0.7).value
        self.smoothing = self.declare_parameter('smoothing_s', 0.25).value
        # Used until the arm's TF exists, e.g. with the placeholder model.
        self.focus = tuple(self.declare_parameter('default_focus', [0.0, 0.0, 0.35]).value)
        self.period = 1.0 / self.declare_parameter('rate_hz', 60.0).value
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_timer(self.period, self.update)

    def position(self, frame):
        transform = self.buffer.lookup_transform(self.fixed_frame, frame, Time())
        offset = transform.transform.translation
        return offset.x, offset.y, offset.z

    def update(self):
        try:
            tool, anchor = self.position(self.tool_frame), self.position(self.anchor_frame)
        except tf2_ros.TransformException:
            goal = self.focus
        else:
            goal = [self.tool_weight * t + (1.0 - self.tool_weight) * a
                    for t, a in zip(tool, anchor)]
        blend = 1.0 - math.exp(-self.period / self.smoothing)
        self.focus = tuple(f + blend * (g - f) for f, g in zip(self.focus, goal))
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.fixed_frame
        message.child_frame_id = self.focus_frame
        offset = message.transform.translation
        offset.x, offset.y, offset.z = self.focus
        message.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(message)


def main():
    rclpy.init()
    node = CameraFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
