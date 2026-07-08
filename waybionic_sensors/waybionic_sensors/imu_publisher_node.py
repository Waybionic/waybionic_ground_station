#!/usr/bin/env python3
"""Publish accelerometer/gyroscope data as sensor_msgs/msg/Imu."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('waybionic_imu_publisher')
        self.declare_parameter('use_mock', True)
        self.declare_parameter('topic', '/waybionic/imu/data_raw')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('parent_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('serial_port', '')

        self.use_mock_ = self.get_parameter('use_mock').get_parameter_value().bool_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.frame_id_ = self.get_parameter('frame_id').get_parameter_value().string_value
        self.parent_frame_id_ = self.get_parameter('parent_frame_id').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.serial_port_ = self.get_parameter('serial_port').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(Imu, topic, 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.02
        self.timer_ = self.create_timer(period, self.publish_imu)

        if self.use_mock_:
            self.get_logger().info(f'Publishing mock IMU data to {topic} ({self.frame_id_})')
        elif self.serial_port_:
            self.get_logger().info(
                f'Live IMU mode configured for serial port {self.serial_port_} '
                '(hardware reader not yet implemented; publishing is disabled)'
            )
        else:
            self.get_logger().warn(
                'Live IMU mode requested but serial_port is empty; '
                'set serial_port when hardware is available'
            )

    def publish_imu(self):
        if not self.use_mock_:
            return

        stamp = self.get_clock().now()
        pulse = math.sin(stamp.nanoseconds / 2e9)
        roll = 0.10 * pulse
        pitch = 0.05 * math.sin(stamp.nanoseconds / 3e9)
        yaw = 0.20 * pulse

        msg = Imu()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id_

        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = 0.10 * pulse
        msg.angular_velocity.y = 0.05 * math.cos(stamp.nanoseconds / 3e9)
        msg.angular_velocity.z = 0.20 * pulse

        msg.linear_acceleration.x = 0.05 * pulse
        msg.linear_acceleration.y = -0.02 * math.sin(stamp.nanoseconds / 4e9)
        msg.linear_acceleration.z = 9.81

        self.publisher_.publish(msg)
        self.publish_tf(stamp, qx, qy, qz, qw)

    def publish_tf(self, stamp, qx, qy, qz, qw):
        transform = TransformStamped()
        transform.header.stamp = stamp.to_msg()
        transform.header.frame_id = self.parent_frame_id_
        transform.child_frame_id = self.frame_id_
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.1
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster_.sendTransform(transform)


def main():
    rclpy.init()
    node = ImuPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
