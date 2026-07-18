#!/usr/bin/env python3
"""Temporary demo publisher for local /diagnostics validation."""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

CYCLE_MODES = ('normal', 'fault', 'stale')
CYCLE_PERIOD_SECONDS = 5.0


def make_status(name, level, message='', value=None, unit=None):
    status = DiagnosticStatus()
    status.name = name
    status.level = level
    status.message = message
    if value is not None:
        status.values.append(KeyValue(key='value', value=str(value)))
    if unit is not None:
        status.values.append(KeyValue(key='unit', value=unit))
    return status


class TemporaryDiagnosticsPublisher(Node):
    def __init__(self):
        super().__init__('temporary_diagnostics_publisher')
        self.declare_parameter('mode', 'normal')
        self.declare_parameter('topic', '/diagnostics')
        self.declare_parameter('publish_rate_hz', 2.0)

        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        if self.mode not in {'normal', 'fault', 'stale', 'cycle'}:
            raise ValueError(
                f"Unsupported mode '{self.mode}'. Use normal, fault, stale, or cycle."
            )

        self.publisher_ = self.create_publisher(DiagnosticArray, topic, 10)
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.5
        self.timer_ = self.create_timer(period, self.publish_diagnostics)
        self.cycle_start_ = self.get_clock().now()

        self.get_logger().info(
            f"Publishing temporary diagnostics to {topic} in '{self.mode}' mode at {rate_hz:.1f} Hz"
        )

    def active_mode(self):
        if self.mode != 'cycle':
            return self.mode

        elapsed = (self.get_clock().now() - self.cycle_start_).nanoseconds / 1e9
        index = int(elapsed / CYCLE_PERIOD_SECONDS) % len(CYCLE_MODES)
        return CYCLE_MODES[index]

    def publish_diagnostics(self):
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = self.build_statuses(self.active_mode())
        self.publisher_.publish(message)

    def build_statuses(self, mode):
        pulse = math.sin(self.get_clock().now().nanoseconds / 4e9)

        if mode == 'normal':
            return [
                make_status(
                    'board.temperature',
                    DiagnosticStatus.OK,
                    value=f'{42.0 + pulse:.1f}',
                    unit='C',
                ),
                make_status(
                    'motor.current',
                    DiagnosticStatus.OK,
                    value=f'{0.8 + pulse * 0.05:.2f}',
                    unit='A',
                ),
                make_status(
                    'imu.roll',
                    DiagnosticStatus.OK,
                    value=f'{1.2 + pulse * 0.1:.1f}',
                    unit='deg',
                ),
                make_status(
                    'imu.pitch',
                    DiagnosticStatus.OK,
                    value=f'{-0.4 + pulse * 0.1:.1f}',
                    unit='deg',
                ),
                make_status(
                    'imu.yaw',
                    DiagnosticStatus.OK,
                    value=f'{12.9 + pulse * 0.2:.1f}',
                    unit='deg',
                ),
            ]

        if mode == 'fault':
            return [
                make_status(
                    'board.temperature',
                    DiagnosticStatus.ERROR,
                    'High temperature detected',
                    value=f'{82.0 + pulse * 0.5:.1f}',
                    unit='C',
                ),
                make_status('motor.current', DiagnosticStatus.OK, value='0.80', unit='A'),
                make_status('imu.roll', DiagnosticStatus.OK, value='1.2', unit='deg'),
                make_status('imu.pitch', DiagnosticStatus.OK, value='-0.4', unit='deg'),
                make_status('imu.yaw', DiagnosticStatus.OK, value='12.9', unit='deg'),
                make_status(
                    'imu.heartbeat',
                    DiagnosticStatus.STALE,
                    'Sensor timeout',
                ),
            ]

        return [
            make_status(
                'board.temperature',
                DiagnosticStatus.STALE,
                'No recent board telemetry',
                value='--',
                unit='C',
            ),
            make_status(
                'motor.current',
                DiagnosticStatus.STALE,
                'No recent motor telemetry',
                value='--',
                unit='A',
            ),
            make_status(
                'imu.roll',
                DiagnosticStatus.STALE,
                'IMU stream stale',
                value='--',
                unit='deg',
            ),
            make_status(
                'imu.pitch',
                DiagnosticStatus.STALE,
                'IMU stream stale',
                value='--',
                unit='deg',
            ),
            make_status(
                'imu.yaw',
                DiagnosticStatus.STALE,
                'IMU stream stale',
                value='--',
                unit='deg',
            ),
            make_status(
                'imu.heartbeat',
                DiagnosticStatus.STALE,
                'Sensor timeout',
            ),
        ]


def main(args=None):
    rclpy.init(args=args)
    node = TemporaryDiagnosticsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
