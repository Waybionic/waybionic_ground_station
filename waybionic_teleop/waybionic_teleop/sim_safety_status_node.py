"""Publish placeholder safety and power state until the carrier-board node is ready."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool

from waybionic_teleop.safety_interface import (
    EMERGENCY_STOP_SET_SERVICE,
    EMERGENCY_STOP_SIGNAL,
    EMERGENCY_STOP_TOPIC,
    LAST_REPORT_AGE_SIGNAL,
    LAST_REPORT_AGE_TOPIC,
    MOTOR_SUPPLY_VOLTAGE_SIGNAL,
    MOTOR_SUPPLY_VOLTAGE_TOPIC,
)


def diagnostic_status(name, level, value, unit, message):
    return DiagnosticStatus(
        name=name,
        level=level,
        message=message,
        values=[KeyValue(key='value', value=value), KeyValue(key='unit', value=unit)],
    )


class SimulatedSafetyStatus(Node):
    """Publish a switchable E-stop and placeholder voltage/report-age telemetry."""

    def __init__(self, **kwargs):
        super().__init__('sim_safety_status', **kwargs)
        voltage = float(self.declare_parameter('motor_supply_voltage', 24.0).value)
        report_period = float(self.declare_parameter('report_period_s', 0.1).value)
        diagnostics_topic = self.declare_parameter('diagnostics_topic', '/diagnostics').value
        if not math.isfinite(voltage) or voltage <= 0.0:
            raise ValueError('motor_supply_voltage must be positive and finite')
        if not math.isfinite(report_period) or report_period <= 0.0:
            raise ValueError('report_period_s must be positive and finite')

        self.motor_supply_voltage = voltage
        self.report_period = report_period
        self.emergency_stop_pressed = False
        self.last_report_time = time.monotonic()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.emergency_stop_publisher = self.create_publisher(Bool, EMERGENCY_STOP_TOPIC, latched)
        self.voltage_publisher = self.create_publisher(Float32, MOTOR_SUPPLY_VOLTAGE_TOPIC, 10)
        self.report_age_publisher = self.create_publisher(Float32, LAST_REPORT_AGE_TOPIC, 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, diagnostics_topic, 10)
        self.create_service(SetBool, EMERGENCY_STOP_SET_SERVICE, self.set_emergency_stop)
        self.create_timer(self.report_period, self.publish_report)
        self.publish_report()

    def set_emergency_stop(self, request, response):
        self.emergency_stop_pressed = bool(request.data)
        response.success = True
        response.message = (
            'Emergency stop pressed' if self.emergency_stop_pressed
            else 'Emergency stop released'
        )
        self.publish_report()
        return response

    def publish_report(self):
        now = time.monotonic()
        report_age = max(0.0, now - self.last_report_time)
        self.emergency_stop_publisher.publish(Bool(data=self.emergency_stop_pressed))
        self.voltage_publisher.publish(Float32(data=self.motor_supply_voltage))
        self.report_age_publisher.publish(Float32(data=report_age))

        emergency_level = (
            DiagnosticStatus.ERROR if self.emergency_stop_pressed else DiagnosticStatus.OK
        )
        statuses = [
            diagnostic_status(
                EMERGENCY_STOP_SIGNAL,
                emergency_level,
                'pressed' if self.emergency_stop_pressed else 'released',
                '',
                'Emergency stop is pressed' if self.emergency_stop_pressed else '',
            ),
            diagnostic_status(
                MOTOR_SUPPLY_VOLTAGE_SIGNAL,
                DiagnosticStatus.OK,
                f'{self.motor_supply_voltage:.1f}',
                'V',
                'Simulated placeholder; no hardware voltage limits applied',
            ),
            diagnostic_status(
                LAST_REPORT_AGE_SIGNAL,
                DiagnosticStatus.OK,
                f'{report_age:.3f}',
                's',
                '',
            ),
        ]
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)
        self.last_report_time = now


def main():
    rclpy.init()
    node = SimulatedSafetyStatus()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
