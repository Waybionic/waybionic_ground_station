#!/usr/bin/env python3
"""
ROS 2 node publishing WayBionic IMU data and sensor health.

The node only wires components together: sample generation lives in
:mod:`waybionic_sensors.mock_source` or a future
:class:`~waybionic_sensors.hardware_reader.ImuHardwareReader`, message
construction in :mod:`waybionic_sensors.imu_messages`, and health reporting in
:mod:`waybionic_sensors.imu_diagnostics`.

Topics
------
``~topic`` (default ``/waybionic/imu/data_raw``)
    ``sensor_msgs/msg/Imu`` with gyroscope and accelerometer data. Orientation is
    always marked unavailable here.
``~demo_orientation_topic`` (default ``/waybionic/imu/data_demo``)
    Only advertised when ``publish_demo_orientation`` is true. Carries a
    synthetic orientation for visualisation.
``/diagnostics``
    ``diagnostic_msgs/msg/DiagnosticArray`` including ``imu.heartbeat``.
"""

from diagnostic_msgs.msg import DiagnosticArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from waybionic_sensors.hardware_reader import UnconfiguredImuReader
from waybionic_sensors.imu_diagnostics import ImuDiagnosticsBuilder
from waybionic_sensors.imu_messages import (
    build_demo_orientation_message,
    build_demo_transform,
    build_raw_imu_message,
)
from waybionic_sensors.mock_source import MockImuSource


class ImuPublisher(Node):
    """Publishes IMU samples, an optional demo orientation, and health."""

    def __init__(self, **kwargs) -> None:
        """
        Declare parameters, build the data source, and start the timers.

        Extra keyword arguments are forwarded to :class:`rclpy.node.Node`, which
        lets tests supply ``parameter_overrides`` without a launch file.
        """
        super().__init__('waybionic_imu_publisher', **kwargs)

        self._declare_parameters()

        self._frame_id = self._string_param('frame_id')
        self._parent_frame_id = self._string_param('parent_frame_id')
        self._publish_demo_orientation = self._bool_param('publish_demo_orientation')
        self._publish_demo_tf = self._bool_param('publish_demo_tf')
        self._angular_velocity_stddev = self._double_param('angular_velocity_stddev')
        self._linear_acceleration_stddev = self._double_param('linear_acceleration_stddev')
        self._orientation_stddev = self._double_param('orientation_stddev')

        rate_hz = self._double_param('publish_rate_hz')
        self._publish_rate_hz = rate_hz if rate_hz > 0.0 else 50.0

        use_mock = self._bool_param('use_mock')
        self._mock_source = None
        self._hardware_reader = None
        if use_mock:
            self._mock_source = MockImuSource(
                stall_after_sec=self._double_param('mock_stall_after_sec'),
            )
            self._source_description = 'mock generator'
        else:
            self._hardware_reader = UnconfiguredImuReader(self._string_param('serial_port'))
            self._hardware_reader.start()
            self._source_description = self._hardware_reader.describe()

        self._imu_publisher = self.create_publisher(Imu, self._string_param('topic'), 10)
        self._demo_publisher = None
        if self._publish_demo_orientation:
            self._demo_publisher = self.create_publisher(
                Imu, self._string_param('demo_orientation_topic'), 10
            )

        self._tf_broadcaster = TransformBroadcaster(self) if self._publish_demo_tf else None

        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self._string_param('diagnostics_topic'), 10
        )
        self._diagnostics_builder = ImuDiagnosticsBuilder(
            stale_timeout_sec=self._double_param('stale_timeout_sec'),
            expected_rate_hz=self._publish_rate_hz,
        )

        self._last_reading = None
        self._samples_since_report = 0
        self._last_report_ns = self._now_ns()

        self._sample_timer = self.create_timer(1.0 / self._publish_rate_hz, self._on_sample_timer)

        diagnostics_period = 1.0 / max(1.0, self._double_param('diagnostics_rate_hz'))
        self._diagnostics_timer = self.create_timer(
            diagnostics_period, self._on_diagnostics_timer
        )

        self._log_startup(use_mock)

    def _declare_parameters(self) -> None:
        """Declare every runtime parameter with its default."""
        self.declare_parameter('use_mock', True)
        self.declare_parameter('topic', '/waybionic/imu/data_raw')
        self.declare_parameter('demo_orientation_topic', '/waybionic/imu/data_demo')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('parent_frame_id', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('diagnostics_rate_hz', 2.0)
        self.declare_parameter('stale_timeout_sec', 1.0)
        # Both demo outputs default off: they are visualisation aids and would
        # otherwise imply the raw sensor knows its own attitude.
        self.declare_parameter('publish_demo_orientation', False)
        self.declare_parameter('publish_demo_tf', False)
        # 0.0 means unknown covariance (ROS all-zero matrix). Do not invent
        # datasheet values; electrical must supply them. See HARDWARE_INTERFACE.md.
        self.declare_parameter('angular_velocity_stddev', 0.0)
        self.declare_parameter('linear_acceleration_stddev', 0.0)
        # Demo topic only: synthetic placeholder, never copied onto data_raw.
        self.declare_parameter('orientation_stddev', 0.05)
        self.declare_parameter('mock_stall_after_sec', 0.0)
        self.declare_parameter('serial_port', '')

    def _bool_param(self, name: str) -> bool:
        """Read a declared boolean parameter."""
        return self.get_parameter(name).get_parameter_value().bool_value

    def _string_param(self, name: str) -> str:
        """Read a declared string parameter."""
        return self.get_parameter(name).get_parameter_value().string_value

    def _double_param(self, name: str) -> float:
        """Read a declared double parameter."""
        return self.get_parameter(name).get_parameter_value().double_value

    def _now_ns(self) -> int:
        """Return the current node time in nanoseconds."""
        return self.get_clock().now().nanoseconds

    def _log_startup(self, use_mock: bool) -> None:
        """Log the active configuration so the mode is obvious in the console."""
        logger = self.get_logger()
        logger.info(
            f'IMU source: {self._source_description}; '
            f'publishing {self._string_param("topic")} '
            f'at {self._publish_rate_hz:.1f} Hz in frame {self._frame_id}'
        )
        logger.info(
            'Raw topic marks orientation unavailable '
            '(orientation_covariance[0] = -1); demo orientation '
            f'{"enabled" if self._publish_demo_orientation else "disabled"}, '
            f'demo TF {"enabled" if self._publish_demo_tf else "disabled"}'
        )
        if not use_mock:
            logger.warning(
                'Live mode selected but no hardware driver is implemented yet. '
                'imu.heartbeat will report STALE until a real reader is supplied.'
            )
        if self._mock_source is not None and self._mock_source.stalled_deliberately:
            logger.warning(
                'mock_stall_after_sec is set; the mock will stop publishing so '
                'the stale heartbeat path can be demonstrated.'
            )

    def _read(self, stamp_ns: int):
        """Return the newest sample from whichever source is configured."""
        if self._mock_source is not None:
            return self._mock_source.read(stamp_ns)
        return self._hardware_reader.read(stamp_ns)

    def _on_sample_timer(self) -> None:
        """Acquire one sample and publish the raw, demo, and TF outputs."""
        stamp_ns = self._now_ns()
        reading = self._read(stamp_ns)
        if reading is None:
            return

        self._last_reading = reading
        self._samples_since_report += 1

        self._imu_publisher.publish(
            build_raw_imu_message(
                reading,
                self._frame_id,
                angular_velocity_stddev=self._angular_velocity_stddev,
                linear_acceleration_stddev=self._linear_acceleration_stddev,
            )
        )

        if self._demo_publisher is None and self._tf_broadcaster is None:
            return

        orientation = self._demo_orientation(reading)
        if orientation is None:
            return

        if self._demo_publisher is not None:
            self._demo_publisher.publish(
                build_demo_orientation_message(
                    reading,
                    self._frame_id,
                    orientation,
                    orientation_stddev=self._orientation_stddev,
                    angular_velocity_stddev=self._angular_velocity_stddev,
                    linear_acceleration_stddev=self._linear_acceleration_stddev,
                )
            )

        if self._tf_broadcaster is not None:
            self._tf_broadcaster.sendTransform(
                build_demo_transform(
                    reading.stamp_ns,
                    self._parent_frame_id,
                    self._frame_id,
                    orientation,
                )
            )

    def _demo_orientation(self, reading):
        """Return the orientation to use for demo outputs, if one is available."""
        if reading.has_orientation:
            return reading.orientation
        if self._mock_source is not None:
            return self._mock_source.demo_orientation(reading.stamp_ns)
        return None

    def _on_diagnostics_timer(self) -> None:
        """Publish IMU health, including the heartbeat, at a steady rate."""
        now_ns = self._now_ns()
        elapsed_sec = max(1e-9, (now_ns - self._last_report_ns) / 1e9)
        measured_rate_hz = self._samples_since_report / elapsed_sec

        self._diagnostics_publisher.publish(
            self._diagnostics_builder.build(
                now_ns,
                self._last_reading,
                measured_rate_hz,
                source_description=self._source_description,
            )
        )

        self._samples_since_report = 0
        self._last_report_ns = now_ns

    def destroy_node(self) -> bool:
        """Release the hardware reader before the node goes away."""
        if self._hardware_reader is not None:
            self._hardware_reader.stop()
        return super().destroy_node()


def main() -> None:
    """Spin the IMU publisher until interrupted."""
    rclpy.init()
    node = ImuPublisher()
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
