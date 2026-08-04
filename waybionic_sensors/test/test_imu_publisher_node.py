"""
Runtime tests that spin the real node and inspect what it publishes.

These stay hardware independent: everything runs against the mock source.
"""

import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from waybionic_sensors.imu_diagnostics import HEARTBEAT_NAME
from waybionic_sensors.imu_messages import ORIENTATION_UNAVAILABLE
from waybionic_sensors.imu_publisher_node import ImuPublisher

RAW_TOPIC = '/test/imu/data_raw'
DEMO_TOPIC = '/test/imu/data_demo'
DIAGNOSTICS_TOPIC = '/test/diagnostics'


class Collector(Node):
    """Subscribes to everything the publisher can emit."""

    def __init__(self) -> None:
        """Create one subscription per output under test."""
        super().__init__('imu_test_collector')
        self.raw = []
        self.demo = []
        self.diagnostics = []
        self.transforms = []
        self.create_subscription(Imu, RAW_TOPIC, self.raw.append, 20)
        self.create_subscription(Imu, DEMO_TOPIC, self.demo.append, 20)
        self.create_subscription(DiagnosticArray, DIAGNOSTICS_TOPIC, self.diagnostics.append, 20)
        self.create_subscription(TFMessage, '/tf', self.transforms.append, 20)


def params(**overrides):
    """Build parameter overrides with test topics already applied."""
    values = {
        'topic': RAW_TOPIC,
        'demo_orientation_topic': DEMO_TOPIC,
        'diagnostics_topic': DIAGNOSTICS_TOPIC,
        'publish_rate_hz': 50.0,
        'diagnostics_rate_hz': 10.0,
    }
    values.update(overrides)

    parameters = []
    for name, value in values.items():
        if isinstance(value, bool):
            parameter_type = Parameter.Type.BOOL
        elif isinstance(value, float):
            parameter_type = Parameter.Type.DOUBLE
        else:
            parameter_type = Parameter.Type.STRING
        parameters.append(Parameter(name, parameter_type, value))
    return parameters


class Harness:
    """Spins the publisher and a collector together for a fixed duration."""

    def __init__(self, duration_sec=1.2, **overrides):
        """Record how long to spin and which parameters to override."""
        self._duration_sec = duration_sec
        self._overrides = overrides

    def __enter__(self):
        """Start both nodes on a background executor."""
        self.publisher = ImuPublisher(parameter_overrides=params(**self._overrides))
        self.collector = Collector()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.publisher)
        self._executor.add_node(self.collector)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        time.sleep(self._duration_sec)
        return self.collector

    def __exit__(self, *exc_info):
        """Tear the executor and nodes down."""
        self._executor.shutdown()
        self._thread.join(timeout=5.0)
        self.publisher.destroy_node()
        self.collector.destroy_node()
        return False


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialise rclpy once for this module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def test_publishes_raw_imu_messages():
    with Harness() as collector:
        assert len(collector.raw) > 10


def test_raw_messages_use_the_configured_frame():
    with Harness(frame_id='imu_link') as collector:
        assert all(message.header.frame_id == 'imu_link' for message in collector.raw)


def test_raw_messages_carry_increasing_timestamps():
    with Harness() as collector:
        stamps = [
            message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            for message in collector.raw
        ]
        assert stamps == sorted(stamps)
        assert stamps[0] > 0


def test_raw_messages_mark_orientation_unavailable():
    with Harness() as collector:
        assert all(
            message.orientation_covariance[0] == ORIENTATION_UNAVAILABLE
            for message in collector.raw
        )


def test_raw_messages_carry_non_zero_measurement_covariance():
    with Harness() as collector:
        message = collector.raw[0]
        assert message.angular_velocity_covariance[0] > 0.0
        assert message.linear_acceleration_covariance[0] > 0.0


def test_publish_rate_follows_the_parameter():
    duration = 1.5
    with Harness(duration_sec=duration, publish_rate_hz=20.0) as collector:
        measured = len(collector.raw) / duration
        assert 12.0 < measured < 28.0


def test_demo_topic_is_silent_by_default():
    with Harness() as collector:
        assert collector.demo == []


def test_demo_topic_publishes_orientation_when_enabled():
    with Harness(publish_demo_orientation=True) as collector:
        assert len(collector.demo) > 10
        assert collector.demo[0].orientation_covariance[0] > 0.0


def test_demo_tf_is_not_broadcast_by_default():
    with Harness() as collector:
        children = {
            transform.child_frame_id
            for message in collector.transforms
            for transform in message.transforms
        }
        assert 'imu_link' not in children


def test_demo_tf_is_broadcast_when_enabled():
    with Harness(publish_demo_tf=True, frame_id='imu_link') as collector:
        children = {
            transform.child_frame_id
            for message in collector.transforms
            for transform in message.transforms
        }
        assert 'imu_link' in children


def test_heartbeat_is_published_at_least_once_per_second():
    duration = 1.5
    with Harness(duration_sec=duration, diagnostics_rate_hz=2.0) as collector:
        heartbeats = [
            status
            for array in collector.diagnostics
            for status in array.status
            if status.name == HEARTBEAT_NAME
        ]
        assert len(heartbeats) >= int(duration)


def test_heartbeat_is_ok_while_the_mock_streams():
    with Harness() as collector:
        levels = [
            status.level
            for array in collector.diagnostics
            for status in array.status
            if status.name == HEARTBEAT_NAME
        ]
        assert DiagnosticStatus.OK in levels


def test_heartbeat_goes_stale_after_the_mock_stalls():
    with Harness(
        duration_sec=2.5, mock_stall_after_sec=0.4, stale_timeout_sec=0.5
    ) as collector:
        levels = [
            status.level
            for array in collector.diagnostics
            for status in array.status
            if status.name == HEARTBEAT_NAME
        ]
        assert DiagnosticStatus.OK in levels
        assert levels[-1] == DiagnosticStatus.STALE


def test_live_mode_without_hardware_reports_stale_and_publishes_nothing():
    with Harness(use_mock=False) as collector:
        assert collector.raw == []
        heartbeats = [
            status
            for array in collector.diagnostics
            for status in array.status
            if status.name == HEARTBEAT_NAME
        ]
        assert heartbeats
        assert all(status.level == DiagnosticStatus.STALE for status in heartbeats)
