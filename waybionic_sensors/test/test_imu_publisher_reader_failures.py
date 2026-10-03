"""
Publisher tests for the lead-approved IMU reader failure policy.

These spin the real node against a scriptable in-memory reader. They do not
add a physical driver or a hardware protocol.
"""

from types import SimpleNamespace
from unittest.mock import patch

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import Node
from rclpy.parameter import Parameter
from scripted_imu_reader import ScriptedImuReader
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from waybionic_sensors.imu_diagnostics import (
    ANGULAR_VELOCITY_NAME,
    HEARTBEAT_NAME,
    LINEAR_ACCELERATION_NAME,
    RATE_NAME,
)
from waybionic_sensors.imu_messages import ORIENTATION_UNAVAILABLE, UNKNOWN_COVARIANCE
from waybionic_sensors.imu_publisher_node import ImuPublisher
from waybionic_sensors.imu_reading import GRAVITY_M_S2, ImuReading

RAW_TOPIC = '/test/imu/data_raw'
DEMO_TOPIC = '/test/imu/data_demo'
DIAGNOSTICS_TOPIC = '/test/diagnostics'

IMU_SIGNAL_NAMES = (
    HEARTBEAT_NAME,
    RATE_NAME,
    ANGULAR_VELOCITY_NAME,
    LINEAR_ACCELERATION_NAME,
)


class Collector(Node):
    """Subscribes to everything the publisher can emit."""

    def __init__(self) -> None:
        """Create one subscription per output under test."""
        super().__init__('imu_reader_failure_collector')
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


T1_NS = 1_000_000_000
T2_NS = 2_000_000_000
IDENTITY = (0.0, 0.0, 0.0, 1.0)
ROS_TIME_MAX_NS = (2 ** 31) * 1_000_000_000 - 1


def make_reading(
    stamp_ns,
    angular_velocity=(0.1, 0.0, -0.2),
    linear_acceleration=(0.0, 0.0, GRAVITY_M_S2),
    orientation=IDENTITY,
):
    """Build a finite in-memory reading for the scripted reader."""
    return ImuReading(
        stamp_ns=stamp_ns,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
        orientation=orientation,
    )


class ControlledPublisher:
    """Publisher plus collector with timers cancelled so polls are explicit."""

    def __init__(self, reader, **overrides):
        """Create both nodes and take over the sample/diagnostics cadence."""
        overrides.setdefault('use_mock', False)
        overrides.setdefault('publish_demo_orientation', True)
        overrides.setdefault('publish_demo_tf', True)
        overrides.setdefault('stale_timeout_sec', 1.0)
        self.reader = reader
        self.publisher = ImuPublisher(
            hardware_reader=reader,
            parameter_overrides=params(**overrides),
        )
        self.publisher.destroy_timer(self.publisher._sample_timer)
        self.publisher.destroy_timer(self.publisher._diagnostics_timer)
        self.collector = Collector()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.publisher)
        self._executor.add_node(self.collector)

    def poll(self):
        """Run one reader cycle and drain published messages."""
        self.publisher._on_sample_timer()
        self._spin()

    def report(self):
        """Publish diagnostics and drain the collector."""
        self.publisher._on_diagnostics_timer()
        self._spin()

    def _spin(self):
        """Pump the executor a few times so subscriptions see publications."""
        for _ in range(8):
            self._executor.spin_once(timeout_sec=0.05)

    def last_levels(self):
        """Return the latest diagnostic level for each IMU signal name."""
        latest = {}
        for array in self.collector.diagnostics:
            for status in array.status:
                latest[status.name] = status.level
        return latest

    def sensor_output_counts(self):
        """Return raw, demo, and individual TF transform counts."""
        return (
            len(self.collector.raw),
            len(self.collector.demo),
            sum(len(message.transforms) for message in self.collector.transforms),
        )

    def destroy(self):
        """Tear down the executor and both nodes."""
        self._executor.shutdown()
        self.publisher.destroy_node()
        self.collector.destroy_node()


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    """Initialise rclpy once for this module."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def harness():
    """Yield a controlled publisher and always destroy it."""
    context = {'value': None}

    def factory(reader, **overrides):
        context['value'] = ControlledPublisher(reader, **overrides)
        return context['value']

    yield factory
    if context['value'] is not None:
        context['value'].destroy()


def failure_state(session):
    """Capture every state field a rejected read must preserve."""
    return (
        session.sensor_output_counts(),
        session.publisher._last_reading,
        session.publisher._samples_since_report,
    )


def assert_failure_preserved(session, before):
    """Assert a rejected read produced no sensor output or health refresh."""
    output_counts, last_reading, sample_count = before
    assert session.sensor_output_counts() == output_counts
    assert session.publisher._last_reading is last_reading
    assert session.publisher._samples_since_report == sample_count
    assert session.publisher.context.ok()


def test_no_sample_keeps_last_valid_and_publishes_nothing_new(harness):
    valid = make_reading(T2_NS)
    session = harness(ScriptedImuReader([valid, None]))

    session.poll()
    assert len(session.collector.raw) == 1
    assert len(session.collector.demo) == 1
    assert session.publisher._last_reading is valid
    assert session.publisher._samples_since_report == 1

    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)
    assert session.publisher._last_reading.stamp_ns == T2_NS

    session.report()
    assert session.last_levels()[HEARTBEAT_NAME] == DiagnosticStatus.STALE


def test_out_of_order_timestamp_is_rejected(harness):
    newer = make_reading(T2_NS)
    older = make_reading(T1_NS)
    session = harness(ScriptedImuReader([newer, older]))

    session.poll()
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)
    assert session.publisher._last_reading.stamp_ns == T2_NS
    header = session.collector.raw[0].header
    published_ns = header.stamp.sec * 10**9 + header.stamp.nanosec
    assert published_ns == T2_NS


def test_nan_angular_velocity_is_rejected(harness):
    valid = make_reading(T2_NS)
    invalid = make_reading(T2_NS + 1, angular_velocity=(float('nan'), 0.0, 0.0))
    session = harness(ScriptedImuReader([valid, invalid]))

    session.poll()
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)


def test_infinity_acceleration_is_rejected(harness):
    valid = make_reading(T2_NS)
    invalid = make_reading(
        T2_NS + 1, linear_acceleration=(0.0, float('inf'), GRAVITY_M_S2)
    )
    session = harness(ScriptedImuReader([valid, invalid]))

    session.poll()
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)


def test_non_finite_orientation_is_rejected(harness):
    valid = make_reading(T2_NS)
    invalid = make_reading(T2_NS + 1, orientation=(0.0, 0.0, float('-inf'), 1.0))
    session = harness(ScriptedImuReader([valid, invalid]))

    session.poll()
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)


def test_malformed_vector_is_rejected_without_crashing(harness):
    valid = make_reading(T2_NS)
    malformed = make_reading(T2_NS + 1, angular_velocity=(0.0, 0.0))
    session = harness(ScriptedImuReader([valid, malformed]))

    session.poll()
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)


@pytest.mark.parametrize(
    'bad_candidate',
    [
        SimpleNamespace(
            stamp_ns=T2_NS + 1,
            angular_velocity=(0.0, 0.0, 0.0),
            linear_acceleration=(0.0, 0.0, GRAVITY_M_S2),
            orientation=IDENTITY,
        ),
        make_reading(
            T2_NS + 1,
            angular_velocity=(component for component in (0.0, 0.0, 0.0)),
        ),
        make_reading(T2_NS + 1, angular_velocity=(10 ** 1000, 0.0, 0.0)),
        make_reading(ROS_TIME_MAX_NS + 1),
    ],
    ids=[
        'foreign-object',
        'one-shot-vector',
        'float-overflow',
        'unrepresentable-timestamp',
    ],
)
def test_contract_unsafe_candidate_is_rejected_without_output_or_crash(
    harness, bad_candidate
):
    valid = make_reading(T2_NS)
    session = harness(ScriptedImuReader([valid, bad_candidate]))

    session.poll()
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)


def test_reader_exception_does_not_kill_the_node(harness):
    first = make_reading(T2_NS)
    recovered = make_reading(T2_NS + 5_000_000)
    session = harness(
        ScriptedImuReader([first, RuntimeError('temporary bus error'), recovered])
    )

    session.poll()
    assert len(session.collector.raw) == 1
    before = failure_state(session)
    session.poll()
    assert_failure_preserved(session, before)

    session.poll()
    assert session.sensor_output_counts() == tuple(
        count + 1 for count in before[0]
    )
    assert session.publisher._last_reading is recovered
    assert session.publisher._samples_since_report == 2


def test_multi_failure_sequence_preserves_last_valid_until_recovery(harness):
    first = make_reading(T2_NS)
    invalid = make_reading(T2_NS + 1, linear_acceleration=(float('nan'), 0.0, GRAVITY_M_S2))
    recovered = make_reading(T2_NS + 10_000_000)
    session = harness(
        ScriptedImuReader(
            [first, None, invalid, RuntimeError('read failed'), recovered]
        )
    )

    session.poll()
    before = failure_state(session)
    for _ in range(3):
        session.poll()
        assert_failure_preserved(session, before)
        assert session.publisher._last_reading.stamp_ns == T2_NS

    session.poll()
    assert session.publisher._last_reading is recovered
    assert session.sensor_output_counts() == tuple(
        count + 1 for count in before[0]
    )
    assert session.publisher._samples_since_report == 2


def test_rejected_readings_cannot_refresh_stale_diagnostics(harness):
    session = harness(ScriptedImuReader(), stale_timeout_sec=0.4)
    now_ns = session.publisher.get_clock().now().nanoseconds
    stale_stamp = now_ns - 5_000_000_000
    valid = make_reading(stale_stamp)
    session.reader.enqueue(valid)
    session.poll()
    assert session.publisher._last_reading is valid
    raw_count = len(session.collector.raw)
    failure_outputs = session.sensor_output_counts()

    session.report()
    levels = session.last_levels()
    for name in IMU_SIGNAL_NAMES:
        assert levels[name] == DiagnosticStatus.STALE, name
    assert session.publisher._samples_since_report == 0

    session.reader.enqueue(None)
    session.reader.enqueue(
        make_reading(now_ns, angular_velocity=(float('nan'), 0.0, 0.0))
    )
    session.reader.enqueue(
        make_reading(now_ns, linear_acceleration=(0.0, 0.0, float('inf')))
    )
    session.reader.enqueue(RuntimeError('still disconnected'))
    session.reader.enqueue(make_reading(stale_stamp - 1))
    for _ in range(5):
        session.poll()

    assert session.publisher._last_reading is valid
    assert session.publisher._last_reading.stamp_ns == stale_stamp
    assert len(session.collector.raw) == raw_count
    assert session.sensor_output_counts() == failure_outputs
    assert session.publisher._samples_since_report == 0

    session.report()
    levels = session.last_levels()
    for name in IMU_SIGNAL_NAMES:
        assert levels[name] == DiagnosticStatus.STALE, name

    fresh = make_reading(session.publisher.get_clock().now().nanoseconds)
    session.reader.enqueue(fresh)
    session.poll()
    session.report()
    assert session.publisher._last_reading is fresh
    assert len(session.collector.raw) == raw_count + 1
    levels = session.last_levels()
    assert levels[HEARTBEAT_NAME] == DiagnosticStatus.OK
    assert levels[ANGULAR_VELOCITY_NAME] == DiagnosticStatus.OK
    assert levels[LINEAR_ACCELERATION_NAME] == DiagnosticStatus.OK
    assert levels[RATE_NAME] != DiagnosticStatus.STALE


def test_in_order_but_old_timestamp_is_accepted_and_stays_stale(harness):
    first = make_reading(T1_NS)
    second = make_reading(T1_NS + 1)
    session = harness(ScriptedImuReader([first, second]), stale_timeout_sec=0.4)

    session.poll()
    session.poll()
    assert session.publisher._last_reading is second
    assert session.publisher._last_reading.stamp_ns == T1_NS + 1
    header = session.collector.raw[-1].header
    published_ns = header.stamp.sec * 10**9 + header.stamp.nanosec
    assert published_ns == T1_NS + 1

    session.report()
    assert session.last_levels()[HEARTBEAT_NAME] == DiagnosticStatus.STALE


def test_injected_reader_lifecycle_uses_start_and_stop():
    reader = ScriptedImuReader([])
    publisher = ImuPublisher(
        hardware_reader=reader,
        parameter_overrides=params(use_mock=False),
    )
    try:
        publisher.destroy_timer(publisher._sample_timer)
        publisher.destroy_timer(publisher._diagnostics_timer)
        assert reader.started is True
        assert reader.stopped is False
    finally:
        publisher.destroy_node()
    assert reader.stopped is True


def test_injected_reader_does_not_get_unconfigured_driver_warning():
    reader = ScriptedImuReader([])
    with patch.object(RcutilsLogger, 'warning') as warning:
        publisher = ImuPublisher(
            hardware_reader=reader,
            parameter_overrides=params(use_mock=False),
        )
    try:
        warning.assert_not_called()
    finally:
        publisher.destroy_node()


def test_unconfigured_reader_still_gets_missing_driver_warning():
    with patch.object(RcutilsLogger, 'warning') as warning:
        publisher = ImuPublisher(parameter_overrides=params(use_mock=False))
    try:
        warning.assert_called_once()
        assert 'no hardware driver is implemented' in warning.call_args.args[0]
    finally:
        publisher.destroy_node()


def test_accepted_raw_sample_keeps_pr11_orientation_and_covariance(harness):
    session = harness(ScriptedImuReader([make_reading(T2_NS)]))
    session.poll()
    message = session.collector.raw[0]
    assert message.header.frame_id == 'imu_link'
    assert message.orientation_covariance[0] == ORIENTATION_UNAVAILABLE
    assert list(message.angular_velocity_covariance) == UNKNOWN_COVARIANCE
    assert list(message.linear_acceleration_covariance) == UNKNOWN_COVARIANCE
    assert session.collector.demo[0].orientation_covariance[0] == pytest.approx(0.05 ** 2)
