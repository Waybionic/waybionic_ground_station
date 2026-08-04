"""Tests for sensor_msgs/msg/Imu construction, especially raw/fused semantics."""

from waybionic_sensors.imu_messages import (
    build_demo_orientation_message,
    build_demo_transform,
    build_raw_imu_message,
    diagonal_covariance,
    ORIENTATION_UNAVAILABLE,
    to_time_msg,
)
from waybionic_sensors.imu_reading import GRAVITY_M_S2, ImuReading

STAMP_NS = 1_234_567_890_123
ANGULAR_STDDEV = 0.01
LINEAR_STDDEV = 0.05
ORIENTATION_STDDEV = 0.5


def make_reading(orientation=None) -> ImuReading:
    """Build a reading with distinguishable per-axis values."""
    return ImuReading(
        stamp_ns=STAMP_NS,
        angular_velocity=(0.1, 0.2, 0.3),
        linear_acceleration=(0.4, 0.5, GRAVITY_M_S2),
        orientation=orientation,
    )


def build_raw():
    """Build the raw message used by most assertions here."""
    return build_raw_imu_message(
        make_reading(),
        'imu_link',
        angular_velocity_stddev=ANGULAR_STDDEV,
        linear_acceleration_stddev=LINEAR_STDDEV,
    )


def test_to_time_msg_splits_seconds_and_nanoseconds():
    stamp = to_time_msg(1_500_000_000)
    assert stamp.sec == 1
    assert stamp.nanosec == 500_000_000


def test_diagonal_covariance_squares_the_standard_deviation():
    covariance = diagonal_covariance(0.5)
    assert covariance[0] == 0.25
    assert covariance[4] == 0.25
    assert covariance[8] == 0.25


def test_diagonal_covariance_leaves_cross_terms_zero():
    covariance = diagonal_covariance(0.5)
    off_diagonal = [covariance[i] for i in (1, 2, 3, 5, 6, 7)]
    assert off_diagonal == [0.0] * 6


def test_raw_message_uses_configured_frame():
    assert build_raw().header.frame_id == 'imu_link'


def test_raw_message_preserves_reading_timestamp():
    header = build_raw().header
    assert header.stamp.sec == STAMP_NS // 1_000_000_000
    assert header.stamp.nanosec == STAMP_NS % 1_000_000_000


def test_raw_message_marks_orientation_unavailable():
    message = build_raw()
    assert message.orientation_covariance[0] == ORIENTATION_UNAVAILABLE


def test_raw_message_uses_identity_orientation_placeholder():
    message = build_raw()
    assert (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
    ) == (0.0, 0.0, 0.0, 1.0)


def test_raw_message_ignores_orientation_present_on_the_reading():
    # Even if a device supplies a fused quaternion, the raw topic must not
    # advertise it, otherwise consumers cannot tell the two topics apart.
    message = build_raw_imu_message(
        make_reading(orientation=(0.1, 0.2, 0.3, 0.9)),
        'imu_link',
        angular_velocity_stddev=ANGULAR_STDDEV,
        linear_acceleration_stddev=LINEAR_STDDEV,
    )
    assert message.orientation_covariance[0] == ORIENTATION_UNAVAILABLE
    assert message.orientation.w == 1.0


def test_raw_message_copies_angular_velocity():
    message = build_raw()
    assert message.angular_velocity.x == 0.1
    assert message.angular_velocity.y == 0.2
    assert message.angular_velocity.z == 0.3


def test_raw_message_copies_linear_acceleration_including_gravity():
    message = build_raw()
    assert message.linear_acceleration.x == 0.4
    assert message.linear_acceleration.y == 0.5
    assert message.linear_acceleration.z == GRAVITY_M_S2


def test_raw_message_populates_measurement_covariances():
    message = build_raw()
    assert message.angular_velocity_covariance[0] == ANGULAR_STDDEV ** 2
    assert message.linear_acceleration_covariance[0] == LINEAR_STDDEV ** 2


def test_raw_message_covariances_are_not_left_at_zero():
    message = build_raw()
    assert any(value > 0.0 for value in message.angular_velocity_covariance)
    assert any(value > 0.0 for value in message.linear_acceleration_covariance)


def test_demo_message_carries_a_usable_orientation():
    message = build_demo_orientation_message(
        make_reading(),
        'imu_link',
        (0.0, 0.0, 0.3826834, 0.9238795),
        orientation_stddev=ORIENTATION_STDDEV,
        angular_velocity_stddev=ANGULAR_STDDEV,
        linear_acceleration_stddev=LINEAR_STDDEV,
    )
    assert message.orientation.z == 0.3826834
    assert message.orientation_covariance[0] == ORIENTATION_STDDEV ** 2
    assert message.orientation_covariance[0] != ORIENTATION_UNAVAILABLE


def test_demo_message_keeps_the_same_measurement_fields():
    message = build_demo_orientation_message(
        make_reading(),
        'imu_link',
        (0.0, 0.0, 0.0, 1.0),
        orientation_stddev=ORIENTATION_STDDEV,
        angular_velocity_stddev=ANGULAR_STDDEV,
        linear_acceleration_stddev=LINEAR_STDDEV,
    )
    assert message.angular_velocity.z == 0.3
    assert message.linear_acceleration.z == GRAVITY_M_S2
    assert message.header.frame_id == 'imu_link'


def test_demo_transform_links_parent_to_imu_frame():
    transform = build_demo_transform(
        STAMP_NS, 'base_link', 'imu_link', (0.0, 0.0, 0.0, 1.0)
    )
    assert transform.header.frame_id == 'base_link'
    assert transform.child_frame_id == 'imu_link'
    assert transform.transform.rotation.w == 1.0
    assert transform.header.stamp.sec == STAMP_NS // 1_000_000_000
