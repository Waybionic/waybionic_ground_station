"""Unit tests for the hardware-independent IMU reading gate."""

from types import SimpleNamespace

import pytest

from waybionic_sensors.imu_reading import GRAVITY_M_S2, ImuReading
from waybionic_sensors.imu_sample_validation import (
    invalid_reading_reason,
    REASON_INVALID,
    REASON_NO_SAMPLE,
    REASON_OUT_OF_ORDER,
    rejection_reason,
)

T1_NS = 1_000_000_000
T2_NS = 2_000_000_000
ROS_TIME_MIN_NS = -(2 ** 31) * 1_000_000_000
ROS_TIME_MAX_NS = (2 ** 31) * 1_000_000_000 - 1


def make_reading(
    stamp_ns=T2_NS,
    angular_velocity=(0.1, 0.0, -0.2),
    linear_acceleration=(0.0, 0.0, GRAVITY_M_S2),
    orientation=None,
):
    """Build a candidate reading with overridable fields."""
    return ImuReading(
        stamp_ns=stamp_ns,
        angular_velocity=angular_velocity,
        linear_acceleration=linear_acceleration,
        orientation=orientation,
    )


def test_none_is_classified_as_no_sample():
    assert rejection_reason(None) == REASON_NO_SAMPLE


def test_valid_reading_is_accepted():
    assert rejection_reason(make_reading()) is None


def test_valid_reading_with_orientation_is_accepted():
    assert rejection_reason(make_reading(orientation=(0.0, 0.0, 0.0, 1.0))) is None


@pytest.mark.parametrize('component', [0, 1, -1, 0.25, -3.5])
def test_normal_int_and_float_components_are_accepted(component):
    reading = make_reading(angular_velocity=(component, 0.0, 0.0))
    assert invalid_reading_reason(reading) is None


def test_bool_component_is_invalid():
    reading = make_reading(angular_velocity=(True, 0.0, 0.0))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_integer_too_large_for_float_is_invalid():
    reading = make_reading(angular_velocity=(10 ** 1000, 0.0, 0.0))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_numeric_conversion_exception_is_rejected_not_raised():
    class MalformedFloat(float):
        def __float__(self):
            raise RuntimeError('cannot convert')

    reading = make_reading(angular_velocity=(MalformedFloat(1.0), 0.0, 0.0))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_nan_angular_velocity_is_invalid():
    reading = make_reading(angular_velocity=(float('nan'), 0.0, 0.0))
    assert invalid_reading_reason(reading) == REASON_INVALID
    assert rejection_reason(reading, last_accepted_stamp_ns=T1_NS) == REASON_INVALID


def test_positive_infinity_acceleration_is_invalid():
    reading = make_reading(linear_acceleration=(0.0, float('inf'), GRAVITY_M_S2))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_negative_infinity_acceleration_is_invalid():
    reading = make_reading(linear_acceleration=(0.0, 0.0, float('-inf')))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_non_finite_orientation_component_is_invalid():
    reading = make_reading(orientation=(0.0, float('nan'), 0.0, 1.0))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_malformed_angular_velocity_shape_is_invalid():
    reading = make_reading(angular_velocity=(0.0, 0.0))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_malformed_acceleration_type_is_invalid():
    reading = make_reading(linear_acceleration='not-a-vector')
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_malformed_orientation_shape_is_invalid():
    reading = make_reading(orientation=(0.0, 0.0, 1.0))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_non_reading_object_with_valid_looking_fields_is_invalid():
    candidate = SimpleNamespace(
        stamp_ns=T2_NS,
        angular_velocity=(0.0, 0.0, 0.0),
        linear_acceleration=(0.0, 0.0, GRAVITY_M_S2),
        orientation=None,
    )
    assert invalid_reading_reason(candidate) == REASON_INVALID


def test_one_shot_vector_iterable_is_invalid_without_being_consumed():
    vector = (component for component in (0.0, 0.0, 0.0))
    reading = make_reading(angular_velocity=vector)
    assert invalid_reading_reason(reading) == REASON_INVALID
    assert tuple(vector) == (0.0, 0.0, 0.0)


@pytest.mark.parametrize('stamp_ns', [0, T2_NS, ROS_TIME_MIN_NS, ROS_TIME_MAX_NS])
def test_ros_time_representable_timestamp_is_accepted(stamp_ns):
    assert invalid_reading_reason(make_reading(stamp_ns=stamp_ns)) is None


@pytest.mark.parametrize(
    'stamp_ns',
    [ROS_TIME_MIN_NS - 1, ROS_TIME_MAX_NS + 1, float('nan'), True],
)
def test_unrepresentable_or_malformed_timestamp_is_invalid(stamp_ns):
    assert invalid_reading_reason(make_reading(stamp_ns=stamp_ns)) == REASON_INVALID


def test_validation_returns_rejection_for_weird_malformed_container():
    class ExplodingTuple(tuple):
        def __len__(self):
            raise RuntimeError('malformed container')

    reading = make_reading(angular_velocity=ExplodingTuple((0.0, 0.0, 0.0)))
    assert invalid_reading_reason(reading) == REASON_INVALID


def test_out_of_order_timestamp_is_rejected():
    older = make_reading(stamp_ns=T1_NS)
    assert rejection_reason(older, last_accepted_stamp_ns=T2_NS) == REASON_OUT_OF_ORDER


def test_equal_timestamp_is_not_out_of_order():
    same = make_reading(stamp_ns=T2_NS)
    assert rejection_reason(same, last_accepted_stamp_ns=T2_NS) is None


def test_old_but_in_order_timestamp_is_accepted():
    later_but_old = make_reading(stamp_ns=T1_NS + 1)
    assert rejection_reason(later_but_old, last_accepted_stamp_ns=T1_NS) is None
