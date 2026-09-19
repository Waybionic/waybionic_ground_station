"""Unit tests for the hardware-independent IMU reading gate."""

from types import SimpleNamespace

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


def test_malformed_stamp_on_a_controlled_object_is_invalid():
    candidate = SimpleNamespace(
        stamp_ns=float('nan'),
        angular_velocity=(0.0, 0.0, 0.0),
        linear_acceleration=(0.0, 0.0, GRAVITY_M_S2),
        orientation=None,
    )
    assert invalid_reading_reason(candidate) == REASON_INVALID


def test_out_of_order_timestamp_is_rejected():
    older = make_reading(stamp_ns=T1_NS)
    assert rejection_reason(older, last_accepted_stamp_ns=T2_NS) == REASON_OUT_OF_ORDER


def test_equal_timestamp_is_not_out_of_order():
    same = make_reading(stamp_ns=T2_NS)
    assert rejection_reason(same, last_accepted_stamp_ns=T2_NS) is None


def test_old_but_in_order_timestamp_is_accepted():
    later_but_old = make_reading(stamp_ns=T1_NS + 1)
    assert rejection_reason(later_but_old, last_accepted_stamp_ns=T1_NS) is None
