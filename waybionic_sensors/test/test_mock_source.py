"""Tests for the synthetic sample generator."""

import math

from waybionic_sensors.imu_reading import GRAVITY_M_S2
from waybionic_sensors.mock_source import euler_to_quaternion, MockImuSource

START_NS = 5_000_000_000
SECOND_NS = 1_000_000_000


def test_first_read_returns_a_sample():
    assert MockImuSource().read(START_NS) is not None


def test_reading_uses_the_supplied_timestamp():
    reading = MockImuSource().read(START_NS)
    assert reading.stamp_ns == START_NS


def test_mock_never_claims_an_orientation():
    # The generator models an accelerometer and gyroscope, which cannot observe
    # absolute attitude. Demo orientation is a separate, explicit call.
    reading = MockImuSource().read(START_NS)
    assert reading.orientation is None
    assert reading.has_orientation is False


def test_acceleration_includes_gravity_on_z():
    reading = MockImuSource().read(START_NS)
    assert reading.linear_acceleration[2] == GRAVITY_M_S2


def test_values_stay_within_the_configured_amplitudes():
    source = MockImuSource(angular_amplitude=0.2, linear_amplitude=0.05)
    for step in range(200):
        reading = source.read(START_NS + step * SECOND_NS // 10)
        assert all(abs(value) <= 0.2 + 1e-9 for value in reading.angular_velocity)
        assert abs(reading.linear_acceleration[0]) <= 0.05 + 1e-9


def test_generator_is_deterministic():
    first = MockImuSource().read(START_NS)
    second = MockImuSource().read(START_NS)
    assert first.angular_velocity == second.angular_velocity
    assert first.linear_acceleration == second.linear_acceleration


def test_motion_changes_over_time():
    source = MockImuSource()
    source.read(START_NS)
    later = source.read(START_NS + 2 * SECOND_NS)
    assert later.angular_velocity != (0.0, 0.0, 0.0)


def test_stall_is_disabled_by_default():
    source = MockImuSource()
    source.read(START_NS)
    assert source.stalled_deliberately is False
    assert source.read(START_NS + 100 * SECOND_NS) is not None


def test_stall_stops_samples_after_the_configured_delay():
    source = MockImuSource(stall_after_sec=2.0)
    assert source.stalled_deliberately is True
    assert source.read(START_NS) is not None
    assert source.read(START_NS + 1 * SECOND_NS) is not None
    assert source.read(START_NS + 3 * SECOND_NS) is None


def test_elapsed_is_measured_from_the_first_sample():
    source = MockImuSource()
    source.read(START_NS)
    assert source.elapsed_sec(START_NS + 3 * SECOND_NS) == 3.0


def test_demo_orientation_is_a_unit_quaternion():
    source = MockImuSource()
    source.read(START_NS)
    x, y, z, w = source.demo_orientation(START_NS + SECOND_NS)
    assert abs(math.sqrt(x * x + y * y + z * z + w * w) - 1.0) < 1e-9


def test_euler_to_quaternion_returns_identity_for_zero_rotation():
    assert euler_to_quaternion(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)


def test_euler_to_quaternion_matches_a_known_yaw():
    _, _, z, w = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert abs(z - math.sqrt(0.5)) < 1e-9
    assert abs(w - math.sqrt(0.5)) < 1e-9
