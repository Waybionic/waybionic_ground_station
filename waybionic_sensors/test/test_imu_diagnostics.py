"""Tests for the /diagnostics output, including heartbeat and stale behaviour."""

from diagnostic_msgs.msg import DiagnosticStatus

from waybionic_sensors.imu_diagnostics import (
    ANGULAR_VELOCITY_NAME,
    HEARTBEAT_NAME,
    ImuDiagnosticsBuilder,
    LINEAR_ACCELERATION_NAME,
    RATE_NAME,
)
from waybionic_sensors.imu_reading import GRAVITY_M_S2, ImuReading

NOW_NS = 10_000_000_000
SECOND_NS = 1_000_000_000


def make_reading(age_sec: float) -> ImuReading:
    """Build a reading that is ``age_sec`` old relative to ``NOW_NS``."""
    return ImuReading(
        stamp_ns=NOW_NS - int(age_sec * SECOND_NS),
        angular_velocity=(0.0, 0.0, 3.0),
        linear_acceleration=(0.0, 0.0, GRAVITY_M_S2),
    )


def build(last_reading, measured_rate_hz=50.0, stale_timeout_sec=1.0):
    """Run the builder with the defaults used across these tests."""
    builder = ImuDiagnosticsBuilder(
        stale_timeout_sec=stale_timeout_sec, expected_rate_hz=50.0
    )
    return builder.build(
        NOW_NS, last_reading, measured_rate_hz, source_description='mock generator'
    )


def status_named(array, name):
    """Return the single status with ``name``, or None."""
    matches = [status for status in array.status if status.name == name]
    return matches[0] if matches else None


def value_of(status):
    """Return the ``value`` key of a status."""
    return next(entry.value for entry in status.values if entry.key == 'value')


def unit_of(status):
    """Return the ``unit`` key of a status."""
    return next(entry.value for entry in status.values if entry.key == 'unit')


def test_heartbeat_is_always_published():
    assert status_named(build(make_reading(0.0)), HEARTBEAT_NAME) is not None
    assert status_named(build(None), HEARTBEAT_NAME) is not None


def test_heartbeat_is_ok_when_samples_are_fresh():
    status = status_named(build(make_reading(0.02)), HEARTBEAT_NAME)
    assert status.level == DiagnosticStatus.OK


def test_heartbeat_goes_stale_past_the_timeout():
    status = status_named(build(make_reading(2.5)), HEARTBEAT_NAME)
    assert status.level == DiagnosticStatus.STALE
    assert 'timeout' in status.message


def test_heartbeat_respects_a_custom_timeout():
    fresh = status_named(build(make_reading(1.5), stale_timeout_sec=3.0), HEARTBEAT_NAME)
    stale = status_named(build(make_reading(1.5), stale_timeout_sec=0.5), HEARTBEAT_NAME)
    assert fresh.level == DiagnosticStatus.OK
    assert stale.level == DiagnosticStatus.STALE


def test_heartbeat_is_stale_when_no_sample_ever_arrived():
    status = status_named(build(None), HEARTBEAT_NAME)
    assert status.level == DiagnosticStatus.STALE
    assert value_of(status) == 'never'


def test_heartbeat_reports_age_in_seconds():
    status = status_named(build(make_reading(0.25)), HEARTBEAT_NAME)
    assert unit_of(status) == 's'
    assert float(value_of(status)) == 0.25


def test_heartbeat_names_the_active_source():
    status = status_named(build(make_reading(0.0)), HEARTBEAT_NAME)
    assert 'mock generator' in status.message


def test_statuses_carry_a_hardware_id():
    for status in build(make_reading(0.0)).status:
        assert status.hardware_id


def test_rate_row_reports_measured_hz():
    status = status_named(build(make_reading(0.0), measured_rate_hz=49.7), RATE_NAME)
    assert unit_of(status) == 'Hz'
    assert float(value_of(status)) == 49.7
    assert status.level == DiagnosticStatus.OK


def test_rate_row_warns_when_well_below_the_configured_rate():
    status = status_named(build(make_reading(0.0), measured_rate_hz=10.0), RATE_NAME)
    assert status.level == DiagnosticStatus.WARN


def test_rate_row_is_stale_without_samples():
    status = status_named(build(None, measured_rate_hz=0.0), RATE_NAME)
    assert status.level == DiagnosticStatus.STALE


def test_telemetry_rows_use_unambiguous_units():
    array = build(make_reading(0.0))
    assert unit_of(status_named(array, ANGULAR_VELOCITY_NAME)) == 'rad/s'
    assert unit_of(status_named(array, LINEAR_ACCELERATION_NAME)) == 'm/s^2'


def test_telemetry_rows_report_vector_magnitudes():
    array = build(make_reading(0.0))
    assert float(value_of(status_named(array, ANGULAR_VELOCITY_NAME))) == 3.0
    assert abs(
        float(value_of(status_named(array, LINEAR_ACCELERATION_NAME))) - GRAVITY_M_S2
    ) < 0.001


def test_telemetry_rows_are_omitted_before_any_sample():
    array = build(None)
    assert status_named(array, ANGULAR_VELOCITY_NAME) is None
    assert status_named(array, LINEAR_ACCELERATION_NAME) is None


def test_array_carries_a_header_stamp():
    array = build(make_reading(0.0))
    assert array.header.stamp.sec == NOW_NS // SECOND_NS


def test_no_orientation_signals_are_published_from_raw_data():
    # imu.roll/pitch/yaw only become meaningful once a real fusion source
    # exists; publishing them from raw data would misrepresent the sensor.
    names = {status.name for status in build(make_reading(0.0)).status}
    assert not {'imu.roll', 'imu.pitch', 'imu.yaw'} & names
