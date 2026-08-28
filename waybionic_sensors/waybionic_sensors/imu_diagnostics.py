"""
Builds the ``/diagnostics`` array describing IMU health.

Signal names, ``value``/``unit`` keys and level mapping follow
``waybionic_rviz_plugins/docs/DIAGNOSTICS_BACKEND_INTEGRATION.md`` so the merged
DiagnosticsPanel renders these rows without any camera- or IMU-specific code.

Separated from the node so the freshness and level logic can be tested without
spinning ROS.
"""

import math
from typing import Optional

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from waybionic_sensors.imu_messages import to_time_msg
from waybionic_sensors.imu_reading import ImuReading

HEARTBEAT_NAME = 'imu.heartbeat'
RATE_NAME = 'imu.rate'
ANGULAR_VELOCITY_NAME = 'imu.angular_velocity'
LINEAR_ACCELERATION_NAME = 'imu.linear_acceleration'

HARDWARE_ID = 'waybionic_sensors/imu'


def _key_values(value: str, unit: str) -> list:
    """Return the ``value``/``unit`` pair the panel looks for first."""
    return [KeyValue(key='value', value=value), KeyValue(key='unit', value=unit)]


def _magnitude(vector) -> float:
    """Return the Euclidean norm of a 3-tuple."""
    return math.sqrt(sum(component * component for component in vector))


class ImuDiagnosticsBuilder:
    """
    Turns the publisher's recent activity into diagnostic statuses.

    ``stale_timeout_sec`` is the sample age past which heartbeat, rate, and
    telemetry rows are reported STALE. It should comfortably exceed one publish
    period so ordinary jitter does not flap the status.

    ``expected_rate_hz`` is the configured publish rate, reported alongside the
    measured rate so a reviewer can see shortfalls.
    """

    def __init__(self, *, stale_timeout_sec: float = 1.0, expected_rate_hz: float = 50.0) -> None:
        """Store the freshness threshold and the configured rate."""
        self._stale_timeout_sec = stale_timeout_sec
        self._expected_rate_hz = expected_rate_hz

    @property
    def stale_timeout_sec(self) -> float:
        """Return the age after which the heartbeat is considered stale."""
        return self._stale_timeout_sec

    def build(
        self,
        now_ns: int,
        last_reading: Optional[ImuReading],
        measured_rate_hz: float,
        *,
        source_description: str,
    ) -> DiagnosticArray:
        """
        Assemble the diagnostics for the current moment.

        ``now_ns`` is the current time in nanoseconds, ``last_reading`` the most
        recent sample or ``None`` if none has arrived, ``measured_rate_hz`` the
        publish rate observed since the last report, and ``source_description``
        a short description of the active data source.
        """
        sample_is_stale = self._is_stale(now_ns, last_reading)

        array = DiagnosticArray()
        array.header.stamp = to_time_msg(now_ns)
        array.status.append(
            self._heartbeat_status(now_ns, last_reading, source_description)
        )
        array.status.append(
            self._rate_status(measured_rate_hz, last_reading is not None, sample_is_stale)
        )

        if last_reading is not None:
            array.status.append(
                self._value_status(
                    ANGULAR_VELOCITY_NAME,
                    _magnitude(last_reading.angular_velocity),
                    'rad/s',
                    'Gyroscope magnitude',
                    stale=sample_is_stale,
                )
            )
            array.status.append(
                self._value_status(
                    LINEAR_ACCELERATION_NAME,
                    _magnitude(last_reading.linear_acceleration),
                    'm/s^2',
                    'Accelerometer magnitude, including gravity',
                    stale=sample_is_stale,
                )
            )

        return array

    def _is_stale(self, now_ns: int, last_reading: Optional[ImuReading]) -> bool:
        """Return True when no sample exists or the newest sample is too old."""
        if last_reading is None:
            return True
        age_sec = max(0.0, (now_ns - last_reading.stamp_ns) / 1e9)
        return age_sec > self._stale_timeout_sec

    def _heartbeat_status(
        self,
        now_ns: int,
        last_reading: Optional[ImuReading],
        source_description: str,
    ) -> DiagnosticStatus:
        """Build the ``imu.heartbeat`` row required by issue #4."""
        status = DiagnosticStatus()
        status.name = HEARTBEAT_NAME
        status.hardware_id = HARDWARE_ID

        if last_reading is None:
            status.level = DiagnosticStatus.STALE
            status.message = f'No IMU samples received from {source_description}'
            status.values = _key_values('never', 's')
            return status

        age_sec = max(0.0, (now_ns - last_reading.stamp_ns) / 1e9)
        status.values = _key_values(f'{age_sec:.2f}', 's')

        if age_sec > self._stale_timeout_sec:
            status.level = DiagnosticStatus.STALE
            status.message = (
                f'No IMU sample for {age_sec:.2f} s '
                f'(timeout {self._stale_timeout_sec:.2f} s)'
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'IMU streaming from {source_description}'

        return status

    def _rate_status(
        self,
        measured_rate_hz: float,
        has_data: bool,
        sample_is_stale: bool,
    ) -> DiagnosticStatus:
        """Build the ``imu.rate`` row comparing measured against expected rate."""
        status = DiagnosticStatus()
        status.name = RATE_NAME
        status.hardware_id = HARDWARE_ID
        status.values = _key_values(f'{measured_rate_hz:.1f}', 'Hz')

        if not has_data or sample_is_stale:
            status.level = DiagnosticStatus.STALE
            status.message = (
                'No IMU samples to measure'
                if not has_data
                else (
                    f'Publishing at {measured_rate_hz:.1f} Hz; '
                    'IMU samples are stale'
                )
            )
            return status

        # Anything below about 80% of the configured rate is worth surfacing but
        # is not a fault on its own, since the publisher is still alive.
        if measured_rate_hz < self._expected_rate_hz * 0.8:
            status.level = DiagnosticStatus.WARN
            status.message = (
                f'Publishing at {measured_rate_hz:.1f} Hz, '
                f'below the configured {self._expected_rate_hz:.1f} Hz'
            )
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Publishing at {measured_rate_hz:.1f} Hz'

        return status

    def _value_status(
        self,
        name: str,
        value: float,
        unit: str,
        message: str,
        *,
        stale: bool = False,
    ) -> DiagnosticStatus:
        """Build a telemetry row; mark it STALE when the last sample is old."""
        status = DiagnosticStatus()
        status.name = name
        status.hardware_id = HARDWARE_ID
        status.values = _key_values(f'{value:.3f}', unit)
        if stale:
            status.level = DiagnosticStatus.STALE
            status.message = f'{message} (stale)'
        else:
            status.level = DiagnosticStatus.OK
            status.message = message
        return status
