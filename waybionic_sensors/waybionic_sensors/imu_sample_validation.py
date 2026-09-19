"""
Hardware-independent gate for candidate IMU readings.

Future drivers still own transport, parsing, unit conversion, and axis
mapping for a specific sensor. This module only decides whether a candidate
:class:`~waybionic_sensors.imu_reading.ImuReading` is safe to accept as
telemetry. It does not invent sensor values, clamp corrupt data, guess
covariance, or rewrite timestamps.
"""

from typing import Optional

REASON_NO_SAMPLE = 'no sample'
REASON_INVALID = 'invalid or non-finite sample'
REASON_OUT_OF_ORDER = 'out-of-order timestamp'


def _is_finite_number(value) -> bool:
    """Return whether ``value`` is a non-bool int or finite float."""
    if isinstance(value, bool) or value is None:
        return False
    if isinstance(value, int):
        return True
    if isinstance(value, float):
        return value == value and value not in (float('inf'), float('-inf'))
    return False


def _is_finite_vector(value, length: int) -> bool:
    """Return whether ``value`` is a sequence of ``length`` finite numbers."""
    if isinstance(value, (str, bytes)) or value is None:
        return False
    try:
        components = tuple(value)
    except TypeError:
        return False
    if len(components) != length:
        return False
    return all(_is_finite_number(component) for component in components)


def _is_stamp_ns(value) -> bool:
    """Return whether ``value`` is an integer nanosecond timestamp."""
    return isinstance(value, int) and not isinstance(value, bool)


def invalid_reading_reason(reading) -> Optional[str]:
    """
    Return why ``reading`` is malformed or non-finite, or ``None`` if usable.

    Checks ``stamp_ns``, the gyroscope vector, the accelerometer vector, and
    orientation when it is present. ``orientation is None`` is valid: raw
    samples are not required to carry a fused attitude.
    """
    if not _is_stamp_ns(getattr(reading, 'stamp_ns', None)):
        return REASON_INVALID
    if not _is_finite_vector(getattr(reading, 'angular_velocity', None), 3):
        return REASON_INVALID
    if not _is_finite_vector(getattr(reading, 'linear_acceleration', None), 3):
        return REASON_INVALID
    orientation = getattr(reading, 'orientation', None)
    if orientation is not None and not _is_finite_vector(orientation, 4):
        return REASON_INVALID
    return None


def rejection_reason(reading, last_accepted_stamp_ns: Optional[int] = None) -> Optional[str]:
    """
    Return why a candidate must not be accepted, or ``None`` to accept it.

    ``reading is None`` means the reader had no new sample. That is not a
    crash and not a fabricated STALE sample; the previous valid reading is
    left untouched so diagnostics can age naturally.
    """
    if reading is None:
        return REASON_NO_SAMPLE
    invalid = invalid_reading_reason(reading)
    if invalid is not None:
        return invalid
    if (
        last_accepted_stamp_ns is not None
        and reading.stamp_ns < last_accepted_stamp_ns
    ):
        return REASON_OUT_OF_ORDER
    return None
