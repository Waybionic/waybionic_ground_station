"""
Hardware-independent gate for candidate IMU readings.

Future drivers still own transport, parsing, unit conversion, and axis
mapping for a specific sensor. This module only decides whether a candidate
:class:`~waybionic_sensors.imu_reading.ImuReading` is safe to accept as
telemetry. It does not invent sensor values, clamp corrupt data, guess
covariance, or rewrite timestamps.
"""

import math
from numbers import Real
from typing import Optional

from waybionic_sensors.imu_reading import ImuReading

REASON_NO_SAMPLE = 'no sample'
REASON_INVALID = 'invalid or non-finite sample'
REASON_OUT_OF_ORDER = 'out-of-order timestamp'

# ``builtin_interfaces/msg/Time`` uses an int32 ``sec`` field and a uint32
# ``nanosec`` field. ``to_time_msg()`` normalises nanoseconds to
# 0..999,999,999, so these are the inclusive ImuReading.stamp_ns bounds that
# preserve the value through Jazzy CDR serialization.
_NANOSECONDS_PER_SECOND = 1_000_000_000
_TIME_SEC_MIN = -(2 ** 31)
_TIME_SEC_MAX = 2 ** 31 - 1
_STAMP_NS_MIN = _TIME_SEC_MIN * _NANOSECONDS_PER_SECOND
_STAMP_NS_MAX = (
    (_TIME_SEC_MAX + 1) * _NANOSECONDS_PER_SECOND - 1
)


def _is_finite_number(value) -> bool:
    """Return whether ``value`` is a finite real safely convertible to float."""
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def _is_finite_vector(value, length: int) -> bool:
    """Return whether ``value`` is a fixed tuple of finite real numbers."""
    if type(value) is not tuple:
        return False
    try:
        return (
            len(value) == length
            and all(_is_finite_number(value[index]) for index in range(length))
        )
    except Exception:
        return False


def _is_stamp_ns(value) -> bool:
    """Return whether ``value`` safely maps to ROS Time's int32 seconds."""
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and _STAMP_NS_MIN <= value <= _STAMP_NS_MAX
    )


def invalid_reading_reason(reading) -> Optional[str]:
    """
    Return why ``reading`` is malformed or non-finite, or ``None`` if usable.

    Checks ``stamp_ns``, the gyroscope vector, the accelerometer vector, and
    orientation when it is present. ``orientation is None`` is valid: raw
    samples are not required to carry a fused attitude.
    """
    if not isinstance(reading, ImuReading):
        return REASON_INVALID
    try:
        if not _is_stamp_ns(reading.stamp_ns):
            return REASON_INVALID
        if not _is_finite_vector(reading.angular_velocity, 3):
            return REASON_INVALID
        if not _is_finite_vector(reading.linear_acceleration, 3):
            return REASON_INVALID
        if (
            reading.orientation is not None
            and not _is_finite_vector(reading.orientation, 4)
        ):
            return REASON_INVALID
    except Exception:
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
