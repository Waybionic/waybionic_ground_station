"""
Hardware-independent IMU sample passed between readers and ROS publishing.

This is the boundary type of the package. A mock generator or a future hardware
driver produces :class:`ImuReading` values, and everything downstream (ROS
message construction, diagnostics, TF) consumes only this type. Adding a real
sensor therefore means writing a reader that returns these values, not editing
the publisher.

Units and axes follow REP-103: right-handed, x forward, y left, z up, SI units.
"""

from dataclasses import dataclass
from typing import Optional, Tuple

Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]

GRAVITY_M_S2 = 9.80665
"""Standard gravity, the value a level accelerometer reports on its z axis."""


@dataclass(frozen=True)
class ImuReading:
    """
    One IMU sample.

    ``stamp_ns`` is the acquisition time in nanoseconds since the epoch of the
    clock that produced it.

    ``angular_velocity`` is the gyroscope reading in rad/s as ``(x, y, z)``.

    ``linear_acceleration`` is the accelerometer reading in m/s^2 as
    ``(x, y, z)``, including gravity, per the ``sensor_msgs/msg/Imu``
    convention.

    ``orientation`` is an absolute orientation as ``(x, y, z, w)``, or ``None``
    when the device provides no fused orientation. Raw accelerometer and
    gyroscope data alone never yield an absolute orientation, so this stays
    ``None`` unless a real fusion source fills it in.
    """

    stamp_ns: int
    angular_velocity: Vector3
    linear_acceleration: Vector3
    orientation: Optional[Quaternion] = None

    @property
    def has_orientation(self) -> bool:
        """Return whether this sample carries a fused absolute orientation."""
        return self.orientation is not None
