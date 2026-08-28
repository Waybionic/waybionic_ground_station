"""
Synthetic IMU sample generation for benchtop work without hardware.

Kept free of ROS types so the motion model can be unit tested directly and so
the node does not mix data generation with message construction.
"""

import math
from typing import Optional

from waybionic_sensors.imu_reading import GRAVITY_M_S2, ImuReading, Quaternion


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert intrinsic roll/pitch/yaw in radians to an ``(x, y, z, w)`` quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class MockImuSource:
    """
    Generates a smooth, repeatable wobble so RViz and the panel have data.

    The motion is a deterministic function of elapsed time, which keeps tests
    stable and makes the output obviously synthetic rather than noise that could
    be mistaken for a real sensor.
    """

    def __init__(
        self,
        *,
        angular_amplitude: float = 0.20,
        linear_amplitude: float = 0.05,
        stall_after_sec: float = 0.0,
    ) -> None:
        """
        Configure the generator.

        ``angular_amplitude`` is the peak angular velocity in rad/s.

        ``linear_amplitude`` is the peak lateral acceleration in m/s^2, on top
        of gravity.

        ``stall_after_sec``, when positive, stops sample production that many
        seconds after the first one. It exists to demonstrate the stale
        heartbeat path without unplugging anything.
        """
        self._angular_amplitude = angular_amplitude
        self._linear_amplitude = linear_amplitude
        self._stall_after_sec = stall_after_sec
        self._start_ns: Optional[int] = None

    @property
    def stalled_deliberately(self) -> bool:
        """Return whether a stall was configured for stale-path demonstrations."""
        return self._stall_after_sec > 0.0

    def elapsed_sec(self, stamp_ns: int) -> float:
        """Return seconds since the first sample, given the current timestamp."""
        if self._start_ns is None:
            return 0.0
        return (stamp_ns - self._start_ns) / 1e9

    def read(self, stamp_ns: int) -> Optional[ImuReading]:
        """Produce the sample for ``stamp_ns``, or ``None`` once stalled."""
        if self._start_ns is None:
            self._start_ns = stamp_ns

        elapsed = self.elapsed_sec(stamp_ns)
        if self._stall_after_sec > 0.0 and elapsed > self._stall_after_sec:
            return None

        slow = math.sin(elapsed * 0.5)
        medium = math.cos(elapsed * 0.8)

        angular_velocity = (
            self._angular_amplitude * 0.5 * slow,
            self._angular_amplitude * 0.25 * medium,
            self._angular_amplitude * slow,
        )
        linear_acceleration = (
            self._linear_amplitude * slow,
            self._linear_amplitude * 0.4 * medium,
            GRAVITY_M_S2,
        )

        return ImuReading(
            stamp_ns=stamp_ns,
            angular_velocity=angular_velocity,
            linear_acceleration=linear_acceleration,
            orientation=None,
        )

    def demo_orientation(self, stamp_ns: int) -> Quaternion:
        """
        Return a synthetic orientation for the demo topic and demo TF.

        This is a display aid only. It is not derived from the accelerometer or
        gyroscope values above and must never be published on the raw topic.
        """
        elapsed = self.elapsed_sec(stamp_ns)
        roll = 0.10 * math.sin(elapsed * 0.5)
        pitch = 0.05 * math.sin(elapsed * 0.33)
        yaw = 0.20 * math.sin(elapsed * 0.5)
        return euler_to_quaternion(roll, pitch, yaw)
