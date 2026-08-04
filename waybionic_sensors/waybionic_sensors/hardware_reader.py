"""
Boundary for a future physical IMU.

The sensor model, transport and packet format are not confirmed yet, so this
module defines only the interface a driver has to satisfy. Nothing here invents
a wire protocol. See ``docs/HARDWARE_INTERFACE.md`` for the questions that must
be answered by electrical before a real reader is written.

To add hardware later, implement :class:`ImuHardwareReader` in its own module
and hand an instance to the node. No change to message construction,
diagnostics, or TF publishing is required.
"""

from abc import ABC, abstractmethod
from typing import Optional

from waybionic_sensors.imu_reading import ImuReading


class ImuHardwareReader(ABC):
    """
    Reads samples from a physical IMU.

    Implementations own transport and parsing only. They must return values
    already converted into the REP-103 units of :class:`ImuReading`, so the unit
    and axis conversion for a specific sensor stays inside its own driver.
    """

    @abstractmethod
    def start(self) -> None:
        """Acquire the device. Raise if the device cannot be opened."""

    @abstractmethod
    def read(self, stamp_ns: int) -> Optional[ImuReading]:
        """
        Return the newest sample, or ``None`` if none is available yet.

        ``stamp_ns`` is the fallback acquisition time to use when the device
        does not supply its own timestamp.
        """

    @abstractmethod
    def stop(self) -> None:
        """Release the device. Must be safe to call when never started."""

    @abstractmethod
    def describe(self) -> str:
        """Return a short human-readable description for logs and diagnostics."""


class UnconfiguredImuReader(ImuHardwareReader):
    """
    Stands in for the real driver until electrical confirms the sensor.

    Never produces samples. Live mode with this reader is still useful: the
    heartbeat goes stale in the diagnostics panel, which is exactly what an
    absent or disconnected sensor should look like.
    """

    def __init__(self, serial_port: str = '') -> None:
        """Record the configured port, if any, purely for the description."""
        self._serial_port = serial_port

    def start(self) -> None:
        """Do nothing. There is no device to acquire."""

    def read(self, stamp_ns: int) -> Optional[ImuReading]:
        """Return ``None`` because no hardware interface exists yet."""
        return None

    def stop(self) -> None:
        """Do nothing. There is no device to release."""

    def describe(self) -> str:
        """Return a description that makes the missing driver obvious in logs."""
        if self._serial_port:
            return (
                f'unconfigured IMU driver (port {self._serial_port}); '
                'awaiting sensor model and packet format from electrical'
            )
        return (
            'unconfigured IMU driver; awaiting sensor model, transport and '
            'packet format from electrical'
        )
