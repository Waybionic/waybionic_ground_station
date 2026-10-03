"""Test-only in-memory IMU reader with a scriptable event sequence."""

from typing import List, Optional, Union

from waybionic_sensors.hardware_reader import ImuHardwareReader
from waybionic_sensors.imu_reading import ImuReading

ScriptEvent = Union[ImuReading, BaseException, None]


class ScriptedImuReader(ImuHardwareReader):
    """
    Deterministic ``ImuHardwareReader`` for publisher validation tests.

    Each ``read()`` consumes the next queued event: an :class:`ImuReading`,
    ``None``, or an exception instance that is raised. When the queue is empty
    the reader returns ``None``, which is the no-sample case.
    """

    def __init__(self, events: Optional[List[ScriptEvent]] = None) -> None:
        """Store the optional initial event sequence."""
        self._events: List[ScriptEvent] = list(events or [])
        self.started = False
        self.stopped = False
        self.read_calls = 0

    def enqueue(self, event: ScriptEvent) -> None:
        """Append one event to be consumed by a later ``read()``."""
        self._events.append(event)

    def start(self) -> None:
        """Record that the publisher acquired this test reader."""
        self.started = True

    def read(self, stamp_ns: int) -> Optional[ImuReading]:
        """Return or raise the next scripted event. ``stamp_ns`` is unused."""
        del stamp_ns
        self.read_calls += 1
        if not self._events:
            return None
        event = self._events.pop(0)
        if isinstance(event, BaseException):
            raise event
        return event

    def stop(self) -> None:
        """Record that the publisher released this test reader."""
        self.stopped = True

    def describe(self) -> str:
        """Return a description that makes the test seam obvious."""
        return 'scripted in-memory test reader'
