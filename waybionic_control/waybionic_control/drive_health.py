"""ROS-independent drive health evaluation for a single axis."""

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math


class DriveHealthState(Enum):
    """Possible drive health states."""

    OK = 'OK'
    FOLLOWING_ERROR = 'FOLLOWING_ERROR'
    STALLED = 'STALLED'
    NOT_RESPONDING = 'NOT_RESPONDING'
    DISABLED = 'DISABLED'


@dataclass(frozen=True)
class DriveHealthThresholds:
    """Configurable drive-health thresholds, expressed in degrees and seconds."""

    following_error_degrees: float = 5.0
    stall_movement_degrees: float = 1.0
    stall_duration_seconds: float = 2.0
    reply_timeout_seconds: float = 0.5

    def __post_init__(self):
        """Reject non-positive or non-finite threshold values."""
        values = (
            self.following_error_degrees,
            self.stall_movement_degrees,
            self.stall_duration_seconds,
            self.reply_timeout_seconds,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise ValueError('Drive health thresholds must be finite and positive')


class DriveHealthMonitor:
    """Evaluate health for one axis using target and reply data.

    Positions and encoder samples must be in degrees. ``reply_age_seconds`` is
    the age of the latest valid reply. ``now_seconds`` must use a monotonic
    clock and be consistent across calls to this monitor.
    """

    def __init__(self, thresholds=None):
        """Create a monitor with optional per-axis thresholds."""
        self.thresholds = thresholds or DriveHealthThresholds()
        self._encoder_samples = deque()

    def evaluate(
            self,
            target_position: float,
            encoder_position: float,
            motion_commanded: bool,
            enabled: bool,
            reply_age_seconds: float,
            now_seconds: float) -> DriveHealthState:
        """Return the highest-priority health state for the latest sample.

        State precedence is DISABLED, NOT_RESPONDING, FOLLOWING_ERROR,
        STALLED, then OK. A disabled or non-responding drive resets stall
        history so unobserved time is not counted as stalled movement.
        """
        values = (target_position, encoder_position, reply_age_seconds, now_seconds)
        if any(not math.isfinite(value) for value in values):
            raise ValueError('Drive health inputs must be finite')
        if reply_age_seconds < 0.0:
            raise ValueError('Reply age cannot be negative')

        if not enabled:
            self._reset_motion_window()
            return DriveHealthState.DISABLED

        if reply_age_seconds > self.thresholds.reply_timeout_seconds:
            self._reset_motion_window()
            return DriveHealthState.NOT_RESPONDING

        stalled = self._is_stalled(
            encoder_position, motion_commanded, now_seconds)
        following_error = abs(target_position - encoder_position)
        if following_error > self.thresholds.following_error_degrees:
            return DriveHealthState.FOLLOWING_ERROR
        if stalled:
            return DriveHealthState.STALLED
        return DriveHealthState.OK

    def _is_stalled(self, encoder_position, motion_commanded, now_seconds):
        if not motion_commanded:
            self._reset_motion_window()
            return False

        if (self._encoder_samples
                and now_seconds < self._encoder_samples[-1][0]):
            self._reset_motion_window()

        if (self._encoder_samples
                and now_seconds == self._encoder_samples[-1][0]):
            self._encoder_samples[-1] = (now_seconds, encoder_position)
        else:
            self._encoder_samples.append((now_seconds, encoder_position))

        window_start = now_seconds - self.thresholds.stall_duration_seconds
        while (len(self._encoder_samples) > 1
               and self._encoder_samples[1][0] <= window_start):
            self._encoder_samples.popleft()

        if self._encoder_samples[0][0] > window_start:
            return False

        samples = list(self._encoder_samples)
        if samples[0][0] < window_start:
            previous, current = samples[:2]
            interval = current[0] - previous[0]
            fraction = (window_start - previous[0]) / interval
            window_position = previous[1] + fraction * (current[1] - previous[1])
            movement = abs(current[1] - window_position)
            first_pair = 1
        else:
            movement = 0.0
            first_pair = 0

        movement += sum(
            abs(current[1] - previous[1])
            for previous, current in zip(
                samples[first_pair:], samples[first_pair + 1:]))
        movement_threshold = self.thresholds.stall_movement_degrees
        return (movement < movement_threshold
                and not math.isclose(movement, movement_threshold))

    def _reset_motion_window(self):
        self._encoder_samples.clear()
