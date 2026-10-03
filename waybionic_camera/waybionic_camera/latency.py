"""Frame rate and delay over a sliding window (pure logic, no ROS)."""

from collections import deque
import math


class FrameStats:
    """Keep the frames of the last few seconds: when each was captured and received."""

    def __init__(self, window_s=5.0):
        self.window = window_s
        self.frames = deque()
        self.total = 0

    def add(self, captured_s, received_s):
        self.frames.append((captured_s, received_s))
        self.total += 1
        while self.frames and received_s - self.frames[0][1] > self.window:
            self.frames.popleft()

    def summary(self, now_s):
        """Return fps and delay in ms (mean, min, max and jitter), or None without frames."""
        while self.frames and now_s - self.frames[0][1] > self.window:
            self.frames.popleft()
        if not self.frames:
            return None
        delays = [1000.0 * (received - captured) for captured, received in self.frames]
        mean = sum(delays) / len(delays)
        span = self.frames[-1][1] - self.frames[0][1]
        return {
            'fps': (len(self.frames) - 1) / span if span > 0 else 0.0,
            'mean_ms': mean,
            'min_ms': min(delays),
            'max_ms': max(delays),
            # Jitter is how much the delay varies from frame to frame.
            'jitter_ms': math.sqrt(sum((delay - mean) ** 2 for delay in delays) / len(delays)),
        }
