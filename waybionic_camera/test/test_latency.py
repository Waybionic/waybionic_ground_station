"""Frame rate, delay and jitter over the sliding window."""

import pytest

from waybionic_camera.latency import FrameStats


def test_rate_delay_and_jitter_of_a_steady_stream():
    stats = FrameStats(window_s=5.0)
    for index in range(90):
        captured = 100.0 + index / 30.0
        # Alternating 35 and 45 ms of delay.
        stats.add(captured, captured + (0.035 if index % 2 else 0.045))
    summary = stats.summary(103.0)
    assert summary['fps'] == pytest.approx(30.0, rel=0.02)
    assert summary['mean_ms'] == pytest.approx(40.0, abs=0.2)
    assert (summary['min_ms'], summary['max_ms']) == pytest.approx((35.0, 45.0))
    assert summary['jitter_ms'] == pytest.approx(5.0, abs=0.1)
    assert stats.total == 90


def test_old_frames_leave_the_window():
    stats = FrameStats(window_s=1.0)
    stats.add(10.0, 10.5)
    stats.add(11.6, 11.62)
    assert stats.summary(11.7)['max_ms'] == pytest.approx(20.0)
    assert stats.summary(13.0) is None
