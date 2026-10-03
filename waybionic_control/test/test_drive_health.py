"""ROS-independent tests for drive-health state evaluation."""

import unittest

from waybionic_control.drive_health import (
    DriveHealthMonitor,
    DriveHealthState,
    DriveHealthThresholds,
)


class TestDriveHealthMonitor(unittest.TestCase):
    """Exercise state decisions and threshold boundaries."""

    def setUp(self):
        self.monitor = DriveHealthMonitor()

    def evaluate(self, **overrides):
        values = {
            'target_position': 0.0,
            'encoder_position': 0.0,
            'motion_commanded': False,
            'enabled': True,
            'reply_age_seconds': 0.0,
            'now_seconds': 0.0,
        }
        values.update(overrides)
        return self.monitor.evaluate(**values)

    def test_ok(self):
        self.assertIs(self.evaluate(), DriveHealthState.OK)

    def test_following_error(self):
        state = self.evaluate(target_position=5.01)
        self.assertIs(state, DriveHealthState.FOLLOWING_ERROR)

    def test_stalled_after_low_movement_window(self):
        self.evaluate(
            target_position=2.0,
            motion_commanded=True,
            now_seconds=0.0)
        state = self.evaluate(
            target_position=2.0,
            encoder_position=0.25,
            motion_commanded=True,
            now_seconds=1.999)
        self.assertIs(state, DriveHealthState.OK)
        state = self.evaluate(
            target_position=2.0,
            encoder_position=0.5,
            motion_commanded=True,
            now_seconds=2.0)
        self.assertIs(state, DriveHealthState.STALLED)

    def test_not_responding(self):
        state = self.evaluate(reply_age_seconds=0.5001)
        self.assertIs(state, DriveHealthState.NOT_RESPONDING)

    def test_disabled(self):
        state = self.evaluate(enabled=False)
        self.assertIs(state, DriveHealthState.DISABLED)

    def test_exact_following_error_boundary_is_ok(self):
        state = self.evaluate(target_position=5.0)
        self.assertIs(state, DriveHealthState.OK)

    def test_exact_reply_timeout_boundary_is_not_stale(self):
        state = self.evaluate(reply_age_seconds=0.5)
        self.assertIs(state, DriveHealthState.OK)

    def test_exact_stall_movement_boundary_is_not_stalled(self):
        self.evaluate(
            target_position=1.0,
            motion_commanded=True,
            now_seconds=0.0)
        state = self.evaluate(
            target_position=1.0,
            encoder_position=1.0,
            motion_commanded=True,
            now_seconds=2.0)
        self.assertIs(state, DriveHealthState.OK)

    def test_motion_command_required_for_stall(self):
        self.evaluate(motion_commanded=False, now_seconds=0.0)
        state = self.evaluate(motion_commanded=False, now_seconds=10.0)
        self.assertIs(state, DriveHealthState.OK)

    def test_exact_rolling_window_movement_boundary_is_not_stalled(self):
        self.evaluate(
            target_position=2.0,
            motion_commanded=True,
            now_seconds=0.0)
        self.evaluate(
            target_position=2.0,
            encoder_position=2.0,
            motion_commanded=True,
            now_seconds=0.2)
        state = self.evaluate(
            target_position=2.0,
            encoder_position=2.0,
            motion_commanded=True,
            now_seconds=2.1)
        self.assertIs(state, DriveHealthState.OK)

    def test_cumulative_movement_counts_out_and_back_travel(self):
        self.evaluate(
            target_position=0.6,
            motion_commanded=True,
            now_seconds=0.0)
        self.evaluate(
            target_position=0.6,
            encoder_position=0.6,
            motion_commanded=True,
            now_seconds=1.0)
        state = self.evaluate(
            motion_commanded=True,
            now_seconds=2.0)
        self.assertIs(state, DriveHealthState.OK)

    def test_disabled_precedes_other_conditions(self):
        state = self.evaluate(
            target_position=20.0,
            enabled=False,
            reply_age_seconds=1.0)
        self.assertIs(state, DriveHealthState.DISABLED)

    def test_not_responding_precedes_following_error(self):
        state = self.evaluate(
            target_position=20.0,
            reply_age_seconds=0.6)
        self.assertIs(state, DriveHealthState.NOT_RESPONDING)

    def test_following_error_precedes_stalled(self):
        self.evaluate(
            target_position=20.0,
            motion_commanded=True,
            now_seconds=0.0)
        state = self.evaluate(
            target_position=20.0,
            encoder_position=0.5,
            motion_commanded=True,
            now_seconds=2.0)
        self.assertIs(state, DriveHealthState.FOLLOWING_ERROR)

    def test_custom_thresholds(self):
        monitor = DriveHealthMonitor(DriveHealthThresholds(
            following_error_degrees=2.0,
            stall_movement_degrees=0.5,
            stall_duration_seconds=1.0,
            reply_timeout_seconds=0.25,
        ))
        state = monitor.evaluate(
            target_position=2.01,
            encoder_position=0.0,
            motion_commanded=False,
            enabled=True,
            reply_age_seconds=0.0,
            now_seconds=0.0,
        )
        self.assertIs(state, DriveHealthState.FOLLOWING_ERROR)


if __name__ == '__main__':
    unittest.main()
