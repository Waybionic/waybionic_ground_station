"""The simulated drive follows the manual's speed, acceleration and heartbeat rules."""

import pytest

from waybionic_teleop import mks_can
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

RUNNING = mks_can.frame(1, mks_can.ABSOLUTE_AXIS, [1])
COMPLETE = mks_can.frame(1, mks_can.ABSOLUTE_AXIS, [2])


def ready_servo():
    servo = SimulatedServo(1)
    servo.receive(mks_can.set_mode(1))
    servo.receive(mks_can.enable(1))
    return servo


def run(servo, seconds, dt=0.001):
    replies = []
    for _ in range(round(seconds / dt)):
        replies += servo.step(dt)
    return replies


def test_moves_are_refused_until_mode_and_enable_are_set():
    assert SimulatedServo(1).receive(mks_can.absolute_axis(1, 0x4000, 600, 0)) == [
        mks_can.frame(1, mks_can.ABSOLUTE_AXIS, [0])]


def test_speed_limit_sets_the_travel_time():
    servo = ready_servo()
    assert servo.receive(mks_can.absolute_axis(1, 0x2000, 60, 0)) == [RUNNING]
    assert run(servo, 0.49) == []
    assert run(servo, 0.02) == [COMPLETE]
    assert servo.axis == 0x2000


def test_acceleration_follows_the_manual_ramp():
    servo = ready_servo()
    servo.receive(mks_can.absolute_axis(1, 50 * mks_can.COUNTS_PER_REV, 3000, 236))
    run(servo, 0.1)
    assert servo.rpm == pytest.approx(100.0, abs=1.0)


def test_encoder_reply_reports_the_position():
    servo = ready_servo()
    servo.receive(mks_can.absolute_axis(1, -1234, 300, 0))
    run(servo, 0.5)
    _, arguments = mks_can.parse(1, servo.receive(mks_can.read_encoder(1))[0])
    assert mks_can.encoder_value(arguments) == -1234


def test_heartbeat_stops_a_moving_motor_when_the_host_goes_quiet():
    servo = ready_servo()
    servo.receive(mks_can.set_heartbeat(1, 100))
    servo.receive(mks_can.absolute_axis(1, 10 * mks_can.COUNTS_PER_REV, 60, 0))
    run(servo, 0.2)
    assert (servo.heartbeat_stops, servo.rpm, servo.target) == (1, 0.0, None)


def test_bus_routes_replies_and_counts_bad_frames():
    bus = SimulatedBus([SimulatedServo(1), SimulatedServo(2)], 500000)
    bus.send(2, mks_can.read_encoder(2))
    assert bus.receive() == (2, mks_can.frame(2, mks_can.READ_ENCODER, bytes(6)))
    assert bus.receive() is None
    bus.send(1, mks_can.read_encoder(2))
    bus.send(9, mks_can.read_encoder(9))
    assert (bus.frames, bus.errors) == (4, 2)
