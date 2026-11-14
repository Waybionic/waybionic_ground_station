"""The simulated drive follows the manual's speed, acceleration and heartbeat rules."""

import threading

import pytest

from waybionic_teleop import mks_can, mks_drive_sim
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


def test_a_long_pause_still_trips_the_heartbeat():
    servo = ready_servo()
    servo.receive(mks_can.set_heartbeat(1, 500))
    servo.receive(mks_can.absolute_axis(1, 100 * mks_can.COUNTS_PER_REV, 60, 0))
    servo.step(2.0)
    # The motor ran for the 500 ms the heartbeat allows, then stopped.
    assert (servo.heartbeat_stops, servo.rpm, servo.target) == (1, 0.0, None)
    assert servo.axis == pytest.approx(0.5 * mks_can.COUNTS_PER_REV, rel=0.05)


def test_set_zero_makes_the_current_position_zero():
    servo = ready_servo()
    servo.receive(mks_can.absolute_axis(1, 5000, 300, 0))
    run(servo, 0.5)
    assert servo.receive(mks_can.set_zero(1)) == [mks_can.frame(1, mks_can.SET_ZERO, [1])]
    _, arguments = mks_can.parse(1, servo.receive(mks_can.read_encoder(1))[0])
    assert mks_can.encoder_value(arguments) == 0


def test_f5_stop_is_immediate_and_a_new_negative_target_rearms_the_drive():
    servo = ready_servo()
    assert servo.receive(mks_can.absolute_axis(1, 0x4000, 300, 0)) == [RUNNING]
    run(servo, 0.05)
    assert servo.rpm > 0.0 and servo.axis > 0.0
    held = servo.axis
    assert servo.receive(mks_can.absolute_axis(1, 0, 0, 0)) == [COMPLETE]
    assert servo.enabled and servo.target is None and servo.rpm == 0.0
    assert run(servo, 0.4) == []
    assert servo.axis == held
    assert servo.receive(mks_can.absolute_axis(1, -0x4000, 300, 0)) == [RUNNING]
    run(servo, 0.05)
    assert servo.target == -0x4000 and servo.rpm < 0.0 and servo.axis < held


@pytest.mark.parametrize('end', [mks_can.stop(1, 0), mks_can.emergency_stop(1), None])
def test_a_target_that_reverses_a_running_move_is_run_to_once_the_moves_stop(end):
    # Firmware V1.0.9 on the bench: the reversing target is kept while later targets steer the
    # move, and the drive runs back to it on its own after a stop or its last target.
    servo = ready_servo()
    servo.receive(mks_can.absolute_axis(1, 0x8000, 300, 0))
    run(servo, 0.05)
    assert servo.rpm > 0.0
    servo.receive(mks_can.absolute_axis(1, -0x1000, 300, 0))
    run(servo, 0.02)
    servo.receive(mks_can.absolute_axis(1, round(servo.axis) + 2000, 300, 0))
    if end is None:
        run(servo, 0.05)
    else:
        servo.receive(end)
    run(servo, 1.0)
    assert servo.axis == -0x1000 and servo.latched is None


def test_a_drive_that_stops_before_it_reverses_runs_only_to_its_last_target():
    servo = ready_servo()
    servo.receive(mks_can.absolute_axis(1, 0x8000, 300, 0))
    run(servo, 0.05)
    servo.receive(mks_can.stop(1, 0))
    servo.receive(mks_can.absolute_axis(1, -0x1000, 300, 0))
    run(servo, 1.0)
    held = servo.axis
    assert servo.receive(mks_can.stop(1, 0)) == [COMPLETE]
    run(servo, 1.0)
    assert servo.axis == held == -0x1000


def test_stalled_test_fault_keeps_replying_without_advancing_and_recovers():
    servo = ready_servo()
    initial_axis = servo.axis
    servo.set_test_fault('stalled')
    assert servo.receive(mks_can.absolute_axis(1, 0x4000, 60, 0)) == [RUNNING]
    run(servo, 0.1)
    assert servo.rpm > 0.0 and servo.target == 0x4000
    assert servo.axis == initial_axis
    reply = servo.receive(mks_can.read_encoder(1))
    _, arguments = mks_can.parse(1, reply[0])
    assert mks_can.encoder_value(arguments) == initial_axis

    servo.set_test_fault()
    run(servo, 0.1)
    assert servo.axis > initial_axis


def test_not_responding_test_fault_suppresses_replies_and_recovers():
    servo = ready_servo()
    servo.set_test_fault('not_responding')
    assert servo.receive(mks_can.read_encoder(1)) == []
    assert servo.receive(mks_can.absolute_axis(1, 0x4000, 60, 0)) == []
    run(servo, 0.1)
    assert servo.axis > 0

    servo.set_test_fault(None)
    assert servo.receive(mks_can.read_encoder(1)) == [
        mks_can.frame(1, mks_can.READ_ENCODER,
                      round(servo.axis).to_bytes(6, 'big', signed=True))]


def test_disabled_test_fault_exposes_enabled_state_and_recovers():
    servo = ready_servo()
    servo.set_test_fault('disabled')
    assert not servo.enabled
    assert servo.receive(mks_can.absolute_axis(1, 0x4000, 60, 0)) == [
        mks_can.frame(1, mks_can.ABSOLUTE_AXIS, [0])]
    run(servo, 0.1)
    assert servo.axis == 0.0

    servo.set_test_fault()
    assert servo.enabled
    assert servo.receive(mks_can.absolute_axis(1, 0x4000, 60, 0)) == [RUNNING]
    run(servo, 0.1)
    assert servo.axis > 0.0


def test_unknown_test_fault_is_rejected():
    with pytest.raises(ValueError, match='unknown simulated drive test fault'):
        SimulatedServo(1).set_test_fault('other')


def test_f7_emergency_stop_is_immediate():
    servo = ready_servo()
    servo.receive(mks_can.absolute_axis(1, 0x4000, 300, 0))
    run(servo, 0.05)
    held = servo.axis
    assert servo.rpm > 0.0
    assert servo.receive(mks_can.emergency_stop(1)) == [
        mks_can.frame(1, mks_can.EMERGENCY_STOP, [1])]
    assert servo.enabled and servo.target is None and servo.rpm == 0.0
    run(servo, 0.4)
    assert servo.axis == held


def test_bus_routes_replies_and_counts_bad_frames():
    bus = SimulatedBus([SimulatedServo(1), SimulatedServo(2)], 500000)
    bus.send(2, mks_can.read_encoder(2))
    assert bus.receive() == (2, mks_can.frame(2, mks_can.READ_ENCODER, bytes(6)))
    assert bus.receive() is None
    bus.send(1, mks_can.read_encoder(2))
    bus.send(9, mks_can.read_encoder(9))
    assert (bus.frames, bus.errors) == (4, 2)


def moving_servos(*can_ids):
    servos = {}
    for can_id in can_ids:
        servo = servos[can_id] = SimulatedServo(can_id)
        for data in (mks_can.set_mode(can_id), mks_can.enable(can_id),
                     mks_can.absolute_axis(can_id, 0x4000, 300, 0)):
            servo.receive(data)
        run(servo, 0.05)
        assert servo.rpm > 0.0
    return servos


def test_a_broadcast_stops_every_drive_and_none_replies():
    servos = moving_servos(1, 2)
    bus = SimulatedBus(servos.values(), 500000)
    # The checksum of a broadcast counts ID 0, so one made for a single drive is malformed.
    assert not bus.send(0, mks_can.emergency_stop(1)) and bus.errors == 1
    assert bus.send(0, mks_can.emergency_stop(0))
    assert bus.receive() is None
    assert all(servo.target is None and servo.rpm == 0.0 for servo in servos.values())


def test_the_drive_simulator_obeys_a_broadcast_without_replying():
    servos, done = moving_servos(1, 2), threading.Event()

    class Bus:
        errors = 0

        def __init__(self):
            self.inbox, self.sent = [(0, mks_can.emergency_stop(0))], []

        def receive(self, timeout=0.0):
            if self.inbox:
                return self.inbox.pop(0)
            done.set()
            return None

        def send(self, can_id, data):
            self.sent.append((can_id, data))

    bus = Bus()
    mks_drive_sim.serve(bus, servos, done)
    assert all(servo.target is None and servo.rpm == 0.0 for servo in servos.values())
    assert bus.sent == [] and bus.errors == 0
