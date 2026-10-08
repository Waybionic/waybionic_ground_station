"""
The host drives MKS servos over a python-can interface exactly as it drives the simulation.

The end-to-end test uses python-can's in-process virtual bus. The CAN CI job sets
WAYBIONIC_CAN_TEST=socketcan:vcan0 to run it over a kernel CAN interface instead.
"""

import math
import os
from pathlib import Path
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from waybionic_teleop import mks_can
from waybionic_teleop.can_bus import CanBus
from waybionic_teleop.mks_drive_sim import serve
from waybionic_teleop.sim_arm_drives_node import SimArmDrives
from waybionic_teleop.sim_drives import SimulatedServo

BITRATE = 1000000
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')


@pytest.fixture
def context(monkeypatch):
    # Keep these nodes out of other tests' discovery.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    rclpy.try_shutdown(context=context)


@pytest.fixture
def make_node(context, parameters):
    nodes = []

    def make(**overrides):
        values = {**parameters('arm_drives.yaml', 'sim_arm_drives'), **overrides}
        node = SimArmDrives(context=context, parameter_overrides=[
            Parameter(name, value=value) for name, value in values.items()])
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        nodes.append((node, executor))
        return node, executor
    yield make
    for node, executor in nodes:
        executor.shutdown()
        node.close()
        node.destroy_node()


def spin_until(executor, done, seconds=3.0):
    deadline = time.monotonic() + seconds
    while not done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    return done()


def spin_for(executor, seconds):
    spin_until(executor, lambda: False, seconds)


def zero(node):
    return node.on_zero(Trigger.Request(), Trigger.Response())


def command(node, **positions):
    node.on_command(JointState(name=list(positions), position=list(positions.values())))


def positions(node):
    return node.map.to_positions(node.counts)


def test_the_bus_drops_its_own_echo_but_not_an_identical_reply():
    channel = uuid.uuid4().hex
    host = CanBus('virtual', channel, BITRATE, receive_own_messages=True)
    drive = CanBus('virtual', channel, BITRATE, receive_own_messages=True)
    try:
        # A successful enable reply is byte for byte the enable command.
        host.send(1, mks_can.enable(1))
        assert host.receive() is None
        assert drive.receive() == (1, mks_can.enable(1))
        drive.send(1, mks_can.frame(1, mks_can.ENABLE, [1]))
        assert drive.receive() is None
        assert host.receive() == (1, mks_can.enable(1))
    finally:
        host.shutdown()
        drive.shutdown()


def test_the_host_drives_mks_servos_over_can(make_node):
    interface, channel = os.environ.get(
        'WAYBIONIC_CAN_TEST', f'virtual:{uuid.uuid4().hex}').split(':', 1)
    drives_bus = CanBus(interface, channel, BITRATE)
    servos = {can_id: SimulatedServo(can_id) for can_id in range(1, 7)}
    # The base was powered on half a turn away from the zero pose.
    servos[1].axis = mks_can.COUNTS_PER_REV / 2
    stop = threading.Event()
    thread = threading.Thread(target=serve, args=(drives_bus, servos, stop), daemon=True)
    thread.start()
    try:
        node, executor = make_node(interface=interface, channel=channel)
        assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
        spin_for(executor, 0.1)
        # Real drives are not driven, and no joint states are published, until zeroed.
        assert node.commanded is None

        assert zero(node).success
        assert spin_until(executor, lambda: node.commanded is not None)
        assert positions(node)['joint_1'] == pytest.approx(0.0)

        command(node, joint_1=0.3, joint_4=0.2)
        assert spin_until(executor, lambda: abs(positions(node)['joint_1'] - 0.3) < 1e-3
                          and abs(positions(node)['joint_4'] - 0.2) < 1e-3)
        # Wrist pitch turns both differential motors the same way.
        assert servos[4].axis == pytest.approx(servos[5].axis)
        assert servos[4].axis == pytest.approx(0.2 / (2 * math.pi) * mks_can.COUNTS_PER_REV,
                                               abs=1.0)
        assert node.bus.errors == 0 and node.bus_errors == 0
    finally:
        stop.set()
        thread.join()
        drives_bus.shutdown()


def test_commands_before_zeroing_are_ignored(make_node):
    node, executor = make_node(zero_on_start=False)
    spin_for(executor, 0.1)
    command(node, joint_1=0.3)
    spin_for(executor, 0.2)
    assert node.commanded is None
    assert all(servo.target is None and servo.axis == 0 for servo in node.bus.drives.values())


def test_zeroing_waits_for_the_commands_to_stop(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    command(node, joint_1=0.1)
    response = zero(node)
    assert not response.success and 'disable teleop' in response.message


def test_a_drive_that_stops_answering_stops_the_arm_until_it_is_zeroed_again(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    elbow = node.bus.drives.pop(3)
    assert spin_until(executor, lambda: node.lost[2])
    assert node.commanded is None and not node.zeroed[2]
    # Every other drive was told to stop where it is.
    assert all(servo.target is None for servo in node.bus.drives.values())

    node.bus.drives[3] = elbow
    assert spin_until(executor, lambda: not node.lost[2] and node.state[2] == 'ready')
    spin_for(executor, 0.1)
    assert node.commanded is None
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None)


def test_commands_that_stop_mid_move_stop_the_drives(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    # One command toward a far target, then silence, as if teleop had exited mid-move.
    node.on_command(JointState(name=['joint_1'], position=[100.0], velocity=[1.0]))
    assert spin_until(executor, lambda: not node.velocities)
    assert spin_until(executor, lambda: node.commanded is not None)
    held = node.commanded['joint_1']
    spin_for(executor, 0.5)
    assert 1.0 < held < 50.0
    assert node.map.to_positions(node.counts)['joint_1'] == pytest.approx(held, abs=0.05)


def test_nothing_moves_until_every_drive_confirms_its_heartbeat(make_node, monkeypatch):
    node, executor = make_node(zero_on_start=False)
    assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
    shoulder = node.bus.drives[2]
    answer = shoulder.receive
    # The shoulder misses its first heartbeat setting, as if the frame were lost.
    missed = []

    def lossy(data):
        if data[0] == mks_can.SET_HEARTBEAT and not missed:
            missed.append(data)
            return []
        return answer(data)
    monkeypatch.setattr(shoulder, 'receive', lossy)
    node.set_up(1)
    assert zero(node).success
    spin_for(executor, 0.2)
    assert node.unconfirmed[1] == {mks_can.SET_HEARTBEAT} and node.commanded is None
    # The setup is sent again, and the arm can then move.
    assert spin_until(executor, lambda: node.commanded is not None)
    assert shoulder.heartbeat_ms == node.heartbeat_ms


def test_a_target_the_interface_refused_is_sent_again(make_node, monkeypatch):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    send, refused = node.bus.send, []

    def flaky(can_id, data):
        if data[0] == mks_can.ABSOLUTE_AXIS and data[1:3] != b'\x00\x00' and not refused:
            refused.append(can_id)
            return False
        return send(can_id, data)
    monkeypatch.setattr(node.bus, 'send', flaky)
    command(node, joint_1=0.3)
    assert spin_until(executor, lambda: abs(positions(node)['joint_1'] - 0.3) < 1e-3)
    assert refused == [1]


def test_commands_without_a_speed_that_stop_also_stop_the_drives(make_node):
    # A gearbox that lets the joint turn at 6 rpm, the slowest documented MKS SERVO42D
    # setting, so a move well inside the joint limits outlasts the 0.5 s command timeout.
    node, executor = make_node(max_rpm=6)
    node.on_description(String(data=URDF))
    assert spin_until(executor, lambda: node.commanded is not None and node.limits)
    upper = node.limits['joint_1'][1]
    # One position-only command, as a publisher that fills in no velocity sends, then silence.
    node.on_command(JointState(name=['joint_1'], position=[upper]))
    assert spin_until(executor, lambda: node.stale)
    # Let the drives finish slowing down, then check that the arm holds where it stopped.
    spin_for(executor, 0.1)
    stopped = positions(node)['joint_1']
    spin_for(executor, 1.0)
    assert positions(node)['joint_1'] == pytest.approx(stopped, abs=0.03)
    # Without the stop it would have kept turning towards the limit at 36 deg/s.
    assert stopped < upper - 1.0


def test_a_paused_host_tick_respects_joint_limit_and_drive_heartbeat(make_node):
    node, executor = make_node(max_rpm=6, command_timeout_s=10.0)
    node.on_description(String(data=URDF))
    assert spin_until(executor, lambda: node.commanded is not None and node.limits)
    upper = node.limits['joint_1'][1]
    node.on_command(JointState(name=['joint_1'], position=[upper - 0.01], velocity=[6.0]))
    node.tick()
    motor = node.bus.drives[1]
    expected = node.map.to_counts({**node.commanded, 'joint_1': upper})[0]
    assert motor.target == expected

    # No wall-clock pause: a scheduler gap reaches the drive before encoder polling
    # can reset the heartbeat. The unrelated publisher timeout remains unarmed.
    node.last_tick -= 2.0
    node.tick()
    assert not node.stale
    assert (motor.heartbeat_stops, motor.rpm, motor.target) == (1, 0.0, None)
    assert 0.0 < motor.axis < expected
    node.tick()
    assert motor.target is None


def test_the_stale_stop_happens_once_and_clears_when_commands_return(make_node):
    node, executor = make_node(max_rpm=6)
    assert spin_until(executor, lambda: node.commanded is not None)
    stop_all, stops = node.stop_all, []

    def counted():
        stops.append(time.monotonic())
        stop_all()
    node.stop_all = counted
    command(node, joint_1=0.5)
    assert spin_until(executor, lambda: node.stale)
    spin_for(executor, 1.0)
    assert len(stops) == 1
    command(node, joint_1=0.5)
    assert not node.stale


def test_explicit_stop_latches_and_rearms_only_after_valid_command(make_node):
    node, executor = make_node(max_rpm=6)
    node.on_description(String(data=URDF))
    assert spin_until(executor, lambda: node.commanded is not None and node.limits)
    command(node, joint_1=1.0)
    node.on_command(JointState())
    assert node.safety_stop and node.stale and node.commanded is None
    stopped_at = node.command_time
    response = node.on_zero(Trigger.Request(), Trigger.Response())
    assert not response.success and 'still arriving' in response.message

    node.on_command(JointState(name=['joint_1'], position=[math.nan]))
    assert node.safety_stop and node.stale and node.commanded is None
    assert node.command_time == stopped_at

    command(node, joint_1=0.3)
    assert not node.safety_stop and not node.stale
    assert node.commanded['joint_1'] == pytest.approx(0.3)
    assert node.command_time > stopped_at


def test_silence_before_the_first_command_leaves_the_drives_holding(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    held, stops, stop_all = dict(node.commanded), [], node.stop_all

    def counted():
        stops.append(time.monotonic())
        stop_all()
    node.stop_all = counted
    # Longer than command_timeout_s: an arm nobody has commanded yet keeps its hold target,
    # and start-up never logs a stale-command warning.
    spin_for(executor, 1.0)
    assert stops == [] and node.commanded == held
