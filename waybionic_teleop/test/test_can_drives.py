"""
The host drives MKS servos over a python-can interface exactly as it drives the simulation.

The end-to-end test uses python-can's in-process virtual bus. The CAN CI job sets
WAYBIONIC_CAN_TEST=socketcan:vcan0 to run it over a kernel CAN interface instead.
"""

import math
import os
import threading
import time
import uuid

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from waybionic_teleop import mks_can
from waybionic_teleop.can_bus import CanBus
from waybionic_teleop.mks_drive_sim import serve
from waybionic_teleop.sim_arm_drives_node import SimArmDrives
from waybionic_teleop.sim_drives import SimulatedServo

BITRATE = 1000000


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


def test_stale_commands_hold_the_last_target(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    node.on_command(JointState(name=['joint_1'], position=[0.0], velocity=[1.0]))
    assert node.velocities == {'joint_1': 1.0}
    assert spin_until(executor, lambda: not node.velocities)
