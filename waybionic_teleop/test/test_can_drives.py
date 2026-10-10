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
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from waybionic_teleop import mks_can
from waybionic_teleop.can_bus import CanBus
from waybionic_teleop.mks_drive_sim import serve
from waybionic_teleop.sim_arm_drives_node import QUIET_BEFORE_ZERO_S, SimArmDrives
from waybionic_teleop.sim_drives import SimulatedServo

BITRATE = 1000000
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
# The arm's drives without the placeholder tool, whose joint has no URDF limit yet.
ARM_DRIVES = ['base_yaw', 'shoulder', 'elbow', 'wrist_left', 'wrist_right']


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


@pytest.fixture
def mock_drives():
    """Answer as six MKS drives on a fresh python-can virtual channel."""
    channel = uuid.uuid4().hex
    bus = CanBus('virtual', channel, BITRATE)
    servos = {can_id: SimulatedServo(can_id) for can_id in range(1, 7)}
    stop = threading.Event()
    thread = threading.Thread(target=serve, args=(bus, servos, stop), daemon=True)
    thread.start()
    yield channel, servos
    stop.set()
    thread.join()
    bus.shutdown()


def spin_until(executor, done, seconds=3.0):
    deadline = time.monotonic() + seconds
    while not done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
    return done()


def spin_for(executor, seconds):
    spin_until(executor, lambda: False, seconds)


def zero(node):
    return node.on_zero(Trigger.Request(), Trigger.Response())


def positions(node):
    return node.map.to_positions(node.counts)


def press_start(node):
    """Load the URDF limits, then release and press Start the way teleop reports it."""
    node.on_description(String(data=URDF))
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))


def command(node, speed=1.0, **targets):
    """Refresh the enable and send one complete command toward the named joint targets."""
    current = positions(node)
    goal = {**(node.commanded or current), **targets}
    velocities = [math.copysign(speed, goal[joint] - current[joint]) if joint in targets
                  else 0.0 for joint in goal]
    node.on_enabled(Bool(data=True))
    node.on_command(JointState(name=list(goal), position=list(goal.values()),
                               velocity=velocities))


def move(node, executor, **targets):
    """Stream commands like teleop until the named joints reach their targets."""
    def reached():
        command(node, **targets)
        return all(abs(positions(node)[joint] - value) < 1e-3
                   for joint, value in targets.items())
    return spin_until(executor, reached)


def enabled_until(node, done=lambda: False):
    """Republish the enable, as teleop does on every update, without sending a command."""
    def check():
        node.on_enabled(Bool(data=True))
        return done()
    return check


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
        node, executor = make_node(interface=interface, channel=channel, drives=ARM_DRIVES)
        assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
        spin_for(executor, 0.1)
        # Real drives are not driven, and no joint states are published, until zeroed.
        press_start(node)
        assert node.commanded is None and not node.authorized

        assert zero(node).success
        assert spin_until(executor, lambda: node.commanded is not None)
        assert positions(node)['joint_1'] == pytest.approx(0.0)

        press_start(node)
        assert node.authorized, node.stop_reason
        assert set(node.limits) == set(node.map.joints)
        assert move(node, executor, joint_1=0.3, joint_4=0.2)
        # Wrist pitch turns both differential motors the same way.
        assert servos[4].axis == pytest.approx(servos[5].axis)
        assert servos[4].axis == pytest.approx(0.2 / (2 * math.pi) * mks_can.COUNTS_PER_REV,
                                               abs=1.0)
        assert node.bus.errors == 0 and node.bus_errors == 0
    finally:
        stop.set()
        thread.join()
        drives_bus.shutdown()


def test_a_real_bus_needs_a_urdf_limit_for_every_drive_joint(make_node, mock_drives):
    channel, servos = mock_drives
    sim, _ = make_node()
    sim.on_description(String(data=URDF))
    assert sim.description_valid and 'tool_grip' not in sim.limits
    # Simulation runs the placeholder tool drive without a limit; real drives never hold or
    # take a command until the URDF limits every joint they move.
    node, executor = make_node(interface='virtual', channel=channel)
    assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None)
    node.on_description(String(data=URDF))
    assert not node.description_valid and 'tool_grip' in node.stop_reason
    press_start(node)
    command(node, joint_1=0.3)
    spin_for(executor, 0.2)
    assert not node.authorized
    assert all(servo.target is None and servo.axis == 0 for servo in servos.values())


def test_commands_before_zeroing_are_ignored(make_node):
    node, executor = make_node(zero_on_start=False)
    spin_for(executor, 0.1)
    press_start(node)
    command(node, joint_1=0.3)
    spin_for(executor, 0.2)
    assert not node.authorized and node.commanded is None
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
    press_start(node)
    elbow = node.bus.drives.pop(3)

    def commanding_until_lost():
        # Teleop keeps commanding a move while the elbow is silent.
        command(node, joint_1=1.0)
        return node.lost[2]
    assert spin_until(executor, commanding_until_lost)
    assert node.commanded is None and not node.zeroed[2] and not node.authorized
    # Every other drive was told to stop where it is.
    assert all(servo.target is None for servo in node.bus.drives.values())

    node.bus.drives[3] = elbow
    assert spin_until(executor, lambda: not node.lost[2] and node.state[2] == 'ready')
    spin_for(executor, QUIET_BEFORE_ZERO_S)
    assert node.commanded is None
    press_start(node)
    assert not node.authorized
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    assert node.authorized


def test_commands_that_stop_mid_move_stop_the_drives(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    # One command toward a far target, then silence while teleop stays enabled, as if its
    # command publisher had died mid-move.
    command(node, speed=0.5, joint_1=3.0)
    assert spin_until(executor, enabled_until(node, lambda: node.stale))
    assert not node.authorized
    held = positions(node)['joint_1']
    spin_for(executor, 0.5)
    assert 0.05 < held < 2.0
    assert positions(node)['joint_1'] == pytest.approx(held, abs=1e-3)


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
    press_start(node)
    assert node.unconfirmed[1] == {mks_can.SET_HEARTBEAT} and not node.authorized
    # The setup is sent again, and the arm can then move.
    assert spin_until(executor, lambda: not node.unconfirmed[1])
    press_start(node)
    assert node.authorized
    assert shoulder.heartbeat_ms == node.heartbeat_ms


def test_a_target_the_interface_refused_is_sent_again(make_node, monkeypatch):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    send, refused = node.bus.send, []

    def flaky(can_id, data):
        if data[0] == mks_can.ABSOLUTE_AXIS and data[1:3] != b'\x00\x00' and not refused:
            refused.append(can_id)
            return False
        return send(can_id, data)
    monkeypatch.setattr(node.bus, 'send', flaky)
    assert move(node, executor, joint_1=0.3)
    assert refused == [1]


def test_a_command_without_speeds_stops_the_drives(make_node):
    node, executor = make_node(max_rpm=6)
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    command(node, joint_1=1.0)
    spin_for(executor, 0.1)
    assert node.bus.drives[1].target is not None
    # Positions alone, as a publisher that fills in no velocity sends, never move the arm on
    # to the last target at whatever speed the drives were given.
    targets = dict(node.commanded)
    node.on_command(JointState(name=list(targets), position=list(targets.values())))
    assert not node.authorized and node.rejected == 1
    assert all(servo.target is None and servo.rpm == 0.0 for servo in node.bus.drives.values())
    stopped = positions(node)['joint_1']
    spin_for(executor, 0.5)
    assert positions(node)['joint_1'] == pytest.approx(stopped, abs=1e-3)


def test_a_paused_host_tick_respects_joint_limit_and_drive_heartbeat(make_node):
    node, executor = make_node(max_rpm=6, command_timeout_s=10.0)
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    upper = node.limits['joint_1'][1]
    command(node, speed=6.0, joint_1=upper - 0.01)
    node.tick()
    motor = node.bus.drives[1]
    expected = node.map.to_counts(node.commanded)[0]
    assert motor.target == expected
    assert expected < node.map.to_counts({**node.commanded, 'joint_1': upper})[0]

    # No wall-clock pause: a scheduler gap reaches the drive before encoder polling
    # can reset the heartbeat. The unrelated publisher timeout remains unarmed.
    node.last_tick -= 2.0
    node.tick()
    assert not node.stale and not node.authorized
    assert (motor.heartbeat_stops, motor.rpm, motor.target) == (1, 0.0, None)
    assert 0.0 < motor.axis < expected
    node.tick()
    assert motor.target is None


def test_the_stale_stop_happens_once_and_clears_when_commands_return(make_node):
    node, executor = make_node(max_rpm=6)
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    stop_all, stops = node.stop_all, []

    def counted(reason):
        stops.append(reason)
        stop_all(reason)
    node.stop_all = counted
    command(node, joint_1=0.5)
    assert spin_until(executor, enabled_until(node, lambda: node.stale))
    spin_for(executor, 1.0)
    assert len(stops) == 1
    command(node, joint_1=0.5)
    assert not node.stale
    # Moving again takes a fresh Start.
    assert not node.authorized


def test_silence_before_the_first_command_leaves_the_drives_holding(make_node):
    node, executor = make_node()
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    held, stops, stop_all = dict(node.commanded), [], node.stop_all

    def counted(reason):
        stops.append(reason)
        stop_all(reason)
    node.stop_all = counted
    # Longer than command_timeout_s: an enabled arm nobody has commanded yet keeps its hold
    # target, and start-up never logs a stale-command warning.
    spin_until(executor, enabled_until(node), 1.0)
    assert stops == [] and node.authorized and node.commanded == held
