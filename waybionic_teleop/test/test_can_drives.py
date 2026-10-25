"""
The host drives MKS servos over a python-can interface exactly as it drives the simulation.

The end-to-end test uses python-can's in-process virtual bus. The CAN CI job sets
WAYBIONIC_CAN_TEST=socketcan:vcan0 to run it over a kernel CAN interface instead. The carrier
tests stand a fake of carrier_bridge's serial port behind the slcan interface.
"""

import io
import math
import os
from pathlib import Path
import threading
import time
import uuid

from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from waybionic_teleop import carrier, mks_can, slcan_bus
from waybionic_teleop.can_bus import CanBus
from waybionic_teleop.mks_drive_sim import serve
from waybionic_teleop.sim_arm_drives_node import QUIET_BEFORE_ZERO_S, SimArmDrives
from waybionic_teleop.sim_drives import SimulatedServo

BITRATE = 1000000
URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
# The arm's five drives, all on the bus, without the placeholder tool, whose joint has no URDF
# limit yet.
ARM_DRIVES = {'drives': ['base_yaw', 'shoulder', 'elbow', 'wrist_left', 'wrist_right'],
              'simulated_drives': []}
# A target sent once must stay within following_error_counts: 3390 of 4096 counts at 26:1.
NEAR_TARGET = 0.05


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


def toward(node, speed=1.0, **targets):
    """Command the named joints a tick ahead of their encoders, as teleop does."""
    current, step = positions(node), speed * node.period
    command(node, speed, **{joint: current[joint] + max(-step, min(step, goal - current[joint]))
                            for joint, goal in targets.items()})


def move(node, executor, **targets):
    """Stream commands like teleop until the named joints reach their targets."""
    def reached():
        toward(node, **targets)
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
        assert drive.receive(1.0) == (1, mks_can.enable(1))
        assert host.receive(0.1) is None
        drive.send(1, mks_can.frame(1, mks_can.ENABLE, [1]))
        assert host.receive(1.0) == (1, mks_can.enable(1))
        assert drive.receive(0.1) is None
    finally:
        host.shutdown()
        drive.shutdown()


class FakeSlcanPort:
    """The serial port of an slcan (LAWICEL) adapter, with MKS drives on its CAN side."""

    def __init__(self, servos):
        self.servos = servos
        self.output = bytearray()
        self.ready = threading.Condition()
        self.write_timeout = None

    @property
    def in_waiting(self):
        with self.ready:
            return len(self.output)

    def read(self, size=1):
        with self.ready:
            if not self.output:
                self.ready.wait(0.001)
            data = bytes(self.output[:size])
            del self.output[:size]
            return data

    def write(self, data):
        with self.ready:
            for line in bytes(data).split(b'\r')[:-1]:
                if line[:1] != b't':
                    self.output += b'\r'
                    continue
                can_id, size = int(line[1:4], 16), int(line[4:5])
                data = bytes.fromhex(line[5:5 + 2 * size].decode())
                # The adapter acknowledges every frame it transmits with a z line.
                self.output += b'z\r'
                for reply in self.servos[can_id].receive(data):
                    self.output += f't{can_id:03X}{len(reply)}{reply.hex().upper()}\r'.encode()
            self.ready.notify_all()
        return len(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        with self.ready:
            self.output.clear()

    def close(self):
        pass

    def fileno(self):
        raise io.UnsupportedOperation('fileno')


def test_every_slcan_reply_is_read_between_the_adapter_acknowledgements(monkeypatch):
    serial = pytest.importorskip('serial')
    servos = {can_id: SimulatedServo(can_id) for can_id in range(1, 7)}
    port = FakeSlcanPort(servos)
    monkeypatch.setattr(serial, 'serial_for_url', lambda *args, **kwargs: port)
    bus = CanBus('slcan', 'fake', BITRATE, sleep_after_open=0)
    drained = []
    try:
        for _ in range(5):
            # A host tick sends two frames per drive; each gets a z line, then the reply.
            for can_id in servos:
                bus.send(can_id, mks_can.read_encoder(can_id))
                bus.send(can_id, mks_can.set_mode(can_id))
            deadline = time.monotonic() + 0.5
            while port.in_waiting and time.monotonic() < deadline:
                time.sleep(0.001)
            time.sleep(0.02)
            replies = []
            # The host drains without waiting, as on every tick.
            while (reply := bus.receive()) is not None:
                replies.append(reply)
            drained.append(len(replies))
    finally:
        bus.shutdown()
    assert drained == [12] * 5 and bus.errors == 0


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
        node, executor = make_node(interface=interface, channel=channel, **ARM_DRIVES)
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
        # Wrist pitch turns both differential motors the same way, through 3:1 belts.
        assert servos[4].axis == pytest.approx(servos[5].axis)
        assert servos[4].axis == pytest.approx(
            0.2 * node.map.drives[3].gear_ratio / (2 * math.pi) * mks_can.COUNTS_PER_REV,
            abs=1.0)
        assert node.bus.errors == 0 and node.bus_errors == 0
    finally:
        stop.set()
        thread.join()
        drives_bus.shutdown()


class FakeCarrier:
    """carrier_bridge's USB serial port, with MKS drives on its CAN side and an e-stop."""

    def __init__(self, servos):
        self.servos = servos
        self.output = bytearray()
        self.writes = []
        self.flags = None
        self.unplugged = self.open = False
        # Return True to refuse a frame, as the carrier does while its CAN controller is busy.
        self.refuse = lambda can_id, data: False
        self.last = self.reported = time.monotonic()

    def check(self):
        if self.unplugged:
            raise OSError(6, 'Device not configured')

    @property
    def in_waiting(self):
        self.check()
        return len(self.output)

    def read(self, size):
        self.check()
        data = bytes(self.output[:size])
        del self.output[:size]
        return data

    def write(self, data):
        self.check()
        self.writes.append(bytes(data))
        now = time.monotonic()
        frames = [(can_id, reply) for can_id, servo in self.servos.items()
                  for reply in servo.step(now - self.last)]
        self.last = now
        for line in bytes(data).split(b'\r')[:-1]:
            if line[:1] != b't':
                # C, Sn, O and an empty line get \r; a frame the carrier puts on the bus, nothing.
                self.open = {b'O': True, b'C': False}.get(line, self.open)
                self.output += b'\r'
                continue
            can_id, command = int(line[1:4], 16), bytes.fromhex(line[5:].decode())
            if not self.open or self.refuse(can_id, command):
                self.output += b'\a'
                continue
            frames += [(can_id, reply) for reply in self.servos[can_id].receive(command)]
        if self.flags is not None and now - self.reported >= 0.1:
            frames.append((carrier.STATUS_ID, bytes([0, self.flags, 0, 0, 0, 0, 0, 0])))
            self.reported = now
        for can_id, reply in frames if self.open else []:
            self.output += b't%03X%d%s\r' % (can_id, len(reply), reply.hex().upper().encode())
        return len(data)

    def close(self):
        pass


@pytest.fixture
def carrier_port(monkeypatch):
    """Put a fake carrier with the arm's five drives where the drive node opens its port."""
    port = FakeCarrier({can_id: SimulatedServo(can_id) for can_id in range(1, 6)})

    def open_port(*args, **kwargs):
        port.check()
        return port
    monkeypatch.setattr(slcan_bus.serial, 'serial_for_url', open_port)
    return port


def carrier_row(node, monkeypatch):
    reports = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', reports.append)
    node.report()
    return next(row for row in reports[-1].status if row.name == 'can.carrier')


def test_the_host_drives_the_arm_through_the_carrier(make_node, carrier_port, monkeypatch):
    port, servos = carrier_port, carrier_port.servos
    node, executor = make_node(interface='slcan', channel='/dev/cu.usbmodem1101',
                               **ARM_DRIVES)
    # The carrier opens at the drives' bit rate, and the monitor waits for its first report.
    assert port.writes[0] == b'C\rS6\rO\r'
    assert carrier_row(node, monkeypatch).level == DiagnosticStatus.WARN
    port.flags = carrier.ESTOP_WIRED
    assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
    spin_for(executor, 0.2)
    row = carrier_row(node, monkeypatch)
    assert row.level == DiagnosticStatus.OK and row.message.startswith('E-stop released')
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    assert node.authorized, node.stop_reason
    assert move(node, executor, joint_1=0.3)

    # Pressed mid-move, the e-stop stops every drive, and Start waits for its release.
    def pressed_mid_move():
        toward(node, joint_1=1.0)
        if positions(node)['joint_1'] > 0.5:
            port.flags = carrier.ESTOP_WIRED | carrier.ESTOP_PRESSED
        return not node.authorized
    assert spin_until(executor, pressed_mid_move)
    assert node.stop_reason == 'E-stop pressed'
    assert 0.5 < positions(node)['joint_1'] < 1.0
    assert all(servo.target is None and servo.rpm == 0.0 for servo in servos.values())
    assert carrier_row(node, monkeypatch).level == DiagnosticStatus.ERROR
    press_start(node)
    assert not node.authorized
    port.flags = carrier.ESTOP_WIRED
    assert spin_until(executor, lambda: not node.estop_pressed)
    press_start(node)
    assert node.authorized, node.stop_reason

    # Unplugged, the drives fall silent and stop the arm; the carrier reopens once it is back.
    port.unplugged = True
    assert spin_until(executor, lambda: all(node.lost))
    assert not node.authorized and not any(node.zeroed)
    assert 'USB link lost' in carrier_row(node, monkeypatch).message
    port.unplugged = False
    assert spin_until(executor, lambda: not any(node.lost))
    assert node.bus.reopens == 1 and port.writes.count(b'C\rS6\rO\r') == 2
    assert node.bus.refused == 0


def test_the_arm_runs_on_the_carrier_with_its_unwired_drives_simulated(make_node, monkeypatch):
    # J2 waits for its PCB and the tool is a placeholder, so only these drives are on the bus.
    port = FakeCarrier({can_id: SimulatedServo(can_id) for can_id in (1, 3, 4, 5)})
    port.flags = carrier.ESTOP_WIRED
    monkeypatch.setattr(slcan_bus.serial, 'serial_for_url', lambda *args, **kwargs: port)
    node, executor = make_node(interface='slcan', channel='/dev/cu.usbmodem1101')
    assert port.writes[0] == b'C\rS6\rO\r'
    assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    assert node.authorized, node.stop_reason
    # J2 has a URDF limit but no drive, so it holds the zero pose; the tool's joint has no limit.
    assert node.limits['joint_2'] == (0.0, 0.0) and 'tool_grip' not in node.limits
    assert move(node, executor, joint_1=0.3, joint_3=-0.2, joint_5=0.2)
    assert positions(node)['joint_2'] == 0.0
    sent = {int(line[1:4], 16) for write in port.writes for line in write.split(b'\r')
            if line[:1] == b't'}
    assert sent == {1, 3, 4, 5}
    reports = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', reports.append)
    node.report()
    shoulder = next(row for row in reports[-1].status if row.name == 'drive.shoulder')
    assert 'simulated' in shoulder.message and shoulder.level == DiagnosticStatus.OK
    # Moving J2 stops the arm rather than put the twin out of step with it.
    command(node, joint_2=0.1)
    assert not node.authorized and 'joint_2' in node.stop_reason


def test_frames_the_carrier_refuses_go_out_again(make_node, carrier_port, monkeypatch):
    port = carrier_port
    port.flags = carrier.ESTOP_WIRED
    seen = set()

    def busy(can_id, data):
        # A busy carrier refuses each move, stop and zero once; sent again, it goes out.
        if data[0] not in (mks_can.ABSOLUTE_AXIS, mks_can.SET_ZERO) or (can_id, data) in seen:
            return False
        seen.add((can_id, data))
        return True
    port.refuse = busy
    node, executor = make_node(interface='slcan', channel='/dev/cu.usbmodem1101',
                               **ARM_DRIVES)
    reports = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', reports.append)
    assert spin_until(executor, lambda: all(state == 'ready' for state in node.state))
    # Each zero refused is sent again while the zero service waits for the replies.
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None and not node.stopping)
    press_start(node)
    assert node.authorized, node.stop_reason
    # Every target is refused once, so the drives move only if refused ones go out again.
    assert move(node, executor, joint_1=0.3, joint_4=0.2)
    assert node.authorized, node.stop_reason
    node.report()
    rows = [row for report in reports for row in report.status if row.name == 'can.carrier']
    assert any(row.level == DiagnosticStatus.WARN and 'frames refused' in row.message
               for row in rows)
    assert node.bus.refused > 0 and node.bus.errors == 0 and node.bus_errors == 0


def test_a_real_bus_needs_a_urdf_limit_for_every_drive_joint(make_node, mock_drives):
    channel, servos = mock_drives
    sim, _ = make_node()
    sim.on_description(String(data=URDF))
    assert sim.description_valid and 'tool_grip' not in sim.limits
    # Simulation runs the placeholder tool drive without a limit; real drives never hold or
    # take a command until the URDF limits every joint they move.
    node, executor = make_node(interface='virtual', channel=channel, simulated_drives=[])
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
        toward(node, joint_1=1.0)
        return node.lost[2]
    assert spin_until(executor, commanding_until_lost)
    assert node.commanded is None and not node.zeroed[2] and not node.authorized
    # Every other drive was told to stop where it is.
    assert all(servo.target is None for servo in node.bus.drives.values())

    node.bus.drives[3] = elbow
    assert spin_until(executor, lambda: not node.lost[2] and not node.unconfirmed[2])
    # The stop it missed while silent went out once it answered again.
    assert not node.stopping and elbow.target is None
    spin_for(executor, QUIET_BEFORE_ZERO_S)
    assert node.commanded is None
    press_start(node)
    assert not node.authorized
    assert zero(node).success
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    assert node.authorized


def test_commands_that_stop_mid_move_stop_the_drives(make_node):
    node, executor = make_node(max_rpm=6)
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    # One slow command, then silence while teleop stays enabled, as if its command publisher
    # had died mid-move.
    command(node, joint_1=NEAR_TARGET)
    assert spin_until(executor, enabled_until(node, lambda: node.stale))
    assert not node.authorized
    held = positions(node)['joint_1']
    spin_for(executor, 0.5)
    assert 0.005 < held < NEAR_TARGET
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
    command(node, joint_1=NEAR_TARGET)
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
    command(node, joint_1=NEAR_TARGET)
    assert spin_until(executor, enabled_until(node, lambda: node.stale))
    spin_for(executor, 1.0)
    assert len(stops) == 1
    command(node, joint_1=NEAR_TARGET)
    assert not node.stale
    # Moving again takes a fresh Start.
    assert not node.authorized


def test_an_empty_command_stops_the_drives_until_a_fresh_start(make_node):
    node, executor = make_node(max_rpm=6)
    assert spin_until(executor, lambda: node.commanded is not None)
    press_start(node)
    command(node, joint_1=NEAR_TARGET)
    spin_for(executor, 0.1)
    assert node.bus.drives[1].target is not None
    node.on_command(JointState())
    assert not node.authorized and node.rejected == 1
    assert all(servo.target is None and servo.rpm == 0.0 for servo in node.bus.drives.values())
    response = zero(node)
    assert not response.success and 'still arriving' in response.message
    # Neither an invalid nor a valid command moves the arm again without a fresh Start.
    node.on_command(JointState(name=['joint_1'], position=[math.nan]))
    command(node, joint_1=0.3)
    spin_for(executor, 0.1)
    assert not node.authorized
    assert all(servo.target is None and servo.rpm == 0.0 for servo in node.bus.drives.values())
    press_start(node)
    # 6 rpm turns the base 0.024 rad/s through its reducer.
    assert move(node, executor, joint_1=0.01)


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


@pytest.mark.parametrize('interface', ['sim', 'virtual'])
def test_the_bus_report_names_the_interface(make_node, interface):
    # The autoplay demo only plays when this says sim.
    node, _ = make_node(interface=interface, channel=uuid.uuid4().hex)
    reports = []
    node.diagnostics_publisher.publish = reports.append
    node.report()
    bus = next(status for status in reports[0].status if status.name == 'can.bus')
    assert {item.key: item.value for item in bus.values}['interface'] == interface
