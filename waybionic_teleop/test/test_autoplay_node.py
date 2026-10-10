"""The autoplay node plays only on fresh reports from simulated drives, and hands over safely."""

from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest
import rclpy
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String

from waybionic_teleop import autoplay_node
from waybionic_teleop.demo_script import SILENCE_S, STALE_S
from waybionic_teleop.gamepad import AXES, BUTTON, BUTTONS

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
DRIVES = ['base_yaw', 'shoulder', 'elbow', 'wrist_left', 'wrist_right', 'tool']
RATE = 120.0
# Teleop and the drives report twice a second.
REPORT_TICKS = round(RATE / 2)
TELEOP = DiagnosticArray(status=[
    DiagnosticStatus(name='teleop.state', values=[KeyValue(key='value', value='enabled')]),
    DiagnosticStatus(name='teleop.group', values=[KeyValue(key='value', value='base')])])


def row(name, level, value, **extra):
    return DiagnosticStatus(name=name, level=level, values=[
        KeyValue(key=key, value=item) for key, item in {'value': value, **extra}.items()])


def drives(interface, gate=DiagnosticStatus.WARN, drive=DiagnosticStatus.OK):
    """Report like the drive node: its bus, command gate (waiting for Start) and each drive."""
    return DiagnosticArray(status=[
        row('can.bus', DiagnosticStatus.OK, '0', interface=interface),
        row('arm.command_gate', gate, 'enabled' if gate == DiagnosticStatus.OK else 'stopped'),
        *(row(f'drive.{name}', drive, '+0.000') for name in DRIVES)])


def joy(*buttons, **axes):
    return Joy(axes=[axes.get(name, 0.0) for name in AXES],
               buttons=[int(name in buttons) for name in BUTTONS])


def pressed(sent, button):
    return [message for _, message in sent if message.buttons[BUTTON[button]]]


class Clock:
    """Stands in for the time module in the node, so the tests set the time."""

    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now


class Bench:
    """The autoplay node on a set clock, with teleop and the drives reporting as asked."""

    def __init__(self, node, clock):
        self.node, self.clock, self.sent = node, clock, []
        node.joy_publisher.publish = lambda message: self.sent.append((clock.now, message))
        node.on_description(String(data=URDF))

    def run(self, seconds, teleop=True, interface='sim', joints=True, operator=None,
            **drive_levels):
        """Tick for this long; return what reached teleop's joy topic, with when it was sent."""
        start = len(self.sent)
        for tick in range(round(seconds * RATE)):
            if tick % REPORT_TICKS == 0:
                if teleop:
                    self.node.on_diagnostics(TELEOP)
                if interface:
                    self.node.on_diagnostics(drives(interface, **drive_levels))
            if joints:
                self.node.on_joint_states(JointState(name=JOINTS, position=[0.0] * len(JOINTS)))
            if operator:
                self.node.on_operator(operator)
            self.node.tick()
            self.clock.now += 1.0 / RATE
        return self.sent[start:]


@pytest.fixture
def context(monkeypatch):
    # Keep these nodes out of other tests' discovery.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    rclpy.try_shutdown(context=context)


@pytest.fixture
def bench(context, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(autoplay_node, 'time', clock)
    node = autoplay_node.Autoplay(context=context)
    yield Bench(node, clock)
    node.destroy_node()


def test_the_demo_plays_on_simulated_drives(bench):
    start = bench.clock.now
    sent = bench.run(3.0)
    # Silence first, so teleop times out however it was left, then Start.
    assert sent[0][0] >= start + SILENCE_S - 1e-6
    assert pressed(sent, 'start')


def test_the_demo_never_plays_on_real_drives(bench):
    assert bench.run(5.0, interface='socketcan') == []
    # The operator's controller still reaches teleop, unchanged.
    stick = joy(left_x=1.0)
    bench.node.on_operator(stick)
    assert bench.sent == [(bench.clock.now, stick)]


@pytest.mark.parametrize('report', [
    {'interface': None},
    # A drive not set up or zeroed, or silent.
    {'drive': DiagnosticStatus.WARN},
    {'drive': DiagnosticStatus.ERROR},
    # The drives have no valid robot description.
    {'gate': DiagnosticStatus.ERROR},
])
def test_the_demo_waits_for_the_drives_to_be_ready(bench, report):
    assert bench.run(5.0, **report) == []


def test_start_waits_for_the_drives_gate_to_wait_for_it(bench):
    # While the gate still passes commands, a Start would only disable teleop.
    assert not pressed(bench.run(4.0, gate=DiagnosticStatus.OK), 'start')
    assert pressed(bench.run(4.0), 'start')


def test_the_demo_plays_on_only_once_the_drives_take_the_start(bench):
    # Teleop reports enabled, but the drives' gate stays shut: the demo starts over.
    sent = bench.run(6.0)
    assert pressed(sent, 'start') and not pressed(sent, 'dpad_down')
    assert pressed(bench.run(1.9), 'start')
    # This time the gate opens, and the demo sets the speed.
    assert pressed(bench.run(3.0, gate=DiagnosticStatus.OK), 'dpad_down')


def test_the_demo_stops_while_another_node_publishes_joy(bench, context):
    assert pressed(bench.run(3.0), 'start')
    other = rclpy.create_node('joystick', context=context)
    try:
        other.create_publisher(Joy, 'joy', 10)
        deadline = time.monotonic() + 5.0
        while bench.node.count_publishers('/joy') < 2 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert bench.run(5.0) == []
    finally:
        other.destroy_node()


def test_taking_over_leaves_teleop_disabled(bench):
    assert pressed(bench.run(3.0), 'start')
    stick = joy(left_x=1.0)
    sent = [message for _, message in bench.run(1.0, operator=stick)]
    # Teleop is still enabled from the demo's Start: B first, then a neutral sample, and only
    # then the operator's own input.
    kinds = ['b' if message.buttons[BUTTON['b']] else 'operator' if message is stick
             else 'neutral' for message in sent]
    assert [kind for index, kind in enumerate(kinds)
            if index == 0 or kind != kinds[index - 1]] == ['b', 'neutral', 'operator']
    ours = [message for message in sent if message is not stick]
    assert not any(any(message.axes) for message in ours)
    assert {tuple(message.buttons) for message in ours} == {
        tuple(joy('b').buttons), tuple(joy().buttons)}
    assert not bench.node.playing


@pytest.mark.parametrize('quiet', ['teleop', 'joints', 'interface'])
def test_the_demo_sends_nothing_on_stale_reports_then_starts_over(bench, quiet):
    assert pressed(bench.run(4.0), 'start')
    silent_from = bench.clock.now
    # Teleop times out once nothing more is sent.
    assert all(at <= silent_from + STALE_S + 1e-6 for at, _ in bench.run(3.0, **{quiet: None}))
    resumed = bench.clock.now
    sent = bench.run(3.0)
    assert sent[0][0] >= resumed + SILENCE_S - 1e-6
    assert pressed(sent, 'start')
