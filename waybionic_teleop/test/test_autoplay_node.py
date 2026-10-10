"""The autoplay node plays only on fresh reports from simulated drives."""

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
RATE = 120.0
# Teleop and the drives report twice a second.
REPORT_TICKS = round(RATE / 2)
TELEOP = DiagnosticArray(status=[
    DiagnosticStatus(name='teleop.state', values=[KeyValue(key='value', value='enabled')]),
    DiagnosticStatus(name='teleop.group', values=[KeyValue(key='value', value='base')])])


def drives(interface):
    return DiagnosticArray(status=[DiagnosticStatus(name='can.bus', values=[
        KeyValue(key='value', value='0'), KeyValue(key='interface', value=interface)])])


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

    def run(self, seconds, interface='sim'):
        """Tick for this long; return what reached teleop's joy topic, with when it was sent."""
        start = len(self.sent)
        for tick in range(round(seconds * RATE)):
            if tick % REPORT_TICKS == 0:
                self.node.on_diagnostics(TELEOP)
                if interface:
                    self.node.on_diagnostics(drives(interface))
            self.node.on_joint_states(JointState(name=JOINTS, position=[0.0] * len(JOINTS)))
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


def test_the_demo_waits_for_the_drives_to_report(bench):
    assert bench.run(5.0, interface=None) == []


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


def test_the_demo_sends_nothing_on_a_stale_drive_report_then_starts_over(bench):
    assert pressed(bench.run(4.0), 'start')
    silent_from = bench.clock.now
    # Teleop times out once nothing more is sent.
    assert all(at <= silent_from + STALE_S + 1e-6 for at, _ in bench.run(3.0, interface=None))
    resumed = bench.clock.now
    sent = bench.run(3.0)
    assert sent[0][0] >= resumed + SILENCE_S - 1e-6
    assert pressed(sent, 'start')
