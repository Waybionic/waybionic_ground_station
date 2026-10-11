"""Controller link diagnostics tests using synthetic UDP controller packets."""

from unittest.mock import MagicMock

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
import rclpy

from waybionic_teleop import gamepad, joy_udp_receiver
from waybionic_teleop.joy_udp_receiver import JoyUdpReceiver


class FakeSocket:
    """A queued UDP socket substitute for receiver tests."""

    def __init__(self):
        self.packets = []

    def bind(self, address):
        self.address = address

    def setblocking(self, value):
        self.blocking = value

    def recvfrom(self, _size):
        if not self.packets:
            raise BlockingIOError
        return self.packets.pop(0)

    def close(self):
        pass


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def receiver(monkeypatch):
    fake_socket = FakeSocket()
    monkeypatch.setattr(
        joy_udp_receiver.socket, 'socket', lambda *_args: fake_socket)
    node = JoyUdpReceiver()
    monkeypatch.setattr(joy_udp_receiver.time, 'monotonic', lambda: 100.0)
    node.report_time = 99.5
    node.diagnostics_publisher.publish = MagicMock()
    node.joy_publisher.publish = MagicMock()
    yield node, fake_socket
    node.destroy_node()


def add_packet(fake_socket, sequence, connected):
    packet = gamepad.pack(
        sequence, [0.0] * len(gamepad.AXES),
        [0] * len(gamepad.BUTTONS), connected=connected)
    fake_socket.packets.append((packet, ('127.0.0.1', 47300)))


def report_status(node):
    node.report()
    message = node.diagnostics_publisher.publish.call_args.args[0]
    return message.status[0]


def values(status):
    return {value.key: value.value for value in status.values}


def test_connected_state_and_input_rate(receiver):
    node, fake_socket = receiver
    add_packet(fake_socket, 1, connected=True)
    node.poll()
    add_packet(fake_socket, 2, connected=True)
    node.poll()

    status = report_status(node)

    assert status.name == 'controller.link'
    assert status.level == DiagnosticStatus.OK
    assert values(status)['state'] == 'CONNECTED'
    assert values(status)['value'] == '4'
    assert values(status)['unit'] == 'Hz'
    assert node.joy_publisher.publish.call_count == 2


def test_no_controller_from_explicit_packet_flag(receiver):
    node, fake_socket = receiver
    add_packet(fake_socket, 1, connected=False)
    node.poll()

    status = report_status(node)

    assert values(status)['state'] == 'NO_CONTROLLER'
    assert status.level == DiagnosticStatus.WARN
    assert 'no controller is connected' in status.message
    assert status.name == 'controller.link'
    node.joy_publisher.publish.assert_not_called()


def test_missing_bridge_packets_are_stale(receiver):
    node, _fake_socket = receiver

    status = report_status(node)

    assert values(status)['state'] == 'BRIDGE_NOT_RUNNING'
    assert status.level == DiagnosticStatus.STALE


def test_stale_teleop_input_is_reported_when_bridge_is_connected(receiver):
    node, fake_socket = receiver
    add_packet(fake_socket, 1, connected=True)
    node.poll()
    node.on_diagnostics(DiagnosticArray(status=[DiagnosticStatus(
        name='teleop.input', level=DiagnosticStatus.STALE)]))

    status = report_status(node)

    assert values(status)['state'] == 'INPUT_STALE'
    assert status.level == DiagnosticStatus.STALE
    assert 'No fresh /joy input' in status.message


def test_bridge_timeout_precedes_no_controller_state(receiver):
    node, fake_socket = receiver
    add_packet(fake_socket, 1, connected=False)
    node.poll()
    node.last_packet = 99.49

    status = report_status(node)

    assert values(status)['state'] == 'BRIDGE_NOT_RUNNING'
    assert status.level == DiagnosticStatus.STALE


def test_no_controller_precedes_stale_teleop_input(receiver):
    node, fake_socket = receiver
    add_packet(fake_socket, 1, connected=False)
    node.poll()
    node.on_diagnostics(DiagnosticArray(status=[DiagnosticStatus(
        name='teleop.input', level=DiagnosticStatus.STALE)]))

    status = report_status(node)

    assert values(status)['state'] == 'NO_CONTROLLER'
    assert status.level == DiagnosticStatus.WARN
