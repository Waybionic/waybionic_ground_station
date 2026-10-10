"""Controller packets from the host bridge in, /joy out."""

import select
import socket

import pytest
import rclpy
from rclpy.parameter import Parameter

from waybionic_teleop import gamepad
from waybionic_teleop.joy_udp_receiver import JoyUdpReceiver

STICK = [0.5, -0.25, 0.0, 1.0, 0.0, -0.75]
START = [int(name == 'start') for name in gamepad.BUTTONS]
NEUTRAL = [0.0] * len(gamepad.AXES), [0] * len(gamepad.BUTTONS)


@pytest.fixture
def receiver(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    # Port 0 binds a free port, so a running ground station cannot interfere.
    node = JoyUdpReceiver(context=context, parameter_overrides=[Parameter('port', value=0)])
    yield node
    node.socket.close()
    node.destroy_node()
    rclpy.try_shutdown(context=context)


@pytest.fixture
def joys(receiver, monkeypatch):
    published = []
    monkeypatch.setattr(receiver.joy_publisher, 'publish', published.append)
    return published


@pytest.fixture
def send(receiver):
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def deliver(data):
        sender.sendto(data, receiver.socket.getsockname())
        assert select.select([receiver.socket], [], [], 2.0)[0], 'the packet never arrived'
        receiver.poll()

    yield deliver
    sender.close()


def test_a_lost_controller_releases_every_control_once_then_goes_quiet(receiver, joys, send):
    send(gamepad.pack(1, *NEUTRAL))
    centred = joys[-1]
    send(gamepad.pack(2, STICK, START))
    for sequence in range(3, 7):
        send(gamepad.pack(sequence, STICK, START, connected=False))
    assert receiver.sequence == 6 and len(joys) == 3
    released = joys[-1]
    assert list(released.axes) == list(centred.axes) == [0.0] * len(gamepad.AXES)
    assert list(released.buttons) == list(centred.buttons) == [0] * len(gamepad.BUTTONS)
    assert released.header.frame_id == centred.header.frame_id
    send(gamepad.pack(7, STICK, START))
    assert len(joys) == 4 and list(joys[-1].buttons) == START
