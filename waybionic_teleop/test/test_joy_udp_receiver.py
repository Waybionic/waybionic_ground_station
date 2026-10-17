"""Controller packets from the host bridge in, /joy out."""

import select
import socket

from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JoyFeedback

from waybionic_teleop import gamepad
from waybionic_teleop.joy_udp_receiver import JoyUdpReceiver, LinkQuality

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


def test_old_and_repeated_packets_are_dropped(receiver, joys, send):
    send(gamepad.pack(10, STICK, START))
    send(gamepad.pack(10, *NEUTRAL))
    send(gamepad.pack(9, *NEUTRAL))
    assert receiver.sequence == 10 and len(joys) == 1 and list(joys[-1].buttons) == START
    send(gamepad.pack(11, *NEUTRAL))
    assert len(joys) == 2 and not any(joys[-1].buttons)


def test_a_restarted_bridge_is_accepted_after_the_timeout(receiver, joys, send):
    send(gamepad.pack(500, STICK, START))
    send(gamepad.pack(1, *NEUTRAL))
    assert receiver.sequence == 500 and len(joys) == 1
    # The bridge was quiet for longer than the timeout, so its count may restart.
    receiver.last_packet -= receiver.timeout + 0.01
    send(gamepad.pack(1, *NEUTRAL))
    assert receiver.sequence == 1 and len(joys) == 2


def test_oversize_packets_are_rejected(receiver, joys, send):
    packet = gamepad.pack(1, STICK, START)
    send(packet + b'\x00')
    send(packet + bytes(100))
    assert receiver.rejected == 2 and not joys
    send(packet)
    assert len(joys) == 1


def test_rumble_goes_back_to_the_bridge_until_its_link_times_out(receiver):
    bridge = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rumble = JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=0.6)
    try:
        bridge.sendto(gamepad.pack(1, *NEUTRAL), receiver.socket.getsockname())
        assert select.select([receiver.socket], [], [], 2.0)[0], 'the packet never arrived'
        receiver.poll()
        receiver.on_feedback(rumble)
        assert select.select([bridge], [], [], 2.0)[0], 'the rumble never came back'
        assert gamepad.unpack_rumble(bridge.recv(64)) == pytest.approx(0.6)
        receiver.last_packet -= receiver.timeout + 0.01
        receiver.poll()
        assert receiver.sender is None
        receiver.on_feedback(rumble)
        assert not select.select([bridge], [], [], 0.2)[0], 'rumble sent over a timed-out link'
    finally:
        bridge.close()


def test_link_quality_counts_loss_jitter_and_the_longest_gap():
    link = LinkQuality()
    for sequence, now in ((1, 0.000), (2, 0.004), (3, 0.008), (6, 0.020), (7, 0.024)):
        link.arrived(sequence, now)
    loss, jitter, gap = link.take()
    # Packets 4 and 5 never came: 2 of 7.
    assert loss == pytest.approx(2 / 7)
    assert gap == pytest.approx(0.012)
    # Intervals 4, 4, 12, 4 ms vary by 0, 8 and 8 ms.
    assert jitter == pytest.approx(0.016 / 3)
    assert link.take() == (0.0, 0.0, 0.0)
    # A bridge that restarted counts from 1 again; that is not loss.
    link.restart()
    link.arrived(1, 5.0)
    link.arrived(2, 5.004)
    assert link.take()[0] == 0.0


def test_the_bridge_times_the_link_and_lost_packets_make_it_warn(receiver, monkeypatch):
    reports = []
    monkeypatch.setattr(receiver.diagnostics_publisher, 'publish', reports.append)
    bridge, stranger = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(2))

    def deliver(sock, data):
        sock.sendto(data, receiver.socket.getsockname())
        assert select.select([receiver.socket], [], [], 2.0)[0], 'the packet never arrived'
        receiver.poll()

    def link():
        receiver.report()
        status = reports[-1].status[0]
        return status, {value.key: value.value for value in status.values}

    try:
        for sequence in (1, 2, 3):
            deliver(bridge, gamepad.pack(sequence, *NEUTRAL))
        # Each report pings the bridge.
        _, values = link()
        assert values['round_trip_ms'] == ''
        assert select.select([bridge], [], [], 2.0)[0], 'the ping never came'
        ping = bridge.recv(64)
        # Only the bridge's answer times the link.
        deliver(stranger, gamepad.pong(ping))
        assert receiver.round_trip is None
        deliver(bridge, gamepad.pong(ping))
        assert receiver.round_trip is not None
        # Packets 4 to 9 never arrive.
        deliver(bridge, gamepad.pack(10, *NEUTRAL))
        status, values = link()
        assert status.level == DiagnosticStatus.WARN and status.message.startswith('Unstable')
        assert values['loss_percent'] == '85.7' and values['round_trip_ms'] != ''
    finally:
        bridge.close()
        stranger.close()


def test_only_the_first_answer_to_a_ping_of_this_node_times_the_link(receiver):
    bridge = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def deliver(data):
        bridge.sendto(data, receiver.socket.getsockname())
        assert select.select([receiver.socket], [], [], 2.0)[0], 'the packet never arrived'
        receiver.poll()

    try:
        deliver(gamepad.pack(1, *NEUTRAL))
        receiver.report()
        assert select.select([bridge], [], [], 2.0)[0], 'the ping never came'
        ping = bridge.recv(64)
        nonce, sent = gamepad.unpack_pong(gamepad.pong(ping))
        # A pong for a ping this node never sent does not count, even with a fresh time.
        deliver(gamepad.pong(gamepad.pack_ping(nonce + 1, sent)))
        assert receiver.round_trip is None
        deliver(gamepad.pong(ping))
        assert receiver.round_trip is not None
        # The same pong again, delayed or replayed, does not refresh the round trip.
        receiver.round_trip_time = None
        deliver(gamepad.pong(ping))
        assert receiver.round_trip_time is None
    finally:
        bridge.close()
