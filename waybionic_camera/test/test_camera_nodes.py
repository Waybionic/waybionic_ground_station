"""Frames sent like camera_bridge sends them come out as stamped images and a delay report."""

import socket
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, CompressedImage

from waybionic_camera.camera_receiver import CameraReceiver, READ_TIMEOUT_S
from waybionic_camera.frames import pack
from waybionic_camera.latency_monitor import LatencyMonitor

NAMESPACE = '/camera_test'
DIAGNOSTICS = f'{NAMESPACE}/diagnostics'
DELAY_NS = 20_000_000
JPEG = b'\xff\xd8frame\xff\xd9'


@pytest.fixture
def nodes():
    context = rclpy.Context()
    rclpy.init(context=context)
    receiver = CameraReceiver(context=context, namespace=NAMESPACE, parameter_overrides=[
        Parameter('port', value=0)])
    monitor = LatencyMonitor(context=context, namespace=NAMESPACE, parameter_overrides=[
        Parameter('diagnostics_topic', value=DIAGNOSTICS)])
    collector = rclpy.create_node('camera_test_collector', context=context)
    received = {'images': [], 'infos': [], 'diagnostics': []}
    collector.create_subscription(
        CompressedImage, f'{NAMESPACE}/doctor_view/left/image_raw/compressed',
        received['images'].append, 50)
    collector.create_subscription(CameraInfo, f'{NAMESPACE}/doctor_view/left/camera_info',
                                  received['infos'].append, 50)
    collector.create_subscription(DiagnosticArray, DIAGNOSTICS, received['diagnostics'].append,
                                  10)
    executor = SingleThreadedExecutor(context=context)
    for node in (receiver, monitor, collector):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    yield receiver, received
    executor.shutdown()
    receiver.close()
    monitor.close()
    for node in (receiver, monitor, collector):
        node.destroy_node()
    rclpy.try_shutdown(context=context)
    spinner.join(timeout=5.0)


@pytest.fixture
def monitor(monkeypatch):
    context = rclpy.Context()
    rclpy.init(context=context)
    node = LatencyMonitor(context=context, namespace=NAMESPACE, parameter_overrides=[
        Parameter('diagnostics_topic', value=DIAGNOSTICS)])
    reports = []
    monkeypatch.setattr(node.publisher, 'publish', reports.append)
    yield node, reports
    node.destroy_node()
    rclpy.try_shutdown(context=context)


def wait_for(done, seconds):
    deadline = time.monotonic() + seconds
    while not done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return done()


def stream(sender, stamps, done, seconds):
    """Send a frame every 20 ms, as a live bridge does, until done() or the time runs out."""
    deadline = time.monotonic() + seconds
    while not done() and time.monotonic() < deadline:
        stamps.append(time.time_ns() - DELAY_NS)
        sender.sendall(pack(len(stamps), stamps[-1], 640, 480, JPEG))
        time.sleep(0.02)
    return done()


def statuses(messages, level):
    return [status for message in messages for status in message.status
            if status.level == level]


def test_bridge_frames_become_stamped_images_and_a_delay_report(nodes):
    receiver, received = nodes
    # Give discovery a moment so the first frames are not lost.
    time.sleep(1.0)
    stamps = []
    with socket.create_connection(('127.0.0.1', receiver.port)) as sender:
        assert stream(sender, stamps, lambda: len(received['images']) >= 35, 5.0)
        # Keep streaming until a report covers those frames.
        covered = len(received['diagnostics'])
        assert stream(sender, stamps, lambda: statuses(
            received['diagnostics'][covered:], DiagnosticStatus.OK), 3.0)
    first = received['images'][0]
    assert first.format == 'jpeg' and bytes(first.data) == JPEG
    stamp = first.header.stamp.sec * 1_000_000_000 + first.header.stamp.nanosec
    assert stamp in stamps
    assert first.header.frame_id == 'doctor_view_left_optical_frame'
    assert (received['infos'][0].width, received['infos'][0].height) == (640, 480)

    report = statuses(received['diagnostics'], DiagnosticStatus.OK)[-1]
    values = {item.key: item.value for item in report.values}
    assert 20.0 <= float(values['mean_ms']) < 60.0
    assert 30.0 < float(values['fps']) < 60.0
    assert report.name == 'camera.doctor_view' and 'at 640x480' in report.message

    # Once the frames stop, the report warns within a second or two, long before it goes stale.
    stopped = len(received['diagnostics'])
    assert wait_for(lambda: any(
        status.message.startswith('No frame for')
        for status in statuses(received['diagnostics'][stopped:], DiagnosticStatus.WARN)), 3.0)


def test_a_silent_connection_gives_way_to_the_next_bridge(nodes):
    receiver, received = nodes
    # A bridge that vanished without closing its connection, then connected again.
    silent = socket.create_connection(('127.0.0.1', receiver.port))
    with silent, socket.create_connection(('127.0.0.1', receiver.port)) as sender:
        sender.sendall(pack(1, time.time_ns(), 640, 480, JPEG))
        assert wait_for(lambda: received['images'], READ_TIMEOUT_S + 3.0)
        silent.settimeout(1.0)
        assert silent.recv(1) == b''


def test_close_wakes_a_receiver_waiting_for_a_bridge(nodes):
    receiver, _ = nodes
    receiver.close()
    assert not receiver.thread.is_alive()


def test_close_ends_a_live_connection_at_once(nodes):
    receiver, received = nodes
    with socket.create_connection(('127.0.0.1', receiver.port)) as sender:
        assert stream(sender, [], lambda: received['images'], 5.0)
        assert receiver.connection.getsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE)
        # The bridge goes quiet, so the receiver is waiting for its next frame.
        time.sleep(0.2)
        start = time.monotonic()
        receiver.close()
        assert time.monotonic() - start < READ_TIMEOUT_S / 2
        assert not receiver.thread.is_alive()
        sender.settimeout(1.0)
        assert sender.recv(1) == b''


def test_a_stalled_feed_warns(monitor):
    node, reports = monitor
    now = node.get_clock().now().nanoseconds * 1e-9
    # A second of frames that stopped half a second ago.
    for index in range(30):
        received = now - 1.5 + index / 30.0
        node.stats.add(received - 0.01, received)
    node.report()
    status = reports[-1].status[0]
    assert status.level == DiagnosticStatus.WARN
    assert status.message.startswith('No frame for')


def test_a_negative_delay_warns_of_clock_skew(monitor):
    node, reports = monitor
    now = node.get_clock().now().nanoseconds * 1e-9
    for index in range(30):
        received = now - 1.0 + index / 30.0
        # Stamped after it arrived: the camera computer's clock is ahead of this one.
        node.stats.add(received + 0.005, received)
    node.report()
    status = reports[-1].status[0]
    assert status.level == DiagnosticStatus.WARN
    assert 'clock skew' in status.message
