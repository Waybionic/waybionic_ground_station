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

from waybionic_camera.camera_receiver import CameraReceiver
from waybionic_camera.frames import pack
from waybionic_camera.latency_monitor import LatencyMonitor

NAMESPACE = '/camera_test'
DIAGNOSTICS = f'{NAMESPACE}/diagnostics'
DELAY_NS = 20_000_000


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


def wait_for(done, seconds):
    deadline = time.monotonic() + seconds
    while not done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return done()


def test_bridge_frames_become_stamped_images_and_a_delay_report(nodes):
    receiver, received = nodes
    # Give discovery a moment so the first frames are not lost.
    time.sleep(1.0)
    stamps = []
    with socket.create_connection(('127.0.0.1', receiver.port)) as sender:
        for number in range(1, 41):
            stamps.append(time.time_ns() - DELAY_NS)
            sender.sendall(pack(number, stamps[-1], 640, 480, b'\xff\xd8frame\xff\xd9'))
            time.sleep(0.02)
    assert wait_for(lambda: len(received['images']) >= 35, 5.0)
    first = received['images'][0]
    assert first.format == 'jpeg' and bytes(first.data) == b'\xff\xd8frame\xff\xd9'
    stamp = first.header.stamp.sec * 1_000_000_000 + first.header.stamp.nanosec
    assert stamp in stamps
    assert first.header.frame_id == 'doctor_view_left_optical_frame'
    assert (received['infos'][0].width, received['infos'][0].height) == (640, 480)

    def report():
        statuses = [status for message in received['diagnostics'] for status in message.status
                    if status.level == DiagnosticStatus.OK]
        return statuses[-1] if statuses else None
    assert wait_for(report, 3.0)
    values = {item.key: item.value for item in report().values}
    assert 20.0 <= float(values['mean_ms']) < 60.0
    assert 30.0 < float(values['fps']) < 60.0
    assert report().name == 'camera.doctor_view' and 'at 640x480' in report().message
