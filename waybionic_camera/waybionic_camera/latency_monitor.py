"""
Report a camera's frame rate and delay on /diagnostics, and optionally log every frame.

The delay is the time a frame arrives minus its header stamp, so it covers capture to arrival
as long as the camera computer's clock matches this one. Any camera topic works: the camera
bridge's compressed frames, or a ROS camera driver's raw images.
"""

import csv

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from waybionic_camera.latency import FrameStats


class LatencyMonitor(Node):
    """Measure the frames on one image topic."""

    def __init__(self, **kwargs):
        super().__init__('camera_latency_monitor', **kwargs)
        topic = self.declare_parameter('topic', 'doctor_view/left/image_raw/compressed').value
        compressed = self.declare_parameter('compressed', True).value
        info_topic = self.declare_parameter('info_topic', 'doctor_view/left/camera_info').value
        self.name = self.declare_parameter('name', 'camera.doctor_view').value
        self.max_delay = float(self.declare_parameter('max_delay_ms', 100.0).value)
        self.min_fps = float(self.declare_parameter('min_fps', 25.0).value)
        diagnostics = self.declare_parameter('diagnostics_topic', '/diagnostics').value
        log_path = self.declare_parameter('log_csv', '').value
        self.topic = self.resolve_topic_name(topic)
        self.stats = FrameStats()
        self.size = None
        self.log = None
        if log_path:
            self.log_file = open(log_path, 'w', newline='', encoding='utf-8')
            self.log = csv.writer(self.log_file)
            self.log.writerow(['frame', 'captured_s', 'received_s', 'delay_ms', 'width',
                               'height'])
        self.create_subscription(CompressedImage if compressed else Image, topic, self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self.on_info, qos_profile_sensor_data)
        self.publisher = self.create_publisher(DiagnosticArray, diagnostics, 10)
        self.create_timer(1.0, self.report)

    def on_info(self, message):
        self.size = (message.width, message.height)

    def on_image(self, message):
        if isinstance(message, Image):
            self.size = (message.width, message.height)
        stamp = message.header.stamp
        captured = stamp.sec + stamp.nanosec * 1e-9
        received = self.get_clock().now().nanoseconds * 1e-9
        self.stats.add(captured, received)
        if self.log is not None:
            width, height = self.size or ('', '')
            self.log.writerow([self.stats.total, f'{captured:.6f}', f'{received:.6f}',
                               f'{1000.0 * (received - captured):.2f}', width, height])

    def report(self):
        summary = self.stats.summary(self.get_clock().now().nanoseconds * 1e-9)
        if summary is None:
            level, value, text = DiagnosticStatus.STALE, '', f'No frames on {self.topic}'
        else:
            slow = summary['fps'] < self.min_fps or summary['mean_ms'] > self.max_delay
            level = DiagnosticStatus.WARN if slow else DiagnosticStatus.OK
            value = f'{summary["mean_ms"]:.0f}'
            size = f' at {self.size[0]}x{self.size[1]}' if self.size else ''
            text = (f'{summary["fps"]:.1f} fps, delay {summary["mean_ms"]:.0f} ms (min '
                    f'{summary["min_ms"]:.0f}, max {summary["max_ms"]:.0f}, jitter '
                    f'{summary["jitter_ms"]:.1f}){size}')
        values = [KeyValue(key='value', value=value), KeyValue(key='unit', value='ms'),
                  KeyValue(key='frames', value=str(self.stats.total))]
        if summary is not None:
            values += [KeyValue(key=key, value=f'{number:.2f}') for key, number in summary.items()]
        message = DiagnosticArray(status=[DiagnosticStatus(
            name=self.name, level=level, message=text, hardware_id=self.topic, values=values)])
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)
        if self.log is not None:
            self.log_file.flush()

    def close(self):
        if self.log is not None:
            self.log_file.close()


def main():
    rclpy.init()
    node = LatencyMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
