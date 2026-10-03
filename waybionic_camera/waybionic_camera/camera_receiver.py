"""
Publish the camera bridge's frames as the doctor view.

Frames arrive over TCP from camera_bridge and are published, still JPEG-compressed, on
doctor_view/left/image_raw/compressed with doctor_view/left/camera_info, both stamped with
the time the frame was captured. RViz shows them with the compressed image transport.
"""

import socket
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage

from waybionic_camera.frames import read_frame


def to_messages(frame_id, stamp_ns, width, height, jpeg):
    """Return the CompressedImage and CameraInfo for one frame."""
    image = CompressedImage(format='jpeg', data=jpeg)
    info = CameraInfo(width=width, height=height)
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    for message in (image, info):
        message.header.frame_id = frame_id
        message.header.stamp.sec, message.header.stamp.nanosec = seconds, nanoseconds
    return image, info


class CameraReceiver(Node):
    """Accept one camera bridge at a time and republish its frames."""

    def __init__(self, **kwargs):
        super().__init__('camera_receiver', **kwargs)
        bind = self.declare_parameter('bind_address', '127.0.0.1').value
        port = int(self.declare_parameter('port', 47310).value)
        self.frame_id = self.declare_parameter(
            'frame_id', 'doctor_view_left_optical_frame').value
        self.image_publisher = self.create_publisher(
            CompressedImage, 'doctor_view/left/image_raw/compressed', 10)
        self.info_publisher = self.create_publisher(CameraInfo, 'doctor_view/left/camera_info', 10)
        self.server = socket.create_server((bind, port))
        self.port = self.server.getsockname()[1]
        self.running = True
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Waiting for camera_bridge on {bind}:{self.port}/tcp')

    def serve(self):
        while self.running:
            try:
                connection, address = self.server.accept()
            except OSError:
                return
            self.get_logger().info(f'Camera bridge connected from {address[0]}')
            with connection:
                try:
                    while (frame := read_frame(connection)) is not None:
                        _, stamp_ns, width, height, jpeg = frame
                        image, info = to_messages(self.frame_id, stamp_ns, width, height, jpeg)
                        self.image_publisher.publish(image)
                        self.info_publisher.publish(info)
                except (OSError, ValueError) as error:
                    self.get_logger().warning(f'Camera stream dropped: {error}')
            self.get_logger().info('Camera bridge disconnected')

    def close(self):
        self.running = False
        self.server.close()


def main():
    rclpy.init()
    node = CameraReceiver()
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
