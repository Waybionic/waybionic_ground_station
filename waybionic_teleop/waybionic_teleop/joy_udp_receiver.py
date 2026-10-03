"""Publish sensor_msgs/Joy from the controller packets that xinput_bridge sends from a host."""

import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy

from waybionic_teleop.gamepad import PACKET_SIZE, unpack


class JoyUdpReceiver(Node):
    """Validate controller packets and republish the newest one on /joy."""

    def __init__(self):
        super().__init__('joy_udp_receiver')
        self.address = (self.declare_parameter('bind_address', '127.0.0.1').value,
                        self.declare_parameter('port', 47300).value)
        self.timeout = self.declare_parameter('timeout_s', 0.5).value
        topic = self.declare_parameter('diagnostics_topic', '/diagnostics').value
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(self.address)
        self.socket.setblocking(False)
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.sequence = None
        self.last_packet = None
        self.connected = False
        self.sender = None
        self.published = 0
        self.rejected = 0
        self.report_time = time.monotonic()
        self.create_timer(0.005, self.poll)
        self.create_timer(0.5, self.report)
        self.get_logger().info('Waiting for controller packets on udp {}:{}'.format(*self.address))

    def poll(self):
        latest = None
        # Bounded so a flood of packets cannot starve the node's other callbacks.
        for _ in range(100):
            try:
                data, sender = self.socket.recvfrom(PACKET_SIZE + 1)
            except BlockingIOError:
                break
            except OSError as error:
                self.get_logger().warning(f'UDP receive failed: {error}',
                                          throttle_duration_sec=5.0)
                break
            try:
                sequence, connected, axes, buttons = unpack(data)
            except ValueError:
                self.rejected += 1
                continue
            now = time.monotonic()
            # A pause longer than the timeout means the bridge restarted and its count reset.
            resumed = self.last_packet is None or now - self.last_packet > self.timeout
            if not resumed and not 0 < (sequence - self.sequence) & 0xFFFFFFFF < 0x80000000:
                continue
            self.sequence, self.last_packet, self.sender = sequence, now, sender
            self.connected = connected
            latest = (axes, buttons) if connected else None
        if latest is not None:
            message = Joy(axes=latest[0], buttons=latest[1])
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'joy'
            self.joy_publisher.publish(message)
            self.published += 1

    def report(self):
        now = time.monotonic()
        rate = self.published / max(now - self.report_time, 1e-3)
        self.published, self.report_time = 0, now
        status = DiagnosticStatus(name='controller.link', level=DiagnosticStatus.OK)
        if self.last_packet is None or now - self.last_packet > self.timeout:
            status.level = DiagnosticStatus.STALE
            status.message = ('No packets on udp {}:{}; run xinput_bridge on the host'
                              .format(*self.address))
        elif not self.connected:
            status.level = DiagnosticStatus.WARN
            status.message = 'Host bridge is running but no controller is connected'
        else:
            status.message = f'Receiving from {self.sender[0]}'
        status.values = [KeyValue(key='value', value=f'{rate:.0f}'),
                         KeyValue(key='unit', value='Hz'),
                         KeyValue(key='rejected', value=str(self.rejected))]
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)


def main():
    rclpy.init()
    node = JoyUdpReceiver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.socket.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
