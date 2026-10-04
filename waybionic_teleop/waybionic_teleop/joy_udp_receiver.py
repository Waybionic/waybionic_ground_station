"""
Publish sensor_msgs/Joy from the controller packets that a host bridge sends over UDP.

The bridge may run on another computer, so the receiver also watches the link: packets lost,
how unevenly they arrive, the longest gap, and the round trip of a ping the bridge answers.
"""

import socket
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy, JoyFeedback

from waybionic_teleop.gamepad import (pack_ping, pack_rumble, PACKET_SIZE, PONG_MAGIC, unpack,
                                      unpack_pong)

# Past these the operator feels the arm stutter or lag, so the monitor warns.
UNSTABLE_LOSS = 0.02
UNSTABLE_GAP_S = 0.1
UNSTABLE_ROUND_TRIP_S = 0.05
PING_EVERY_S = 0.5


class LinkQuality:
    """Loss, arrival jitter and the longest gap between packets, per report window."""

    def __init__(self):
        self.received = self.lost = self.variations = 0
        self.variation = self.worst_gap = 0.0
        self.restart()

    def restart(self):
        """Forget the last packet, as after a bridge restart that reset its count."""
        self.sequence = self.time = self.interval = None

    def arrived(self, sequence, now):
        if self.sequence is not None:
            self.lost += max(((sequence - self.sequence) & 0xFFFFFFFF) - 1, 0)
            interval = now - self.time
            self.worst_gap = max(self.worst_gap, interval)
            if self.interval is not None:
                self.variation += abs(interval - self.interval)
                self.variations += 1
            self.interval = interval
        self.received += 1
        self.sequence, self.time = sequence, now

    def take(self):
        """Return (loss fraction, mean jitter in s, longest gap in s), then start a new window."""
        total = self.received + self.lost
        result = (self.lost / total if total else 0.0,
                  self.variation / self.variations if self.variations else 0.0,
                  self.worst_gap)
        self.received = self.lost = self.variations = 0
        self.variation = self.worst_gap = 0.0
        return result


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
        # Same rumble topic as game_controller_node, passed back to the host bridge.
        self.create_subscription(JoyFeedback, 'joy/set_feedback', self.on_feedback, 10)
        self.sequence = None
        self.last_packet = None
        self.connected = False
        self.sender = None
        self.published = 0
        self.rejected = 0
        self.link = LinkQuality()
        self.pings = 0
        self.round_trip, self.round_trip_time = None, None
        self.report_time = time.monotonic()
        # Every millisecond, so a packet waits at most that long before it reaches teleop.
        self.create_timer(0.001, self.poll)
        self.create_timer(PING_EVERY_S, self.report)
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
            now = time.monotonic()
            if data[:4] == PONG_MAGIC:
                self.on_pong(data, now)
                continue
            try:
                sequence, connected, axes, buttons = unpack(data)
            except ValueError:
                self.rejected += 1
                continue
            # A pause longer than the timeout means the bridge restarted and its count reset.
            resumed = self.last_packet is None or now - self.last_packet > self.timeout
            if not resumed and not 0 < (sequence - self.sequence) & 0xFFFFFFFF < 0x80000000:
                continue
            if resumed:
                self.link.restart()
            self.link.arrived(sequence, now)
            self.sequence, self.last_packet, self.sender = sequence, now, sender
            self.connected = connected
            latest = (axes, buttons) if connected else None
        if latest is not None:
            message = Joy(axes=latest[0], buttons=latest[1])
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'joy'
            self.joy_publisher.publish(message)
            self.published += 1

    def on_pong(self, data, now):
        try:
            _, sent = unpack_pong(data)
        except ValueError:
            self.rejected += 1
            return
        if 0.0 <= now - sent < 10.0:
            self.round_trip, self.round_trip_time = now - sent, now

    def on_feedback(self, message):
        if message.type != JoyFeedback.TYPE_RUMBLE or message.id != 0 or self.sender is None:
            return
        try:
            self.socket.sendto(pack_rumble(message.intensity), self.sender)
        except OSError as error:
            self.get_logger().warning(f'Rumble not sent: {error}', throttle_duration_sec=5.0)

    def report(self):
        now = time.monotonic()
        rate = self.published / max(now - self.report_time, 1e-3)
        self.published, self.report_time = 0, now
        loss, jitter, gap = self.link.take()
        if self.sender is not None:
            self.pings += 1
            try:
                self.socket.sendto(pack_ping(self.pings, now), self.sender)
            except OSError as error:
                self.get_logger().warning(f'Ping not sent: {error}', throttle_duration_sec=5.0)
        # Bridges from before the ping was added never answer it.
        round_trip = (self.round_trip if self.round_trip_time is not None
                      and now - self.round_trip_time < 4 * PING_EVERY_S else None)
        quality = (f'{100.0 * loss:.1f}% lost, jitter {1000.0 * jitter:.1f} ms, '
                   f'longest gap {1000.0 * gap:.0f} ms, round trip '
                   + ('unknown' if round_trip is None else f'{1000.0 * round_trip:.1f} ms'))
        status = DiagnosticStatus(name='controller.link', level=DiagnosticStatus.OK)
        if self.last_packet is None or now - self.last_packet > self.timeout:
            status.level = DiagnosticStatus.STALE
            status.message = ('No packets on udp {}:{}; run the controller bridge on the host'
                              .format(*self.address))
        elif not self.connected:
            status.level = DiagnosticStatus.WARN
            status.message = 'Host bridge is running but no controller is connected'
        elif (loss > UNSTABLE_LOSS or gap > UNSTABLE_GAP_S
              or (round_trip or 0.0) > UNSTABLE_ROUND_TRIP_S):
            status.level = DiagnosticStatus.WARN
            status.message = f'Unstable link from {self.sender[0]}: {quality}'
        else:
            status.message = f'Receiving from {self.sender[0]}: {quality}'
        status.values = [
            KeyValue(key='value', value=f'{rate:.0f}'), KeyValue(key='unit', value='Hz'),
            KeyValue(key='rejected', value=str(self.rejected)),
            KeyValue(key='loss_percent', value=f'{100.0 * loss:.1f}'),
            KeyValue(key='jitter_ms', value=f'{1000.0 * jitter:.1f}'),
            KeyValue(key='longest_gap_ms', value=f'{1000.0 * gap:.0f}'),
            KeyValue(key='round_trip_ms',
                     value='' if round_trip is None else f'{1000.0 * round_trip:.1f}')]
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
