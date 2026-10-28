"""ROS 2 node: receive host UDP joint states and publish /joint_states for RViz.

Always republishes the latest (or zero) pose at a fixed rate so RobotModel/TF
stay healthy even before the Windows teleop starts streaming.
"""

from __future__ import annotations

import socket
import sys

from .viz_udp import DEFAULT_PORT, JOINT_NAMES, unpack_joint_state


def main(argv: list[str] | None = None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
    except ImportError:
        print("rclpy/sensor_msgs required (run inside ROS 2 Jazzy).", file=sys.stderr)
        return 1

    class UdpJointStateReceiver(Node):
        def __init__(self) -> None:
            super().__init__("embedded_udp_joint_state_receiver")
            self.declare_parameter("port", DEFAULT_PORT)
            self.declare_parameter("bind", "0.0.0.0")
            self.declare_parameter("rate_hz", 50.0)
            port = int(self.get_parameter("port").value)
            bind = self.get_parameter("bind").get_parameter_value().string_value
            rate_hz = float(self.get_parameter("rate_hz").value)

            self._names = list(JOINT_NAMES)
            self._positions = [0.0] * len(self._names)
            self._velocities = [0.0] * len(self._names)

            self._pub = self.create_publisher(JointState, "/joint_states", 10)
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((bind, port))
            self._sock.setblocking(False)
            self.get_logger().info(
                f"Listening for host joint states on UDP {bind}:{port} "
                f"(publishing zeros until teleop connects)"
            )
            self.create_timer(0.01, self._poll_udp)
            self.create_timer(1.0 / max(rate_hz, 1.0), self._publish)

        def _poll_udp(self) -> None:
            while True:
                try:
                    data, _addr = self._sock.recvfrom(65535)
                except BlockingIOError:
                    break
                except Exception as exc:
                    self.get_logger().warn(f"UDP read error: {exc}")
                    break
                try:
                    names, positions, velocities = unpack_joint_state(data)
                except Exception as exc:
                    self.get_logger().warn(f"Bad UDP packet: {exc}")
                    continue
                self._names = names
                self._positions = positions
                self._velocities = velocities

        def _publish(self) -> None:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self._names)
            msg.position = list(self._positions)
            msg.velocity = list(self._velocities)
            self._pub.publish(msg)

        def destroy_node(self) -> bool:
            try:
                self._sock.close()
            except Exception:
                pass
            return super().destroy_node()

    rclpy.init(args=argv)
    node = UdpJointStateReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
