"""ROS 2 bridge: Float64 joint velocities -> UNO ASCII serial.

Topics / services (initial):
  /embedded/j1_velocity   std_msgs/Float64   >0 CW, <0 CCW, ~0 STOP
  /embedded/j3_velocity   std_msgs/Float64
  /embedded/wrist_cmd     std_msgs/String    w|s|a|d|stop  (or DIFF_*)
  /embedded/estop         std_srvs/Trigger
  /embedded/enable        std_srvs/Trigger
  /embedded/disable       std_srvs/Trigger
  /embedded/firmware_line std_msgs/String    (firmware stdout)

Parameters:
  port (string, default /dev/ttyACM0)
  baud (int, default 115200)
  watchdog_s (double, default 0.5)

Run inside a sourced ROS 2 Jazzy workspace that contains this package, or:
  python -m waybionic_embedded_bridge.ros_serial_bridge_node --ros-args -p port:=COM3

On Windows Docker Desktop, prefer keyboard_teleop / host serial — containers
cannot open COM ports. Use native WSL+usbipd or Linux for this node with USB.
"""

from __future__ import annotations

import sys

from . import protocol
from .serial_client import UnoSerialClient


def main(argv: list[str] | None = None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64, String
        from std_srvs.srv import Trigger
    except ImportError:
        print(
            "rclpy is not available in this Python. "
            "Use keyboard_teleop on Windows, or run inside ROS 2 Jazzy.",
            file=sys.stderr,
        )
        return 1

    class EmbeddedSerialBridge(Node):
        def __init__(self) -> None:
            super().__init__("waybionic_embedded_serial_bridge")
            self.declare_parameter("port", "/dev/ttyACM0")
            self.declare_parameter("baud", protocol.BAUD)
            self.declare_parameter("watchdog_s", 0.5)
            port = self.get_parameter("port").get_parameter_value().string_value
            baud = int(self.get_parameter("baud").value)
            watchdog_s = float(self.get_parameter("watchdog_s").value)

            self._line_pub = self.create_publisher(String, "/embedded/firmware_line", 10)

            def on_line(line: str) -> None:
                msg = String()
                msg.data = line
                self._line_pub.publish(msg)
                self.get_logger().info(f"FW: {line}")

            self._client = UnoSerialClient(
                port=port, baud=baud, watchdog_s=watchdog_s, on_line=on_line
            )
            self.get_logger().info(f"Opening serial {port} @ {baud}")
            self._client.open()

            self.create_subscription(Float64, "/embedded/j1_velocity", self._on_j1, 10)
            self.create_subscription(Float64, "/embedded/j3_velocity", self._on_j3, 10)
            self.create_subscription(String, "/embedded/wrist_cmd", self._on_wrist, 10)
            self.create_service(Trigger, "/embedded/estop", self._srv_estop)
            self.create_service(Trigger, "/embedded/enable", self._srv_enable)
            self.create_service(Trigger, "/embedded/disable", self._srv_disable)
            self.get_logger().info("Embedded serial bridge ready (J2 ignored by design)")

        def _on_j1(self, msg: Float64) -> None:
            cmd = protocol.velocity_to_joint_cmd("j1", msg.data)
            if cmd:
                self._client.send(cmd)

        def _on_j3(self, msg: Float64) -> None:
            cmd = protocol.velocity_to_joint_cmd("j3", msg.data)
            if cmd:
                self._client.send(cmd)

        def _on_wrist(self, msg: String) -> None:
            raw = msg.data.strip().lower()
            if raw in protocol.DIFF:
                self._client.send(protocol.DIFF[raw])
            elif raw.upper().startswith("DIFF_"):
                self._client.send(raw.upper())
            else:
                self.get_logger().warn(f"Unknown wrist_cmd: {msg.data!r}")

        def _srv_estop(self, _req, res):
            self._client.estop()
            res.success = True
            res.message = "ESTOP sent"
            return res

        def _srv_enable(self, _req, res):
            self._client.enable()
            res.success = True
            res.message = "ENABLE sent"
            return res

        def _srv_disable(self, _req, res):
            self._client.disable()
            res.success = True
            res.message = "DISABLE sent"
            return res

        def destroy_node(self) -> bool:
            try:
                self._client.close()
            except Exception:
                pass
            return super().destroy_node()

    rclpy.init(args=argv)
    node = EmbeddedSerialBridge()
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
