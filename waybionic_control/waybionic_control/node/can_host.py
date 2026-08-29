import struct
import time

import can
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CanHostNode(Node):
    def __init__(self):
        super().__init__('can_host')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.get_logger().info('Host connecting to software virtual CAN bus...')
        self.bus = can.interface.Bus(bustype='udp_multicast', channel='224.0.0.1')

        self.last_seen = {i: 0.0 for i in range(1, 7)}

        self.create_timer(0.05, self.read_bus)  # 20 Hz read loop
        self.create_timer(1.0, self.publish_diagnostics)  # 1 Hz diag loop
        self.get_logger().info('Host node started. Listening for joint data.')

    def read_bus(self):
        while True:
            msg = self.bus.recv(0.0)
            if msg is None:
                break

            if 0x101 <= msg.arbitration_id <= 0x106:
                joint_id = msg.arbitration_id - 0x100
                self.last_seen[joint_id] = time.time()

                if len(msg.data) == 5:
                    position, health = struct.unpack('<fB', msg.data)
                    self.publish_joint_state(joint_id, position)

    def publish_joint_state(self, joint_id, position):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f'joint_{joint_id}']
        js.position = [position]
        self.joint_pub.publish(js)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        current_time = time.time()
        for joint_id in range(1, 7):
            status = DiagnosticStatus()
            status.name = f'can.bus: Joint {joint_id} Heartbeat'
            status.hardware_id = f'joint_{joint_id}'

            if current_time - self.last_seen[joint_id] > 0.5:
                status.level = DiagnosticStatus.ERROR
                status.message = 'STALE (No heartbeat)'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'OK'

            diag_array.status.append(status)

        self.diag_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    node = CanHostNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
