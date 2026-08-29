import struct

import can
import rclpy
from rclpy.node import Node


class MockDrivesNode(Node):
    def __init__(self):
        super().__init__('mock_drives')
        self.declare_parameter('simulate_stale_joint', False)

        self.get_logger().info('Connecting to software virtual CAN bus...')
        self.bus = can.interface.Bus(bustype='udp_multicast', channel='224.0.0.1')

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.count = 0
        self.get_logger().info('Mock drives started. Broadcasting 6 joints at 10Hz.')

    def timer_callback(self):
        simulate_stale = self.get_parameter('simulate_stale_joint').value

        for joint_id in range(1, 7):
            if simulate_stale and joint_id == 6 and self.count > 30:
                continue

            fake_position = 0.0
            health_status = 1
            data = struct.pack('<fB', fake_position, health_status)

            msg = can.Message(arbitration_id=0x100 + joint_id, data=data, is_extended_id=False)

            try:
                self.bus.send(msg)
            except can.CanError as e:
                self.get_logger().error(f'CAN error: {e}')

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MockDrivesNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
