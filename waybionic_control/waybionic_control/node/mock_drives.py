import can
import rclpy
from rclpy.node import Node

from waybionic_control.protocol import codec


class MockDrivesNode(Node):
    def __init__(self):
        super().__init__('mock_drives')
        self.declare_parameter('simulate_faults', True)

        self.get_logger().info('Connecting to software virtual CAN bus...')
        self.bus = can.interface.Bus(bustype='udp_multicast', channel='224.0.0.1')

        self.positions = {i: 0.0 for i in range(1, 7)}
        self.velocities = {i: 0.0 for i in range(1, 7)}
        self.targets = {i: 0.0 for i in range(1, 7)}

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.count = 0
        self.get_logger().info('Mock drives started. Broadcasting 6 joints at 10Hz.')

    def timer_callback(self):
        simulate_faults = self.get_parameter('simulate_faults').value

        # 1. Read incoming command frames from the host
        while True:
            msg = self.bus.recv(0.0)
            if msg is None:
                break
            if codec.CMD_BASE_ID + 1 <= msg.arbitration_id <= codec.CMD_BASE_ID + 6:
                joint_id = msg.arbitration_id - codec.CMD_BASE_ID
                target_pos, target_vel = codec.decode_target_command(msg.data)
                self.targets[joint_id] = target_pos

        # 2. Simulate movement and broadcast state frames back to the host
        for joint_id in range(1, 7):
            # Simulate STALE fault (Joint 6 dies after 30 ticks)
            if simulate_faults and joint_id == 6 and self.count > 30:
                continue

            # Basic simulation: smoothly move toward the target position
            diff = self.targets[joint_id] - self.positions[joint_id]
            self.velocities[joint_id] = diff * 2.0
            self.positions[joint_id] += diff * 0.5

            health_status = 1
            fault_code = 0

            # Simulate HARDWARE FAULT (Joint 4 throws error 0xAA after 50 ticks)
            if simulate_faults and joint_id == 4 and self.count > 50:
                health_status = 0
                fault_code = 0xAA  # Fake error code (e.g., Motor Overcurrent)

            # Pack 10 bytes of data (CAN-FD allows > 8 bytes)
            data = codec.encode_joint_state(
                self.positions[joint_id],
                self.velocities[joint_id],
                health_status,
                fault_code
            )

            msg = can.Message(
                arbitration_id=codec.STATE_BASE_ID + joint_id,
                data=data,
                is_extended_id=False,
                is_fd=True
            )

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
