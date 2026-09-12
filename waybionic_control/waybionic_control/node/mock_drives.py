# Copyright 2026 Waybionic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import can
import rclpy
from rclpy.node import Node

from waybionic_control.protocol import codec


class MockDrivesNode(Node):
    """Node simulating 6 CAN-based joint controllers."""

    def __init__(self):
        """Initialize the MockDrivesNode and connect to the virtual CAN bus."""
        super().__init__('mock_drives')
        self.declare_parameter('simulate_faults', True)
        self.declare_parameter('can_interface', 'vcan0')

        can_interface = self.get_parameter('can_interface').value
        self.get_logger().info(f'Connecting to virtual CAN bus on {can_interface}...')

        try:
            self.bus = can.interface.Bus(bustype='socketcan', channel=can_interface, fd=True)
        except Exception as e:
            self.get_logger().warning(f'SocketCAN failed ({e}), falling back to udp_multicast')
            self.bus = can.interface.Bus(bustype='udp_multicast', channel='224.0.0.1', fd=True)

        self.positions = {i: 0.0 for i in range(1, 7)}
        self.velocities = {i: 0.0 for i in range(1, 7)}
        self.targets = {i: 0.0 for i in range(1, 7)}

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0
        self.get_logger().info('Mock drives started. Broadcasting 6 joints at 10Hz.')

    def timer_callback(self):
        simulate_faults = self.get_parameter('simulate_faults').value

        # Process max 100 messages per tick to prevent infinite blocking
        for _ in range(100):
            msg = self.bus.recv(0.0)
            if msg is None:
                break
            if codec.CMD_BASE_ID + 1 <= msg.arbitration_id <= codec.CMD_BASE_ID + 6:
                joint_id = msg.arbitration_id - codec.CMD_BASE_ID
                try:
                    target_pos, target_vel = codec.decode_target_command(msg.data)
                    self.targets[joint_id] = target_pos
                except ValueError as e:
                    self.get_logger().warning(f'Ignored bad command: {e}')

        for joint_id in range(1, 7):
            if simulate_faults and joint_id == 6 and self.count > 30:
                continue

            diff = self.targets[joint_id] - self.positions[joint_id]
            self.velocities[joint_id] = diff * 2.0
            self.positions[joint_id] += diff * 0.5

            health_status = 1
            fault_code = 0

            if simulate_faults and joint_id == 4 and self.count > 50:
                health_status = 0
                fault_code = 0xAA

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
