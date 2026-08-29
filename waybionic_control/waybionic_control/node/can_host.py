import time

import can
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from waybionic_control.protocol import codec


class CanHostNode(Node):
    def __init__(self):
        super().__init__('can_host')

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Subscribe to incoming commands from the high-level ROS 2 system
        self.cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.command_callback, 10)

        self.get_logger().info('Host connecting to software virtual CAN bus...')
        self.bus = can.interface.Bus(bustype='udp_multicast', channel='224.0.0.1')

        # State tracking for diagnostics
        self.last_seen = {i: 0.0 for i in range(1, 7)}
        self.faults = {i: 0 for i in range(1, 7)}
        self.last_cmd_time = 0.0

        self.create_timer(0.05, self.read_bus)  # 20 Hz read loop
        self.create_timer(1.0, self.publish_diagnostics)  # 1 Hz diag loop
        self.get_logger().info('Host node started. Ready for bidirectional CAN.')

    def command_callback(self, msg):
        self.last_cmd_time = time.time()
        # Parse the incoming ROS command and send it down the CAN bus
        for i, name in enumerate(msg.name):
            if name.startswith('joint_'):
                try:
                    joint_id = int(name.split('_')[1])
                    if 1 <= joint_id <= 6:
                        target_pos = msg.position[i] if i < len(msg.position) else 0.0
                        target_vel = msg.velocity[i] if i < len(msg.velocity) else 0.0

                        data = codec.encode_target_command(target_pos, target_vel)
                        can_msg = can.Message(
                            arbitration_id=codec.CMD_BASE_ID + joint_id,
                            data=data,
                            is_extended_id=False,
                            is_fd=True
                        )
                        self.bus.send(can_msg)
                except (ValueError, IndexError, can.CanError) as e:
                    self.get_logger().error(f'Command error: {e}')

    def read_bus(self):
        while True:
            msg = self.bus.recv(0.0)
            if msg is None:
                break

            if codec.STATE_BASE_ID + 1 <= msg.arbitration_id <= codec.STATE_BASE_ID + 6:
                joint_id = msg.arbitration_id - codec.STATE_BASE_ID
                self.last_seen[joint_id] = time.time()

                pos, vel, health, fault = codec.decode_joint_state(msg.data)
                self.faults[joint_id] = fault
                self.publish_joint_state(joint_id, pos, vel)

    def publish_joint_state(self, joint_id, pos, vel):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f'joint_{joint_id}']
        js.position = [pos]
        js.velocity = [vel]
        self.joint_pub.publish(js)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        current_time = time.time()

        # 1. Bus Alive Status
        bus_stat = DiagnosticStatus(
          name='can.bus: Link Status',
          level=DiagnosticStatus.OK,
          message='ACTIVE'
        )

        diag_array.status.append(bus_stat)

        # 2. Command Age Status
        cmd_stat = DiagnosticStatus(name='can.bus: Command Age')
        cmd_age = current_time - self.last_cmd_time
        if self.last_cmd_time == 0.0:
            cmd_stat.level = DiagnosticStatus.WARN
            cmd_stat.message = 'NO COMMANDS RECEIVED YET'
        elif cmd_age > 1.0:
            cmd_stat.level = DiagnosticStatus.WARN
            cmd_stat.message = f'STALE COMMANDS ({cmd_age:.1f}s ago)'
        else:
            cmd_stat.level = DiagnosticStatus.OK
            cmd_stat.message = f'ACTIVE ({cmd_age:.1f}s ago)'
        diag_array.status.append(cmd_stat)

        # 3. Individual Joint Status
        for joint_id in range(1, 7):
            status = DiagnosticStatus()
            status.name = f'can.bus: Joint {joint_id} Health'
            status.hardware_id = f'joint_{joint_id}'

            # Add raw fault code as key/value pair
            status.values.append(
                KeyValue(key='fault_code', value=hex(self.faults[joint_id]))
            )

            if current_time - self.last_seen[joint_id] > 0.5:
                status.level = DiagnosticStatus.ERROR
                status.message = 'STALE (No heartbeat)'
            elif self.faults[joint_id] != 0:
                status.level = DiagnosticStatus.ERROR
                status.message = f'HARDWARE FAULT (Code: {hex(self.faults[joint_id])})'
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
