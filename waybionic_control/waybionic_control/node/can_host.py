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

import time

import can
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from waybionic_control.protocol import codec


class CanHostNode(Node):
    """Node that translates ROS JointState messages to CAN frames and monitors health."""

    def __init__(self, **kwargs):
        super().__init__('can_host', **kwargs)

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        self.cmd_sub = self.create_subscription(
            JointState, '/joint_commands', self.command_callback, 10)

        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('transport', 'socketcan')

        can_interface = self.get_parameter('can_interface').value
        self.transport = self.get_parameter('transport').value
        self.bus = None
        self.bus_error = None

        if self.transport == 'socketcan':
            try:
                self.bus = can.interface.Bus(
                    bustype='socketcan', channel=can_interface, fd=True)
                self.get_logger().info(f'SocketCAN active on {can_interface}')
            except (can.CanError, OSError) as e:
                self.bus_error = str(e)
                self.get_logger().error(
                    f'SocketCAN init failed on {can_interface}: {e}. Node is '
                    'degraded; no frames sent or received. Set '
                    'transport:=udp_multicast to use UDP explicitly.')
        elif self.transport == 'udp_multicast':
            try:
                self.bus = can.interface.Bus(
                    bustype='udp_multicast', channel='224.0.0.1', fd=True)
                self.get_logger().warning(
                    'udp_multicast transport selected. This is NOT a physical '
                    'CAN link and must not be used for hardware validation.')
            except (can.CanError, OSError) as e:
                self.bus_error = str(e)
                self.get_logger().error(f'udp_multicast init failed: {e}')
        else:
            self.bus_error = f'unknown transport {self.transport!r}'
            self.get_logger().error(
                f'Unknown transport {self.transport!r}; '
                "expected 'socketcan' or 'udp_multicast'.")

        self.last_seen = {i: 0.0 for i in range(1, 7)}
        self.faults = {i: 0 for i in range(1, 7)}
        self.healths = {i: 1 for i in range(1, 7)}
        self.last_cmd_time = 0.0

        self.create_timer(0.05, self.read_bus)
        self.create_timer(1.0, self.publish_diagnostics)
        self.get_logger().info('Host node started. Ready for bidirectional CAN.')

    def command_callback(self, msg):
        if self.bus is None:
            self.get_logger().warning(
                'Command dropped: no CAN transport available',
                throttle_duration_sec=5.0)
            return

        self.last_cmd_time = time.time()
        for i, name in enumerate(msg.name):
            if name.startswith('joint_'):
                if i >= len(msg.position):
                    self.get_logger().warning(f'Rejecting command {name}: missing position')
                    continue

                try:
                    joint_id = int(name.split('_')[1])
                    if 1 <= joint_id <= 6:
                        target_pos = msg.position[i]
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
        if self.bus is None:
            return

        # Process max 100 messages per tick to prevent infinite blocking
        for _ in range(100):
            msg = self.bus.recv(0.0)
            if msg is None:
                break

            if codec.STATE_BASE_ID + 1 <= msg.arbitration_id <= codec.STATE_BASE_ID + 6:
                joint_id = msg.arbitration_id - codec.STATE_BASE_ID
                try:
                    pos, vel, health, fault = codec.decode_joint_state(msg.data)
                    self.last_seen[joint_id] = time.time()
                    self.faults[joint_id] = fault
                    self.healths[joint_id] = health
                    self.publish_joint_state(joint_id, pos, vel)
                except ValueError as e:
                    self.get_logger().warning(f'Ignored bad state: {e}')

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

        if self.bus is None:
            bus_stat = DiagnosticStatus(
                name='can.bus: Link Status',
                level=DiagnosticStatus.ERROR,
                message=f'DOWN ({self.transport} init failed: {self.bus_error})'
            )
        else:
            bus_stat = DiagnosticStatus(
                name='can.bus: Link Status',
                level=DiagnosticStatus.OK,
                message=f'ACTIVE ({self.transport})'
            )
        bus_stat.values.append(
            KeyValue(key='transport', value=str(self.transport)))
        diag_array.status.append(bus_stat)

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

        for joint_id in range(1, 7):
            status = DiagnosticStatus()
            status.name = f'can.bus: Joint {joint_id} Health'
            status.hardware_id = f'joint_{joint_id}'

            status.values.append(KeyValue(key='fault_code', value=hex(self.faults[joint_id])))
            status.values.append(KeyValue(key='health', value=str(self.healths[joint_id])))

            if current_time - self.last_seen[joint_id] > 0.5:
                status.level = DiagnosticStatus.ERROR
                status.message = 'STALE (No heartbeat)'
            elif self.faults[joint_id] != 0:
                status.level = DiagnosticStatus.ERROR
                status.message = f'HARDWARE FAULT (Code: {hex(self.faults[joint_id])})'
            elif self.healths[joint_id] == 0:
                status.level = DiagnosticStatus.ERROR
                status.message = 'UNHEALTHY (health=0, no fault code)'
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
