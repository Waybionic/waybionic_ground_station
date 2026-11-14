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
from diagnostic_msgs.msg import DiagnosticStatus
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node

from waybionic_control.protocol import codec


class MockDrivesNode(Node):
    """Node simulating 6 CAN-based joint controllers."""

    TEST_HEALTH_FAULTS = {
        'following_error', 'stalled', 'not_responding', 'disabled'}

    def __init__(self, **kwargs):
        """Initialize the MockDrivesNode and connect to the configured transport."""
        super().__init__('mock_drives', **kwargs)
        self.declare_parameter('simulate_faults', True)
        self.declare_parameter('can_interface', 'vcan0')
        self.declare_parameter('transport', 'socketcan')

        can_interface = self.get_parameter('can_interface').value
        self.transport = self.get_parameter('transport').value
        self.bus = None

        if self.transport == 'socketcan':
            try:
                self.bus = can.interface.Bus(
                    bustype='socketcan', channel=can_interface, fd=True)
                self.get_logger().info(f'SocketCAN active on {can_interface}')
            except (can.CanError, OSError) as e:
                self.get_logger().error(
                    f'SocketCAN init failed on {can_interface}: {e}. Node is '
                    'degraded; no frames will be published.')
        elif self.transport == 'udp_multicast':
            try:
                self.bus = can.interface.Bus(
                    bustype='udp_multicast', channel='224.0.0.1', fd=True)
                self.get_logger().warning(
                    'udp_multicast transport selected. This is NOT a physical '
                    'CAN link and must not be used for hardware validation.')
            except (can.CanError, OSError) as e:
                self.get_logger().error(f'udp_multicast init failed: {e}')
        else:
            self.get_logger().error(
                f'Unknown transport {self.transport!r}; '
                "expected 'socketcan' or 'udp_multicast'.")

        self.positions = {i: 0.0 for i in range(1, 7)}
        self.velocities = {i: 0.0 for i in range(1, 7)}
        self.targets = {i: 0.0 for i in range(1, 7)}
        self.enabled = {i: True for i in range(1, 7)}
        self.test_health_faults = {}
        self.active_test_health_fault = None
        self.test_health_fault_publisher = self.create_publisher(
            DiagnosticStatus, '~/test_health_fault', 10)
        self.declare_parameter('test_health_fault', 'none')
        self.add_on_set_parameters_callback(self.on_test_health_fault_parameter)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.count = 0
        self.get_logger().info('Mock drives started. Broadcasting 6 joints at 10Hz.')

    def on_test_health_fault_parameter(self, parameters):
        """Apply a test-only ``joint_id:fault`` parameter or clear with ``none``."""
        requested = next(
            (parameter.value for parameter in parameters
             if parameter.name == 'test_health_fault'), None)
        if requested is None:
            return SetParametersResult(successful=True)

        requested = requested.strip().lower()
        if requested == 'none':
            if self.active_test_health_fault is not None:
                joint_id, _ = self.active_test_health_fault
                self.set_test_health_fault(joint_id, None)
                self.publish_test_health_fault(joint_id, None)
                self.active_test_health_fault = None
            return SetParametersResult(successful=True)

        try:
            joint_text, fault = requested.split(':', 1)
            joint_id = int(joint_text)
        except ValueError:
            return SetParametersResult(
                successful=False,
                reason='Expected none or <joint_id>:<fault>',
            )

        if joint_id not in self.positions:
            return SetParametersResult(successful=False, reason='Joint ID must be 1 through 6')
        if fault not in self.TEST_HEALTH_FAULTS:
            return SetParametersResult(
                successful=False,
                reason=f'Fault must be one of {sorted(self.TEST_HEALTH_FAULTS)}',
            )

        previous = self.active_test_health_fault
        if previous is not None and previous[0] != joint_id:
            self.set_test_health_fault(previous[0], None)
            self.publish_test_health_fault(previous[0], None)
        self.set_test_health_fault(joint_id, fault)
        self.publish_test_health_fault(joint_id, fault)
        self.active_test_health_fault = (joint_id, fault)
        return SetParametersResult(successful=True)

    def publish_test_health_fault(self, joint_id, fault):
        """Publish the mock-only enabled state consumed by mock-mode diagnostics."""
        status = DiagnosticStatus(
            name='mock_drives.test_health_fault',
            hardware_id=f'joint_{joint_id}',
            message=fault or 'none',
        )
        self.test_health_fault_publisher.publish(status)

    def set_test_health_fault(self, joint_id, fault):
        """Set a test-only health fault for one simulated joint, or clear it."""
        if joint_id not in self.positions:
            raise ValueError(f'Unknown simulated joint {joint_id}')
        if fault is not None and fault not in self.TEST_HEALTH_FAULTS:
            raise ValueError(f'Unknown test health fault {fault!r}')
        self.test_health_faults[joint_id] = fault

    def timer_callback(self):
        if self.bus is None:
            return

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
            test_fault = self.test_health_faults.get(joint_id)
            self.enabled[joint_id] = test_fault != 'disabled'
            if test_fault == 'not_responding':
                continue
            if simulate_faults and joint_id == 6 and 30 < self.count <= 70:
                continue

            if test_fault in ('following_error', 'stalled'):
                self.velocities[joint_id] = 0.0
            else:
                diff = self.targets[joint_id] - self.positions[joint_id]
                self.velocities[joint_id] = diff * 2.0
                self.positions[joint_id] += diff * 0.5

            health_status = 1
            fault_code = 0

            # Provisional fault scenarios, pending Electrical confirmation.
            if simulate_faults:
                if joint_id == 4 and 50 < self.count <= 90:
                    health_status = 0
                    fault_code = 0xAA
                elif joint_id == 5 and 60 < self.count <= 100:
                    health_status = 0
                    fault_code = 0

            try:
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
                self.bus.send(msg)
            except ValueError as e:
                self.get_logger().warning(f'Skipped bad state for joint {joint_id}: {e}')
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
