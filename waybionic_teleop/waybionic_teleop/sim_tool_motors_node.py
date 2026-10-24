"""Publish simulated claw positions and four tool-motor diagnostics."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState

from std_msgs.msg import Bool

from waybionic_teleop import mks_can
from waybionic_teleop.tool_board_frames import BOARD_MAX_RPM, move_frames, stop_frames
from waybionic_teleop.tool_sim import (
    SimulatedTool, tool_map_from_parameters, ToolSafety)


def status(name, level, value, unit, message, **extra):
    """Build one diagnostic status with its numeric value and metadata."""
    values = [KeyValue(key='value', value=str(value)),
              KeyValue(key='unit', value=unit)]
    values += [KeyValue(key=key, value=str(item))
               for key, item in extra.items()]
    return DiagnosticStatus(name=name, level=level, message=message,
                            values=values)


class SimToolMotors(Node):
    """Run a simulation-only tool board with two claws and four motors."""

    def __init__(self):
        """Load calibration and start command, state and status IO."""
        super().__init__('sim_tool_motors',
                         automatically_declare_parameters_from_overrides=True)
        params = {name: self.get_parameter(name).value
                  for name in self.list_parameters([], 0).names}
        self.board_safe_rpm = params['board_safe_rpm']
        self.board_acceleration_code = params['board_acceleration_code']
        if (not isinstance(self.board_safe_rpm, int) or isinstance(self.board_safe_rpm, bool)
                or not 1 <= self.board_safe_rpm <= BOARD_MAX_RPM):
            raise ValueError('board_safe_rpm must be 1-60')
        if (not isinstance(self.board_acceleration_code, int)
                or isinstance(self.board_acceleration_code, bool)
                or not 0 <= self.board_acceleration_code <= 255):
            raise ValueError('board_acceleration_code must be an F5 byte')
        self.sim = SimulatedTool(tool_map_from_parameters(params))
        stalled = params.get('simulated_stalled_motor', '')
        self.sim.set_stalled_motors([stalled] if stalled else [])
        self.safety = ToolSafety(self.sim, float(params['command_timeout_s']),
                                 time.monotonic())
        self.last_frames = stop_frames()
        self.tolerance = float(params['pair_tolerance'])
        self.rejected = 0
        self.create_subscription(Bool, 'teleop_enabled', self.on_enabled, 10)
        self.create_subscription(
            JointState, 'tool_commands', self.on_command, 10)
        self.state_publisher = self.create_publisher(
            JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            params.get('diagnostics_topic', '/diagnostics'), 10)
        self.create_timer(1.0 / float(params['rate_hz']), self.tick)
        self.create_timer(0.5, self.report)
        self.get_logger().info(
            'Four simulated tool motors ready; no hardware connection')

    def on_enabled(self, message):
        """Hold the tool whenever teleop disables its command stream."""
        if self.safety.update_enable(message.data, time.monotonic()):
            self.last_frames = stop_frames()

    def on_command(self, message):
        """Accept only a complete, finite pair of claw targets."""
        names = set(self.sim.target_positions)
        positions = {
            name: value for name, value in zip(message.name, message.position)
            if name in names}
        if (set(positions) != names
                or not all(math.isfinite(v) for v in positions.values())):
            self.rejected += 1
            return
        try:
            if self.safety.command(positions, time.monotonic()):
                self.last_frames = move_frames(
                    self.sim.targets, self.sim.speeds,
                    acceleration=self.board_acceleration_code,
                    max_rpm=self.board_safe_rpm)
        except ValueError:
            self.rejected += 1

    def tick(self):
        """Move the motors and publish the two derived claw positions."""
        now = time.monotonic()
        if self.safety.tick(now):
            self.last_frames = stop_frames()
        positions, _, _ = self.sim.feedback()
        message = JointState(
            name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def report(self):
        """Report claws and motors with targets, limits and pair error."""
        positions, disagreement, counts = self.sim.feedback()
        statuses = []
        gate_level = (DiagnosticStatus.OK if self.safety.enabled
                      else DiagnosticStatus.WARN)
        gate_message = ('teleop enabled' if self.safety.enabled
                        else self.safety.stop_reason)
        statuses.append(status(
            'tool.command_gate', gate_level, self.safety.enabled, 'enabled',
            gate_message, requires_release=self.safety.requires_release))
        for claw in self.sim.map.claws:
            position = positions[claw.name]
            error = disagreement[claw.name]
            at_limit = position <= claw.lower or position >= claw.upper
            stalled = [motor.name for motor in claw.motors
                       if motor.name in self.sim.stalled_motors]
            level = (DiagnosticStatus.WARN if stalled or error > self.tolerance
                     else DiagnosticStatus.OK)
            statuses.append(status(
                f'tool.{claw.name}', level, f'{position:.3f}', 'provisional',
                ('simulated motor stall' if stalled else
                 'paired motors disagree' if error > self.tolerance else
                 'at limit' if at_limit else 'simulated'),
                target=f'{self.sim.target_positions[claw.name]:.3f}',
                lower=claw.lower, upper=claw.upper,
                pair_error=f'{error:.3f}', at_limit=at_limit,
                stalled_motors=','.join(stalled)))
            endpoints = [
                self.sim.map.target_steps({
                    candidate.name: (bound if candidate.name == claw.name
                                     else candidate.reference)
                    for candidate in self.sim.map.claws})
                for bound in (claw.lower, claw.upper)]
            for motor in claw.motors:
                low_steps, high_steps = sorted(
                    endpoint[motor.name] for endpoint in endpoints)
                motor_stalled = motor.name in self.sim.stalled_motors
                statuses.append(status(
                    f'tool.motor.{motor.name}',
                    DiagnosticStatus.WARN if motor_stalled
                    else DiagnosticStatus.OK,
                    counts[motor.name], 'steps',
                    'simulated motor stall' if motor_stalled else 'simulated',
                    stalled=motor_stalled,
                    target_steps=self.sim.targets[motor.name],
                    lower_steps=low_steps, upper_steps=high_steps,
                    at_limit=counts[motor.name] in (low_steps, high_steps),
                    direction=motor.direction,
                    can_id=self.last_frames[motor.name][0],
                    last_command=mks_can.hex_frame(
                        *self.last_frames[motor.name])))
        statuses.append(status('tool.commands', DiagnosticStatus.OK,
                               self.rejected, 'rejected', 'simulation only'))
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)


def main():
    """Run the simulated tool until ROS shuts down."""
    rclpy.init()
    node = SimToolMotors()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
