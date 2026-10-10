"""Publish simulated claw positions and four tool-motor diagnostics."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState

from waybionic_teleop import mks_can
from waybionic_teleop.tool_board_frames import move_frames
from waybionic_teleop.tool_sim import SimulatedTool, tool_map_from_parameters


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
        self.sim = SimulatedTool(tool_map_from_parameters(params))
        self.last_frames = move_frames(self.sim.targets, self.sim.speeds)
        self.tolerance = float(params['pair_tolerance'])
        self.rejected = 0
        self.last_tick = time.monotonic()
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
            self.sim.command(positions)
            self.last_frames = move_frames(self.sim.targets, self.sim.speeds)
        except ValueError:
            self.rejected += 1

    def tick(self):
        """Move the motors and publish the two derived claw positions."""
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.1), now
        self.sim.tick(dt)
        positions, _, _ = self.sim.feedback()
        message = JointState(
            name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def report(self):
        """Report claws and motors with targets, limits and pair error."""
        positions, disagreement, counts = self.sim.feedback()
        statuses = []
        for claw in self.sim.map.claws:
            position = positions[claw.name]
            error = disagreement[claw.name]
            at_limit = position <= claw.lower or position >= claw.upper
            level = (DiagnosticStatus.WARN if error > self.tolerance
                     else DiagnosticStatus.OK)
            statuses.append(status(
                f'tool.{claw.name}', level, f'{position:.3f}', 'provisional',
                'paired motors disagree' if level else
                ('at limit' if at_limit else 'simulated'),
                target=f'{self.sim.target_positions[claw.name]:.3f}',
                lower=claw.lower, upper=claw.upper,
                pair_error=f'{error:.3f}', at_limit=at_limit))
            for motor in claw.motors:
                statuses.append(status(
                    f'tool.motor.{motor.name}', DiagnosticStatus.OK,
                    counts[motor.name], 'steps', 'simulated',
                    target_steps=self.sim.targets[motor.name],
                    direction=motor.direction,
                    can_id=self.last_frames[motor.name][0],
                    last_command=mks_can.hex_frame(*self.last_frames[motor.name])))
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
