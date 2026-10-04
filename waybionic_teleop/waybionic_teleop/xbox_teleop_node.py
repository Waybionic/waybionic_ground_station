"""
Drive the arm from an Xbox controller: /joy in, /joint_commands out.

Mapping and limits come from config/xbox_teleop.yaml and the robot_description joint limits;
the logic lives in :mod:`waybionic_teleop.teleop`. The node also reports its state on
/diagnostics and draws a placeholder end effector and a status label in RViz.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy, JoyFeedback
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.kinematics import ArmKinematics, joint_limits
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

JAW_SIZE = (0.006, 0.014, 0.03)
JAW_OPEN_GAP = 0.024
# One short rumble each time a move is stopped at a limit.
RUMBLE_INTENSITY = 0.6
RUMBLE_S = 0.25


def stop_reason(blocked):
    """Name what stopped a move, one word per line for the RViz label."""
    return '\n'.join('OUT-OF-REACH' if item == 'reach'
                     else 'STOPPED-' + item.replace(': ', '-').replace(' ', '-')
                     for item in blocked[:2])


def status(name, level, value, unit, message):
    return DiagnosticStatus(name=name, level=level, message=message, values=[
        KeyValue(key='value', value=value), KeyValue(key='unit', value=unit)])


class XboxTeleop(Node):
    """Turn controller input into streamed joint targets while the operator has it enabled."""

    def __init__(self, **kwargs):
        super().__init__('xbox_teleop', automatically_declare_parameters_from_overrides=True,
                         **kwargs)
        params = {name: self.get_parameter(name).value
                  for name in self.list_parameters([], 0).names}
        self.config = config_from_parameters(params)
        self.timeout = float(params['input_timeout_s'])
        self.tool_frame = params['tool_frame']
        self.base_frame = params['base_frame']
        self.table_height = float(params.get('table_height', 0.0))
        self.clearance = float(params.get('collision_clearance', 0.01))
        self.teleop = None
        self.problem = 'Waiting for robot_description'
        self.joy = None
        self.joy_time = None
        self.joy_count = 0
        self.measured = {}
        self.ticks = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.load, latched)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.command_publisher = self.create_publisher(JointState, 'joint_commands', 10)
        # game_controller_node rumbles a local controller; joy_udp_receiver passes it on.
        self.feedback_publisher = self.create_publisher(JoyFeedback, 'joy/set_feedback', 10)
        self.rumble_until = None
        self.was_blocked = False
        self.marker_publisher = self.create_publisher(MarkerArray, 'waybionic/teleop/markers', 10)
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, params.get('diagnostics_topic', '/diagnostics'), 10)
        self.last_tick = self.report_time = time.monotonic()
        self.create_timer(1.0 / float(params['rate_hz']), self.tick)
        self.create_timer(0.5, self.report)

    def load(self, message):
        if self.teleop is not None and self.teleop.enabled:
            # The new description replaces the teleop state, so hold the arm where it is first.
            self.teleop.disable(self.measured, 'Robot description changed')
            self.publish_command()
            self.get_logger().warning('Robot description changed: teleop disabled, arm held')
        joints = {joint for group in self.config.groups for joint in group.joints}
        try:
            limits = joint_limits(message.data, joints)
        except ValueError as error:
            self.teleop, self.problem = None, str(error)
            self.get_logger().error(f'Teleop disabled: {error}')
            return
        try:
            kinematics = ArmKinematics.from_urdf(message.data, self.base_frame, self.tool_frame)
        except ValueError as error:
            kinematics = None
            self.get_logger().warning(f'Cartesian group unavailable: {error}')
        try:
            collision = ArmCollision.from_urdf(message.data, self.table_height, self.clearance)
        except ValueError as error:
            collision = None
            self.get_logger().warning(f'No table or base collision checks: {error}')
        self.teleop, self.problem = ArmTeleop(self.config, limits, kinematics, collision), ''
        self.get_logger().info('Xbox teleop ready (press Start to enable): ' + '; '.join(
            f'{group.name}: {group.describe()}' for group in self.teleop.groups))

    def on_joy(self, message):
        self.joy, self.joy_time = message, time.monotonic()
        self.joy_count += 1

    def on_joint_states(self, message):
        self.measured.update((name, position) for name, position
                             in zip(message.name, message.position) if math.isfinite(position))

    def fresh(self, now):
        return self.joy_time is not None and now - self.joy_time <= self.timeout

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.1), now
        self.ticks += 1
        if self.teleop is None:
            return
        fresh = self.fresh(now)
        if not fresh and self.teleop.enabled:
            self.teleop.disable(self.measured, 'Controller lost; press Start (Menu) to enable',
                                warning=True)
            self.publish_command()
        axes, buttons = (self.joy.axes, self.joy.buttons) if fresh else ((), ())
        if self.teleop.update(axes, buttons, self.measured, dt):
            self.publish_command()
        self.feel_limits(now)
        if self.ticks % 3 == 0:
            self.publish_markers(fresh)

    def feel_limits(self, now):
        blocked = self.teleop.enabled and bool(self.teleop.blocked)
        if blocked and not self.was_blocked:
            self.rumble(RUMBLE_INTENSITY)
            self.rumble_until = now + RUMBLE_S
        elif self.rumble_until is not None and now >= self.rumble_until:
            self.rumble(0.0)
            self.rumble_until = None
        self.was_blocked = blocked

    def rumble(self, intensity):
        self.feedback_publisher.publish(
            JoyFeedback(type=JoyFeedback.TYPE_RUMBLE, id=0, intensity=intensity))

    def publish_command(self):
        targets = self.teleop.targets
        if not targets:
            return
        message = JointState(name=list(targets), position=list(targets.values()),
                             velocity=[self.teleop.velocities[joint] for joint in targets])
        message.header.stamp = self.get_clock().now().to_msg()
        self.command_publisher.publish(message)

    def summary(self, fresh):
        # RViz text markers misplace words separated by spaces, so use one word per line.
        teleop = self.teleop
        if teleop.enabled:
            state = 'ENABLED'
            if teleop.blocked:
                state += '\n' + stop_reason(teleop.blocked)
        else:
            state = 'DISABLED\nPRESS-MENU' if fresh else 'NO-CONTROLLER'
        return f'{teleop.active_group.name.upper()}\n{self.speed_percent():.0f}%\n{state}'

    def speed_percent(self):
        return 100.0 * self.config.speed_levels[self.teleop.level]

    def publish_markers(self, fresh):
        low, high = self.config.tool_limits
        closed = min(max((self.measured.get(self.config.tool_joint, low) - low) / (high - low),
                         0.0), 1.0)
        gap = JAW_OPEN_GAP * (1.0 - closed)
        markers = MarkerArray()
        # Placeholder jaws until Biomedical provides the end effector model.
        for index, side in enumerate((-1.0, 1.0)):
            jaw = Marker(ns='tool', id=index, type=Marker.CUBE, action=Marker.ADD,
                         frame_locked=True)
            jaw.header.frame_id = self.tool_frame
            jaw.pose.position.x = side * (gap + JAW_SIZE[0]) / 2.0
            jaw.pose.position.z = JAW_SIZE[2] / 2.0
            jaw.pose.orientation.w = 1.0
            jaw.scale.x, jaw.scale.y, jaw.scale.z = JAW_SIZE
            jaw.color.r, jaw.color.g, jaw.color.b, jaw.color.a = 0.3, 0.75, 1.0, 0.9
            markers.markers.append(jaw)
        label = Marker(ns='status', id=0, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD,
                       text=self.summary(fresh))
        label.header.frame_id = self.base_frame
        label.pose.position.x, label.pose.position.y, label.pose.position.z = 0.3, -0.3, 0.1
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        enabled = self.teleop.enabled and fresh
        label.color.r, label.color.g, label.color.b, label.color.a = (
            (1.0, 0.35, 0.3, 1.0) if enabled and self.teleop.blocked
            else (0.5, 1.0, 0.5, 1.0) if enabled else (1.0, 0.8, 0.3, 1.0))
        markers.markers.append(label)
        self.marker_publisher.publish(markers)

    def report(self):
        now = time.monotonic()
        rate = self.joy_count / max(now - self.report_time, 1e-3)
        self.joy_count, self.report_time = 0, now
        fresh = self.fresh(now)
        input_level = DiagnosticStatus.OK if fresh else DiagnosticStatus.STALE
        statuses = [status('teleop.input', input_level, f'{rate:.0f}', 'Hz',
                           '' if fresh else 'No /joy input from a controller')]
        teleop = self.teleop
        if teleop is None:
            statuses.append(status('teleop.state', DiagnosticStatus.ERROR, 'disabled', '',
                                   self.problem))
        else:
            group = teleop.active_group
            cartesian = group.mode == 'cartesian'
            hint = 'B stops, Y switches group, A holds to go home'
            note = teleop.note or ('At limit: ' + ', '.join(teleop.blocked) if teleop.blocked
                                   else ('LB moves along one axis; ' if cartesian else '') + hint)
            speed = (f'{1000.0 * teleop.linear_speed:.1f} mm/s' if cartesian
                     else f'{math.degrees(teleop.speed):.0f} deg/s')
            low, high = self.config.tool_limits
            closed = (self.measured.get(self.config.tool_joint, low) - low) / (high - low)
            statuses += [
                status('teleop.state', DiagnosticStatus.WARN if teleop.warning
                       else DiagnosticStatus.OK, 'enabled' if teleop.enabled else 'disabled',
                       '', note),
                status('teleop.group', DiagnosticStatus.OK, group.name, '', group.describe()),
                status('teleop.speed', DiagnosticStatus.OK, f'{self.speed_percent():.0f}', '%',
                       f'{speed} max; D-pad up/down changes it'),
                status('teleop.tool', DiagnosticStatus.OK, f'{100.0 * closed:.0f}', '%',
                       'closed (placeholder end effector); RT closes, LT opens'),
            ]
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)


def main():
    rclpy.init()
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
