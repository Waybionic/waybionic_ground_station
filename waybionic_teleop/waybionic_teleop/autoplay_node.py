"""
Play the controller demo whenever nobody is using the controller.

The real controller arrives on joy_operator and is passed on to teleop's joy topic. The demo
takes over joy as soon as teleop is ready, and again after idle_s seconds without a button,
stick or trigger moving; the moment someone touches the controller, it stops, presses B so
teleop is left disabled, and passes the controller through. It only plays while the drives
report that they are simulated and ready and no other node publishes joy, presses Start only
while the drives' command gate waits for it, and sends nothing while teleop's state, the joint
states or the drives' report are out of date, so teleop times out.
The demo draws the tool tip's path and a caption in RViz.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from waybionic_teleop.demo_script import (
    DemoPlayer, operator_active, PIVOT, STALE_S, TeleopState)
from waybionic_teleop.kinematics import ArmKinematics
from waybionic_teleop.ros_shutdown import is_shutdown_error

HOME_TOLERANCE_RAD = 0.01


class Autoplay(Node):
    """Switch teleop's controller input between the operator and the scripted demo."""

    def __init__(self, **kwargs):
        super().__init__('autoplay', **kwargs)
        self.idle = float(self.declare_parameter('idle_s', 30.0).value)
        self.deadzone = float(self.declare_parameter('deadzone', 0.15).value)
        rate = float(self.declare_parameter('rate_hz', 120.0).value)
        topic = self.declare_parameter('diagnostics_topic', '/diagnostics').value
        self.player = DemoPlayer(rate)
        self.state = TeleopState()
        self.joints = {}
        self.drive_interface, self.drives_reported = None, -math.inf
        self.unready = []
        self.kinematics = None
        self.playing = False
        self.handover = None
        self.reason = None
        # Nobody has used the controller yet, so the demo starts as soon as teleop is ready.
        self.last_used = -math.inf
        self.ticks = 0
        self.caption, self.trail, self.fixed = None, [], None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(Joy, 'joy_operator', self.on_operator, 10)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.create_subscription(DiagnosticArray, topic, self.on_diagnostics, 10)
        self.joy_publisher = self.create_publisher(Joy, 'joy', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'waybionic/teleop/markers', 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f'Demo plays after {self.idle:.0f} s without controller input; touch it to stop')

    def on_description(self, message):
        try:
            self.kinematics = ArmKinematics.from_urdf(message.data)
        except ValueError as error:
            self.get_logger().warning(f'No tool path in RViz: {error}')

    def on_joint_states(self, message):
        self.joints.update(zip(message.name, message.position))
        self.state.measured = time.monotonic()

    def on_diagnostics(self, message):
        now = time.monotonic()
        rows = {status.name: status for status in message.status}
        for status in message.status:
            values = {item.key: item.value for item in status.values}
            if status.name == 'teleop.state':
                self.state.enabled = values.get('value') == 'enabled'
                self.state.reported = now
            elif status.name == 'teleop.group':
                self.state.group = values.get('value')
            elif status.name == 'can.bus':
                self.drive_interface, self.drives_reported = values.get('interface'), now
                self.on_drives(rows)

    def on_drives(self, rows):
        """Read the command gate and drive rows the drive node reports with its bus."""
        gate = rows.get('arm.command_gate')
        drives = [status for name, status in rows.items() if name.startswith('drive.')]
        self.unready = [status.name for status in drives if status.level != DiagnosticStatus.OK]
        if gate is None or gate.level == DiagnosticStatus.ERROR:
            self.unready.insert(0, 'arm.command_gate')
        self.state.drives_ready = bool(drives) and not self.unready
        self.state.gate_open = gate is not None and gate.level == DiagnosticStatus.OK

    def on_operator(self, message):
        if operator_active(message.axes, message.buttons, self.deadzone):
            self.last_used = time.monotonic()
            if self.playing:
                self.playing = False
                # The demo's Start left teleop enabled; B disables it before handing over.
                self.handover = self.player.press('b')
                self.draw()
                self.get_logger().info('Controller in use: demo stopped; press Start to drive')
        if not self.playing and self.handover is None:
            self.joy_publisher.publish(message)

    def blocked(self, now):
        """Return why the demo cannot play now, or None when it can."""
        if (self.kinematics is None or not self.state.fresh(now)
                or now - self.drives_reported > STALE_S):
            return 'waiting for teleop, the joint states and the drives to report'
        if self.drive_interface != 'sim':
            return f'the drives are not simulated ({self.drive_interface})'
        if not self.state.drives_ready:
            return 'the drives are not ready: ' + (', '.join(self.unready) or 'no drive rows')
        if self.count_publishers(self.joy_publisher.topic_name) > 1:
            return 'another node publishes joy'
        return None

    def tick(self):
        now = time.monotonic()
        if self.handover is not None:
            sample = next(self.handover, None)
            if sample is None:
                self.handover = None
            self.send(sample)
            return
        reason = self.blocked(now)
        if reason != self.reason:
            self.reason = reason
            if reason:
                self.get_logger().info(f'Demo on hold: {reason}')
                # Send nothing, so teleop times out; the script starts over once all is well.
                self.player.restart()
                self.draw()
        if reason:
            return
        if not self.playing:
            if now - self.last_used < self.idle:
                return
            self.playing = True
            self.player.restart()
            self.get_logger().info('Controller idle: demo playing')
        self.state.at_home = all(abs(self.joints.get(joint, math.inf)) < HOME_TOLERANCE_RAD
                                 for joint in self.kinematics.joints)
        self.send(self.player.step(self.state))
        self.ticks += 1
        if self.ticks % 4 == 0:
            self.draw()

    def send(self, sample):
        if sample is None:
            return
        message = Joy(axes=sample[0], buttons=sample[1])
        message.header.stamp = self.get_clock().now().to_msg()
        self.joy_publisher.publish(message)

    def draw(self):
        caption = self.player.caption if self.playing else None
        tip = None
        if self.kinematics and all(joint in self.joints for joint in self.kinematics.joints):
            tip = self.kinematics.forward(self.joints)[0]
        if caption != self.caption:
            # Each part of the demo starts a new path; the pivot also marks the fixed tip.
            self.trail = []
            self.fixed = tip if caption == PIVOT else None
            self.caption = caption
        if caption and tip is not None:
            self.trail.append(tip)
        items = [self.marker('demo_trail', Marker.LINE_STRIP, len(self.trail) > 1),
                 self.marker('demo_point', Marker.SPHERE, self.fixed is not None),
                 # RViz aborts on text with no visible glyphs, so an empty caption is deleted.
                 self.marker('demo_caption', Marker.TEXT_VIEW_FACING, bool(caption))]
        trail, point, text = items
        trail.points = [Point(x=x, y=y, z=z) for x, y, z in self.trail]
        trail.scale.x = 0.004
        trail.color.r, trail.color.g, trail.color.b, trail.color.a = 1.0, 0.45, 0.1, 1.0
        if self.fixed is not None:
            point.pose.position.x, point.pose.position.y, point.pose.position.z = self.fixed
        point.scale.x = point.scale.y = point.scale.z = 0.024
        point.color.r, point.color.g, point.color.b, point.color.a = 1.0, 0.15, 0.15, 0.6
        text.text = caption or ''
        text.pose.position.x, text.pose.position.y, text.pose.position.z = 0.42, -0.02, 0.36
        text.scale.z = 0.05
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        self.marker_publisher.publish(MarkerArray(markers=items))

    @staticmethod
    def marker(namespace, kind, shown):
        item = Marker(ns=namespace, id=0, type=kind,
                      action=Marker.ADD if shown else Marker.DELETE)
        item.header.frame_id = 'base_link'
        item.pose.orientation.w = 1.0
        return item


def main():
    rclpy.init()
    node = Autoplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        if not is_shutdown_error(error):
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
