#!/usr/bin/env python3
"""Sweep each arm joint in turn and check the TF it produces (simulated joint states only)."""

import math
import xml.etree.ElementTree as ET

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import tf2_ros

ANGLE_TOLERANCE = math.radians(0.1)
OFFSET_TOLERANCE = 1e-4


def floats(text, default):
    return tuple(float(value) for value in text.split()) if text else default


def multiply(a, b):
    """Multiply quaternions given as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def from_rpy(roll, pitch, yaw):
    """Return the URDF fixed-axis roll, pitch, yaw rotation as a quaternion."""
    x = (math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2))
    y = (0.0, math.sin(pitch / 2), 0.0, math.cos(pitch / 2))
    z = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return multiply(z, multiply(y, x))


def twist(zero, measured, axis):
    """Return the angle of measured relative to zero about axis."""
    x, y, z, w = multiply((-zero[0], -zero[1], -zero[2], zero[3]), measured)
    along = x * axis[0] + y * axis[1] + z * axis[2]
    return math.remainder(2.0 * math.atan2(along, w), 2 * math.pi)


def parse_joint(element):
    origin = element.find('origin')
    axis = floats(element.find('axis').get('xyz') if element.find('axis') is not None else '',
                  (1.0, 0.0, 0.0))
    length = math.sqrt(sum(value * value for value in axis))
    limit = element.find('limit')
    lower, upper = -math.pi, math.pi
    if element.get('type') == 'revolute' and limit is not None:
        lower, upper = float(limit.get('lower', 0.0)), float(limit.get('upper', 0.0))
    return {
        'name': element.get('name'),
        'parent': element.find('parent').get('link'),
        'child': element.find('child').get('link'),
        'axis': tuple(value / length for value in axis),
        'xyz': floats(origin.get('xyz') if origin is not None else '', (0.0, 0.0, 0.0)),
        'zero': from_rpy(*floats(origin.get('rpy') if origin is not None else '',
                                 (0.0, 0.0, 0.0))),
        'lower': lower,
        'upper': upper,
    }


class JointDemo(Node):
    """Move one joint at a time and verify that robot_state_publisher follows it."""

    def __init__(self):
        super().__init__('joint_demo')
        speed = self.declare_parameter('speed_deg_s', 30.0).value
        if not speed > 0.0:
            raise ValueError(f'speed_deg_s must be positive, not {speed}')
        self.speed = math.radians(speed)
        self.fraction = self.declare_parameter('amplitude_fraction', 0.5).value
        self.dwell = self.declare_parameter('dwell_s', 0.5).value
        self.loop = self.declare_parameter('loop', True).value
        topic = self.declare_parameter('diagnostics_topic', '/diagnostics').value
        self.period = 1.0 / 30.0
        self.joints = []
        self.followers = {}
        self.positions = {}
        self.results = {}
        self.errors = {}
        self.current = []
        self.plan = []
        self.step_index = 0
        self.remaining = None
        self.passes = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.load, latched)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.create_timer(self.period, self.step)
        self.create_timer(0.5, self.report)
        self.get_logger().info(
            'Joint demo waiting for robot_description (no hardware is commanded)')

    def load(self, message):
        robot = ET.fromstring(message.data)
        self.joints, self.followers = [], {}
        for element in robot.findall('joint'):
            if element.get('type') not in ('revolute', 'continuous'):
                continue
            joint = parse_joint(element)
            mimic = element.find('mimic')
            if mimic is None:
                self.joints.append(joint)
                continue
            joint['multiplier'] = float(mimic.get('multiplier', 1.0))
            joint['offset'] = float(mimic.get('offset', 0.0))
            self.followers.setdefault(mimic.get('joint'), []).append(joint)
        self.positions = {joint['name']: 0.0 for joint in self.joints}
        self.results = {joint['name']: 'pending' for joint in self.joints}
        self.errors = {}
        self.plan = []
        dwell_ticks = max(1, round(self.dwell / self.period))
        for index, joint in enumerate(self.joints):
            high = self.fraction * max(joint['upper'], 0.0)
            low = self.fraction * min(joint['lower'], 0.0)
            self.plan.append(('start', index, None))
            for target in (high, low):
                self.plan += [('move', index, target), ('dwell', index, dwell_ticks),
                              ('check', index, None)]
            self.plan += [('move', index, 0.0), ('finish', index, None)]
        self.step_index = 0
        self.get_logger().info(
            f'Testing {len(self.joints)} joints in turn: '
            + ', '.join(joint['name'] for joint in self.joints))

    def step(self):
        if not self.joints:
            return
        if self.step_index >= len(self.plan):
            if not self.loop:
                self.publish_joints()
                return
            self.step_index = 0
        kind, index, value = self.plan[self.step_index]
        joint = self.joints[index]
        if kind == 'start':
            if self.results[joint['name']] == 'pending':
                self.results[joint['name']] = 'testing'
            self.current = []
            self.step_index += 1
        elif kind == 'move':
            current = self.positions[joint['name']]
            limit = self.speed * self.period
            if abs(value - current) <= limit:
                self.positions[joint['name']] = value
                self.step_index += 1
            else:
                self.positions[joint['name']] = current + math.copysign(limit, value - current)
        elif kind == 'dwell':
            self.remaining = value if self.remaining is None else self.remaining - 1
            if self.remaining <= 0:
                self.remaining = None
                self.step_index += 1
        elif kind == 'check':
            self.check(joint)
            self.step_index += 1
        else:
            self.finish(joint, index)
            self.step_index += 1
        self.publish_joints()

    def check(self, joint):
        command = self.positions[joint['name']]
        problems = self.current
        for moved in [joint] + self.followers.get(joint['name'], []):
            expected = command * moved.get('multiplier', 1.0) + moved.get('offset', 0.0)
            try:
                transform = self.buffer.lookup_transform(moved['parent'], moved['child'], Time())
            except tf2_ros.TransformException:
                problems.append(f"no TF {moved['parent']} -> {moved['child']}")
                continue
            rotation = transform.transform.rotation
            angle = twist(moved['zero'], (rotation.x, rotation.y, rotation.z, rotation.w),
                          moved['axis'])
            offset = transform.transform.translation
            shift = math.dist((offset.x, offset.y, offset.z), moved['xyz'])
            if abs(math.remainder(angle - expected, 2 * math.pi)) > ANGLE_TOLERANCE:
                problems.append(f"{moved['child']} at {math.degrees(angle):+.1f} deg, "
                                f'expected {math.degrees(expected):+.1f}')
            if shift > OFFSET_TOLERANCE:
                problems.append(f"{moved['child']} shifted {shift * 1000:.1f} mm")

    def finish(self, joint, index):
        problems = list(dict.fromkeys(self.current))
        self.errors[joint['name']] = problems
        self.results[joint['name']] = 'fail' if problems else 'pass'
        high = math.degrees(self.fraction * max(joint['upper'], 0.0))
        low = math.degrees(self.fraction * min(joint['lower'], 0.0))
        followers = ', '.join(moved['name'] for moved in self.followers.get(joint['name'], []))
        summary = (f"{joint['name']} ({joint['parent']} -> {joint['child']}"
                   + (f'; drives {followers}' if followers else '') + '): '
                   + ('; '.join(problems) if problems else
                      f'pass, followed {high:+.0f} and {low:+.0f} deg'))
        (self.get_logger().error if problems else self.get_logger().info)(summary)
        if index == len(self.joints) - 1:
            self.passes += 1
            passed = sum(result == 'pass' for result in self.results.values())
            self.get_logger().info(
                f'Joint test pass {self.passes}: {passed}/{len(self.joints)} passed')

    def publish_joints(self):
        message = JointState(name=list(self.positions), position=list(self.positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.joint_publisher.publish(message)

    def report(self):
        if not self.joints:
            return
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        levels = {'pass': DiagnosticStatus.OK, 'testing': DiagnosticStatus.OK,
                  'pending': DiagnosticStatus.WARN, 'fail': DiagnosticStatus.ERROR}
        for joint in self.joints:
            result = self.results[joint['name']]
            angle = math.degrees(self.positions[joint['name']])
            status = DiagnosticStatus(name='arm.' + joint['name'], level=levels[result])
            problems = '; '.join(self.errors.get(joint['name'], []))
            status.message = {'pending': 'Not tested yet', 'fail': problems}.get(result, '')
            status.values = [KeyValue(key='value', value=f'{angle:+.1f}'),
                             KeyValue(key='unit', value='deg'),
                             KeyValue(key='result', value=result)]
            message.status.append(status)
        results = list(self.results.values())
        passed = results.count('pass')
        overall = 'fail' if 'fail' in results else 'pass' if self.passes else 'running'
        summary = DiagnosticStatus(name='arm.joint_test', level={
            'fail': DiagnosticStatus.ERROR, 'pass': DiagnosticStatus.OK,
            'running': DiagnosticStatus.WARN}[overall])
        summary.message = {'fail': 'A joint did not follow its command',
                           'running': 'First sweep in progress'}.get(overall, '')
        summary.values = [KeyValue(key='value', value=f'{passed}/{len(self.joints)}'),
                          KeyValue(key='unit', value='joints'),
                          KeyValue(key='result', value=overall)]
        message.status.append(summary)
        self.diagnostics_publisher.publish(message)


def main():
    rclpy.init()
    node = JointDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
