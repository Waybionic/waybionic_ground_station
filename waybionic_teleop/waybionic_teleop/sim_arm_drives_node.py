"""
Run the arm's joint drives in simulation: /joint_commands in, CAN frames, /joint_states out.

The node talks to the drives only through CAN frames from :mod:`waybionic_teleop.mks_can`, so
replacing :class:`~waybionic_teleop.sim_drives.SimulatedBus` with a real CAN interface keeps
the command, feedback and diagnostics path unchanged. Nothing here reaches hardware.

Each drive goes through the start-up in :mod:`waybionic_teleop.drive_link`. The arm moves only
while every drive is ready: if one stops answering, the others are stopped, /joint_states pauses
and the commands are dropped. Once all drives are ready again, the node holds the measured pose
and ignores /joint_commands until they start from it, so teleop has to be disabled and enabled
again before the arm follows. ~/zero (std_srvs/Trigger) sets every drive's encoder to 0 with
92h; call it only with the arm in the zero pose, where it points straight up.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from waybionic_teleop import mks_can
from waybionic_teleop.drive_link import DriveLink
from waybionic_teleop.drive_map import drive_map_from_parameters
from waybionic_teleop.kinematics import joint_limits
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

REPLY_TIMEOUT_S = 0.5
ZERO_TIMEOUT_S = 2.0
# How far a command may be from the measured pose and still be taken after a hold.
SYNC_TOLERANCE_RAD = 0.05


def status(name, level, value, unit, message, **extra):
    values = [KeyValue(key='value', value=value), KeyValue(key='unit', value=unit)]
    values += [KeyValue(key=key, value=str(item)) for key, item in extra.items()]
    return DiagnosticStatus(name=name, level=level, message=message, values=values)


class SimArmDrives(Node):
    """Stream joint targets to simulated MKS drives and publish their encoder feedback."""

    def __init__(self):
        super().__init__('sim_arm_drives', automatically_declare_parameters_from_overrides=True)
        params = {name: self.get_parameter(name).value
                  for name in self.list_parameters([], 0).names}
        self.map = drive_map_from_parameters(params, mks_can.COUNTS_PER_REV)
        self.acc = int(params['acc'])
        self.max_rpm = int(params['max_rpm'])
        self.period = 1.0 / float(params['rate_hz'])
        self.bitrate = int(params['bitrate'])
        drives = self.map.drives
        self.index = {drive.can_id: index for index, drive in enumerate(drives)}
        self.bus = SimulatedBus([SimulatedServo(drive.can_id) for drive in drives], self.bitrate)
        self.links = [DriveLink(drive.can_id, int(params['heartbeat_ms']), REPLY_TIMEOUT_S)
                      for drive in drives]
        self.sent = [None] * len(drives)
        self.last_command = [''] * len(drives)
        self.commanded = None
        self.synced = False
        self.velocities = {}
        self.limits = {}
        self.rejected = 0
        self.held = 0
        self.bus_errors = 0
        self.zeroing = None
        self.zero_deadline = None
        topic = params.get('diagnostics_topic', '/diagnostics')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(JointState, 'joint_commands', self.on_command, 10)
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        # Its own group, so the ticks that carry the 92h frames keep running while it waits.
        self.create_service(Trigger, '~/zero', self.on_zero,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.last_tick = time.monotonic()
        self.report_time, self.report_frames, self.report_bits = self.last_tick, 0, 0
        self.create_timer(1.0 / float(params['rate_hz']), self.tick)
        self.create_timer(0.5, self.report)
        self.get_logger().info(
            'Simulated drives: '
            + ', '.join(f'{drive.name}=CAN {drive.can_id}' for drive in drives)
            + ' (placeholder MKS SERVO map, no hardware)')

    def on_description(self, message):
        try:
            self.limits = joint_limits(message.data, self.map.joints, required=False)
        except ValueError as error:
            self.get_logger().error(f'Joint limits unavailable: {error}')

    @property
    def counts(self):
        return [link.count for link in self.links]

    def on_command(self, message):
        if self.commanded is None:
            return
        commands = {}
        for index, joint in enumerate(message.name):
            if joint not in self.commanded or index >= len(message.position):
                continue
            position = message.position[index]
            velocity = message.velocity[index] if index < len(message.velocity) else 0.0
            if not (math.isfinite(position) and math.isfinite(velocity)):
                self.rejected += 1
                continue
            commands[joint] = position, velocity
        if not self.synced:
            # Commands from before a hold would make the arm jump, so wait for one that starts
            # where the arm is, such as teleop's first command after it is enabled.
            measured = self.map.to_positions(self.counts)
            if not commands or any(abs(position - measured[joint]) > SYNC_TOLERANCE_RAD
                                   for joint, (position, _) in commands.items()):
                self.held += 1
                self.get_logger().warning(
                    'Holding: /joint_commands are not at the measured pose; disable and enable '
                    'teleop', throttle_duration_sec=5.0)
                return
            self.synced = True
        for joint, (position, velocity) in commands.items():
            self.commanded[joint], self.velocities[joint] = position, velocity

    async def on_zero(self, request, response):
        waiting = [drive.name for drive, link in zip(self.map.drives, self.links)
                   if not link.ready]
        if waiting:
            response.message = 'drives not ready: ' + ', '.join(waiting)
            return response
        self.hold()
        for link in self.links:
            link.zero()
        self.zeroing = Future()
        self.zero_deadline = time.monotonic() + ZERO_TIMEOUT_S
        response.success, response.message = await self.zeroing
        self.zeroing = None
        log = self.get_logger().info if response.success else self.get_logger().error
        log(f'Set zero: {response.message}')
        return response

    def check_zero(self, now):
        failed = [drive.name for drive, link in zip(self.map.drives, self.links)
                  if link.zeroed is False]
        waiting = [drive.name for drive, link in zip(self.map.drives, self.links)
                   if not link.ready]
        if failed:
            result = False, ('not zeroed: ' + ', '.join(failed)
                             + '; the drives disagree, call ~/zero again in the zero pose')
        elif not waiting:
            result = True, f'{len(self.links)} drives zeroed'
        elif now > self.zero_deadline:
            result = False, 'timed out waiting for ' + ', '.join(waiting)
        else:
            return
        self.zero_deadline = None
        self.zeroing.set_result(result)

    def hold(self):
        """Stop every drive that still answers and drop the commands until all are ready."""
        for drive, link in zip(self.map.drives, self.links):
            if link.ready:
                self.bus.send(drive.can_id, mks_can.stop(drive.can_id, self.acc))
        self.commanded, self.synced, self.velocities = None, False, {}
        self.sent = [None] * len(self.links)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.1), now
        # Speeds follow the encoders, so the targets are refreshed every tick.
        if self.commanded is not None and self.synced:
            self.send_targets()
        self.bus.step(dt)
        for drive, link in zip(self.map.drives, self.links):
            if (data := link.poll(now)) is not None:
                self.bus.send(drive.can_id, data)
        while (reply := self.bus.receive()) is not None:
            self.on_reply(*reply, now)
        if self.zero_deadline is not None:
            self.check_zero(now)
        waiting = [(drive.name, link) for drive, link in zip(self.map.drives, self.links)
                   if not link.ready]
        if waiting:
            if self.commanded is not None:
                self.get_logger().error('Arm stopped, drives not ready: ' + ', '.join(
                    f'{name} ({link.describe()})' for name, link in waiting))
                self.hold()
            return
        positions = self.map.to_positions(self.counts)
        if self.commanded is None:
            # Hold wherever the drives already are until a command starts from there.
            self.commanded = dict(positions)
        message = JointState(name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def send_targets(self):
        moves = self.map.synchronized(self.commanded, self.velocities, self.counts, self.period,
                                      self.max_rpm, self.limits)
        for index, (drive, (axis, speed)) in enumerate(zip(self.map.drives, moves)):
            if self.sent[index] == (axis, speed):
                continue
            try:
                data = mks_can.absolute_axis(drive.can_id, axis, speed, self.acc)
            except ValueError as error:
                self.get_logger().warning(f'{drive.name}: {error}', throttle_duration_sec=2.0)
                continue
            self.bus.send(drive.can_id, data)
            self.sent[index] = (axis, speed)
            self.last_command[index] = mks_can.hex_frame(drive.can_id, data)

    def on_reply(self, can_id, data, now):
        index = self.index.get(can_id)
        try:
            code, arguments = mks_can.parse(can_id, data)
            if index is None:
                raise ValueError(f'reply from unknown CAN ID {can_id}')
            self.links[index].on_reply(code, arguments, now)
        except ValueError as error:
            self.bus_errors += 1
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)

    def report(self):
        now = time.monotonic()
        elapsed = max(now - self.report_time, 1e-3)
        frames = (self.bus.frames - self.report_frames) / elapsed
        load = (self.bus.bits - self.report_bits) / elapsed / self.bitrate * 100.0
        self.report_time, self.report_frames, self.report_bits = (
            now, self.bus.frames, self.bus.bits)
        errors = self.bus_errors + self.bus.errors
        level = (DiagnosticStatus.ERROR if errors else
                 DiagnosticStatus.WARN if load > 70.0 else DiagnosticStatus.OK)
        statuses = [status(
            'can.bus', level, f'{frames:.0f}', 'frames/s',
            f'{load:.0f}% worst-case load at {self.bitrate // 1000} kbit/s (simulated MKS bus)',
            load_percent=f'{load:.1f}', errors=errors, rejected_commands=self.rejected,
            held_commands=self.held)]
        if self.commanded is not None:
            positions = self.map.to_positions(self.counts)
            for joint, position in positions.items():
                target = math.degrees(self.commanded.get(joint, position))
                statuses.append(status(f'arm.{joint}', DiagnosticStatus.OK,
                                       f'{math.degrees(position):+.1f}', 'deg',
                                       f'target {target:+.1f} deg'))
        for index, (drive, link) in enumerate(zip(self.map.drives, self.links)):
            level = (DiagnosticStatus.OK if link.ready else
                     DiagnosticStatus.ERROR if link.problem else DiagnosticStatus.WARN)
            turns = '' if link.count is None else f'{link.count / mks_can.COUNTS_PER_REV:+.3f}'
            statuses.append(status(
                f'drive.{drive.name}', level, turns, 'rev',
                f'CAN ID {drive.can_id}: {link.describe()}'
                + (f'; last {self.last_command[index]}' if self.last_command[index] else ''),
                can_id=drive.can_id, phase=link.phase, dropouts=link.dropouts,
                last_command=self.last_command[index]))
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)


def main():
    rclpy.init()
    node = SimArmDrives()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
