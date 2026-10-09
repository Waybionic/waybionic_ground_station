"""
Run the arm's joint drives in simulation: /joint_commands in, CAN frames, /joint_states out.

The node talks to the drives only through CAN frames from :mod:`waybionic_teleop.mks_can`, so
replacing :class:`~waybionic_teleop.sim_drives.SimulatedBus` with a real CAN interface keeps
the command, feedback and diagnostics path unchanged. Nothing here reaches hardware.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from waybionic_teleop import mks_can
from waybionic_teleop.drive_map import drive_map_from_parameters
from waybionic_teleop.kinematics import joint_limits
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

REPLY_TIMEOUT_S = 0.5


def status(name, level, value, unit, message, **extra):
    values = [KeyValue(key='value', value=value), KeyValue(key='unit', value=unit)]
    values += [KeyValue(key=key, value=str(item)) for key, item in extra.items()]
    return DiagnosticStatus(name=name, level=level, message=message, values=values)


class SimArmDrives(Node):
    """Stream joint targets to simulated MKS drives and publish their encoder feedback."""

    def __init__(self, **kwargs):
        super().__init__('sim_arm_drives', automatically_declare_parameters_from_overrides=True,
                         **kwargs)
        params = {name: self.get_parameter(name).value
                  for name in self.list_parameters([], 0).names}
        self.map = drive_map_from_parameters(params, mks_can.COUNTS_PER_REV)
        self.acc = int(params['acc'])
        self.max_rpm = int(params['max_rpm'])
        rate_hz = float(params['rate_hz'])
        self.bitrate = int(params['bitrate'])
        heartbeat_ms = int(params['heartbeat_ms'])
        if (not 0 <= self.acc <= 255 or not 1 <= self.max_rpm <= mks_can.MAX_SPEED_RPM
                or not math.isfinite(rate_hz) or rate_hz <= 0 or self.bitrate <= 0
                or not 1 <= heartbeat_ms <= 0xFFFFFFFF):
            raise ValueError('invalid simulated drive rate, acceleration, speed or heartbeat')
        self.period = 1.0 / rate_hz
        self.heartbeat_s = heartbeat_ms / 1000.0
        unmodeled = set(params['unmodeled_joints'])
        if not unmodeled <= set(self.map.joints):
            raise ValueError('unmodeled_joints must name drives in the simulated map')
        self.modeled_joints = [joint for joint in self.map.joints if joint not in unmodeled]
        if not self.modeled_joints:
            raise ValueError('at least one drive joint needs a URDF limit')
        drives = self.map.drives
        self.index = {drive.can_id: index for index, drive in enumerate(drives)}
        self.bus = SimulatedBus([SimulatedServo(drive.can_id) for drive in drives], self.bitrate)
        self.counts = [None] * len(drives)
        self.heard = [None] * len(drives)
        self.state = ['starting'] * len(drives)
        self.sent = [None] * len(drives)
        self.last_command = [''] * len(drives)
        self.commanded = None
        self.limits = {}
        self.description_valid = False
        self.authorized = False
        self.awaiting_release = True
        self.stop_reason = 'Waiting for robot_description and a released Start button'
        self.rejected = 0
        self.bus_errors = 0
        # Same start-up sequence the real drives need: bus FOC mode, replies and "move complete"
        # reports on, shaft enabled, and a heartbeat stop if the host goes quiet.
        for drive in drives:
            for data in (mks_can.set_mode(drive.can_id),
                         mks_can.set_response(drive.can_id, respond=True, active=True),
                         mks_can.enable(drive.can_id),
                         mks_can.set_heartbeat(drive.can_id, heartbeat_ms)):
                self.bus.send(drive.can_id, data)
        topic = params.get('diagnostics_topic', '/diagnostics')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(Bool, 'teleop_enabled', self.on_enabled, latched)
        self.create_subscription(JointState, 'joint_commands', self.on_command, 10)
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.last_tick = time.monotonic()
        self.report_time, self.report_frames, self.report_bits = self.last_tick, 0, 0
        self.create_timer(self.period, self.tick)
        self.create_timer(0.5, self.report)
        self.get_logger().info(
            'Simulated drives: '
            + ', '.join(f'{drive.name}=CAN {drive.can_id}' for drive in drives)
            + ' (placeholder MKS SERVO map, no hardware)')

    def on_description(self, message):
        self.disarm('Robot description changed', require_release=True)
        try:
            self.limits = joint_limits(message.data, self.modeled_joints)
        except ValueError as error:
            self.limits = {}
            self.description_valid = False
            self.stop_reason = f'Joint limits unavailable: {error}'
            self.get_logger().error(self.stop_reason)
            return
        self.description_valid = True

    def feedback_fresh(self, now):
        return all(heard is not None and now - heard <= REPLY_TIMEOUT_S for heard in self.heard)

    def on_enabled(self, message):
        if not message.data:
            self.disarm('Teleop disabled')
            self.awaiting_release = False
        elif (not self.awaiting_release and self.description_valid and None not in self.counts
              and self.feedback_fresh(time.monotonic())):
            if not self.authorized:
                self.commanded = self.map.to_positions(self.counts)
            self.authorized = True
            self.stop_reason = ''

    def disarm(self, reason, require_release=False):
        if self.authorized:
            # F5h with speed zero stops a moving servo without steering it back to a stale pose.
            for index, drive in enumerate(self.map.drives):
                data = mks_can.absolute_axis(drive.can_id, 0, 0, self.acc)
                self.bus.send(drive.can_id, data)
                self.last_command[index] = mks_can.hex_frame(drive.can_id, data)
        self.authorized = False
        self.awaiting_release = self.awaiting_release or require_release
        self.commanded = None
        self.sent = [None] * len(self.map.drives)
        self.stop_reason = reason

    def on_command(self, message):
        if not self.authorized or self.commanded is None:
            return
        names = message.name
        if (len(names) != len(self.map.joints) or set(names) != set(self.map.joints)
                or len(message.position) != len(names) or len(message.velocity) != len(names)
                or any(not math.isfinite(value) for value in message.position)
                or any(not math.isfinite(value) for value in message.velocity)):
            self.reject_command('incomplete, duplicate or nonfinite joint command')
            return
        positions = dict(zip(names, message.position))
        measured = self.map.to_positions(self.counts)
        for joint, (lower, upper) in self.limits.items():
            position, actual = positions[joint], measured[joint]
            # An already-outside encoder may move toward the URDF range, never farther out.
            if ((position < lower and (actual >= lower or position < actual))
                    or (position > upper and (actual <= upper or position > actual))):
                self.reject_command(f'{joint} target outside its URDF range')
                return
        self.commanded = positions

    def reject_command(self, reason):
        self.rejected += 1
        self.disarm(reason, require_release=True)
        self.get_logger().warning(f'{reason}; release Start before re-enabling',
                                  throttle_duration_sec=2.0)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        if dt >= self.heartbeat_s:
            # Advance elapsed time before any new frame. Otherwise a stale target or encoder
            # request resets the simulated heartbeat and masks a real host pause.
            self.bus.step(dt)
            self.disarm('Host paused past the drive heartbeat', require_release=True)
        else:
            if self.authorized and not self.feedback_fresh(now):
                self.disarm('Drive encoder feedback stale', require_release=True)
            if self.authorized and self.commanded is not None:
                self.send_targets()
            self.bus.step(dt)
        for drive in self.map.drives:
            self.bus.send(drive.can_id, mks_can.read_encoder(drive.can_id))
        while (reply := self.bus.receive()) is not None:
            self.on_reply(*reply, now)
        if None in self.counts:
            return
        positions = self.map.to_positions(self.counts)
        if self.commanded is None:
            # Hold the actual encoder pose while disarmed, including after a host pause.
            self.commanded = dict(positions)
        message = JointState(name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def send_targets(self):
        try:
            moves = self.map.synchronized(self.commanded, self.counts, self.period, self.max_rpm)
            # Preflight every int24 axis and speed before sending any drive a new target.
            frames = [mks_can.absolute_axis(drive.can_id, axis, speed, self.acc)
                      for drive, (axis, speed) in zip(self.map.drives, moves)]
        except (ValueError, OverflowError) as error:
            self.reject_command(f'Cannot synchronize drives: {error}')
            return
        for index, (drive, move, data) in enumerate(zip(self.map.drives, moves, frames)):
            if self.sent[index] == move:
                continue
            self.bus.send(drive.can_id, data)
            self.sent[index] = move
            self.last_command[index] = mks_can.hex_frame(drive.can_id, data)

    def on_reply(self, can_id, data, now):
        index = self.index.get(can_id)
        try:
            code, arguments = mks_can.parse(can_id, data)
            if index is None:
                raise ValueError(f'reply from unknown CAN ID {can_id}')
            if code == mks_can.READ_ENCODER:
                self.counts[index] = mks_can.encoder_value(arguments)
            elif code == mks_can.ABSOLUTE_AXIS and arguments:
                self.state[index] = mks_can.RUN_STATUS.get(arguments[0], f'status {arguments[0]}')
            elif arguments[:1] == b'\x00':
                self.state[index] = f'setup {code:02X}h failed'
            elif self.state[index] == 'starting':
                self.state[index] = 'ready'
        except ValueError as error:
            self.bus_errors += 1
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.heard[index] = now

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
            load_percent=f'{load:.1f}', errors=errors, rejected_commands=self.rejected)]
        gate_level = (DiagnosticStatus.OK if self.authorized else
                      DiagnosticStatus.ERROR if not self.description_valid else
                      DiagnosticStatus.WARN)
        statuses.append(status('arm.command_gate', gate_level,
                               'enabled' if self.authorized else 'stopped', '',
                               self.stop_reason or 'Receiving validated teleop setpoints'))
        if None not in self.counts:
            positions = self.map.to_positions(self.counts)
            for joint, position in positions.items():
                target = math.degrees(self.commanded.get(joint, position)
                                      if self.commanded is not None else position)
                statuses.append(status(f'arm.{joint}', DiagnosticStatus.OK,
                                       f'{math.degrees(position):+.1f}', 'deg',
                                       f'target {target:+.1f} deg'))
        for index, drive in enumerate(self.map.drives):
            silent = self.heard[index] is None or now - self.heard[index] > REPLY_TIMEOUT_S
            failed = 'failed' in self.state[index]
            turns = '' if self.counts[index] is None else (
                f'{self.counts[index] / mks_can.COUNTS_PER_REV:+.3f}')
            statuses.append(status(
                f'drive.{drive.name}',
                DiagnosticStatus.ERROR if silent or failed else DiagnosticStatus.OK, turns, 'rev',
                f'CAN ID {drive.can_id}: ' + ('no reply' if silent else self.state[index])
                + (f'; last {self.last_command[index]}' if self.last_command[index] else ''),
                can_id=drive.can_id, last_command=self.last_command[index]))
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
