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
from sensor_msgs.msg import JointState

from waybionic_teleop import mks_can
from waybionic_teleop.drive_map import drive_map_from_parameters
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

REPLY_TIMEOUT_S = 0.5


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
        self.bitrate = int(params['bitrate'])
        drives = self.map.drives
        self.index = {drive.can_id: index for index, drive in enumerate(drives)}
        self.bus = SimulatedBus([SimulatedServo(drive.can_id) for drive in drives], self.bitrate)
        self.counts = [None] * len(drives)
        self.heard = [None] * len(drives)
        self.state = ['starting'] * len(drives)
        self.sent = [None] * len(drives)
        self.last_command = [''] * len(drives)
        self.commanded = None
        self.velocities = {}
        self.pending = False
        self.rejected = 0
        self.bus_errors = 0
        # Same start-up sequence the real drives need: bus FOC mode, replies and "move complete"
        # reports on, shaft enabled, and a heartbeat stop if the host goes quiet.
        for drive in drives:
            for data in (mks_can.set_mode(drive.can_id),
                         mks_can.set_response(drive.can_id, respond=True, active=True),
                         mks_can.enable(drive.can_id),
                         mks_can.set_heartbeat(drive.can_id, int(params['heartbeat_ms']))):
                self.bus.send(drive.can_id, data)
        topic = params.get('diagnostics_topic', '/diagnostics')
        self.create_subscription(JointState, 'joint_commands', self.on_command, 10)
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.last_tick = time.monotonic()
        self.report_time, self.report_frames, self.report_bits = self.last_tick, 0, 0
        self.create_timer(1.0 / float(params['rate_hz']), self.tick)
        self.create_timer(0.5, self.report)
        self.get_logger().info(
            'Simulated drives: '
            + ', '.join(f'{drive.name}=CAN {drive.can_id}' for drive in drives)
            + ' (placeholder MKS SERVO map, no hardware)')

    def on_command(self, message):
        if self.commanded is None:
            return
        for index, joint in enumerate(message.name):
            if joint not in self.commanded or index >= len(message.position):
                continue
            position = message.position[index]
            velocity = message.velocity[index] if index < len(message.velocity) else 0.0
            if not (math.isfinite(position) and math.isfinite(velocity)):
                self.rejected += 1
                continue
            self.commanded[joint], self.velocities[joint] = position, velocity
            self.pending = True

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = min(now - self.last_tick, 0.1), now
        if self.pending:
            self.pending = False
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
            # Hold wherever the drives already are until the first command arrives.
            self.commanded = dict(positions)
        message = JointState(name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def send_targets(self):
        counts = self.map.to_counts(self.commanded)
        speeds = self.map.to_rpm(self.velocities)
        for index, (drive, axis, rpm) in enumerate(zip(self.map.drives, counts, speeds)):
            # Allow some speed margin so each streamed target is reached before the next one.
            speed = min(self.max_rpm, math.ceil(rpm * 1.5) + 1)
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
        if None not in self.counts:
            positions = self.map.to_positions(self.counts)
            for joint, position in positions.items():
                target = math.degrees(self.commanded.get(joint, position))
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
