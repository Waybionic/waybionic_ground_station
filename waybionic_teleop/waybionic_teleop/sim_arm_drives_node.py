"""
Run the arm's joint drives: /joint_commands in, MKS CAN frames, /joint_states out.

The drives are simulated on an in-process bus unless ``interface`` names a bus: ``slcan`` for
the WayBionic carrier on a USB serial port, or another python-can interface (socketcan,
udp_multicast, ...) with ``channel`` naming the adapter. The same frames then go to the real MKS
SERVO drives, and the command, feedback and diagnostics path is identical.

The encoders count from where the drives were powered on, so nothing moves until every drive
has been zeroed with the arm in the zero pose: at start-up in simulation, and through the
``~/zero`` service on real drives. A drive that stops answering is treated as power-cycled:
every drive stops and the arm must be zeroed again.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from waybionic_teleop import carrier, mks_can
from waybionic_teleop.drive_map import drive_map_from_parameters
from waybionic_teleop.kinematics import joint_limits
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

REPLY_TIMEOUT_S = 0.5
# The carrier reports ten times a second.
CARRIER_TIMEOUT_S = 0.5
# Zeroing waits for the joint commands to stop, so teleop must be disabled first.
QUIET_BEFORE_ZERO_S = 1.0
# Every drive must confirm these before it moves, so none runs without its heartbeat stop.
SETUP = (mks_can.SET_MODE, mks_can.SET_RESPONSE, mks_can.ENABLE, mks_can.SET_HEARTBEAT)


def status(name, level, value, unit, message, **extra):
    values = [KeyValue(key='value', value=value), KeyValue(key='unit', value=unit)]
    values += [KeyValue(key=key, value=str(item)) for key, item in extra.items()]
    return DiagnosticStatus(name=name, level=level, message=message, values=values)


class SimArmDrives(Node):
    """Stream joint targets to MKS drives and publish their encoder feedback."""

    def __init__(self, **kwargs):
        super().__init__('sim_arm_drives', automatically_declare_parameters_from_overrides=True,
                         **kwargs)
        params = {name: self.get_parameter(name).value
                  for name in self.list_parameters([], 0).names}
        self.map = drive_map_from_parameters(params, mks_can.COUNTS_PER_REV)
        self.acc = int(params['acc'])
        self.max_rpm = int(params['max_rpm'])
        self.period = 1.0 / float(params['rate_hz'])
        self.bitrate = int(params['bitrate'])
        self.heartbeat_ms = int(params['heartbeat_ms'])
        self.command_timeout = float(params.get('command_timeout_s', 0.5))
        self.interface = params.get('interface', 'sim')
        channel = params.get('channel', '')
        drives = self.map.drives
        self.index = {drive.can_id: index for index, drive in enumerate(drives)}
        if self.interface == 'sim':
            self.bus = SimulatedBus([SimulatedServo(drive.can_id) for drive in drives],
                                    self.bitrate)
            self.bus_label = 'simulated MKS bus'
        elif not channel:
            raise ValueError(f'the {self.interface} interface needs a channel, '
                             'such as can0, /dev/ttyACM0 or 239.74.163.2')
        elif self.interface == 'slcan':
            # pyserial is only needed for the carrier.
            from waybionic_teleop.slcan_bus import SlcanBus
            self.bus = SlcanBus(channel, self.bitrate, int(params.get('tty_baudrate', 1000000)))
            self.bus_label = f'carrier on {channel}'
        else:
            # python-can is only needed for other adapters.
            from waybionic_teleop.can_bus import CanBus
            self.bus = CanBus(self.interface, channel, self.bitrate)
            self.bus_label = f'{self.interface} {channel}'
        self.carrier, self.carrier_time = None, None
        self.counts = [None] * len(drives)
        self.heard = [None] * len(drives)
        self.unanswered = [0] * len(drives)
        # Counted in polls rather than seconds, so a host that stalls does not trip it.
        self.max_unanswered = max(3, round(REPLY_TIMEOUT_S / self.period))
        self.lost = [False] * len(drives)
        self.zeroed = [False] * len(drives)
        self.unconfirmed = [set(SETUP) for _ in drives]
        self.setup_time = [None] * len(drives)
        self.state = ['starting'] * len(drives)
        self.sent = [None] * len(drives)
        self.last_command = [''] * len(drives)
        self.commanded = None
        self.velocities = {}
        self.command_time = -math.inf
        self.limits = {}
        self.rejected = 0
        self.bus_errors = 0
        for index in range(len(drives)):
            self.set_up(index)
        zero_on_start = params.get('zero_on_start', self.interface == 'sim')
        if zero_on_start:
            self.zero()
        topic = params.get('diagnostics_topic', '/diagnostics')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(JointState, 'joint_commands', self.on_command, 10)
        self.create_service(Trigger, '~/zero', self.on_zero)
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.last_tick = time.monotonic()
        self.report_time, self.report_frames, self.report_bits = self.last_tick, 0, 0
        self.report_errors = 0
        self.create_timer(1.0 / float(params['rate_hz']), self.tick)
        self.create_timer(0.5, self.report)
        drive_list = ', '.join(f'{drive.name}=CAN {drive.can_id}' for drive in drives)
        if self.interface == 'sim':
            self.get_logger().info(
                f'Simulated drives: {drive_list} (no hardware)')
        else:
            self.get_logger().info(f'MKS drives on {self.bus_label}: {drive_list}')
        if not zero_on_start:
            self.get_logger().warning(f'Not zeroed. {self.zero_hint()}')

    def zero_hint(self):
        return ('Put the arm in the zero pose with teleop disabled, then run: ros2 service call '
                f'{self.get_fully_qualified_name()}/zero std_srvs/srv/Trigger')

    def set_up(self, index):
        # Bus FOC mode, replies and "move complete" reports on, shaft enabled, and a heartbeat
        # stop if the host goes quiet.
        can_id = self.map.drives[index].can_id
        self.unconfirmed[index] = set(SETUP)
        self.setup_time[index] = time.monotonic()
        for data in (mks_can.set_mode(can_id),
                     mks_can.set_response(can_id, respond=True, active=True),
                     mks_can.enable(can_id),
                     mks_can.set_heartbeat(can_id, self.heartbeat_ms)):
            self.bus.send(can_id, data)

    def stop_all(self):
        """Stop every drive where it is and drop the targets until the arm is zeroed again."""
        for drive in self.map.drives:
            self.bus.send(drive.can_id, mks_can.stop(drive.can_id, self.acc))
        self.commanded, self.velocities = None, {}
        self.sent = [None] * len(self.map.drives)

    def zero(self):
        """Make the current pose every drive's zero."""
        self.stop_all()
        for drive in self.map.drives:
            self.bus.send(drive.can_id, mks_can.set_zero(drive.can_id))
        self.zeroed = [False] * len(self.map.drives)

    def on_zero(self, request, response):
        if time.monotonic() - self.command_time < QUIET_BEFORE_ZERO_S:
            response.message = 'Joint commands are still arriving; disable teleop (Start) first'
            return response
        self.zero()
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        while not all(self.zeroed) and time.monotonic() < deadline:
            time.sleep(0.005)
            self.drain(time.monotonic())
        missing = [drive.name for drive, done in zip(self.map.drives, self.zeroed) if not done]
        response.success = not missing
        response.message = ('Zeroed every drive' if response.success
                            else 'No zero reply from ' + ', '.join(missing))
        self.get_logger().info(response.message)
        return response

    def on_description(self, message):
        try:
            self.limits = joint_limits(message.data, self.map.joints, required=False)
        except ValueError as error:
            self.get_logger().error(f'Joint limits unavailable: {error}')

    def on_command(self, message):
        self.command_time = time.monotonic()
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
        # Pass the new targets on now rather than at the next tick.
        if all(self.zeroed) and None not in self.counts and not self.estop_pressed:
            self.send_targets()
            self.drain(time.monotonic())

    @property
    def estop_pressed(self):
        # The last report counts even if the carrier has since gone quiet.
        return self.carrier is not None and self.carrier['estop'] == 'pressed'

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        if any(self.velocities.values()) and now - self.command_time > self.command_timeout:
            # The commands stopped mid-move, say because teleop exited: stop where the arm is
            # rather than run on to the last target.
            self.stop_all()
            self.get_logger().warning('Joint commands stopped mid-move; every drive is stopped')
        self.check_replies()
        # Speeds follow the encoders, so the targets are refreshed every tick.
        if self.commanded is not None and not self.estop_pressed:
            self.send_targets()
        self.bus.step(dt)
        for index, drive in enumerate(self.map.drives):
            self.bus.send(drive.can_id, mks_can.read_encoder(drive.can_id))
            self.unanswered[index] += 1
        self.drain(now)
        if not all(self.zeroed) or None in self.counts:
            return
        positions = self.map.to_positions(self.counts)
        if self.commanded is None and not any(self.unconfirmed):
            # Hold wherever the drives already are until the first command arrives.
            self.commanded = dict(positions)
        message = JointState(name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def check_replies(self):
        now = time.monotonic()
        for index, drive in enumerate(self.map.drives):
            if self.unconfirmed[index] and now - self.setup_time[index] > REPLY_TIMEOUT_S:
                # A lost setup frame or reply: ask again rather than move without it.
                self.set_up(index)
            if (self.heard[index] is None or self.lost[index]
                    or self.unanswered[index] <= self.max_unanswered):
                continue
            self.lost[index], self.zeroed[index] = True, False
            self.stop_all()
            self.get_logger().error(f'{drive.name} stopped answering; every drive is stopped')

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
            # A frame the interface refused is sent again on the next tick.
            if self.bus.send(drive.can_id, data):
                self.sent[index] = (axis, speed)
                self.last_command[index] = mks_can.hex_frame(drive.can_id, data)

    def drain(self, now):
        while (reply := self.bus.receive()) is not None:
            self.on_reply(*reply, now)

    def on_reply(self, can_id, data, now):
        if can_id == carrier.STATUS_ID:
            pressed = self.estop_pressed
            try:
                self.carrier, self.carrier_time = carrier.parse_status(data), now
            except ValueError as error:
                self.bus_errors += 1
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)
                return
            if self.estop_pressed and not pressed:
                # The e-stop cuts the drives' power; no targets go out until it is released.
                self.stop_all()
                self.get_logger().error('E-stop pressed; every drive is stopped')
            return
        index = self.index.get(can_id)
        try:
            code, arguments = mks_can.parse(can_id, data)
            if index is None:
                raise ValueError(f'reply from unknown CAN ID {can_id}')
        except ValueError as error:
            self.bus_errors += 1
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        if self.lost[index]:
            # It may have been power-cycled, which loses its settings and its zero.
            self.lost[index], self.state[index] = False, 'starting'
            self.set_up(index)
            self.get_logger().warning(
                f'{self.map.drives[index].name} answers again. {self.zero_hint()}')
        try:
            if code == mks_can.READ_ENCODER:
                self.counts[index] = mks_can.encoder_value(arguments)
            elif code == mks_can.ABSOLUTE_AXIS and arguments:
                self.state[index] = mks_can.RUN_STATUS.get(arguments[0], f'status {arguments[0]}')
            elif code == mks_can.SET_ZERO and arguments[:1] == b'\x01':
                # Later encoder replies were read after the zero.
                self.zeroed[index], self.counts[index] = True, None
            elif arguments[:1] == b'\x00':
                self.state[index] = f'setup {code:02X}h failed'
            elif code in SETUP:
                self.unconfirmed[index].discard(code)
                if not self.unconfirmed[index] and self.state[index] == 'starting':
                    self.state[index] = 'ready'
        except ValueError as error:
            self.bus_errors += 1
            self.get_logger().warning(str(error), throttle_duration_sec=2.0)
            return
        self.heard[index], self.unanswered[index] = now, 0

    def report(self):
        now = time.monotonic()
        elapsed = max(now - self.report_time, 1e-3)
        frames = (self.bus.frames - self.report_frames) / elapsed
        load = (self.bus.bits - self.report_bits) / elapsed / self.bitrate * 100.0
        errors = self.bus_errors + self.bus.errors
        new_errors = errors - self.report_errors
        self.report_time, self.report_frames, self.report_bits, self.report_errors = (
            now, self.bus.frames, self.bus.bits, errors)
        level = (DiagnosticStatus.ERROR if new_errors else
                 DiagnosticStatus.WARN if load > 70.0 else DiagnosticStatus.OK)
        statuses = [status(
            'can.bus', level, f'{frames:.0f}', 'frames/s',
            f'{load:.0f}% worst-case load at {self.bitrate // 1000} kbit/s ({self.bus_label})',
            load_percent=f'{load:.1f}', errors=errors, rejected_commands=self.rejected)]
        if hasattr(self.bus, 'connected'):
            statuses.append(self.carrier_status(now))
        if not all(self.zeroed):
            statuses.append(status('arm.zero', DiagnosticStatus.WARN, 'no', '',
                                   f'Not zeroed. {self.zero_hint()}'))
        elif None not in self.counts:
            positions = self.map.to_positions(self.counts)
            commanded = self.commanded or {}
            for joint, position in positions.items():
                target = math.degrees(commanded.get(joint, position))
                statuses.append(status(f'arm.{joint}', DiagnosticStatus.OK,
                                       f'{math.degrees(position):+.1f}', 'deg',
                                       f'target {target:+.1f} deg'))
        for index, drive in enumerate(self.map.drives):
            silent = self.heard[index] is None or now - self.heard[index] > REPLY_TIMEOUT_S
            failed = 'failed' in self.state[index]
            turns = '' if self.counts[index] is None else (
                f'{self.counts[index] / mks_can.COUNTS_PER_REV:+.3f}')
            text = 'no reply' if silent else self.state[index]
            if not silent and not self.zeroed[index]:
                text += ', not zeroed'
            level = (DiagnosticStatus.ERROR if silent or failed else
                     DiagnosticStatus.OK if self.zeroed[index] else DiagnosticStatus.WARN)
            statuses.append(status(
                f'drive.{drive.name}', level, turns, 'rev',
                f'CAN ID {drive.can_id}: {text}'
                + (f'; last {self.last_command[index]}' if self.last_command[index] else ''),
                can_id=drive.can_id, last_command=self.last_command[index]))
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)

    def carrier_status(self, now):
        """Report the USB link to the carrier and what the carrier says about the bus."""
        reopens = self.bus.reopens
        if not self.bus.connected:
            return status('can.carrier', DiagnosticStatus.ERROR, 'unplugged', '',
                          f'USB link lost ({self.bus.problem}); retrying every second',
                          reopens=reopens)
        if self.carrier_time is None or now - self.carrier_time > CARRIER_TIMEOUT_S:
            return status('can.carrier', DiagnosticStatus.WARN, 'silent', '',
                          'No status from the carrier: is carrier_bridge flashed?',
                          reopens=reopens)
        report = self.carrier
        estop = report['estop'] or 'not wired'
        supply = 'not wired' if report['supply_v'] is None else f'{report["supply_v"]:.1f} V'
        level = DiagnosticStatus.ERROR if report['estop'] == 'pressed' else DiagnosticStatus.OK
        return status('can.carrier', level, estop, 'e-stop',
                      f'E-stop {estop}, supply {supply}, {report["can_errors"]} CAN errors',
                      supply=supply, can_errors=report['can_errors'],
                      failed_writes=report['failed_writes'],
                      refused_lines=report['refused_lines'], reopens=reopens)

    def close(self):
        """Stop the drives and release the bus; the drives' heartbeat would also stop them."""
        self.stop_all()
        self.bus.shutdown()


def main():
    rclpy.init()
    node = None
    try:
        node = SimArmDrives()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
