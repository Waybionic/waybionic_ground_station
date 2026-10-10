"""
Run the arm's joint drives: /joint_commands in, MKS CAN frames, /joint_states out.

The drives are simulated on an in-process bus unless ``interface`` names a python-can interface
(socketcan, slcan, udp_multicast, ...) and ``channel`` the adapter, in which case the same
frames go to real MKS SERVO drives. The command, feedback and diagnostics path is identical.

The encoders count from where the drives were powered on, so nothing moves until every drive
has been zeroed with the arm in the zero pose: at start-up in simulation, and through the
``~/zero`` service on real drives. A drive that stops answering is treated as power-cycled:
every drive stops and the arm must be zeroed again.

Joint commands are accepted only after a valid robot_description and a fresh teleop enable
that teleop keeps republishing. Whoever publishes ``/joint_commands`` must keep publishing
while the arm is meant to move: once ``command_timeout_s`` passes with no command, every
drive stops where it is, whatever speed it was last given, so a publisher that dies cannot
leave the arm running on to its last target. After any stop, teleop must be disabled and
enabled again before the arm moves.
"""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from waybionic_teleop import mks_can
from waybionic_teleop.drive_map import drive_map_from_parameters
from waybionic_teleop.kinematics import joint_limits
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo

REPLY_TIMEOUT_S = 0.5
# Zeroing waits for the joint commands to stop, so teleop must be disabled first.
QUIET_BEFORE_ZERO_S = 1.0
# Every drive must confirm these before it moves, so none runs without its heartbeat stop.
SETUP = (mks_can.SET_MODE, mks_can.SET_RESPONSE, mks_can.ENABLE, mks_can.SET_HEARTBEAT)
# F5 replies of a drive that no longer follows its targets: run failed, as when stall
# protection releases the motor, and stopped at an end limit.
RUN_FAULTS = (0, 3)


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
        rate_hz = float(params['rate_hz'])
        self.bitrate = int(params['bitrate'])
        self.heartbeat_ms = int(params['heartbeat_ms'])
        self.enable_timeout = float(params['enable_timeout_s'])
        self.command_timeout = float(params.get('command_timeout_s', 0.5))
        self.following_error = int(params.get('following_error_counts', 4096))
        self.following_ticks = int(params.get('following_error_ticks', 12))
        if (not 0 <= self.acc <= 255 or not 1 <= self.max_rpm <= mks_can.MAX_SPEED_RPM
                or not math.isfinite(rate_hz) or rate_hz <= 0 or self.bitrate <= 0
                or not 1 <= self.heartbeat_ms <= 0xFFFFFFFF
                or self.following_error < 1 or self.following_ticks < 1
                or not 0 < self.command_timeout < math.inf
                or not 0 < self.enable_timeout < math.inf):
            raise ValueError('invalid drive rate, acceleration, speed, heartbeat, following '
                             'error, command timeout or enable timeout')
        self.period = 1.0 / rate_hz
        self.heartbeat_s = self.heartbeat_ms / 1000.0
        self.interface = params.get('interface', 'sim')
        channel = params.get('channel', '')
        unmodeled = set(params['unmodeled_joints'])
        if self.interface != 'sim':
            # Real drives move only joints whose URDF limits are known.
            unmodeled = set()
        elif not unmodeled <= set(self.map.joints):
            raise ValueError('unmodeled_joints must name drives in the simulated map')
        self.modeled_joints = [joint for joint in self.map.joints if joint not in unmodeled]
        if not self.modeled_joints:
            raise ValueError('at least one drive joint needs a URDF limit')
        drives = self.map.drives
        self.index = {drive.can_id: index for index, drive in enumerate(drives)}
        if self.interface == 'sim':
            self.bus = SimulatedBus([SimulatedServo(drive.can_id) for drive in drives],
                                    self.bitrate)
            self.bus_label = 'simulated MKS bus'
        elif not channel:
            raise ValueError(f'the {self.interface} interface needs a channel, '
                             'such as can0, /dev/ttyACM0 or 239.74.163.2')
        else:
            # python-can is only needed for a real bus.
            from waybionic_teleop.can_bus import CanBus
            self.bus = CanBus(self.interface, channel, self.bitrate)
            self.bus_label = f'{self.interface} {channel}'
        self.counts = [None] * len(drives)
        # Host time since each drive's last encoder reading, which bounds how far it can turn.
        self.since_count = [0.0] * len(drives)
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
        # Drives whose stop frame the interface has not accepted yet.
        self.stopping = set()
        # Ticks in a row each drive has been more than following_error from its target.
        self.behind = [0] * len(drives)
        self.heartbeat_stops = [0] * len(drives)
        self.last_command = [''] * len(drives)
        self.commanded = None
        self.velocities = {}
        self.command_time = -math.inf
        # True while no fresh command is outstanding: before the first one, and after a stale
        # one has already stopped the drives. It keeps the stop and its warning to one shot.
        self.stale = True
        self.limits = {}
        self.description_valid = False
        self.authorized = False
        self.enabled_at = -math.inf
        self.awaiting_release = True
        self.rejected = 0
        self.bus_errors = 0
        for index in range(len(drives)):
            self.set_up(index)
        zero_on_start = params.get('zero_on_start', self.interface == 'sim')
        if zero_on_start:
            self.zero()
        self.stop_reason = 'Waiting for robot_description and a released Start button'
        topic = params.get('diagnostics_topic', '/diagnostics')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'robot_description', self.on_description, latched)
        self.create_subscription(Bool, 'teleop_enabled', self.on_enabled, latched)
        self.create_subscription(JointState, 'joint_commands', self.on_command, 10)
        self.create_service(Trigger, '~/zero', self.on_zero)
        self.state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, topic, 10)
        self.last_tick = time.monotonic()
        self.report_time, self.report_frames, self.report_bits = self.last_tick, 0, 0
        self.report_errors = 0
        self.create_timer(self.period, self.tick)
        self.create_timer(0.5, self.report)
        drive_list = ', '.join(f'{drive.name}=CAN {drive.can_id}' for drive in drives)
        if self.interface == 'sim':
            self.get_logger().info(
                f'Simulated drives: {drive_list} (placeholder MKS SERVO map, no hardware)')
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
        self.state[index] = 'starting'
        for data in (mks_can.set_mode(can_id),
                     mks_can.set_response(can_id, respond=True, active=True),
                     mks_can.enable(can_id),
                     mks_can.set_heartbeat(can_id, self.heartbeat_ms)):
            self.bus.send(can_id, data)

    def stop_all(self, reason):
        """Stop every drive where it is; teleop must be enabled afresh before it moves."""
        self.stop_drives()
        self.disarm(reason, require_release=True)

    def zero(self):
        """Make the current pose every drive's zero."""
        self.stop_all('Zeroing the drives')
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
        now = time.monotonic()
        if not message.data:
            self.disarm('Teleop disabled')
            self.awaiting_release = False
        elif (not self.awaiting_release and self.description_valid and all(self.zeroed)
              and not any(self.unconfirmed) and not self.stopping and None not in self.counts
              and self.feedback_fresh(now)):
            if not self.authorized:
                self.commanded = self.map.to_positions(self.counts)
            self.authorized = True
            self.enabled_at = now
            self.stop_reason = ''

    def disarm(self, reason, require_release=False):
        if self.authorized:
            self.stop_drives()
        self.authorized = False
        self.awaiting_release = self.awaiting_release or require_release
        self.commanded = None
        self.velocities.clear()
        self.sent = [None] * len(self.map.drives)
        self.stop_reason = reason

    def on_command(self, message):
        self.command_time, self.stale = time.monotonic(), False
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
        self.velocities = dict(zip(names, message.velocity))

    def reject_command(self, reason):
        self.rejected += 1
        self.disarm(reason, require_release=True)
        self.get_logger().warning(f'{reason}; release Start before re-enabling',
                                  throttle_duration_sec=2.0)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        # The old motion must consume the entire elapsed host time before any new frame
        # can reset its heartbeat. A pause requires a released and newly pressed Start.
        self.bus.step(dt)
        self.since_count = [elapsed + dt for elapsed in self.since_count]
        if self.interface == 'sim':
            for index, drive in enumerate(self.map.drives):
                # A simulated drive taken off the bus, as if unplugged, reports nothing.
                servo = self.bus.drives.get(drive.can_id)
                if servo is not None and servo.heartbeat_stops != self.heartbeat_stops[index]:
                    self.heartbeat_stops[index] = servo.heartbeat_stops
                    self.sent[index] = None
        if not self.stale and now - self.command_time > self.command_timeout:
            # The commands stopped, say because teleop exited or its host died: stop where the
            # arm is rather than run on to the last target, whatever speed it was given.
            self.stale = True
            if self.authorized:
                self.stop_all(f'No joint command for {self.command_timeout:.1f} s')
                self.get_logger().warning(
                    f'No joint command for {self.command_timeout:.1f} s; every drive is stopped')
        self.check_replies()
        self.send_stops()
        if dt >= self.heartbeat_s:
            self.disarm('Host paused past the drive heartbeat', require_release=True)
        else:
            if self.authorized and not self.feedback_fresh(now):
                self.disarm('Drive encoder feedback stale', require_release=True)
            # Teleop republishes its enable every tick, so silence means it stopped or died.
            if self.authorized and now - self.enabled_at > self.enable_timeout:
                self.disarm('Teleop enable timed out', require_release=True)
            if self.authorized and self.commanded is not None:
                self.send_targets()
        for index, drive in enumerate(self.map.drives):
            self.bus.send(drive.can_id, mks_can.read_encoder(drive.can_id))
            self.unanswered[index] += 1
        self.drain(now)
        self.check_following()
        if not all(self.zeroed) or None in self.counts:
            return
        positions = self.map.to_positions(self.counts)
        if self.commanded is None:
            # Hold the actual encoder pose while disarmed, including after a host pause.
            self.commanded = dict(positions)
        message = JointState(name=list(positions), position=list(positions.values()))
        message.header.stamp = self.get_clock().now().to_msg()
        self.state_publisher.publish(message)

    def check_replies(self):
        now = time.monotonic()
        for index in range(len(self.map.drives)):
            if self.unconfirmed[index] and now - self.setup_time[index] > REPLY_TIMEOUT_S:
                # A lost setup frame or reply: ask again rather than move without it.
                self.set_up(index)
            if (self.heard[index] is None or self.lost[index]
                    or self.unanswered[index] <= self.max_unanswered):
                continue
            self.lose(index, 'stopped answering')

    def lose(self, index, reason):
        """Treat a drive as power-cycled: every drive stops until the arm is zeroed again."""
        name = self.map.drives[index].name
        self.lost[index], self.zeroed[index] = True, False
        self.stop_all(f'{name} {reason}')
        self.get_logger().error(f'{name} {reason}; every drive is stopped')

    def check_following(self):
        for index, (sent, count) in enumerate(zip(self.sent, self.counts)):
            axis = sent[0] if self.authorized and sent else None
            if axis is None or count is None or abs(axis - count) <= self.following_error:
                self.behind[index] = 0
                continue
            self.behind[index] += 1
            if self.behind[index] >= self.following_ticks:
                self.fault(index, f'is {abs(axis - count)} counts from its target')

    def fault(self, index, reason):
        """Stop every drive after one stops following; the arm must be zeroed again."""
        name = self.map.drives[index].name
        self.zeroed[index] = False
        self.stop_all(f'{name} {reason}')
        self.get_logger().error(f'{name} {reason}; every drive is stopped. {self.zero_hint()}')

    def stop_drives(self):
        """Stop every drive with the manual's F5 zero-speed, zero-acceleration frame."""
        self.velocities.clear()
        self.stopping.update(index for index, sent in enumerate(self.sent) if sent != (None, 0))
        self.send_stops()

    def send_stops(self):
        # A refused stop goes out again on every tick, as the encoder polls keep the drive's
        # heartbeat from stopping it.
        for index in sorted(self.stopping):
            drive = self.map.drives[index]
            data = mks_can.stop(drive.can_id, 0)
            if self.bus.send(drive.can_id, data):
                self.stopping.discard(index)
                self.sent[index] = (None, 0)
                self.last_command[index] = mks_can.hex_frame(drive.can_id, data)

    def send_targets(self):
        try:
            moves = self.map.synchronized(self.commanded, self.counts, self.period, self.max_rpm,
                                          self.velocities)
            motor_rates = self.map.to_rpm(self.velocities)
            commands = []
            # Preflight every int24 axis and frame before sending any coupled motor an update.
            for drive, (axis, speed), rpm in zip(self.map.drives, moves, motor_rates):
                if not mks_can.MIN_AXIS <= axis <= mks_can.MAX_AXIS:
                    raise ValueError(f'{drive.name}: axis {axis} is outside int24')
                if rpm == 0.0:
                    key = (None, 0)
                    data = mks_can.absolute_axis(drive.can_id, 0, 0, 0)
                else:
                    key = (axis, speed)
                    data = mks_can.absolute_axis(drive.can_id, axis, speed, self.acc)
                commands.append((key, data))
        except (ValueError, OverflowError) as error:
            self.reject_command(f'Cannot synchronize drives: {error}')
            return
        for index, (drive, (key, data)) in enumerate(zip(self.map.drives, commands)):
            if self.sent[index] == key:
                continue
            # A frame the interface refused is sent again on the next tick.
            if self.bus.send(drive.can_id, data):
                self.sent[index] = key
                self.last_command[index] = mks_can.hex_frame(drive.can_id, data)

    def drain(self, now):
        while (reply := self.bus.receive()) is not None:
            self.on_reply(*reply, now)

    def on_reply(self, can_id, data, now):
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
            self.lost[index] = False
            self.set_up(index)
            self.get_logger().warning(
                f'{self.map.drives[index].name} answers again. {self.zero_hint()}')
        failed = None
        jump = None
        try:
            if code == mks_can.READ_ENCODER:
                count, previous = mks_can.encoder_value(arguments), self.counts[index]
                # The farthest the drive can turn since its last reading, with a period of
                # slack for a late reply; a longer step means its encoder count restarted.
                most = self.max_rpm / 60.0 * mks_can.COUNTS_PER_REV * (
                    self.since_count[index] + 2 * self.period)
                if previous is not None and abs(count - previous) > most:
                    jump = count - previous
                self.counts[index], self.since_count[index] = count, 0.0
            elif code == mks_can.ABSOLUTE_AXIS and arguments:
                self.state[index] = mks_can.RUN_STATUS.get(arguments[0], f'status {arguments[0]}')
                if arguments[0] in RUN_FAULTS:
                    failed = self.state[index]
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
        if jump is not None:
            # A drive that browned out and restarted answers well before it counts as silent.
            self.lose(index, f'encoder jumped {jump:+d} counts')
        # Only stop frames go out while disarmed, and a released motor refuses those too.
        if failed and self.authorized:
            self.fault(index, failed)

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
        gate_level = (DiagnosticStatus.OK if self.authorized else
                      DiagnosticStatus.ERROR if not self.description_valid else
                      DiagnosticStatus.WARN)
        statuses.append(status('arm.command_gate', gate_level,
                               'enabled' if self.authorized else 'stopped', '',
                               self.stop_reason or 'Receiving validated teleop setpoints'))
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
            failed = ('failed' in self.state[index]
                      or self.state[index] in [mks_can.RUN_STATUS[code] for code in RUN_FAULTS])
            turns = '' if self.counts[index] is None else (
                f'{self.counts[index] / mks_can.COUNTS_PER_REV:+.3f}')
            text = 'no reply' if silent else self.state[index]
            # A drive is never driven before it confirms every setup frame.
            pending = ', '.join(f'{code:02X}h' for code in sorted(self.unconfirmed[index]))
            if not silent and pending:
                text += f', setup {pending} unconfirmed'
            if not silent and not self.zeroed[index]:
                text += ', not zeroed'
            level = (DiagnosticStatus.ERROR if silent or failed else
                     DiagnosticStatus.WARN if pending or not self.zeroed[index] else
                     DiagnosticStatus.OK)
            statuses.append(status(
                f'drive.{drive.name}', level, turns, 'rev',
                f'CAN ID {drive.can_id}: {text}'
                + (f'; last {self.last_command[index]}' if self.last_command[index] else ''),
                can_id=drive.can_id, last_command=self.last_command[index]))
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)

    def close(self):
        """Stop the drives and release the bus; the drives' heartbeat would also stop them."""
        self.stop_all('Shutting down')
        self.bus.shutdown()


def main():
    rclpy.init()
    node = None
    try:
        node = SimArmDrives()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError as error:
        # SIGINT can shut down the context before spin recreates its wait set.
        if rclpy.ok() or 'the given context is not valid' not in str(error):
            raise
    except RuntimeError as error:
        # Jazzy may surface this binding error when SIGINT invalidates a subscription.
        if (rclpy.ok() or not str(error).startswith(
                "Unable to convert call argument '0' to Python object")):
            raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
