"""
Run the arm's joint drives: /joint_commands in, MKS CAN frames, /joint_states out.

The drives are simulated on an in-process bus unless ``interface`` names a bus: ``slcan`` for
the WayBionic carrier on a USB serial port, or another python-can interface (socketcan,
udp_multicast, ...) with ``channel`` naming the adapter. The same frames then go to the real MKS
SERVO drives, and the command, feedback and diagnostics path is identical.

The encoders count from where the drives were powered on, so nothing moves until every drive
has been zeroed with the arm in the zero pose: at start-up in simulation, and through the
``~/zero`` service on real drives. A drive that stops answering is treated as power-cycled:
every drive stops and the arm must be zeroed again. With ``zero_state`` naming a file, each
zero is saved there, and a drive keeps it, at start-up or once it answers again, if it shows
it has not lost power since: see ``check_zero``.

``arm_profile`` names a file (waybionic_teleop/arm_profile.py) that lists the drives this arm
has fitted; each joint no fitted drive moves is published at its locked angle, and a command
must leave it there.

Joint commands are accepted only after a valid robot_description and a fresh teleop enable
that teleop keeps republishing. Whoever publishes ``/joint_commands`` must keep publishing
while the arm is meant to move: once ``command_timeout_s`` passes with no command, every
drive stops where it is, whatever speed it was last given, so a publisher that dies cannot
leave the arm running on to its last target. After any stop, teleop must be disabled and
enabled again before the arm moves.

A drive never gets a target behind a move it is running. MKS firmware V1.0.9 can keep such a
target and run to it on its own once the moves stop, even after a stop frame, so a drive is
stopped first, and after any stop of a running move it starts a new one only once its encoder
has stopped advancing. The firmware also keeps an occasional target sent mid-move, and runs
back to it once the move stops, however it stops. So every stop of a running move is followed
at once by a target where the drive is to stay, sent again every tick for ``HOLD_S``, which
takes such a run over within a tick. A drive that turns more than ``following_error_counts``
after its stop while the arm is enabled stops every drive.
"""

import datetime
import math
import os
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from waybionic_control.drive_health import (
    DriveHealthMonitor,
    DriveHealthState,
    DriveHealthThresholds,
)
from waybionic_teleop import carrier, mks_can
from waybionic_teleop.arm_profile import fit_drive_parameters, load_profile
from waybionic_teleop.drive_map import drive_map_from_parameters
from waybionic_teleop.frame_log import RecordingBus
from waybionic_teleop.kinematics import joint_limits
from waybionic_teleop.ros_shutdown import is_shutdown_error
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo
import yaml

REPLY_TIMEOUT_S = 0.5
REPORT_PERIOD_S = 0.5
# The carrier reports ten times a second.
CARRIER_TIMEOUT_S = 0.5
# Zeroing waits for the joint commands to stop, so teleop must be disabled first.
QUIET_BEFORE_ZERO_S = 1.0
# How long closing waits for the carrier to take the last stops.
CLOSE_TIMEOUT_S = 0.5
# Every drive must confirm these before it moves, so none runs without its heartbeat stop.
SETUP = (mks_can.SET_MODE, mks_can.SET_RESPONSE, mks_can.ENABLE, mks_can.SET_HEARTBEAT)
# F5 replies of a drive that no longer follows its targets: run failed, as when stall
# protection releases the motor, and stopped at an end limit.
RUN_FAULTS = (0, 3)
# Teleop sends a locked joint's angle back unchanged; anything beyond rounding is a move.
LOCKED_TOLERANCE_RAD = 1e-6
# A drive stopped mid-move starts again once an encoder reading this long after its stop no
# longer advances the old way, or once it reports the stop complete and barely moves. Bench
# reversals on firmware V1.0.9 settled in 17 ms (median) and 42 ms at most at 120 Hz.
SETTLE_S = 0.012
SETTLED_STEP_COUNTS = 5
# A drive at rest starts a move only toward a target this far ahead in the commanded
# direction, so a target that its stop overshot never starts a move the wrong way.
START_DEADBAND_COUNTS = 30
# After the stop of a running move, a drive gets its hold target right behind the stop and
# again every tick for HOLD_S, at HOLD_RPM at most. On the bench (firmware V1.0.9, base, 80
# stops each), no run back to a kept target started with the hold right behind the stop, and
# none went past 107 counts; with the hold 1 ms later, 5 started and ended within 437 counts,
# and with only a re-target once a run showed, 7 ran up to 2515 counts.
HOLD_RPM = 60
HOLD_S = 0.25
# A drive keeps its zero across a restart of this node only while its zero status says it was
# zeroed since power-on and its raw-minus-zeroed encoder offset is the one saved at the zero,
# give or take the counts it turns between two replies at rest (bench: 2 at most).
ZERO_MATCH_COUNTS = 16
ZERO_CHECK_TIMEOUT_S = 2.0


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
        # Joints whose drive this arm lacks hold still at the profile's angles.
        profile = load_profile(params.get('arm_profile', ''))
        params = fit_drive_parameters(params, profile)
        self.locked = dict(profile.locked_joints)
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
        health_thresholds = DriveHealthThresholds(
            following_error_degrees=float(params['health_following_error_degrees']),
            stall_movement_degrees=float(params['health_stall_movement_degrees']),
            stall_duration_seconds=float(params['health_stall_duration_seconds']),
            reply_timeout_seconds=float(params['health_reply_timeout_seconds']),
        )
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
        elif not unmodeled <= set(self.map.joints) | set(self.locked):
            raise ValueError('unmodeled_joints must name drives in the simulated map')
        unmodeled -= set(self.locked)
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
        if frame_log := params.get('frame_log', ''):
            self.bus = RecordingBus(self.bus, os.path.expanduser(frame_log))
            self.get_logger().info(f'Recording every drive frame to {frame_log}')
        self.test_fault = None
        if self.interface == 'sim':
            self.declare_parameter('test_fault', 'none')
        self.carrier, self.carrier_time = None, None
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
        # Each drive's running move: +1 or -1 counts up or down, 0 at rest. After the stop of
        # a running move, (time of the stop, its direction) until the drive has settled.
        self.direction = [0] * len(drives)
        self.settling = [None] * len(drives)
        # Where each drive was told to stay at its last stop, which it must keep, and until
        # when that target goes out again every tick, with the tick it last went out.
        self.held = [None] * len(drives)
        self.hold_until = [-math.inf] * len(drives)
        self.hold_tick = [-1] * len(drives)
        self.ticks = 0
        self.completed = [-math.inf] * len(drives)
        self.drive_health_monitors = [
            DriveHealthMonitor(health_thresholds) for _ in drives]
        self.drive_health_states = [None] * len(drives)
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
        # The file the zero is saved in, each drive's saved offset (see check_zero), the
        # offsets read since the last zero, and the drives whose zero is being checked.
        self.zero_state = os.path.expanduser(params.get('zero_state', '') or '')
        self.saved_zero, self.zero_offsets, self.checks = {}, {}, {}
        if self.interface == 'sim':
            self.add_on_set_parameters_callback(self.on_test_fault_parameter)
        for index in range(len(drives)):
            self.set_up(index)
        zero_on_start = params.get('zero_on_start', self.interface == 'sim')
        if zero_on_start:
            self.zero()
        else:
            self.saved_zero = self.load_zero()
            for index in self.saved_zero:
                self.check_zero(index, 'restore')
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
        self.report_errors = self.report_refused = 0
        self.create_timer(self.period, self.tick)
        self.create_timer(REPORT_PERIOD_S, self.report)
        drive_list = ', '.join(f'{drive.name}=CAN {drive.can_id}' for drive in drives)
        if self.interface == 'sim':
            self.get_logger().info(f'Simulated drives: {drive_list} (no hardware)')
        else:
            self.get_logger().info(f'MKS drives on {self.bus_label}: {drive_list}')
        if self.checks:
            self.get_logger().info(
                f'Checking whether the drives kept the zero saved in {self.zero_state}')
        elif not zero_on_start:
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
        # A hold in the old coordinates would move the arm after the zero, so none goes out.
        self.zeroed = [False] * len(self.map.drives)
        self.forget_zero()
        self.stop_all('Zeroing the drives')
        self.held = [None] * len(self.map.drives)
        self.hold_until = [-math.inf] * len(self.map.drives)
        for drive in self.map.drives:
            self.bus.send(drive.can_id, mks_can.set_zero(drive.can_id))

    def on_zero(self, request, response):
        if time.monotonic() - self.command_time < QUIET_BEFORE_ZERO_S:
            response.message = 'Joint commands are still arriving; disable teleop (Start) first'
            return response
        self.zero()
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        while not all(self.zeroed) and time.monotonic() < deadline:
            time.sleep(0.005)
            self.drain(time.monotonic(), zeroing=True)
        missing = [drive.name for drive, done in zip(self.map.drives, self.zeroed) if not done]
        response.success = not missing
        response.message = ('Zeroed every drive' if response.success
                            else 'No zero reply from ' + ', '.join(missing))
        self.get_logger().info(response.message)
        return response

    def check_zero(self, index, purpose):
        """
        Read a drive's zero fingerprint, to save it after a zero or restore it with ``restore``.

        The raw encoder (35h) counts from power-on and a zero leaves it alone, so it minus the
        zeroed count stays the same from one zero until the drive loses power, and the zero
        status (3Bh) says whether the drive was zeroed since power-on. A drive keeps its saved
        zero only if its status shows a zero, which a power cycle clears, and its offset is the
        saved one, which a zero made elsewhere would change.
        """
        self.checks[index] = {'purpose': purpose, 'raw': None, 'count': None, 'set': None,
                              'deadline': time.monotonic() + ZERO_CHECK_TIMEOUT_S}

    def send_checks(self, now):
        for index, check in list(self.checks.items()):
            drive = self.map.drives[index]
            if self.heard[index] is None or now - self.heard[index] > REPLY_TIMEOUT_S:
                # The check times out only while the drive answers, as the carrier may still
                # be starting.
                check['deadline'] = now + ZERO_CHECK_TIMEOUT_S
                continue
            if now > check['deadline']:
                del self.checks[index]
                self.get_logger().warning(
                    f'{drive.name} did not report its zero status, so its zero is '
                    + ('not saved' if check['purpose'] == 'save'
                       else f'not restored. {self.zero_hint()}'))
                continue
            if check['raw'] is None:
                self.bus.send(drive.can_id, mks_can.read_raw_encoder(drive.can_id))
            if check['set'] is None:
                self.bus.send(drive.can_id, mks_can.read_zero_status(drive.can_id))

    def finish_check(self, index):
        check = self.checks[index]
        if None in (check['raw'], check['count'], check['set']):
            return
        del self.checks[index]
        name = self.map.drives[index].name
        offset = check['raw'] - check['count']
        if check['purpose'] == 'save':
            if not check['set']:
                self.get_logger().warning(f'{name} does not report its zero, so it is not saved')
                return
            self.zero_offsets[index] = offset
            if len(self.zero_offsets) == len(self.map.drives):
                self.save_zero()
            return
        saved = self.saved_zero[index]
        if not check['set']:
            reason = 'lost power since its last zero'
        elif abs(offset - saved) > ZERO_MATCH_COUNTS:
            reason = f'was zeroed elsewhere since (offset {offset}, saved {saved})'
        else:
            self.zeroed[index] = True
            # Its last hold is from before it went quiet, so it gets a new one at its next stop.
            self.held[index], self.hold_until[index] = None, -math.inf
            self.get_logger().info(f'{name} kept its zero')
            if all(self.zeroed):
                self.get_logger().info('Every drive kept its zero; teleop can be enabled')
            return
        self.get_logger().warning(f'{name} {reason}, so it is not zeroed. {self.zero_hint()}')

    def load_zero(self):
        """Return each drive's saved offset by index, for the drives the zero file names."""
        if not self.zero_state:
            return {}
        try:
            with open(self.zero_state, encoding='utf-8') as file:
                saved = yaml.safe_load(file)['drives']
            return {index: int(saved[drive.name]['offset'])
                    for index, drive in enumerate(self.map.drives)
                    if drive.name in saved and int(saved[drive.name]['can_id']) == drive.can_id}
        except FileNotFoundError:
            return {}
        except (OSError, yaml.YAMLError, AttributeError, KeyError, TypeError,
                ValueError) as error:
            self.get_logger().warning(f'Ignoring the zero saved in {self.zero_state}: {error}')
            return {}

    def save_zero(self):
        drives = {drive.name: {'can_id': drive.can_id, 'offset': self.zero_offsets[index]}
                  for index, drive in enumerate(self.map.drives)}
        saved = {'saved': datetime.datetime.now().isoformat(timespec='seconds'),
                 'drives': drives}
        partial = self.zero_state + '.partial'
        try:
            os.makedirs(os.path.dirname(self.zero_state) or '.', exist_ok=True)
            with open(partial, 'w', encoding='utf-8') as file:
                yaml.safe_dump(saved, file, sort_keys=False)
            os.replace(partial, self.zero_state)
        except OSError as error:
            self.get_logger().warning(f'Could not save the zero to {self.zero_state}: {error}')
            return
        self.saved_zero = dict(self.zero_offsets)
        self.get_logger().info(f'Saved the zero to {self.zero_state}')

    def forget_zero(self):
        """Drop the saved zero, so no later start or reconnect restores it."""
        self.saved_zero, self.zero_offsets, self.checks = {}, {}, {}
        if not self.zero_state:
            return
        try:
            os.remove(self.zero_state)
        except FileNotFoundError:
            pass
        except OSError as error:
            self.get_logger().warning(f'Could not remove {self.zero_state}: {error}')

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
              and self.feedback_fresh(now) and not self.estop_pressed):
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
        # A drive whose stop has not gone out yet still runs its last move.
        self.sent = [sent if index in self.stopping else None
                     for index, sent in enumerate(self.sent)]
        # The stops just sent must settle before a fresh Start moves those drives again.
        now = time.monotonic()
        self.settling = [(now, direction) if direction else settling
                         for direction, settling in zip(self.direction, self.settling)]
        self.direction = [0] * len(self.map.drives)
        self.stop_reason = reason

    def on_command(self, message):
        self.command_time, self.stale = time.monotonic(), False
        if not self.authorized or self.commanded is None:
            return
        names = message.name
        expected = set(self.map.joints) | set(self.locked)
        if (len(names) != len(expected) or set(names) != expected
                or len(message.position) != len(names) or len(message.velocity) != len(names)
                or any(not math.isfinite(value) for value in message.position)
                or any(not math.isfinite(value) for value in message.velocity)):
            self.reject_command('incomplete, duplicate or nonfinite joint command')
            return
        positions = dict(zip(names, message.position))
        velocities = dict(zip(names, message.velocity))
        for joint, angle in self.locked.items():
            if abs(positions.pop(joint) - angle) > LOCKED_TOLERANCE_RAD:
                self.reject_command(f'{joint} is locked at {angle:g} rad')
                return
            del velocities[joint]
        measured = self.map.to_positions(self.counts)
        for joint, (lower, upper) in self.limits.items():
            position, actual = positions[joint], measured[joint]
            # An already-outside encoder may move toward the URDF range, never farther out.
            if ((position < lower and (actual >= lower or position < actual))
                    or (position > upper and (actual <= upper or position > actual))):
                self.reject_command(f'{joint} target outside its URDF range')
                return
        self.commanded = positions
        self.velocities = velocities
        # Send now rather than at the next tick, unless that tick will stop the drives.
        now = time.monotonic()
        if (now - self.last_tick < self.heartbeat_s and self.feedback_fresh(now)
                and now - self.enabled_at <= self.enable_timeout):
            self.send_targets()
            self.drain(now)

    @property
    def estop_pressed(self):
        # The last report counts even if the carrier has since gone quiet.
        return self.carrier is not None and self.carrier['estop'] == 'pressed'

    def reject_command(self, reason):
        self.rejected += 1
        self.disarm(reason, require_release=True)
        self.get_logger().warning(f'{reason}; release Start before re-enabling',
                                  throttle_duration_sec=2.0)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        self.ticks += 1
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
        self.send_holds(now)
        self.send_checks(now)
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
        positions.update(self.locked)
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
        self.checks.pop(index, None)
        self.stop_all(f'{name} {reason}')
        self.get_logger().error(f'{name} {reason}; every drive is stopped')

    def check_following(self):
        for index, (sent, count) in enumerate(zip(self.sent, self.counts)):
            # A stopped drive must stay where it stopped, armed or not, as its firmware can
            # run on to a kept target after any stop.
            target = None
            if self.authorized and sent:
                target = self.held[index] if sent[0] is None else sent[0]
            elif not self.authorized and index not in self.stopping and self.zeroed[index]:
                target = self.held[index]
            if target is None or count is None or abs(target - count) <= self.following_error:
                self.behind[index] = 0
                continue
            self.behind[index] += 1
            if self.behind[index] < self.following_ticks:
                continue
            if self.authorized and sent[0] is not None:
                self.fault(index, f'is {abs(target - count)} counts from its target')
                continue
            # Its encoder still counts true, so the arm keeps its zero. Its last stop did not
            # hold it, so it gets another, and no hold drives it back that far.
            name = self.map.drives[index].name
            reason = f'{name} turned {count - target:+d} counts after its stop'
            self.held[index], self.hold_until[index] = None, -math.inf
            self.stopping.add(index)
            self.stop_all(reason)
            self.get_logger().error(f'{reason}; every drive is stopped')

    def fault(self, index, reason):
        """Stop every drive after one stops following; the arm must be zeroed again."""
        name = self.map.drives[index].name
        self.zeroed[index] = False
        # A restart must not bring back a zero the arm no longer follows.
        self.forget_zero()
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
        now = time.monotonic()
        for index in sorted(self.stopping):
            drive = self.map.drives[index]
            data = mks_can.stop(drive.can_id, 0)
            # The hold is where the drive is once the stop goes out, not when it was asked for.
            target, running = self.hold_target(index, now)
            if self.bus.send(drive.can_id, data):
                self.stopping.discard(index)
                self.sent[index], self.held[index] = (None, 0), target
                self.last_command[index] = mks_can.hex_frame(drive.can_id, data)
                self.hold_until[index] = -math.inf
                if running:
                    self.start_hold(index, now)

    def send_targets(self):
        try:
            moves = self.map.synchronized(self.commanded, self.counts, self.period, self.max_rpm,
                                          self.velocities)
            motor_rates = self.map.shaft_rpm(self.velocities)
            commands = []
            # Preflight every int24 axis and frame before sending any coupled motor an update.
            for drive, (axis, speed) in zip(self.map.drives, moves):
                if not mks_can.MIN_AXIS <= axis <= mks_can.MAX_AXIS:
                    raise ValueError(f'{drive.name}: axis {axis} is outside int24')
                commands.append(mks_can.absolute_axis(drive.can_id, axis, speed, self.acc))
        except (ValueError, OverflowError) as error:
            self.reject_command(f'Cannot synchronize drives: {error}')
            return
        for index, (drive, (axis, speed), rpm, data) in enumerate(
                zip(self.map.drives, moves, motor_rates, commands)):
            ahead, direction = axis - self.counts[index], self.direction[index]
            if (rpm == 0.0 or self.settling[index] is not None
                    or (direction and ahead * direction < 0)):
                # A refused stop goes out again on the next tick, like a refused target.
                self.hold(index)
            elif direction or (abs(ahead) > START_DEADBAND_COUNTS and ahead * rpm > 0):
                if self.sent[index] != (axis, speed) and self.bus.send(drive.can_id, data):
                    self.sent[index], self.held[index] = (axis, speed), None
                    self.hold_until[index] = -math.inf
                    self.direction[index] = direction or (1 if ahead > 0 else -1)
                    self.last_command[index] = mks_can.hex_frame(drive.can_id, data)

    def hold(self, index):
        """Stop one armed drive where it is; a running move must then settle."""
        if self.sent[index] != (None, 0):
            drive = self.map.drives[index]
            data = mks_can.stop(drive.can_id, 0)
            now = time.monotonic()
            target, running = self.hold_target(index, now)
            if not self.bus.send(drive.can_id, data):
                return
            self.sent[index], self.held[index] = (None, 0), target
            self.last_command[index] = mks_can.hex_frame(drive.can_id, data)
            if running:
                self.start_hold(index, now)
            else:
                self.hold_until[index] = -math.inf
        if self.direction[index]:
            self.settling[index] = (time.monotonic(), self.direction[index])
            self.direction[index] = 0

    def hold_target(self, index, now):
        """Return where a drive about to be stopped is to stay, and if it must be held there."""
        sent, count = self.sent[index], self.counts[index]
        if self.held[index] is not None and now < self.hold_until[index]:
            # A stop ends the drive's run to its hold, after which it must keep holding.
            return self.held[index], True
        if not sent or sent[0] is None or count is None:
            return count, False
        # The running move's end, though never more than a lagging drive covers in two ticks,
        # or since its last count if that is older.
        axis, speed = sent
        reach = round(max(2 * self.period, self.since_count[index] + self.period) * speed
                      / 60.0 * mks_can.COUNTS_PER_REV) + START_DEADBAND_COUNTS
        target = min(max(axis, count - reach), count + reach)
        return min(max(target, mks_can.MIN_AXIS), mks_can.MAX_AXIS), True

    def start_hold(self, index, now):
        self.hold_until[index] = now + HOLD_S
        self.send_hold(index, now)

    def send_holds(self, now):
        for index in range(len(self.map.drives)):
            if now < self.hold_until[index] and self.hold_tick[index] != self.ticks:
                self.send_hold(index, now)

    def send_hold(self, index, now):
        """Send a drive its hold target, unless it may have lost the zero that target uses."""
        held, count, heard = self.held[index], self.counts[index], self.heard[index]
        if (held is None or count is None or heard is None or now - heard > REPLY_TIMEOUT_S
                or abs(held - count) > self.following_error or index in self.stopping
                or not self.zeroed[index] or self.lost[index] or self.estop_pressed):
            return
        can_id = self.map.drives[index].can_id
        # Not a last command: a refused hold simply goes out again on the next tick.
        if self.bus.send(can_id, mks_can.absolute_axis(
                can_id, held, min(HOLD_RPM, self.max_rpm), self.acc)):
            self.hold_tick[index] = self.ticks

    def settle(self, index, step):
        """Let a stopped drive move again once an encoder step shows it no longer runs on."""
        stopped_at, old = self.settling[index]
        if ((self.completed[index] > stopped_at and abs(step) < SETTLED_STEP_COUNTS)
                or (time.monotonic() - stopped_at > SETTLE_S and step * old <= 0)):
            self.settling[index] = None

    def drain(self, now, zeroing=False):
        while (reply := self.bus.receive()) is not None:
            self.on_reply(*reply, now)
        # The carrier answers a frame it refuses after send() has already returned True.
        if self.interface == 'slcan':
            for can_id, data in self.bus.take_refused():
                self.on_refused(can_id, data, zeroing)

    def on_refused(self, can_id, data, zeroing):
        """Send a refused frame again on the next tick, unless a later frame replaced it."""
        index = self.index.get(can_id)
        if index is None:
            return
        if data[0] == mks_can.ABSOLUTE_AXIS:
            if self.last_command[index] != mks_can.hex_frame(can_id, data):
                return
            # While disarmed the last command is a stop, as holds never are one, and
            # send_stops repeats a stop until it is sent.
            if self.authorized:
                self.sent[index] = None
            else:
                self.stopping.add(index)
        elif data[0] in self.unconfirmed[index]:
            self.setup_time[index] = -math.inf
        elif data[0] == mks_can.SET_ZERO and zeroing:
            # Only while the zero service waits: a late zero would move the arm's zero pose.
            self.bus.send(can_id, data)

    def on_reply(self, can_id, data, now):
        if can_id == carrier.STATUS_ID and self.interface == 'slcan':
            pressed = self.estop_pressed
            try:
                self.carrier, self.carrier_time = carrier.parse_status(data), now
            except ValueError as error:
                self.bus_errors += 1
                self.get_logger().warning(str(error), throttle_duration_sec=2.0)
                return
            if self.estop_pressed and not pressed:
                # The e-stop cuts the drives' power; Start is refused until it is released.
                # A drive may restart with a new count, so no hold goes out after the release.
                self.stop_all('E-stop pressed')
                self.hold_until = [-math.inf] * len(self.map.drives)
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
            self.lost[index] = False
            self.set_up(index)
            name = self.map.drives[index].name
            if index in self.saved_zero:
                self.check_zero(index, 'restore')
                self.get_logger().warning(
                    f'{name} answers again; checking whether it kept its zero')
            else:
                self.get_logger().warning(f'{name} answers again. {self.zero_hint()}')
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
                if index in self.checks:
                    self.checks[index]['count'] = count
                if self.settling[index] is not None and previous is not None:
                    self.settle(index, count - previous)
            elif code == mks_can.READ_RAW_ENCODER:
                if index in self.checks:
                    self.checks[index]['raw'] = mks_can.encoder_value(arguments)
            elif code == mks_can.READ_ZERO_STATUS:
                if index in self.checks:
                    self.checks[index]['set'] = mks_can.zero_is_set(arguments)
            elif code == mks_can.ABSOLUTE_AXIS and arguments:
                self.state[index] = mks_can.RUN_STATUS.get(arguments[0], f'status {arguments[0]}')
                if arguments[0] in RUN_FAULTS:
                    failed = self.state[index]
                elif arguments[0] == 2:
                    self.completed[index] = time.monotonic()
            elif code == mks_can.SET_ZERO and arguments[:1] == b'\x01':
                # Later encoder replies were read after the zero.
                self.zeroed[index], self.counts[index] = True, None
                if self.zero_state:
                    self.check_zero(index, 'save')
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
        if index in self.checks:
            self.finish_check(index)
        # Only stops and holds go out while disarmed, and a released motor refuses those too.
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
            load_percent=f'{load:.1f}', errors=errors, rejected_commands=self.rejected,
            interface=self.interface)]
        if self.interface == 'slcan':
            statuses.append(self.carrier_status(now))
        gate_level = (DiagnosticStatus.OK if self.authorized else
                      DiagnosticStatus.ERROR if not self.description_valid else
                      DiagnosticStatus.WARN)
        statuses.append(status('arm.command_gate', gate_level,
                               'enabled' if self.authorized else 'stopped', '',
                               self.stop_reason or 'Receiving validated teleop setpoints'))
        if not all(self.zeroed):
            restoring = any(check['purpose'] == 'restore' for check in self.checks.values())
            statuses.append(status(
                'arm.zero', DiagnosticStatus.WARN, 'checking' if restoring else 'no', '',
                'Checking whether the drives kept their saved zero' if restoring
                else f'Not zeroed. {self.zero_hint()}'))
        elif None not in self.counts:
            positions = self.map.to_positions(self.counts)
            commanded = self.commanded or {}
            for joint, position in positions.items():
                target = math.degrees(commanded.get(joint, position))
                statuses.append(status(f'arm.{joint}', DiagnosticStatus.OK,
                                       f'{math.degrees(position):+.1f}', 'deg',
                                       f'target {target:+.1f} deg'))
        health_motor_rates = (self.map.to_rpm(self.velocities) if self.velocities else
                              [0.0] * len(self.map.drives))
        health_targets = (self.map.to_counts(self.commanded)
                          if self.commanded is not None else None)
        for index, drive in enumerate(self.map.drives):
            silent = self.heard[index] is None or now - self.heard[index] > REPLY_TIMEOUT_S
            failed = ('failed' in self.state[index]
                      or self.state[index] in [mks_can.RUN_STATUS[code] for code in RUN_FAULTS])
            turns = '' if self.counts[index] is None else (
                f'{self.counts[index] / mks_can.COUNTS_PER_REV:+.3f}')
            count = self.counts[index]
            heard = self.heard[index]
            counts_to_joint_degrees = (
                360.0 / mks_can.COUNTS_PER_REV / drive.gear_ratio)
            if count is None or heard is None:
                self.drive_health_states[index] = None
            else:
                target = health_targets[index] if health_targets is not None else count
                enabled = mks_can.ENABLE not in self.unconfirmed[index]
                if self.interface == 'sim':
                    servo = self.bus.drives.get(drive.can_id)
                    if servo is not None:
                        enabled = enabled and servo.enabled
                self.drive_health_states[index] = self.drive_health_monitors[index].evaluate(
                    target_position=target * counts_to_joint_degrees,
                    encoder_position=count * counts_to_joint_degrees,
                    motion_commanded=self.authorized and health_motor_rates[index] > 0.0,
                    enabled=enabled,
                    reply_age_seconds=now - heard,
                    now_seconds=now,
                )
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
            health_state = self.drive_health_states[index]
            health_values = {}
            if health_state is not None:
                target = health_targets[index] if health_targets is not None else count
                health_values = {
                    'state': health_state.value,
                    'target_encoder_error_deg': (
                        f'{(target - count) * counts_to_joint_degrees:.3f}'),
                    'reply_age_sec': f'{now - heard:.3f}',
                }
                if not failed:
                    if health_state is DriveHealthState.NOT_RESPONDING:
                        level = DiagnosticStatus.STALE
                    elif health_state is not DriveHealthState.OK:
                        level = DiagnosticStatus.ERROR
            statuses.append(status(
                f'drive.{drive.name}', level, turns, 'rev',
                f'CAN ID {drive.can_id}: {text}'
                + (f'; last {self.last_command[index]}' if self.last_command[index] else ''),
                can_id=drive.can_id, last_command=self.last_command[index],
                **health_values))
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_publisher.publish(message)

    def on_test_fault_parameter(self, parameters):
        """Apply a simulator-only ``drive_name:fault`` parameter or clear with ``none``."""
        requested = next(
            (parameter.value for parameter in parameters if parameter.name == 'test_fault'),
            None)
        if requested is None:
            return SetParametersResult(successful=True)
        if self.interface != 'sim':
            return SetParametersResult(
                successful=False, reason='simulated drive faults require interface:=sim')

        requested = requested.strip().lower()
        if requested == 'none':
            if self.test_fault is not None:
                can_id, _ = self.test_fault
                self.bus.drives[can_id].set_test_fault(None)
                self.test_fault = None
            return SetParametersResult(successful=True)

        try:
            drive_name, fault = requested.split(':', 1)
            drive = next(drive for drive in self.map.drives if drive.name == drive_name)
        except (ValueError, StopIteration):
            return SetParametersResult(
                successful=False, reason='Expected none or <drive_name>:<fault>')
        if fault not in SimulatedServo.TEST_FAULTS:
            return SetParametersResult(
                successful=False,
                reason=f'Fault must be one of {sorted(SimulatedServo.TEST_FAULTS)}')

        if fault == 'disabled' and self.authorized:
            self.disarm('Simulated drive test fault: disabled', require_release=True)
        if self.test_fault is not None:
            can_id, _ = self.test_fault
            self.bus.drives[can_id].set_test_fault(None)
        self.bus.drives[drive.can_id].set_test_fault(fault)
        self.test_fault = (drive.can_id, fault)
        return SetParametersResult(successful=True)

    def carrier_status(self, now):
        """Report the USB link to the carrier and what the carrier says about the bus."""
        refused, self.report_refused = self.bus.refused - self.report_refused, self.bus.refused
        link = {'reopens': self.bus.reopens, 'refused_frames': self.bus.refused}
        if not self.bus.connected:
            return status('can.carrier', DiagnosticStatus.ERROR, 'unplugged', '',
                          f'USB link lost ({self.bus.problem}); retrying every second', **link)
        if self.carrier_time is None or now - self.carrier_time > CARRIER_TIMEOUT_S:
            return status('can.carrier', DiagnosticStatus.WARN, 'silent', '',
                          'No status from the carrier: '
                          + (self.bus.problem or 'is carrier_bridge flashed?'), **link)
        report = self.carrier
        estop = report['estop'] or 'not wired'
        supply = 'not wired' if report['supply_v'] is None else f'{report["supply_v"]:.1f} V'
        message = f'E-stop {estop}, supply {supply}, {report["can_errors"]} CAN errors'
        level = DiagnosticStatus.OK
        if refused:
            # Each refused target or stop goes out again, a tick late.
            level, message = DiagnosticStatus.WARN, f'{message}, {refused} frames refused'
        if report['estop'] == 'pressed':
            level = DiagnosticStatus.ERROR
        return status('can.carrier', level, estop, 'e-stop', message,
                      supply=supply, can_errors=report['can_errors'],
                      failed_writes=report['failed_writes'],
                      refused_lines=report['refused_lines'], **link)

    def close(self):
        """Stop the drives and release the bus; the drives' heartbeat would also stop them."""
        try:
            self.stop_all('Shutting down')
            if self.interface == 'slcan':
                # The carrier may refuse a stop after send() returned True, and no later tick
                # would send it again, so write the stops now, then wait for the carrier's
                # answers and repeat refused stops before closing it. The holds behind the
                # stops go out every period meanwhile, as on any tick.
                deadline = time.monotonic() + CLOSE_TIMEOUT_S
                self.drain(time.monotonic())
                last_hold = time.monotonic()
                while self.bus.connected and time.monotonic() < deadline:
                    now = time.monotonic()
                    holding = any(now < until for until in self.hold_until)
                    if not (self.stopping or self.bus.unanswered or holding):
                        break
                    time.sleep(0.002)
                    now = time.monotonic()
                    self.drain(now)
                    self.send_stops()
                    if now - last_hold >= self.period:
                        last_hold, self.ticks = now, self.ticks + 1
                        self.send_holds(now)
        finally:
            # Even if a second Ctrl+C lands here, the queued stops and C still go out.
            self.bus.shutdown()


def main():
    rclpy.init()
    node = None
    try:
        node = SimArmDrives()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as error:
        if not is_shutdown_error(error):
            raise
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
