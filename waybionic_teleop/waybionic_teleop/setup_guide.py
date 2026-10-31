"""
Walk the operator through starting the arm: what to do next, from the /diagnostics rows.

The drive and teleop nodes already report the carrier, each drive, the controller and the zero
on /diagnostics. This node turns those rows into five steps, in the order the operator does
them: carrier, drives, controller, zero and ready. It publishes them on ``arm/setup`` (a latched
DiagnosticArray of ``setup.<step>`` rows) for the RViz setup panel, and says each change in the
terminal. A row's message is the instruction, and its values are ``title``, ``state`` (OK,
TODO, BLOCKED or WORKING), ``action`` (``zero`` when the panel is to offer the zero button) and
``current`` (true for the first step that is not done).

It only reads and reports. The zero is the drive node's own ``~/zero`` service.
"""

from dataclasses import dataclass
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from waybionic_teleop.ros_shutdown import is_shutdown_error

TOPIC = 'arm/setup'
OK, TODO, BLOCKED, WORKING = 'ok', 'todo', 'blocked', 'working'
LEVELS = {OK: DiagnosticStatus.OK, WORKING: DiagnosticStatus.OK, TODO: DiagnosticStatus.WARN,
          BLOCKED: DiagnosticStatus.ERROR}
# The drive node reports twice a second; a row this old means whoever sent it has gone.
ROW_TIMEOUT_S = 3.0
PUBLISH_PERIOD_S = 0.5
# The carrier and the drives need a few seconds to answer after a start, so a problem is only
# reported once it has lasted this long; until then the step is still being checked.
BLOCK_GRACE_S = 5.0
TITLES = {'carrier': 'Carrier', 'drives': 'Drives', 'controller': 'Controller',
          'zero': 'Zero', 'ready': 'Ready'}
WAITS = 'Waits for the steps above'


@dataclass
class Step:
    """One thing the operator has to get right, and what to do about it."""

    name: str
    state: str
    message: str
    action: str = ''

    @property
    def title(self):
        return TITLES[self.name]


def value(row, key='value'):
    """Return one of a row's values, or '' if it has none."""
    if row is None:
        return ''
    return next((item.value for item in row.values if item.key == key), '')


def carrier_step(rows):
    if 'arm.command_gate' not in rows:
        return Step('carrier', WORKING, 'Waiting for the ground station to start...')
    carrier = rows.get('can.carrier')
    if carrier is None:
        interface = value(rows.get('can.bus'), 'interface')
        if interface == 'sim':
            return Step('carrier', OK, 'Simulated drives, no hardware')
        return Step('carrier', OK, f'Using the {interface or "CAN"} adapter')
    state = value(carrier)
    if state == 'unplugged':
        return Step('carrier', TODO, "Plug the carrier's USB-C cable into this computer")
    if state == 'silent':
        return Step('carrier', BLOCKED,
                    'The carrier is plugged in but silent: replug it, or flash its firmware '
                    '(./scripts/macos.sh flash)')
    if state == 'pressed':
        return Step('carrier', BLOCKED, 'The E-stop is pressed: release it')
    return Step('carrier', OK, 'Carrier connected')


def drives_step(rows):
    drives = {name[len('drive.'):]: row for name, row in rows.items()
              if name.startswith('drive.')}
    if not drives:
        return Step('drives', WORKING, 'Waiting for the drives...')
    off = [name for name, row in drives.items() if 'disabled (' in row.message]
    on = {name: row for name, row in drives.items() if name not in off}
    missing = [f'{name} (CAN {value(row, "can_id")})' for name, row in on.items()
               if 'no reply' in row.message]
    if missing:
        return Step('drives', BLOCKED,
                    f'No answer from {", ".join(missing)}: check its power and CAN wiring, or '
                    'switch its joint off in Joint control')
    failed = [f'{name} ({row.message.split(": ", 1)[-1]})' for name, row in on.items()
              if row.level == DiagnosticStatus.ERROR]
    if failed:
        return Step('drives', BLOCKED, f'Drive fault: {", ".join(failed)}')
    if any('unconfirmed' in row.message for row in on.values()):
        return Step('drives', WORKING, 'Setting up the drives...')
    message = f'{len(on)} drives answer'
    if off:
        message += f', {len(off)} switched off'
    return Step('drives', OK, message)


def controller_step(rows):
    row = rows.get('teleop.input')
    if row is None:
        return Step('controller', WORKING, 'Waiting for teleop to start...')
    if row.level == DiagnosticStatus.OK:
        return Step('controller', OK, 'Controller connected')
    return Step('controller', TODO,
                'Connect an Xbox controller (USB or Bluetooth) and press any button')


def zero_step(rows):
    zero = rows.get('arm.zero')
    if zero is not None:
        if value(zero) == 'checking':
            return Step('zero', WORKING, 'Checking whether the drives kept their saved zero...')
        return Step('zero', TODO,
                    'Put the arm in its zero pose (upright, as RViz shows it), then press Zero',
                    action='zero')
    if any(name.startswith('arm.') and name not in ('arm.zero', 'arm.command_gate')
           for name in rows):
        return Step('zero', OK, 'Zeroed')
    return Step('zero', WORKING, 'Reading the drives...')


def ready_step(rows):
    state = rows.get('teleop.state')
    if state is None:
        return Step('ready', WORKING, 'Waiting for teleop to start...')
    if state.level == DiagnosticStatus.ERROR:
        return Step('ready', BLOCKED, state.message)
    if value(state) == 'enabled':
        return Step('ready', OK, 'Teleop is on: the sticks move the arm, B stops it')
    message = 'Press Start (the Menu button) on the controller'
    if state.level == DiagnosticStatus.WARN and state.message:
        message = state.message
    return Step('ready', TODO, message)


def evaluate(rows):
    """
    Return the five steps for the fresh /diagnostics rows, {name: DiagnosticStatus}.

    The controller can be plugged in at any time. The others need what comes before them: the
    drives are on the carrier's bus, and the zero and the Start press need drives that answer.
    """
    carrier = carrier_step(rows)
    drives = drives_step(rows) if carrier.state == OK else Step('drives', TODO, WAITS)
    controller = controller_step(rows)
    zero = zero_step(rows) if drives.state == OK else Step('zero', TODO, WAITS)
    before = (carrier, drives, controller, zero)
    ready = (ready_step(rows) if all(step.state == OK for step in before)
             else Step('ready', TODO, WAITS))
    return [*before, ready]


def current(steps):
    """Return the first step that is not done, or None once the arm is ready."""
    return next((step for step in steps if step.state != OK), None)


class SetupGuide(Node):
    """Publish the start-up steps as the other nodes' rows change."""

    def __init__(self, **kwargs):
        super().__init__('setup_guide', automatically_declare_parameters_from_overrides=True,
                         **kwargs)
        topic = (self.get_parameter('diagnostics_topic').value
                 if self.has_parameter('diagnostics_topic') else '/diagnostics')
        self.rows = {}
        self.blocked_since = {}
        self.announced = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(DiagnosticArray, TOPIC, latched)
        self.create_subscription(DiagnosticArray, topic, self.on_diagnostics, 10)
        self.create_timer(PUBLISH_PERIOD_S, self.publish_steps)

    def on_diagnostics(self, message):
        now = time.monotonic()
        for row in message.status:
            self.rows[row.name] = (row, now)

    def fresh_rows(self):
        now = time.monotonic()
        return {name: row for name, (row, seen) in self.rows.items()
                if now - seen <= ROW_TIMEOUT_S}

    def announce(self, text):
        self.get_logger().info(text)

    def publish_steps(self):
        now = time.monotonic()
        steps = evaluate(self.fresh_rows())
        for step in steps:
            if step.state != BLOCKED:
                self.blocked_since.pop(step.name, None)
            elif now - self.blocked_since.setdefault(step.name, now) < BLOCK_GRACE_S:
                step.state, step.message = WORKING, f'Checking the {step.title.lower()}...'
        first = current(steps)
        statuses = []
        for step in steps:
            statuses.append(DiagnosticStatus(
                name=f'setup.{step.name}', level=LEVELS[step.state], message=step.message,
                values=[KeyValue(key='title', value=step.title),
                        KeyValue(key='state', value=step.state),
                        KeyValue(key='action', value=step.action),
                        KeyValue(key='current', value=str(step is first).lower())]))
        message = DiagnosticArray(status=statuses)
        message.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(message)
        summary = (first.name, first.state, first.message) if first else None
        if summary != self.announced:
            self.announced = summary
            self.announce(f'Setup: {first.title} - {first.message}' if first
                          else 'Setup: ready, the arm can be operated')


def main():
    rclpy.init()
    node = SetupGuide()
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
