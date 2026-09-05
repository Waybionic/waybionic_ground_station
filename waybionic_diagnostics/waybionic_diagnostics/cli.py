import argparse
import sys
import time
from typing import Dict, List, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node


STALE_AFTER_SECONDS = 5.0


def status_name(level: int) -> str:
    """Convert a ROS diagnostic level into the CLI status name."""
    if level == DiagnosticStatus.OK:
        return 'OK'
    if level == DiagnosticStatus.WARN:
        return 'WARN'
    if level == DiagnosticStatus.ERROR:
        return 'FAULT'
    if level == DiagnosticStatus.STALE:
        return 'STALE'
    return 'WARN'


def extract_value_and_unit(status: DiagnosticStatus):
    """Extract value/unit using the same rules as the RViz diagnostics source."""
    value: Optional[str] = None
    unit: Optional[str] = None

    for key_value in status.values:
        key = key_value.key.lower()

        if key == 'value':
            value = key_value.value
            continue

        if key == 'unit':
            unit = key_value.value
            continue

        if value is None and key_value.value:
            value = key_value.value
            unit = key_value.key

    return value, unit


class DiagnosticsCliNode(Node):
    """ROS 2 subscriber and state holder for the diagnostics CLI."""

    def __init__(self, topic: str):
        super().__init__('waybionic_diagnostics_cli')

        self.topic = topic
        self.latest: List[DiagnosticStatus] = []
        self.last_received_monotonic: Optional[float] = None

        self.subscription = self.create_subscription(
            DiagnosticArray,
            topic,
            self.diagnostics_callback,
            10,
        )

    def diagnostics_callback(self, message: DiagnosticArray):
        """Store the most recent diagnostic array."""
        self.latest = list(message.status)
        self.last_received_monotonic = time.monotonic()

    def data_age(self) -> Optional[float]:
        """Return seconds since the most recent diagnostic message."""
        if self.last_received_monotonic is None:
            return None

        return max(0.0, time.monotonic() - self.last_received_monotonic)


def overall_status(statuses: List[str]) -> str:
    """Calculate the worst status in a diagnostic snapshot."""
    priority = {
        'OK': 0,
        'WARN': 1,
        'STALE': 2,
        'FAULT': 3,
    }

    if not statuses:
        return 'STALE'

    return max(statuses, key=lambda status: priority.get(status, 1))


def clear_screen():
    """Clear the terminal for watch mode."""
    sys.stdout.write('\033[2J\033[H')
    sys.stdout.flush()


def print_snapshot(node: DiagnosticsCliNode, clear: bool = False):
    """Render the latest diagnostic state."""
    if clear:
        clear_screen()

    print('WAYBIONIC DIAGNOSTICS')
    print('─' * 100)

    if not node.latest:
        age = node.data_age()

        if age is None:
            print(f'Waiting for messages on {node.topic}...')
        else:
            print(
                f'No diagnostic entries received. '
                f'Last message was {age:.1f}s ago.'
            )

        print('─' * 100)
        print('Overall: STALE')
        return

    age = node.data_age()
    stream_stale = age is not None and age > STALE_AFTER_SECONDS

    print(
        f'{"Signal":<32} '
        f'{"Status":<8} '
        f'{"Value":<16} '
        f'{"Age":<10} '
        f'Message'
    )
    print('─' * 100)

    rendered_statuses = []

    for status in node.latest:
        normalized = status_name(status.level)

        if stream_stale and normalized in ('OK', 'WARN'):
            normalized = 'STALE'

        value, unit = extract_value_and_unit(status)

        if value is None:
            display_value = '-'
        elif unit:
            display_value = f'{value} {unit}'
        else:
            display_value = value

        if age is None:
            display_age = '-'
        else:
            display_age = f'{age:.1f}s'

        message = status.message or '-'

        if stream_stale and status.level in (
            DiagnosticStatus.OK,
            DiagnosticStatus.WARN,
        ):
            message = f'No recent update from {node.topic}'

        print(
            f'{status.name:<32.32} '
            f'{normalized:<8} '
            f'{display_value:<16.16} '
            f'{display_age:<10} '
            f'{message}'
        )

        rendered_statuses.append(normalized)

    print('─' * 100)
    print(f'Overall: {overall_status(rendered_statuses)}')

    if age is not None:
        print(f'Diagnostics stream age: {age:.1f}s')


def wait_for_first_message(node: DiagnosticsCliNode, timeout: float = 5.0):
    """Wait briefly for the first diagnostics message."""
    deadline = time.monotonic() + timeout

    while rclpy.ok() and node.last_received_monotonic is None:
        if time.monotonic() >= deadline:
            return False

        rclpy.spin_once(node, timeout_sec=0.1)

    return node.last_received_monotonic is not None


def run_snapshot(node: DiagnosticsCliNode):
    """Run one diagnostic snapshot."""
    wait_for_first_message(node)
    print_snapshot(node)


def run_watch(node: DiagnosticsCliNode):
    """Continuously display diagnostics."""
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            print_snapshot(node, clear=True)
            time.sleep(0.3)
    except KeyboardInterrupt:
        pass


def main(args=None):
    """Run the Waybionic diagnostics CLI."""
    parser = argparse.ArgumentParser(
        description='Waybionic ROS 2 diagnostics viewer.'
    )

    parser.add_argument(
        '--topic',
        default='/diagnostics',
        help='ROS 2 DiagnosticArray topic (default: /diagnostics)',
    )

    parser.add_argument(
        '--watch',
        action='store_true',
        help='Continuously monitor diagnostics.',
    )

    parsed_args, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = DiagnosticsCliNode(parsed_args.topic)

    try:
        if parsed_args.watch:
            run_watch(node)
        else:
            run_snapshot(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
