import importlib.util
from pathlib import Path

import diagnostic_msgs.msg
import rclpy


PUBLISHER_PATH = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'temporary_diagnostics_publisher.py'
)
MODULE_SPEC = importlib.util.spec_from_file_location(
    'temporary_diagnostics_publisher', PUBLISHER_PATH
)
PUBLISHER = importlib.util.module_from_spec(MODULE_SPEC)
MODULE_SPEC.loader.exec_module(PUBLISHER)


def statuses_for(mode):
    rclpy.init()
    node = PUBLISHER.TemporaryDiagnosticsPublisher.__new__(
        PUBLISHER.TemporaryDiagnosticsPublisher
    )
    node.get_clock = lambda: rclpy.clock.Clock()
    try:
        return node.build_statuses(mode)
    finally:
        rclpy.shutdown()


def status_values(status):
    return {item.key: item.value for item in status.values}


def test_normal_fixture_contract():
    statuses = statuses_for('normal')
    assert [status.name for status in statuses] == [
        'board.temperature', 'motor.current', 'imu.roll', 'imu.pitch', 'imu.yaw'
    ]
    assert all(status.level == diagnostic_msgs.msg.DiagnosticStatus.OK for status in statuses)
    assert [status_values(status)['unit'] for status in statuses] == [
        'C', 'A', 'deg', 'deg', 'deg'
    ]
    assert all(status_values(status)['value'] for status in statuses)


def test_fault_fixture_contract():
    statuses = {status.name: status for status in statuses_for('fault')}
    assert statuses['board.temperature'].level == diagnostic_msgs.msg.DiagnosticStatus.ERROR
    assert status_values(statuses['board.temperature']) == {'value': statuses['board.temperature'].values[0].value, 'unit': 'C'}
    assert statuses['board.temperature'].message == 'High temperature detected'
    assert statuses['imu.heartbeat'].level == diagnostic_msgs.msg.DiagnosticStatus.STALE
    assert not statuses['imu.heartbeat'].values


def test_stale_fixture_uses_unavailable_values():
    statuses = {status.name: status for status in statuses_for('stale')}
    for name, unit in (
        ('board.temperature', 'C'),
        ('motor.current', 'A'),
        ('imu.roll', 'deg'),
        ('imu.pitch', 'deg'),
        ('imu.yaw', 'deg'),
    ):
        status = statuses[name]
        assert status.level == diagnostic_msgs.msg.DiagnosticStatus.STALE
        assert status_values(status) == {'value': '--', 'unit': unit}