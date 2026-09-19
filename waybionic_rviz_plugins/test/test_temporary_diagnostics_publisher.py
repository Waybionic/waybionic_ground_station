import importlib.util
import os
import signal
import subprocess
import sys
import textwrap
import time
import uuid
from pathlib import Path

import diagnostic_msgs.msg
import pytest
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


def _unique_topic_name():
    return f'/waybionic/test/seg_{uuid.uuid4().hex}/diagnostics'


def _wait_for(predicate, timeout=5.0, period=0.05, message='condition timeout'):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period)
    raise AssertionError(message)


def _init_rclpy(domain_id=None):
    if domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    if not rclpy.ok():
        rclpy.init()


def _shutdown_rclpy():
    if rclpy.ok():
        rclpy.shutdown()


def _launch_publisher(mode, topic, ros_domain_id=None):
    env = os.environ.copy()
    if ros_domain_id is None:
        ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '1'))
    env['ROS_DOMAIN_ID'] = str(ros_domain_id)

    proc = subprocess.Popen(
        [
            sys.executable,
            str(PUBLISHER_PATH),
            '--ros-args',
            '-p', f'mode:={mode}',
            '-p', f'topic:={topic}',
            '-p', 'publish_rate_hz:=10.0',
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return proc


def statuses_for(mode):
    _init_rclpy(domain_id=100)
    node = PUBLISHER.TemporaryDiagnosticsPublisher.__new__(
        PUBLISHER.TemporaryDiagnosticsPublisher
    )
    node.get_clock = lambda: rclpy.clock.Clock()
    try:
        return node.build_statuses(mode)
    finally:
        _shutdown_rclpy()


def status_values(status):
    return {item.key: item.value for item in status.values}


def test_normal_fixture_contract():
    statuses = statuses_for('normal')
    assert [status.name for status in statuses] == [
        'board.temperature', 'motor.current', 'imu.roll', 'imu.pitch', 'imu.yaw'
    ]
    assert all(
        status.level == diagnostic_msgs.msg.DiagnosticStatus.OK
        for status in statuses
    )
    assert [status_values(status)['unit'] for status in statuses] == [
        'C', 'A', 'deg', 'deg', 'deg'
    ]
    assert all(status_values(status)['value'] for status in statuses)


def test_fault_fixture_contract():
    statuses = {status.name: status for status in statuses_for('fault')}
    assert (
        statuses['board.temperature'].level
        == diagnostic_msgs.msg.DiagnosticStatus.ERROR
    )
    assert status_values(statuses['board.temperature']) == {
        'value': statuses['board.temperature'].values[0].value,
        'unit': 'C',
    }
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


def test_publisher_stop_restart_uses_isolated_topics_and_recovery():
    domain_id = 101
    topic = _unique_topic_name()
    proc = _launch_publisher('normal', topic, ros_domain_id=domain_id)
    try:
        _init_rclpy(domain_id=domain_id)
        node = rclpy.create_node(f'publisher_stop_restart_{uuid.uuid4().hex}')
        received = []

        def callback(message):
            received.append(message)

        subscription = node.create_subscription(
            diagnostic_msgs.msg.DiagnosticArray,
            topic,
            callback,
            10,
        )

        def pump():
            rclpy.spin_once(node, timeout_sec=0.05)
            return bool(received)

        _wait_for(
            pump,
            timeout=5.0,
            message=f'No {topic} messages before shutdown',
        )
        before_stop_count = len(received)
        proc.send_signal(signal.SIGINT)
        _wait_for(lambda: proc.poll() is not None, timeout=5.0, period=0.05)
        time.sleep(0.6)
        assert len(received) == before_stop_count

        restarted = _launch_publisher('stale', topic, ros_domain_id=domain_id)
        try:
            _wait_for(
                lambda: (rclpy.spin_once(node, timeout_sec=0.05), bool(received))[-1]
                and len(received) > before_stop_count,
                timeout=5.0,
                message='Publisher did not resume after restart',
            )
            final_statuses = {
                status.name: status for status in received[-1].status
            }
            for signal_name in (
                'board.temperature',
                'motor.current',
                'imu.roll',
                'imu.pitch',
                'imu.yaw',
            ):
                assert final_statuses[signal_name].level == (
                    diagnostic_msgs.msg.DiagnosticStatus.STALE
                )
                assert status_values(final_statuses[signal_name]) == {
                    'value': '--',
                    'unit': final_statuses[signal_name].values[1].value,
                }
        finally:
            restarted.send_signal(signal.SIGINT)
            _wait_for(lambda: restarted.poll() is not None, timeout=5.0)
            subscription.destroy()
            node.destroy_node()
            _shutdown_rclpy()
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            _wait_for(lambda: proc.poll() is not None, timeout=5.0)
        proc.stdout.close()
        proc.stderr.close()


def test_missing_publisher_is_detected_without_cross_domain_noise():
    domain_id = 102
    topic = _unique_topic_name()
    _init_rclpy(domain_id=domain_id)
    node = rclpy.create_node(f'missing_publisher_{uuid.uuid4().hex}')
    received = []

    def callback(message):
        received.append(message)

    subscription = node.create_subscription(
        diagnostic_msgs.msg.DiagnosticArray,
        topic,
        callback,
        10,
    )
    try:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            if received:
                break
        assert not received, f'Unexpected diagnostics on quiet topic {topic}'
    finally:
        subscription.destroy()
        node.destroy_node()
        _shutdown_rclpy()


def test_status_mismatch_is_detected():
    _init_rclpy(domain_id=103)
    try:
        statuses = {status.name: status for status in statuses_for('fault')}
        with pytest.raises(AssertionError):
            assert statuses['board.temperature'].level == (
                diagnostic_msgs.msg.DiagnosticStatus.OK
            )
    finally:
        _shutdown_rclpy()


def test_failing_test_command_returns_nonzero(tmp_path):
    failing_test = tmp_path / 'failing_test.py'
    failing_test.write_text(
        textwrap.dedent(
            """
            def test_failures_are_reported():
                assert False, 'intentional failure for coverage'
            """
        ),
        encoding='utf-8',
    )

    result = subprocess.run(
        [sys.executable, '-m', 'pytest', '-q', str(failing_test)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    assert '1 failed' in (result.stdout + result.stderr)

