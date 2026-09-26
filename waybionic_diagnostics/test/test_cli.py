import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from waybionic_diagnostics.cli import (
    DiagnosticsCliNode,
    extract_value_and_unit,
    overall_status,
    print_snapshot,
    status_name,
)


def test_status_name_mapping():
    assert status_name(DiagnosticStatus.OK) == 'OK'
    assert status_name(DiagnosticStatus.WARN) == 'WARN'
    assert status_name(DiagnosticStatus.ERROR) == 'FAULT'
    assert status_name(DiagnosticStatus.STALE) == 'STALE'
    assert status_name(99) == 'WARN'


def test_extract_value_and_unit():
    status = DiagnosticStatus()
    status.values = [
        KeyValue(key='value', value='42.5'),
        KeyValue(key='unit', value='C'),
    ]

    value, unit = extract_value_and_unit(status)

    assert value == '42.5'
    assert unit == 'C'


def test_extract_value_and_unit_case_insensitive():
    status = DiagnosticStatus()
    status.values = [
        KeyValue(key='VALUE', value='0.85'),
        KeyValue(key='UNIT', value='A'),
    ]

    value, unit = extract_value_and_unit(status)

    assert value == '0.85'
    assert unit == 'A'


def test_extract_value_and_unit_fallback():
    status = DiagnosticStatus()
    status.values = [
        KeyValue(key='temperature', value='42.0'),
    ]

    value, unit = extract_value_and_unit(status)

    assert value == '42.0'
    assert unit == 'temperature'


def test_extract_value_and_unit_empty():
    status = DiagnosticStatus()

    value, unit = extract_value_and_unit(status)

    assert value is None
    assert unit is None


def test_overall_status():
    assert overall_status([]) == 'STALE'
    assert overall_status(['OK']) == 'OK'
    assert overall_status(['OK', 'WARN']) == 'WARN'
    assert overall_status(['OK', 'STALE']) == 'STALE'
    assert overall_status(['STALE', 'FAULT']) == 'FAULT'
    assert overall_status(['WARN', 'FAULT', 'STALE']) == 'FAULT'


def test_diagnostics_callback_updates_state():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray

    rclpy.init()

    node = DiagnosticsCliNode('/test_diagnostics')

    try:
        message = DiagnosticArray()
        status = DiagnosticStatus()
        status.name = 'board.temperature'
        status.level = DiagnosticStatus.OK
        status.message = 'Normal Temperature'
        status.values = [
            KeyValue(key='value', value='42.0'),
            KeyValue(key='unit', value='C'),
        ]
        message.status = [status]

        before = time.monotonic()
        node.diagnostics_callback(message)

        assert len(node.latest) == 1
        assert node.latest[0].name == 'board.temperature'
        assert node.latest[0].level == DiagnosticStatus.OK
        assert node.last_received_monotonic is not None
        assert node.last_received_monotonic >= before
        assert node.data_age() is not None
        assert node.data_age() >= 0.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_diagnostics_callback_merges_entries_from_multiple_publishers():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray

    rclpy.init()
    node = DiagnosticsCliNode('/test_diagnostics')

    try:
        can_message = DiagnosticArray()
        can_status = DiagnosticStatus()
        can_status.name = 'can.bus'
        can_status.level = DiagnosticStatus.ERROR
        can_message.status = [can_status]

        imu_message = DiagnosticArray()
        imu_status = DiagnosticStatus()
        imu_status.name = 'imu.orientation'
        imu_status.level = DiagnosticStatus.OK
        imu_message.status = [imu_status]

        node.diagnostics_callback(can_message)
        node.diagnostics_callback(imu_message)

        statuses = {status.name: status.level for status in node.latest}
        assert statuses == {
            'can.bus': DiagnosticStatus.ERROR,
            'imu.orientation': DiagnosticStatus.OK,
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_run_snapshot_collects_queued_publishers(monkeypatch, capsys):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from waybionic_diagnostics import cli

    rclpy.init()
    node = DiagnosticsCliNode('/test_diagnostics')
    can_message = DiagnosticArray()
    can_status = DiagnosticStatus()
    can_status.name = 'can.bus'
    can_status.level = DiagnosticStatus.ERROR
    can_message.status = [can_status]
    imu_message = DiagnosticArray()
    imu_status = DiagnosticStatus()
    imu_status.name = 'imu.orientation'
    imu_status.level = DiagnosticStatus.OK
    imu_message.status = [imu_status]

    try:
        node.diagnostics_callback(imu_message)
        node.diagnostics_callback(can_message)
        monkeypatch.setattr(cli, 'SNAPSHOT_COLLECTION_SECONDS', 0.0)
        cli.run_snapshot(node)

        output = capsys.readouterr().out
        assert 'can.bus' in output
        assert 'Overall: FAULT' in output
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_diagnostics_callback_uses_header_stamp_for_sample_age(monkeypatch, capsys):
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray

    rclpy.init()
    node = DiagnosticsCliNode('/test_diagnostics')

    try:
        message = DiagnosticArray()
        message.header.stamp.sec = 60
        status = DiagnosticStatus()
        status.name = 'board.temperature'
        status.level = DiagnosticStatus.OK
        message.status = [status]

        monkeypatch.setattr('waybionic_diagnostics.cli.time.time', lambda: 70.0)
        node.diagnostics_callback(message)

        assert node.status_age(status) == 10.0
        print_snapshot(node)
        assert 'STALE' in capsys.readouterr().out
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_main_forwards_ros_args_to_rclpy_init(monkeypatch):
    from waybionic_diagnostics import cli

    captured = {}

    class FakeNode:
        def __init__(self, topic):
            captured['topic'] = topic

        def destroy_node(self):
            captured['destroyed'] = True

    monkeypatch.setattr(cli, 'DiagnosticsCliNode', FakeNode)
    monkeypatch.setattr(cli, 'run_snapshot', lambda node: captured.setdefault('snapshot', node))
    monkeypatch.setattr(cli.rclpy, 'init', lambda args=None: captured.setdefault('init_args', args))
    monkeypatch.setattr(cli.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(cli.rclpy, 'shutdown', lambda: captured.setdefault('shutdown', True))

    cli.main(['--topic', '/diagnostics_custom', '--ros-args', '-r', '__node:=renamed'])

    assert captured['topic'] == '/diagnostics_custom'
    assert captured['init_args'] == ['--ros-args', '-r', '__node:=renamed']
    assert 'snapshot' in captured
    assert captured['destroyed'] is True
    assert 'shutdown' not in captured
