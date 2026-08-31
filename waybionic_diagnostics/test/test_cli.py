import time

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from waybionic_diagnostics.cli import (
    DiagnosticsCliNode,
    extract_value_and_unit,
    overall_status,
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
