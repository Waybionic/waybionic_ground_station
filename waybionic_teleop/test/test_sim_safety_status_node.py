from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import rclpy
from std_srvs.srv import SetBool

from waybionic_teleop.safety_interface import (
    EMERGENCY_STOP_SIGNAL,
    LAST_REPORT_AGE_SIGNAL,
    MOTOR_SUPPLY_VOLTAGE_SIGNAL,
)
from waybionic_teleop.sim_safety_status_node import SimulatedSafetyStatus


@pytest.fixture
def safety_node(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    node = SimulatedSafetyStatus(context=context)
    diagnostics = []
    estop = []
    voltage = []
    report_age = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', diagnostics.append)
    monkeypatch.setattr(node.emergency_stop_publisher, 'publish', estop.append)
    monkeypatch.setattr(node.voltage_publisher, 'publish', voltage.append)
    monkeypatch.setattr(node.report_age_publisher, 'publish', report_age.append)
    yield node, diagnostics, estop, voltage, report_age
    node.destroy_node()
    rclpy.try_shutdown(context=context)


def test_publishes_safety_power_and_report_age_contract(safety_node):
    node, diagnostics, estop, voltage, report_age = safety_node
    node.publish_report()
    rows = {row.name: row for row in diagnostics[-1].status}

    assert set(rows) == {
        EMERGENCY_STOP_SIGNAL,
        MOTOR_SUPPLY_VOLTAGE_SIGNAL,
        LAST_REPORT_AGE_SIGNAL,
    }
    assert rows[EMERGENCY_STOP_SIGNAL].level == DiagnosticStatus.OK
    assert rows[EMERGENCY_STOP_SIGNAL].values[0].value == 'released'
    assert rows[EMERGENCY_STOP_SIGNAL].values[1].value == ''
    assert rows[MOTOR_SUPPLY_VOLTAGE_SIGNAL].values[0].value == '24.0'
    assert rows[MOTOR_SUPPLY_VOLTAGE_SIGNAL].values[1].value == 'V'
    assert rows[LAST_REPORT_AGE_SIGNAL].values[1].value == 's'
    assert estop[-1].data is False
    assert voltage[-1].data == pytest.approx(24.0)
    assert report_age[-1].data >= 0.0


def test_set_bool_toggles_estop_and_emits_fault_diagnostics(safety_node):
    node, diagnostics, estop, _, _ = safety_node
    response = node.set_emergency_stop(SetBool.Request(data=True), SetBool.Response())
    rows = {row.name: row for row in diagnostics[-1].status}

    assert response.success
    assert response.message == 'Emergency stop pressed'
    assert node.emergency_stop_pressed
    assert estop[-1].data is True
    assert rows[EMERGENCY_STOP_SIGNAL].level == DiagnosticStatus.ERROR
    assert rows[EMERGENCY_STOP_SIGNAL].values[0].value == 'pressed'
    assert rows[EMERGENCY_STOP_SIGNAL].message == 'Emergency stop is pressed'

    response = node.set_emergency_stop(SetBool.Request(data=False), SetBool.Response())
    rows = {row.name: row for row in diagnostics[-1].status}
    assert response.success
    assert response.message == 'Emergency stop released'
    assert rows[EMERGENCY_STOP_SIGNAL].level == DiagnosticStatus.OK
    assert rows[EMERGENCY_STOP_SIGNAL].values[0].value == 'released'


def test_nonpositive_voltage_is_rejected():
    context = rclpy.Context()
    rclpy.init(context=context)
    try:
        with pytest.raises(ValueError, match='motor_supply_voltage'):
            SimulatedSafetyStatus(
                context=context,
                parameter_overrides=[
                    rclpy.parameter.Parameter('motor_supply_voltage', value=0.0)
                ],
            )
    finally:
        rclpy.try_shutdown(context=context)
