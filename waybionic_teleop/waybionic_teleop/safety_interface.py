"""Stable ROS interface names shared by simulated and carrier-board safety sources."""

EMERGENCY_STOP_TOPIC = '/waybionic/safety/emergency_stop'
EMERGENCY_STOP_SET_SERVICE = '/waybionic/safety/emergency_stop/set'
MOTOR_SUPPLY_VOLTAGE_TOPIC = '/waybionic/power/motor_supply_voltage'
LAST_REPORT_AGE_TOPIC = '/waybionic/safety/last_report_age'

EMERGENCY_STOP_SIGNAL = 'safety.emergency_stop'
MOTOR_SUPPLY_VOLTAGE_SIGNAL = 'power.motor_supply_voltage'
LAST_REPORT_AGE_SIGNAL = 'safety.last_report_age'
