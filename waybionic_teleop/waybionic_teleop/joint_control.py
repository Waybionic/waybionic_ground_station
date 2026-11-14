"""
The arm's joint enable states, as the drive node publishes them on ``arm/joint_control``.

The topic carries a latched DiagnosticArray with one row per joint, ``joint_1`` to
``joint_5`` and then ``tool_grip``. A row's name is the joint, its message says what it is
doing, and its values are ``state`` (one of ENABLED, DISABLED or NO_DRIVE), ``drives`` (the
drives that move it, comma separated) and ``angle`` (radians, where the joint was last seen).
The drive node serves ``arm/<joint>/set_enabled`` (std_srvs/SetBool) for the joints that have
a drive. The RViz joint control panel and the Xbox teleop node both read this topic.
"""

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

TOPIC = 'arm/joint_control'
ENABLED, DISABLED, NO_DRIVE = 'enabled', 'disabled', 'no_drive'


def joint_status(joint, state, drives, angle, message):
    """Return the row for one joint."""
    return DiagnosticStatus(
        name=joint, level=DiagnosticStatus.OK if state == ENABLED else DiagnosticStatus.WARN,
        message=message, values=[KeyValue(key='state', value=state),
                                 KeyValue(key='drives', value=','.join(drives)),
                                 KeyValue(key='angle', value=repr(float(angle)))])


def disabled_joints(message):
    """Return the joints a joint control message says are switched off."""
    return {row.name for row in message.status
            if any(item.key == 'state' and item.value == DISABLED for item in row.values)}
