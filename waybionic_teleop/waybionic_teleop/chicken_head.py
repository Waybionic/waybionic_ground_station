"""
Chicken head mode's state, as the Xbox teleop node publishes it on ``arm/chicken_head``.

The topic carries a latched DiagnosticArray with one row, ``chicken_head``. Its message says what
is happening or why the mode cannot start, and its ``state`` value is one of ``off``,
``approach`` (the tip is moving to the head spot), ``hold`` (the tip is held while the sticks tilt
and roll the tool), ``blocked`` (the way to the head spot is blocked) or ``unavailable``. The node
serves ``arm/chicken_head/set_enabled`` (std_srvs/SetBool) to switch the mode on or off, and
answers with why it cannot start, for example before Start has been pressed. The RViz chicken head
panel reads the topic and calls the service.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

TOPIC = 'arm/chicken_head'
SERVICE = 'arm/chicken_head/set_enabled'
ROW = 'chicken_head'
OFF, APPROACH, HOLD, BLOCKED, UNAVAILABLE = 'off', 'approach', 'hold', 'blocked', 'unavailable'


def head_message(state, message):
    """Return the latched message for a state and what to tell the operator."""
    level = DiagnosticStatus.OK if state in (OFF, APPROACH, HOLD) else DiagnosticStatus.WARN
    return DiagnosticArray(status=[DiagnosticStatus(
        name=ROW, level=level, message=message, values=[KeyValue(key='state', value=state)])])
