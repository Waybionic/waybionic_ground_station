"""
Demo mode's state, as the Xbox teleop node publishes it on ``arm/demo``.

The topic carries a latched DiagnosticArray with one row, ``demo``. Its message says what is
happening or why the mode stopped by itself, and its ``state`` value is one of ``off``,
``running`` (the arm is moving slowly around the pose it started from), ``stopping`` (it is
returning to that pose) or ``unavailable``. The row is a warning when the mode cannot be used or
stopped because something was in the way. The node serves ``arm/demo/set_enabled``
(std_srvs/SetBool) to switch the mode on or off, and answers with why it cannot start, for example
before Start has been pressed. The RViz motion modes panel reads the topic and calls the service.
"""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

TOPIC = 'arm/demo'
SERVICE = 'arm/demo/set_enabled'
ROW = 'demo'
OFF, RUNNING, STOPPING, UNAVAILABLE = 'off', 'running', 'stopping', 'unavailable'


def demo_message(state, message):
    """Return the latched message for a state and what to tell the operator."""
    # An off mode only has something to say when it stopped itself.
    warning = state == UNAVAILABLE or (state == OFF and bool(message))
    level = DiagnosticStatus.WARN if warning else DiagnosticStatus.OK
    return DiagnosticArray(status=[DiagnosticStatus(
        name=ROW, level=level, message=message, values=[KeyValue(key='state', value=state)])])
