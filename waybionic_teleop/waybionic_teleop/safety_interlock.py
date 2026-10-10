"""Emergency-stop status policy, independent of ROS."""


class EmergencyStopInterlock:
    def __init__(self, timeout, required=False):
        self.timeout = float(timeout)
        self.required = bool(required)
        self.pressed = False
        self.last_report_at = None

    def update(self, pressed, now):
        self.pressed = bool(pressed)
        self.last_report_at = now

    def is_pressed(self, now):
        if self.pressed:
            return True
        return self.required and (
            self.last_report_at is None or now - self.last_report_at > self.timeout
        )

    def block_reason(self, now):
        if not self.is_pressed(now):
            return ''
        if self.pressed:
            return 'Emergency stop pressed; release it and press Start to enable'
        return 'Emergency stop status missing or stale; treated as pressed'

    def display_state(self, now):
        if self.pressed:
            return 'PRESSED'
        if self.last_report_at is None:
            return 'PRESSED-FAILSAFE' if self.required else 'UNKNOWN'
        if now - self.last_report_at > self.timeout:
            return 'PRESSED-FAILSAFE' if self.required else 'STALE'
        return 'RELEASED'
