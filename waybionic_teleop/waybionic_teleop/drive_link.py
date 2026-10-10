"""
Bring one MKS drive up over CAN and keep track of whether it is still answering.

Start-up sends the setup frames one at a time and waits for each drive to confirm it: 82h bus
FOC mode, 8Ch replies and "move complete" reports on, F3h shaft enabled, 98h heartbeat stop.
Then one 31h encoder read gives the drive's position, and the drive is ready. A ready drive is
read every tick. A drive that leaves a request unanswered for the timeout starts again from
82h, because it may have lost power and with it its settings. A drive that refuses a frame is
retried after a pause. Set zero (92h) is followed by a fresh encoder read.
"""

from waybionic_teleop import mks_can

SETUP = (mks_can.SET_MODE, mks_can.SET_RESPONSE, mks_can.ENABLE, mks_can.SET_HEARTBEAT)
RETRY_S = 1.0


class DriveLink:
    """Start-up and reply tracking for one drive; the caller sends what poll() returns."""

    def __init__(self, can_id, heartbeat_ms, timeout):
        self.can_id = can_id
        self.heartbeat_ms = heartbeat_ms
        self.timeout = timeout
        self.phase = 'setup'
        self.step = 0
        self.code = None
        self.since = None
        self.retry_at = None
        self.count = None
        self.run_status = ''
        self.problem = ''
        self.dropouts = 0
        # True once a requested 92h is confirmed, False if it was refused or went unanswered.
        self.zeroed = None

    @property
    def ready(self):
        return self.phase == 'ready'

    def describe(self):
        if self.ready:
            return self.run_status or 'ready'
        if self.phase == 'failed':
            return f'{self.problem}, retrying'
        step = {'setup': f'setup {SETUP[self.step]:02X}h', 'encoder': 'encoder read',
                'zero': 'set zero'}[self.phase]
        return f'{self.problem}; {step}' if self.problem else step

    def poll(self, now):
        """Return the frame to send this tick, or None."""
        if self.since is not None and now - self.since > self.timeout:
            self.restart(f'no reply to {self.code:02X}h')
        if self.phase == 'failed':
            if now < self.retry_at:
                return None
            self.restart(self.problem)
        # Setup waits for each confirmation; a ready drive is read every tick.
        if self.since is not None and not self.ready:
            return None
        code = self.request()
        if self.since is None:
            self.code, self.since = code, now
        return self.frame(code)

    def request(self):
        if self.phase == 'setup':
            return SETUP[self.step]
        if self.phase == 'zero':
            return mks_can.SET_ZERO
        return mks_can.READ_ENCODER

    def frame(self, code):
        if code == mks_can.SET_MODE:
            return mks_can.set_mode(self.can_id)
        if code == mks_can.SET_RESPONSE:
            return mks_can.set_response(self.can_id, respond=True, active=True)
        if code == mks_can.ENABLE:
            return mks_can.enable(self.can_id)
        if code == mks_can.SET_HEARTBEAT:
            return mks_can.set_heartbeat(self.can_id, self.heartbeat_ms)
        if code == mks_can.SET_ZERO:
            return mks_can.set_zero(self.can_id)
        return mks_can.read_encoder(self.can_id)

    def on_reply(self, code, arguments, now):
        """Handle one parsed reply from this drive."""
        if code == mks_can.READ_ENCODER:
            self.count = mks_can.encoder_value(arguments)
            if code == self.code:
                self.since = None
                if self.phase == 'encoder':
                    self.phase, self.problem = 'ready', ''
            return
        if code == mks_can.ABSOLUTE_AXIS and arguments:
            self.run_status = mks_can.RUN_STATUS.get(arguments[0], f'status {arguments[0]}')
            return
        if code != self.code or self.phase not in ('setup', 'zero'):
            # A late or repeated confirmation of a frame this drive already moved past.
            return
        if arguments[:1] != b'\x01':
            if self.phase == 'zero':
                self.zeroed = False
            self.phase, self.problem = 'failed', f'{code:02X}h refused'
            self.since, self.retry_at = None, now + RETRY_S
            return
        self.since = None
        if self.phase == 'zero':
            self.zeroed = True
            self.phase, self.count = 'encoder', None
            return
        self.step += 1
        if self.step == len(SETUP):
            self.phase = 'encoder'

    def zero(self):
        """Set the current position as encoder 0 (92h); only a ready drive can be zeroed."""
        if not self.ready:
            raise ValueError(f'CAN ID {self.can_id} is not ready')
        self.phase, self.code, self.since, self.zeroed = 'zero', None, None, None

    def restart(self, problem):
        if self.ready:
            self.dropouts += 1
        if self.phase == 'zero':
            self.zeroed = False
        self.phase, self.step, self.code, self.since = 'setup', 0, None, None
        self.count, self.run_status, self.problem = None, '', problem
