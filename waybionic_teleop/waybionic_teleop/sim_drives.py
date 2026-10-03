"""Simulated MKS SERVO drives on an in-process CAN bus, standing in for the real drives."""

from collections import deque
import math

from waybionic_teleop import mks_can


def ramp_rpm_per_s(acc):
    """Return the manual's acceleration: 1 rpm every (256 - acc) * 50 us; None when acc is 0."""
    return None if acc == 0 else 1.0 / ((256 - acc) * 50e-6)


class SimulatedServo:
    """Answer bus commands and move like an MKS SERVO42D/57D in SR_vFOC mode."""

    def __init__(self, can_id):
        self.can_id = can_id
        self.mode = None
        self.enabled = False
        self.respond = True
        self.active = True
        self.heartbeat_ms = 0
        self.quiet_ms = 0.0
        self.heartbeat_stops = 0
        self.axis = 0.0
        self.rpm = 0.0
        self.target = None
        self.speed = 0
        self.acc = 0

    def receive(self, data):
        """Handle one command and return the reply frames."""
        code, arguments = mks_can.parse(self.can_id, data)
        self.quiet_ms = 0.0
        if code == mks_can.READ_ENCODER and not arguments:
            value = round(self.axis).to_bytes(6, 'big', signed=True)
            return [mks_can.frame(self.can_id, code, value)]
        if code == mks_can.SET_MODE and len(arguments) == 1:
            self.mode, status = arguments[0], 1
        elif code == mks_can.SET_RESPONSE and len(arguments) == 2:
            self.respond, self.active, status = bool(arguments[0]), bool(arguments[1]), 1
        elif code == mks_can.SET_HEARTBEAT and len(arguments) == 4:
            self.heartbeat_ms, status = int.from_bytes(arguments, 'big'), 1
        elif code == mks_can.ENABLE and len(arguments) == 1:
            self.enabled, status = bool(arguments[0]), 1
            if not self.enabled:
                self.target, self.rpm = None, 0.0
        elif code == mks_can.ABSOLUTE_AXIS and len(arguments) == 6:
            status = self.start_move(arguments)
        else:
            return []
        return [mks_can.frame(self.can_id, code, [status])] if self.respond else []

    def start_move(self, arguments):
        if not self.enabled or self.mode != mks_can.MODE_SR_VFOC:
            return 0
        self.speed = min(int.from_bytes(arguments[:2], 'big'), mks_can.MAX_SPEED_RPM)
        self.acc = arguments[2]
        # Speed 0 is the manual's stop command: slow down with acc, or stop at once if acc is 0.
        self.target = int.from_bytes(arguments[3:], 'big', signed=True) if self.speed else None
        return 1

    def step(self, dt):
        """Advance the motor by dt seconds and return any frames it sends unprompted."""
        self.quiet_ms += dt * 1000.0
        if self.heartbeat_ms and self.quiet_ms > self.heartbeat_ms and (
                self.target is not None or self.rpm):
            self.target, self.rpm = None, 0.0
            self.heartbeat_stops += 1
        ramp = ramp_rpm_per_s(self.acc)
        goal = 0.0
        if self.target is not None:
            remaining = (self.target - self.axis) / mks_can.COUNTS_PER_REV
            # Fastest speed that can still stop at the target with this acceleration.
            reachable = math.inf if ramp is None else math.sqrt(120.0 * ramp * abs(remaining))
            goal = math.copysign(min(self.speed, reachable), remaining)
        if ramp is None:
            self.rpm = goal
        else:
            change = ramp * dt
            self.rpm += min(max(goal - self.rpm, -change), change)
        move = self.rpm / 60.0 * dt * mks_can.COUNTS_PER_REV
        if self.target is None:
            self.axis += move
            return []
        remaining = self.target - self.axis
        if abs(remaining) < 0.5 or (move * remaining > 0 and abs(move) >= abs(remaining)):
            self.axis, self.rpm, self.target = float(self.target), 0.0, None
            if self.respond and self.active:
                return [mks_can.frame(self.can_id, mks_can.ABSOLUTE_AXIS, [2])]
            return []
        self.axis += move
        return []


class SimulatedBus:
    """Deliver frames to simulated drives and queue their replies like a CAN receive buffer."""

    def __init__(self, drives, bitrate):
        self.drives = {drive.can_id: drive for drive in drives}
        self.bitrate = bitrate
        self.replies = deque()
        self.frames = 0
        self.bits = 0
        self.errors = 0

    def send(self, can_id, data):
        self._count(data)
        drive = self.drives.get(can_id)
        if drive is None:
            self.errors += 1
            return
        try:
            replies = drive.receive(data)
        except ValueError:
            self.errors += 1
            return
        self._queue(can_id, replies)

    def receive(self):
        """Return the next (can_id, data) reply, or None when the buffer is empty."""
        return self.replies.popleft() if self.replies else None

    def step(self, dt):
        for can_id, drive in self.drives.items():
            self._queue(can_id, drive.step(dt))

    def _queue(self, can_id, replies):
        for reply in replies:
            self._count(reply)
            self.replies.append((can_id, reply))

    def _count(self, data):
        # Worst-case length of a standard data frame including stuff bits and interframe space.
        self.frames += 1
        self.bits += 47 + 8 * len(data) + (34 + 8 * len(data) - 1) // 4
