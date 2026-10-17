"""
Talk to the WayBionic carrier (UNO R4 WiFi and a CAN transceiver) over its USB serial link.

The carrier speaks a subset of the Lawicel SLCAN protocol (waybionic_can/arduino). python-can's
slcan interface drains the port after every frame and reads one byte at a time, which costs
milliseconds per frame over USB. This writes a control tick's frames in one go and parses
whatever has arrived in one read. If the USB link drops, it keeps reopening the port, and a
carrier that resets (or sits closed after the port opens) is opened again once it goes quiet.

The carrier says nothing about a frame it puts on the bus, and answers BELL to one it refuses,
such as a frame that arrives while its CAN controller is still busy. Each frame is therefore
followed by an empty line, which the carrier answers with CR, so every answer can be matched to
the line it belongs to. take_refused() hands back the refused frames to be sent again. No more
lines go unanswered at once than the carrier can queue; a frame beyond that is refused here.
"""

from collections import deque
import time

import serial

from waybionic_teleop import mks_can

BITRATE_CODES = {125000: b'4', 250000: b'5', 500000: b'6', 1000000: b'8'}
DEFAULT_BAUDRATE = 1000000
REOPEN_EVERY_S = 1.0
# A carrier that stops reading its port fails the write, so the link is reopened rather than
# the control loop blocking.
WRITE_TIMEOUT_S = 0.1
# The carrier sends its status ten times a second while open, so this much quiet means it is
# closed: it reset, or it was still booting when the port opened.
SILENCE_S = 1.0
# The carrier queues 16 frames and holds the answers to the lines after a queued frame until it
# leaves the queue, 32 answers at most. A frame it refuses for room needs two (its BELL and the
# CR after it), so with more lines unanswered than it can queue, a stalled bus could overflow
# them and later answers would be pinned on the wrong lines. Keeping to the queue's size means it
# never refuses a frame for room.
CARRIER_QUEUE = 16
# The carrier answers within milliseconds, and drops a frame it cannot send after 10 ms, so no
# answer for this long with no room left means it stopped answering its lines.
ANSWER_TIMEOUT_S = 0.5
BELL = 0x07
CR = 0x0D


def parse_line(line):
    """Return (can_id, data) for a standard data frame line such as b't0012313', else None."""
    if len(line) < 5 or line[:1] != b't':
        return None
    try:
        can_id, length = int(line[1:4], 16), int(line[4:5])
        data = bytes.fromhex(line[5:].decode('ascii'))
    except (ValueError, UnicodeDecodeError):
        return None
    return (can_id, data) if len(data) == length <= 8 else None


class SlcanBus:
    """Send and receive standard 11-bit frames through the carrier, without blocking."""

    def __init__(self, port, bitrate, baudrate=DEFAULT_BAUDRATE, open_port=None):
        if bitrate not in BITRATE_CODES:
            raise ValueError(f'the carrier runs CAN at {sorted(BITRATE_CODES)} bit/s')
        self.port, self.bitrate = port, bitrate
        self.open_port = open_port or (
            lambda: serial.serial_for_url(port, baudrate=baudrate, timeout=0,
                                          write_timeout=WRITE_TIMEOUT_S))
        self.frames = self.bits = self.errors = self.refused = self.reopens = 0
        self.problem = ''
        self.outgoing, self.incoming, self.replies = bytearray(), bytearray(), deque()
        # Lines queued, then written but unanswered: a frame's (can_id, data) or a command.
        self.queued, self.pending = [], deque()
        self.refusals, self.bell = [], False
        # When a frame was first refused here for room, until the carrier answers a line.
        self.full_since = None
        self.heard = self.next_try = 0.0
        # A missing adapter at start-up is a configuration error, so let it raise.
        self.serial = self.open_port()
        self._start()

    @property
    def connected(self):
        return self.serial is not None

    @property
    def unanswered(self):
        """Return how many lines, queued or written, the carrier has not answered yet."""
        return len(self.queued) + len(self.pending)

    def send(self, can_id, data):
        """Queue a frame for the next write; return False while the carrier is unplugged."""
        data = bytes(data)
        if not 0 <= can_id <= 0x7FF or len(data) > 8:
            raise ValueError(f'cannot send {can_id:X}#{data.hex()} as a standard frame')
        if self.serial is None:
            return False
        if self.unanswered >= CARRIER_QUEUE:
            now = time.monotonic()
            if self.full_since is None:
                self.full_since = now
            elif now - self.full_since > ANSWER_TIMEOUT_S:
                # Start again rather than pin a later answer, such as a refusal, on the wrong line.
                self._lost('the carrier stopped answering its lines')
                return False
            # Not counted as refused: a burst such as the drives' setup simply waits a tick.
            self.refusals.append((can_id, data))
            return True
        self.outgoing += b't%03X%d%s\r\r' % (can_id, len(data), data.hex().upper().encode())
        self.queued.append((can_id, data))
        self._count(data)
        return True

    def take_refused(self):
        """Return the frames refused since the last call, here or by the carrier, oldest first."""
        refused, self.refusals = self.refusals, []
        return refused

    def receive(self, timeout=0.0):
        """Write what is queued, then return the next (can_id, data) frame, or None."""
        self._flush()
        if not self.replies:
            self._read()
        return self.replies.popleft() if self.replies else None

    def step(self, dt):
        """Reopen a lost port, or reopen the bridge if the carrier has gone quiet."""
        now = time.monotonic()
        if self.serial is not None:
            if now - self.heard > SILENCE_S:
                self._start()
            return
        if now < self.next_try:
            return
        self.next_try = now + REOPEN_EVERY_S
        try:
            self.serial = self.open_port()
        except (serial.SerialException, OSError, ValueError) as error:
            self.problem = f'{self.port}: {error}'
            return
        self.reopens += 1
        self.problem = ''
        self._start()

    def shutdown(self):
        if self.serial is not None:
            self.outgoing += b'C\r'
            self.queued.append(b'C')
            self._flush()
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def _start(self):
        # Close in case it was left open, set the bit rate, open. Each answers \r, or \a.
        commands = [b'C', b'S' + BITRATE_CODES[self.bitrate], b'O']
        self.outgoing[:0] = b''.join(command + b'\r' for command in commands)
        self.queued[:0] = commands
        # A carrier that reset will never answer the lines written before.
        self.pending.clear()
        self.bell, self.full_since = False, None
        self.heard = time.monotonic()
        self._flush()

    def _flush(self):
        if self.serial is None or not self.outgoing:
            return
        try:
            self.serial.write(self.outgoing)
        except (serial.SerialException, OSError) as error:
            self._lost(error)
            return
        self.outgoing.clear()
        self.pending.extend(self.queued)
        self.queued.clear()

    def _read(self):
        if self.serial is None:
            return
        try:
            chunk = self.serial.read(self.serial.in_waiting or 1)
        except (serial.SerialException, OSError) as error:
            self._lost(error)
            return
        if not chunk:
            return
        self.incoming += chunk
        start = 0
        for end, byte in enumerate(self.incoming):
            if byte not in (CR, BELL):
                continue
            line = bytes(self.incoming[start:end])
            start = end + 1
            if byte == BELL:
                self._refused()
            elif not line:
                self._answered()
            elif (frame := parse_line(line)) is not None:
                # A closed carrier still answers lines, but passes on no frames.
                self.heard = time.monotonic()
                self.replies.append(frame)
                self._count(frame[1])
        del self.incoming[:start]
        if len(self.incoming) > 64:
            # No line is that long, so this is noise, such as a baud rate mismatch.
            self.errors += 1
            self.incoming.clear()

    def _refused(self):
        # A frame's BELL comes before the CR of its empty line; a command answers one or the other.
        self.full_since = None
        if self.pending and isinstance(self.pending[0], tuple):
            self.bell = True
            return
        if self.pending and self.pending.popleft() == b'O':
            # The carrier keeps the bit rate it first opened at until it resets.
            self.problem = f'the carrier would not open at {self.bitrate} bit/s; replug it'
        self.errors += 1

    def _answered(self):
        self.full_since = None
        if not self.pending:
            return
        line = self.pending.popleft()
        if line == b'O':
            self.problem = ''
        if self.bell and isinstance(line, tuple):
            self.refused += 1
            self.refusals.append(line)
        self.bell = False

    def _lost(self, error):
        self.problem = f'{self.port}: {error}'
        self.errors += 1
        try:
            self.serial.close()
        except (serial.SerialException, OSError):
            pass
        self.serial = None
        self.outgoing.clear()
        self.queued.clear()
        self.pending.clear()
        self.bell, self.full_since = False, None
        self.incoming.clear()
        self.replies.clear()
        self.next_try = time.monotonic() + REOPEN_EVERY_S

    def _count(self, data):
        self.frames += 1
        self.bits += mks_can.frame_bits(data)
