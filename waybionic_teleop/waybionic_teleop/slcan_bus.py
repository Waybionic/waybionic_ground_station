"""
Talk to the WayBionic carrier (UNO R4 WiFi and a CAN transceiver) over its USB serial link.

The carrier speaks a subset of the Lawicel SLCAN protocol (waybionic_can/arduino). python-can's
slcan interface drains the port after every frame and reads one byte at a time, which costs
milliseconds per frame over USB. This writes a control tick's frames in one go and parses
whatever has arrived in one read. If the USB link drops, it keeps reopening the port, and a
carrier that resets (or sits closed after the port opens) is opened again once it goes quiet.
"""

from collections import deque
import time

import serial

from waybionic_teleop import mks_can

BITRATE_CODES = {125000: b'4', 250000: b'5', 500000: b'6', 1000000: b'8'}
DEFAULT_BAUDRATE = 1000000
REOPEN_EVERY_S = 1.0
# The carrier sends its status ten times a second while open, so this much quiet means it is
# closed: it reset, or it was still booting when the port opened.
SILENCE_S = 1.0
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
            lambda: serial.serial_for_url(port, baudrate=baudrate, timeout=0))
        self.frames = self.bits = self.errors = self.reopens = 0
        self.problem = ''
        self.outgoing, self.incoming, self.replies = bytearray(), bytearray(), deque()
        self.heard = self.next_try = 0.0
        # A missing adapter at start-up is a configuration error, so let it raise.
        self.serial = self.open_port()
        self._start()

    @property
    def connected(self):
        return self.serial is not None

    def send(self, can_id, data):
        """Queue a frame for the next write; return False while the carrier is unplugged."""
        data = bytes(data)
        if not 0 <= can_id <= 0x7FF or len(data) > 8:
            raise ValueError(f'cannot send {can_id:X}#{data.hex()} as a standard frame')
        if self.serial is None:
            return False
        self.outgoing += b't%03X%d%s\r' % (can_id, len(data), data.hex().upper().encode())
        self._count(data)
        return True

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
            self._flush()
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def _start(self):
        # Close in case it was left open, set the bit rate, open. Each answers \r, or \a.
        self.outgoing[:0] = b'C\rS' + BITRATE_CODES[self.bitrate] + b'\rO\r'
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
        self.heard = time.monotonic()
        self.incoming += chunk
        start = 0
        for end, byte in enumerate(self.incoming):
            if byte not in (CR, BELL):
                continue
            line = bytes(self.incoming[start:end])
            start = end + 1
            if byte == BELL:
                # The carrier refused a command, such as a frame it could not queue.
                self.errors += 1
            elif (frame := parse_line(line)) is not None:
                self.replies.append(frame)
                self._count(frame[1])
        del self.incoming[:start]
        if len(self.incoming) > 64:
            # No line is that long, so this is noise, such as a baud rate mismatch.
            self.errors += 1
            self.incoming.clear()

    def _lost(self, error):
        self.problem = f'{self.port}: {error}'
        self.errors += 1
        try:
            self.serial.close()
        except (serial.SerialException, OSError):
            pass
        self.serial = None
        self.outgoing.clear()
        self.incoming.clear()
        self.replies.clear()
        self.next_try = time.monotonic() + REOPEN_EVERY_S

    def _count(self, data):
        self.frames += 1
        self.bits += mks_can.frame_bits(data)
