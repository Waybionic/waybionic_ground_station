"""Carry MKS frames on a python-can interface, with the same calls as the simulated bus."""

from collections import deque
import time

import can

from waybionic_teleop import mks_can


class _Reader(can.BufferedReader):
    """Queue received frames, counting receive errors instead of ending the reading thread."""

    def __init__(self, bus):
        super().__init__()
        self.bus = bus

    def on_error(self, exc):
        self.bus.errors += 1
        # Back off, so an adapter that has gone away does not spin this thread.
        time.sleep(0.01)


class CanBus:
    """Send and receive standard 11-bit frames on a python-can bus without blocking."""

    def __init__(self, interface, channel, bitrate, **options):
        self.bus = can.Bus(interface=interface, channel=channel, bitrate=bitrate, **options)
        self.bitrate = bitrate
        # MKS replies reuse the command's CAN ID, so a bus that hands frames back to their
        # sender (UDP multicast always does) would make commands look like replies.
        echoes = interface == 'udp_multicast' or options.get('receive_own_messages')
        self.echoes = deque(maxlen=256) if echoes else None
        self.frames = 0
        self.bits = 0
        self.errors = 0
        # A thread reads the interface as lines arrive. An slcan adapter also answers every
        # frame sent with a line that recv() returns as None, so reading until the first None
        # would leave most replies behind.
        self.reader = _Reader(self)
        self.notifier = can.Notifier(self.bus, [self.reader], timeout=0.1)

    def send(self, can_id, data):
        """Transmit a frame; return False if the interface refused it."""
        self._count(data)
        message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
        except can.CanError:
            self.errors += 1
            return False
        if self.echoes is not None:
            self.echoes.append((can_id, bytes(data)))
        return True

    def receive(self, timeout=0.0):
        """Return the next (can_id, data) frame from another node, or None."""
        while True:
            message = self.reader.get_message(timeout)
            timeout = 0.0
            if message is None:
                return None
            if message.is_error_frame:
                self.errors += 1
                continue
            if message.is_extended_id or message.is_remote_frame:
                continue
            frame = (message.arbitration_id, bytes(message.data))
            if self.echoes and frame in self.echoes:
                while self.echoes.popleft() != frame:
                    pass
                continue
            self._count(frame[1])
            return frame

    def step(self, dt):
        """Real drives move on their own."""

    def shutdown(self):
        self.notifier.stop()
        self.bus.shutdown()

    def _count(self, data):
        self.frames += 1
        self.bits += mks_can.frame_bits(data)
