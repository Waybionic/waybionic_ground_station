"""Carry MKS frames on a python-can interface, with the same calls as the simulated bus."""

from collections import deque

import can

from waybionic_teleop import mks_can


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

    def send(self, can_id, data):
        self._count(data)
        message = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
        except can.CanError:
            self.errors += 1
            return
        if self.echoes is not None:
            self.echoes.append((can_id, bytes(data)))

    def receive(self, timeout=0.0):
        """Return the next (can_id, data) frame from another node, or None."""
        while True:
            try:
                message = self.bus.recv(timeout)
            except can.CanError:
                self.errors += 1
                return None
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
        self.bus.shutdown()

    def _count(self, data):
        self.frames += 1
        self.bits += mks_can.frame_bits(data)
