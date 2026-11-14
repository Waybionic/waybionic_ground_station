"""Record every frame the drive node sends, receives or gets refused, for bench analysis."""

import time


def parse_line(line):
    """Return (host time, kind, can_id, data) for a line that RecordingBus wrote."""
    stamp, kind, frame = line.split()
    can_id, data = frame.split('#')
    return float(stamp), kind, int(can_id, 16), bytes.fromhex(data)


class RecordingBus:
    """
    Pass frames through to a bus, writing one line per frame to a file.

    Each line is the host's wall-clock time, the kind and the frame, such as
    ``1793438841.369542 tx 001#F5008200FFE3A09A``. Kinds: tx (queued to the bus), unsent (the
    bus was unplugged), rx (a frame the bus received) and refused (a frame the carrier or the
    bus refused after send() returned True).

    The recording never gets in the way of the drives: if the file cannot be written or closed,
    such as on a full disk, report() is called once with the reason and recording stops.
    """

    def __init__(self, bus, path, report=None):
        self.bus = bus
        self.path = path
        self.report = report or (lambda message: None)
        self.file = open(path, 'a', encoding='ascii', buffering=1 << 16)

    def __getattr__(self, name):
        return getattr(self.bus, name)

    def send(self, can_id, data):
        sent = self.bus.send(can_id, data)
        self._write('tx' if sent else 'unsent', can_id, data)
        return sent

    def receive(self, *args, **kwargs):
        frame = self.bus.receive(*args, **kwargs)
        if frame is not None:
            self._write('rx', *frame)
        return frame

    def take_refused(self):
        refused = self.bus.take_refused()
        for can_id, data in refused:
            self._write('refused', can_id, data)
        return refused

    def shutdown(self):
        try:
            self.bus.shutdown()
        finally:
            self._stop_recording()

    def _write(self, kind, can_id, data):
        if self.file is None:
            return
        try:
            self.file.write(
                f'{time.time():.6f} {kind} {can_id:03X}#{bytes(data).hex().upper()}\n')
        except OSError as error:
            self._stop_recording(error)

    def _stop_recording(self, error=None):
        file, self.file = self.file, None
        if file is None:
            return
        try:
            file.close()
        except OSError as close_error:
            error = error or close_error
        if error is not None:
            self.report(f'Stopped recording drive frames to {self.path}: {error}')
