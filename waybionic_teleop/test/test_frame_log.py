import errno

from waybionic_teleop import mks_can
from waybionic_teleop.frame_log import parse_line, RecordingBus
from waybionic_teleop.sim_drives import SimulatedBus, SimulatedServo


def test_every_frame_sent_received_or_refused_is_written_in_order(tmp_path):
    path = tmp_path / 'frames.log'
    sim = SimulatedBus([SimulatedServo(1)], 500000)
    bus = RecordingBus(sim, str(path))
    assert bus.send(1, mks_can.read_encoder(1))
    reply = bus.receive()
    assert reply is not None and bus.receive() is None
    # Everything else, such as the simulated drives, still comes from the wrapped bus.
    assert bus.drives is sim.drives and bus.frames == sim.frames
    sim.take_refused = lambda: [(1, mks_can.stop(1, 0))]
    assert bus.take_refused() == [(1, mks_can.stop(1, 0))]
    bus.shutdown()
    lines = [parse_line(line) for line in path.read_text().splitlines()]
    assert [line[1:] for line in lines] == [
        ('tx', 1, mks_can.read_encoder(1)),
        ('rx', *reply),
        ('refused', 1, mks_can.stop(1, 0)),
    ]
    assert lines[0][0] <= lines[1][0] <= lines[2][0]


class FullDisk:
    """A log file whose writes (and, with fail_close, final flush) fail like a full disk."""

    def __init__(self, fail_write=True, fail_close=True):
        self.fail_write, self.fail_close = fail_write, fail_close
        self.writes = self.closes = 0

    def write(self, text):
        self.writes += 1
        if self.fail_write:
            raise OSError(errno.ENOSPC, 'No space left on device')

    def close(self):
        self.closes += 1
        if self.fail_close:
            raise OSError(errno.ENOSPC, 'No space left on device')


def recording_bus_on(file, tmp_path, messages):
    sim = SimulatedBus([SimulatedServo(1)], 500000)
    bus = RecordingBus(sim, str(tmp_path / 'frames.log'), report=messages.append)
    bus.file.close()
    bus.file = file
    sim.stopped = False
    sim.shutdown = lambda: setattr(sim, 'stopped', True)
    return bus, sim


def test_a_failing_log_never_interrupts_the_bus(tmp_path):
    messages = []
    disk = FullDisk()
    bus, sim = recording_bus_on(disk, tmp_path, messages)
    for _ in range(3):
        # The stops at shutdown go through send(), so a log error must not reach the caller.
        assert bus.send(1, mks_can.read_encoder(1))
        assert bus.receive() is not None
    assert disk.writes == 1
    assert len(messages) == 1
    assert 'No space left on device' in messages[0] and 'frames.log' in messages[0]
    bus.shutdown()
    assert sim.stopped and disk.closes == 1
    assert len(messages) == 1


def test_a_log_that_cannot_be_closed_is_reported_without_blocking_shutdown(tmp_path):
    messages = []
    disk = FullDisk(fail_write=False)
    bus, sim = recording_bus_on(disk, tmp_path, messages)
    assert bus.send(1, mks_can.read_encoder(1))
    assert messages == []
    bus.shutdown()
    assert sim.stopped and disk.closes == 1
    assert len(messages) == 1 and 'No space left on device' in messages[0]
    bus.shutdown()
    assert disk.closes == 1
