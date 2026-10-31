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
