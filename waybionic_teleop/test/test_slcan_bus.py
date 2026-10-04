"""The carrier's serial link: batched writes, line parsing, and recovering from a lost USB."""

import os
import shutil
import subprocess
import time

import pytest

from waybionic_teleop import carrier, mks_can, slcan_bus
from waybionic_teleop.slcan_bus import SlcanBus


class FakeSerial:
    """A USB serial port that can be unplugged."""

    def __init__(self):
        self.writes, self.inbox, self.unplugged, self.closed = [], bytearray(), False, False

    def _check(self):
        if self.unplugged:
            raise OSError(6, 'Device not configured')

    @property
    def in_waiting(self):
        self._check()
        return len(self.inbox)

    def read(self, size):
        self._check()
        chunk = bytes(self.inbox[:size])
        del self.inbox[:size]
        return chunk

    def write(self, data):
        self._check()
        self.writes.append(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


@pytest.fixture
def clock(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(slcan_bus.time, 'monotonic', lambda: now[0])
    return now


@pytest.fixture
def ports():
    return []


@pytest.fixture
def bus(clock, ports):
    def open_port():
        ports.append(FakeSerial())
        return ports[-1]
    return SlcanBus('/dev/cu.usbmodem1101', 1000000, open_port=open_port)


def test_it_opens_the_bridge_at_the_bit_rate_then_writes_a_tick_in_one_go(bus, ports):
    assert ports[0].writes == [b'C\rS8\rO\r']
    assert bus.send(1, mks_can.read_encoder(1)) and bus.send(2, mks_can.read_encoder(2))
    assert len(ports[0].writes) == 1
    assert bus.receive() is None
    assert ports[0].writes[1] == b't00123132\rt00223133\r'


def test_replies_are_parsed_across_reads_and_refusals_counted(bus, ports):
    ports[0].inbox += b'\r\r\rt0013F5'
    assert bus.receive() is None
    ports[0].inbox += b'01F7\r\at7F080005000000000000\r'
    assert bus.receive() == (1, bytes.fromhex('F501F7'))
    assert bus.receive() == (carrier.STATUS_ID, bytes.fromhex('0005000000000000'))
    assert bus.receive() is None
    assert bus.errors == 1


def test_an_unplugged_carrier_is_reopened_and_set_up_again(bus, ports, clock):
    ports[0].unplugged = True
    bus.send(1, mks_can.read_encoder(1))
    assert bus.receive() is None
    assert not bus.connected and 'Device not configured' in bus.problem
    assert not bus.send(1, mks_can.read_encoder(1))
    bus.step(0.0)
    assert len(ports) == 1
    clock[0] += slcan_bus.REOPEN_EVERY_S
    bus.step(0.0)
    assert bus.connected and bus.reopens == 1 and ports[1].writes == [b'C\rS8\rO\r']


def test_a_carrier_that_went_quiet_is_opened_again(bus, ports, clock):
    ports[0].inbox += b't7F080005000000000000\r'
    bus.receive()
    clock[0] += slcan_bus.SILENCE_S / 2
    bus.step(0.0)
    assert len(ports[0].writes) == 1
    clock[0] += slcan_bus.SILENCE_S
    bus.step(0.0)
    assert ports[0].writes[-1] == b'C\rS8\rO\r'


def test_the_carrier_status_frame_decodes():
    status = carrier.parse_status(bytes.fromhex('0707' + '5DC0' + '0002' + '0304'))
    assert status == {'sequence': 7, 'estop': 'pressed', 'supply_v': 24.0, 'can_errors': 2,
                      'failed_writes': 3, 'refused_lines': 4}
    unwired = carrier.parse_status(bytes(8))
    assert unwired['estop'] is None and unwired['supply_v'] is None
    with pytest.raises(ValueError):
        carrier.parse_status(bytes(7))


def slcan_sim():
    try:
        from ament_index_python.packages import get_package_prefix
        path = os.path.join(get_package_prefix('waybionic_can'), 'lib', 'waybionic_can',
                            'slcan_sim')
    except (ImportError, LookupError, ValueError):
        path = shutil.which('slcan_sim')
    return path if path and os.access(path, os.X_OK) else None


@pytest.mark.skipif(slcan_sim() is None, reason='waybionic_can slcan_sim is not built')
def test_the_simulated_carrier_answers_through_the_real_bridge_code():
    sim = subprocess.Popen([slcan_sim(), '--ids', '1', '--move-ms', '100'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        port = sim.stdout.readline().strip()
        bus = SlcanBus(port, 500000)
        frames = []

        def wait_for(done, seconds=2.0):
            deadline = time.monotonic() + seconds
            while not done() and time.monotonic() < deadline:
                bus.step(0.0)
                frame = bus.receive()
                if frame is None:
                    time.sleep(0.002)
                else:
                    frames.append(frame)
            return done()

        assert wait_for(lambda: any(can_id == carrier.STATUS_ID for can_id, _ in frames))
        bus.send(1, mks_can.read_encoder(1))
        assert wait_for(lambda: (1, bytes.fromhex('3100000000000032')) in frames)
        assert bus.errors == 0
        bus.shutdown()
    finally:
        sim.terminate()
        sim.wait()
