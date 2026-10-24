import socket
import threading

import pytest
import serial

from waybionic_teleop import carrier_relay


class LostCarrier:
    """A carrier that has just been unplugged."""

    in_waiting = 0

    def read(self, size=1):
        raise serial.SerialException('device unplugged')

    def write(self, data):
        raise serial.SerialException('device unplugged')


def loop_carrier():
    return serial.serial_for_url('loop://', timeout=carrier_relay.POLL_S)


def start(link):
    """Run relay() in a thread; return our end of the connection, stop, thread and result."""
    ours, theirs = socket.socketpair()
    ours.settimeout(2.0)
    stop = threading.Event()
    result = []
    thread = threading.Thread(
        target=lambda: result.append(carrier_relay.relay(link, theirs, stop)), daemon=True)
    thread.start()
    return ours, stop, thread, result


def test_frames_pass_both_ways():
    link = loop_carrier()
    ours, stop, thread, result = start(link)
    frame = b't00121234\r'
    ours.sendall(frame)
    echoed = b''
    while len(echoed) < len(frame):
        echoed += ours.recv(64)
    assert echoed == frame
    stop.set()
    thread.join(2.0)
    assert result == ['stopped']
    ours.close()
    link.close()


def test_lost_carrier_closes_the_connection():
    ours, stop, thread, result = start(LostCarrier())
    assert ours.recv(64) == b''
    thread.join(2.0)
    assert result == ['serial']
    ours.close()


def test_stopping_closes_the_connection():
    link = loop_carrier()
    ours, stop, thread, result = start(link)
    stop.set()
    assert ours.recv(64) == b''
    thread.join(2.0)
    assert result == ['stopped']
    ours.close()
    link.close()


def test_ground_station_disconnect_keeps_the_carrier_open():
    link = loop_carrier()
    ours, stop, thread, result = start(link)
    ours.close()
    thread.join(2.0)
    assert result == ['client']
    assert link.is_open
    link.close()


def test_no_connection_while_the_carrier_is_unplugged():
    probe = socket.create_server(('127.0.0.1', 0))
    tcp_port = probe.getsockname()[1]
    probe.close()
    stop = threading.Event()

    def unplugged():
        raise serial.SerialException('could not open port')

    thread = threading.Thread(
        target=carrier_relay.serve,
        args=(unplugged, '127.0.0.1', tcp_port, stop, lambda message: None), daemon=True)
    thread.start()
    try:
        with pytest.raises(OSError):
            socket.create_connection(('127.0.0.1', tcp_port), timeout=1.0)
    finally:
        stop.set()
        thread.join(2.0)


def test_port_in_use_exits_with_a_message():
    taken = socket.create_server(('127.0.0.1', 0))
    tcp_port = taken.getsockname()[1]
    stop = threading.Event()
    try:
        with pytest.raises(SystemExit, match='already in use'):
            carrier_relay.serve(loop_carrier, '127.0.0.1', tcp_port, stop, lambda message: None)
    finally:
        taken.close()
