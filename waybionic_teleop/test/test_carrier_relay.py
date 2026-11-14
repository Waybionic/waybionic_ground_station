import socket
import threading
import time
from types import SimpleNamespace

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


class ComPorts:
    """The COM ports Windows lists, which a test changes as the carrier is plugged in."""

    def __init__(self, *devices):
        self.devices, self.opened = list(devices), []

    def arduino(self, device):
        return SimpleNamespace(device=device, vid=carrier_relay.ARDUINO_VID)

    def listed(self):
        return [self.arduino(device) if device.startswith('COM') else
                SimpleNamespace(device=device, vid=0x1A86) for device in self.devices]

    def open_serial(self, name, baudrate, timeout):
        self.opened.append((name, baudrate))
        return name


def opener(ports, port=None):
    return carrier_relay.carrier_opener(
        port, 1000000, open_serial=ports.open_serial, list_com_ports=ports.listed,
        log=lambda message: None)


def test_a_carrier_plugged_in_after_the_relay_started_is_found():
    ports = ComPorts()
    open_carrier = opener(ports)
    with pytest.raises(serial.SerialException, match='no Arduino COM port found.*plug in'):
        open_carrier()
    ports.devices.append('COM5')
    assert open_carrier() == 'COM5'
    assert ports.opened == [('COM5', 1000000)]


def test_a_replugged_carrier_is_found_under_its_new_com_number():
    ports = ComPorts('COM5')
    open_carrier = opener(ports)
    assert open_carrier() == 'COM5'
    ports.devices[:] = ['COM7']
    assert open_carrier() == 'COM7'


def test_other_serial_devices_are_not_mistaken_for_the_carrier():
    ports = ComPorts('/dev/ttyUSB0')
    with pytest.raises(serial.SerialException, match='no Arduino COM port found'):
        opener(ports)()
    assert ports.opened == []


def test_the_relay_does_not_guess_between_several_arduinos():
    ports = ComPorts('COM3', 'COM5')
    with pytest.raises(serial.SerialException, match=r'COM3, COM5.*--port'):
        opener(ports)()
    assert ports.opened == []


def test_a_port_that_is_given_is_opened_without_looking_for_it():
    ports = ComPorts()
    assert opener(ports, port='COM9')() == 'COM9'


def test_the_relay_starts_listening_once_the_carrier_is_plugged_in():
    probe = socket.create_server(('127.0.0.1', 0))
    tcp_port = probe.getsockname()[1]
    probe.close()
    ports = ComPorts()
    messages, stop = [], threading.Event()
    opener_with_loop = carrier_relay.carrier_opener(
        None, 1000000, open_serial=lambda name, baudrate, timeout: loop_carrier(),
        list_com_ports=ports.listed, log=messages.append)
    thread = threading.Thread(
        target=carrier_relay.serve, daemon=True,
        args=(opener_with_loop, '127.0.0.1', tcp_port, stop, messages.append))
    thread.start()
    try:
        time.sleep(0.2)
        with pytest.raises(OSError):
            socket.create_connection(('127.0.0.1', tcp_port), timeout=0.5)
        assert any('plug in the carrier' in message for message in messages)
        ports.devices.append('COM5')
        deadline = time.monotonic() + 4.0
        while True:
            try:
                client = socket.create_connection(('127.0.0.1', tcp_port), timeout=0.5)
                break
            except OSError:
                assert time.monotonic() < deadline, 'the relay never listened'
                time.sleep(0.05)
        client.settimeout(2.0)
        client.sendall(b't00121234\r')
        assert client.recv(64) == b't00121234\r'
        client.close()
        assert 'Carrier on COM5' in messages
    finally:
        stop.set()
        thread.join(3.0)
