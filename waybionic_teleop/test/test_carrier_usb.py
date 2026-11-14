"""Handing the carrier's USB port back for an upload must pick the selected board's bridge."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

SCRIPT = Path(__file__).resolve().parents[2] / 'scripts' / 'carrier_usb.py'


@pytest.fixture
def usb():
    spec = importlib.util.spec_from_file_location('carrier_usb', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Port:
    def __init__(self, device, pid, location):
        self.device, self.vid, self.pid, self.location = device, 0x2341, pid, location


class Machine:
    """USB ports that change when the carrier gives its port back after a few polls."""

    def __init__(self, usb, ports, reenumerated, polls_until_bridge=3):
        self.usb, self.ports, self.reenumerated = usb, list(ports), reenumerated
        self.polls_until_bridge = polls_until_bridge
        self.opened = []

    def comports(self):
        if self.opened:
            self.polls_until_bridge -= 1
            if self.polls_until_bridge <= 0:
                return self.reenumerated
        return self.ports

    def serial(self):
        link = SimpleNamespace(port=None, baudrate=None, dtr=None, close=lambda: None)
        link.open = lambda: self.opened.append((link.port, link.baudrate, link.dtr))
        return link


@pytest.fixture
def machine(usb, monkeypatch):
    def make(ports, reenumerated, **kwargs):
        fake = Machine(usb, ports, reenumerated, **kwargs)
        monkeypatch.setattr(usb.list_ports, 'comports', fake.comports)
        monkeypatch.setattr(usb, 'Serial', fake.serial)
        monkeypatch.setattr(usb.time, 'sleep', lambda seconds: None)
        return fake
    return make


def test_the_selected_carriers_own_bridge_is_returned_not_another_boards(usb, machine):
    carrier = Port('/dev/cu.usbmodem1101', usb.WIFI_NATIVE_PID, '20-1')
    other_bridge = Port('/dev/cu.usbmodem2201', usb.WIFI_BRIDGE_PID, '20-2')
    own_bridge = Port('/dev/cu.usbmodem3301', usb.WIFI_BRIDGE_PID, '20-1')
    fake = machine([carrier, other_bridge], [other_bridge, own_bridge])
    assert usb.upload_port('/dev/cu.usbmodem1101') == '/dev/cu.usbmodem3301'
    assert fake.opened == [('/dev/cu.usbmodem1101', usb.HAND_BACK_BAUD, True)]


def test_the_bridge_may_keep_the_carriers_device_name(usb, machine):
    carrier = Port('/dev/cu.usbmodem1101', usb.WIFI_NATIVE_PID, '20-1')
    other_bridge = Port('/dev/cu.usbmodem2201', usb.WIFI_BRIDGE_PID, '20-2')
    own_bridge = Port('/dev/cu.usbmodem1101', usb.WIFI_BRIDGE_PID, '20-1')
    machine([carrier, other_bridge], [other_bridge, own_bridge])
    assert usb.upload_port('/dev/cu.usbmodem1101') == '/dev/cu.usbmodem1101'


def test_without_a_usb_location_only_a_new_bridge_counts(usb, machine):
    carrier = Port('/dev/cu.usbmodem1101', usb.WIFI_NATIVE_PID, None)
    other_bridge = Port('/dev/cu.usbmodem2201', usb.WIFI_BRIDGE_PID, None)
    new_bridge = Port('/dev/cu.usbmodem3301', usb.WIFI_BRIDGE_PID, None)
    machine([carrier, other_bridge], [other_bridge, new_bridge])
    assert usb.upload_port('/dev/cu.usbmodem1101') == '/dev/cu.usbmodem3301'


def test_a_bridge_already_there_is_never_taken_for_the_handed_back_one(usb, machine, monkeypatch):
    carrier = Port('/dev/cu.usbmodem1101', usb.WIFI_NATIVE_PID, None)
    other_bridge = Port('/dev/cu.usbmodem2201', usb.WIFI_BRIDGE_PID, None)
    machine([carrier, other_bridge], [other_bridge], polls_until_bridge=1)
    monkeypatch.setattr(usb, 'HAND_BACK_TIMEOUT_S', 0.05)
    with pytest.raises(SystemExit, match='did not hand USB back'):
        usb.upload_port('/dev/cu.usbmodem1101')


def test_a_board_already_in_bridge_mode_is_used_as_it_is(usb, machine):
    bridge = Port('/dev/cu.usbmodem2201', usb.WIFI_BRIDGE_PID, '20-2')
    fake = machine([bridge], [bridge])
    assert usb.upload_port('/dev/cu.usbmodem2201') == '/dev/cu.usbmodem2201'
    assert fake.opened == []
