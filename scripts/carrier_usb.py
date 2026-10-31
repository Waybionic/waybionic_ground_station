#!/usr/bin/env python3
"""
Find the carrier (Arduino UNO R4) on USB and get it ready for an upload.

On the UNO R4 WiFi the carrier firmware moves the USB-C port from the ESP32-S3 bridge to the
RA4M1's own USB (waybionic_can/arduino/README.md, "USB link"), but uploads still go through
the bridge. This hands the port back when needed and prints the port to upload to:

    python scripts/carrier_usb.py upload-port [--port PORT]
"""

import argparse
import sys
import time

from serial import Serial
from serial.tools import list_ports

ARDUINO_VID = 0x2341
WIFI_BRIDGE_PID = 0x1002
WIFI_NATIVE_PID = 0x006D
MINIMA_PID = 0x0069
BOARD_PIDS = {
    WIFI_BRIDGE_PID: 'UNO R4 WiFi (USB bridge)',
    WIFI_NATIVE_PID: 'UNO R4 WiFi (carrier firmware)',
    MINIMA_PID: 'UNO R4 Minima',
}
# The carrier hands the port back when it sees this baud rate with DTR still high
# (UsbHandBack.h); about half a second on the bench.
HAND_BACK_BAUD = 1200
HAND_BACK_TIMEOUT_S = 10.0


def boards():
    return [port for port in list_ports.comports()
            if port.vid == ARDUINO_VID and port.pid in BOARD_PIDS]


def describe(ports):
    return ', '.join(f'{port.device} [{BOARD_PIDS[port.pid]}]' for port in ports) or 'none'


def only_board(device=None):
    found = boards()
    if device:
        found = [port for port in found if port.device == device]
    if len(found) != 1:
        wanted = f' at {device}' if device else ''
        sys.exit(f'error: expected one UNO R4{wanted}; found {describe(boards())}')
    return found[0]


def hand_back(device):
    """Ask the carrier firmware at device to give the USB port back to the bridge."""
    link = Serial()
    link.port = device
    link.baudrate = HAND_BACK_BAUD
    link.dtr = True
    link.open()
    try:
        deadline = time.monotonic() + HAND_BACK_TIMEOUT_S
        while time.monotonic() < deadline:
            bridges = [port for port in boards() if port.pid == WIFI_BRIDGE_PID]
            if bridges:
                return bridges[0].device
            time.sleep(0.05)
    finally:
        try:
            link.close()
        except OSError:
            pass
    sys.exit(f'error: the carrier at {device} did not hand USB back to its bridge within '
             f'{HAND_BACK_TIMEOUT_S:.0f} s; unplug it, plug it back in and try again')


def upload_port(device=None):
    board = only_board(device)
    if board.pid == WIFI_NATIVE_PID:
        return hand_back(board.device)
    return board.device


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('command', choices=['upload-port'])
    parser.add_argument('--port', help='the board to use when several are plugged in')
    args = parser.parse_args()
    print(upload_port(args.port))


if __name__ == '__main__':
    main()
