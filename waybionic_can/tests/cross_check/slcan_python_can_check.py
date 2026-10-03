#!/usr/bin/env python3
"""
Drive the SLCAN bridge with python-can's real slcan interface, through slcan_sim.

slcan_sim runs the same SlcanBridge as carrier_bridge.ino on a pseudo-terminal, in front of a
simulated MKS drive with CAN ID 1. Needs python-can and pyserial.

    python3 slcan_python_can_check.py --sim path/to/slcan_sim
"""

import argparse
import subprocess
import sys

import can


def expect(bus, can_id, data_hex, what):
    message = bus.recv(timeout=2.0)
    if message is None:
        sys.exit(f'FAIL {what}: no reply')
    got = f'{message.arbitration_id:03X}#{bytes(message.data).hex().upper()}'
    want = f'{can_id:03X}#{data_hex}'
    print(f'RX {got}  ({what})')
    if got != want or message.is_extended_id:
        sys.exit(f'FAIL {what}: expected {want}')


def send(bus, can_id, data_hex):
    print(f'TX {can_id:03X}#{data_hex}')
    bus.send(can.Message(arbitration_id=can_id, data=bytes.fromhex(data_hex),
                         is_extended_id=False))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', required=True)
    options = parser.parse_args()

    sim = subprocess.Popen([options.sim, '--ids', '1', '--move-ms', '200'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    try:
        pty = sim.stdout.readline().strip()
        bus = can.Bus(interface='slcan', channel=pty, bitrate=500000, sleep_after_open=0.1)
        hw, sw = bus.get_version(timeout=1.0)
        print(f'bridge version hw={hw} sw={sw}')
        # Frames are the PR #24 manual vectors (waybionic_teleop/test/test_mks_can.py).
        send(bus, 1, '3132')
        expect(bus, 1, '3100000000000032', '31h encoder = 0')
        send(bus, 1, '820588')
        expect(bus, 1, '820184', '82h SR_vFOC ok')
        send(bus, 1, 'F301F5')
        expect(bus, 1, 'F301F5', 'F3h enable ok')
        send(bus, 1, 'F502580200400092')
        expect(bus, 1, 'F501F7', 'F5h running')
        expect(bus, 1, 'F502F8', 'F5h run complete')
        send(bus, 1, '3132')
        expect(bus, 1, '3100000000400072', '31h encoder = 0x4000')
        send(bus, 1, '3133')  # bad MKS checksum: the drive must stay silent
        if bus.recv(timeout=0.5) is not None:
            sys.exit('FAIL bad checksum got a reply')
        print('bad checksum: no reply (correct)')
        bus.shutdown()
        print('PASS')
    finally:
        sim.terminate()
        sim.wait()


if __name__ == '__main__':
    main()
