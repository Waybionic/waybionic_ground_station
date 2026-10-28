"""Deeper serial probe: capture boot text, try line endings and a few commands."""
from __future__ import annotations

import sys
import time

import serial


def try_once(port: str, baud: int, dtr: bool, cmds: list[bytes], settle: float) -> None:
    print(f"\n=== port={port} baud={baud} dtr={dtr} settle={settle}s ===")
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.2
    ser.dtr = dtr
    ser.rts = dtr
    ser.open()
    time.sleep(settle)
    boot = ser.read(4096)
    print("BOOT:", repr(boot[:500]))
    for cmd in cmds:
        ser.reset_input_buffer()
        print("TX:", repr(cmd))
        ser.write(cmd)
        ser.flush()
        time.sleep(0.4)
        rx = ser.read(4096)
        print("RX:", repr(rx[:500]))
    ser.close()


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    cmds = [
        b"POS\n",
        b"POS\r\n",
        b"pos\n",
        b"HELP\n",
        b"?\n",
        b"STATUS\n",
    ]
    for baud in (115200, 9600, 57600, 1000000):
        try:
            try_once(port, baud, True, cmds[:2], 2.5)
            try_once(port, baud, False, cmds[:2], 0.5)
        except Exception as exc:
            print(f"FAIL baud={baud}: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
