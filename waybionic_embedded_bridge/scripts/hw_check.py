"""Hardware/firmware smoke check for the UNO ASCII stack (no teleop loop)."""
from __future__ import annotations

import sys
import time

import serial
import serial.tools.list_ports


def list_ports() -> None:
    print("=== Ports ===")
    for p in serial.tools.list_ports.comports():
        mark = " <<<" if "2341" in (p.hwid or "") else ""
        print(f"  {p.device}: {p.description} [{p.hwid}]{mark}")


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
    list_ports()
    print(f"\n=== Opening {port} @ 115200 ===")
    ser = serial.Serial(port, 115200, timeout=0.3)
    time.sleep(2.0)  # allow CDC reset if any
    boot = ser.read(4096)
    print(f"boot bytes ({len(boot)}): {boot[:200]!r}")

    def tx(cmd: str) -> None:
        print(f"TX: {cmd}")
        ser.write((cmd + "\n").encode())
        ser.flush()
        time.sleep(0.4)
        rx = ser.read(4096)
        print(f"RX ({len(rx)}): {rx[:300]!r}")

    for cmd in ("POS", "ENABLE", "J1_CW"):
        tx(cmd)

    print("Waiting 2s while J1_CW should be running (watch the arm)...")
    time.sleep(2.0)
    tx("J1_STOP")
    tx("DISABLE")
    ser.close()
    print("\nDone. If arm never moved during the 2s wait, problem is on UNO/CAN/power/E-stop.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
