"""Safe serial smoke test: open port, query POS, optional ENABLE/DISABLE (no motion).

  python -m waybionic_embedded_bridge.probe_serial --port COM3
"""

from __future__ import annotations

import argparse
import time

from . import protocol
from .serial_client import UnoSerialClient


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--baud", type=int, default=protocol.BAUD)
    parser.add_argument("--no-reset-wait", action="store_true")
    parser.add_argument(
        "--enable-disable",
        action="store_true",
        help="Also send ENABLE then DISABLE (still no motion commands)",
    )
    args = parser.parse_args(argv)

    lines: list[str] = []

    def on_line(line: str) -> None:
        print(f"< {line}")
        lines.append(line)

    client = UnoSerialClient(
        port=args.port,
        baud=args.baud,
        watchdog_s=0.0,
        on_line=on_line,
        reset_settle_s=0.0 if args.no_reset_wait else 2.0,
    )
    try:
        print(f"Opening {args.port} @ {args.baud} ...")
        client.open()
        print("Sending POS")
        client.pos()
        time.sleep(0.8)
        if args.enable_disable:
            print("Sending ENABLE")
            client.enable()
            time.sleep(0.3)
            print("Sending DISABLE")
            client.disable()
            time.sleep(0.3)
        if lines:
            print(f"OK: received {len(lines)} line(s) from firmware.")
            return 0
        print("WARN: no response lines. Port may be wrong, or firmware silent on POS.")
        return 2
    except Exception as exc:
        print(f"FAIL: {exc}")
        return 1
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
