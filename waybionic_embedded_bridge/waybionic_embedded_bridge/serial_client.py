"""USB serial client for the UNO R4 ASCII motor controller."""

from __future__ import annotations

import time
from typing import Callable, Iterator

import serial

from . import protocol


class UnoSerialClient:
    """Newline-terminated ASCII link. Gentle on UNO R4 USB CDC."""

    def __init__(
        self,
        port: str,
        baud: int = protocol.BAUD,
        watchdog_s: float = 0.0,
        on_line: Callable[[str], None] | None = None,
        reset_settle_s: float = 0.0,
        min_gap_s: float = 0.02,
    ) -> None:
        self.port = port
        self.baud = baud
        self.reset_settle_s = reset_settle_s
        self.min_gap_s = min_gap_s
        self._on_line = on_line
        self._ser: serial.Serial | None = None
        self._last_tx = 0.0
        self._rx_buf = bytearray()
        _ = watchdog_s

    def open(self) -> None:
        # Avoid DTR/RTS toggling that resets/hangs UNO R4 CDC on open/close.
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baud
        ser.timeout = 0
        ser.dsrdtr = False
        ser.rtscts = False
        ser.open()
        try:
            ser.dtr = False
            ser.rts = False
        except Exception:
            pass
        self._ser = ser
        if self.reset_settle_s > 0:
            time.sleep(self.reset_settle_s)
        time.sleep(0.3)
        try:
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception:
            pass
        self._rx_buf.clear()

    def close(self) -> None:
        try:
            self.send(protocol.STOP)
            time.sleep(0.05)
        except Exception:
            pass
        if self._ser is not None:
            try:
                if self._ser.is_open:
                    self._ser.close()
            except Exception:
                pass
        self._ser = None

    def __enter__(self) -> "UnoSerialClient":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def send(self, command: str, bump_watchdog: bool = True) -> None:
        _ = bump_watchdog
        line = command.strip()
        if not line:
            return
        if not self._ser or not self._ser.is_open:
            raise RuntimeError(f"Serial port {self.port} is not open")
        # Pace TX so USB CDC + CAN sketch don't wedge.
        gap = self.min_gap_s - (time.monotonic() - self._last_tx)
        if gap > 0:
            time.sleep(gap)
        self._ser.write((line + "\n").encode("ascii", errors="strict"))
        self._ser.flush()
        self._last_tx = time.monotonic()

    def poll_lines(self) -> Iterator[str]:
        """Drain RX and yield complete newline-terminated lines."""
        if not self._ser or not self._ser.is_open:
            return
        waiting = self._ser.in_waiting
        if waiting:
            chunk = self._ser.read(waiting)
            if chunk:
                self._rx_buf.extend(chunk)
        while True:
            nl = self._rx_buf.find(b"\n")
            if nl < 0:
                break
            raw = bytes(self._rx_buf[:nl])
            del self._rx_buf[: nl + 1]
            line = raw.decode("ascii", errors="replace").strip("\r").strip()
            if not line:
                continue
            if self._on_line is not None:
                self._on_line(line)
            yield line

    def enable(self) -> None:
        self.send(protocol.ENABLE)

    def disable(self) -> None:
        self.send(protocol.DISABLE)

    def stop(self) -> None:
        self.send(protocol.STOP)

    def estop(self) -> None:
        self.send(protocol.ESTOP)

    def pos(self) -> None:
        self.send(protocol.POS)
