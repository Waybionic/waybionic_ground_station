"""Locally generated diagnostic data for the prototype dashboard."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Protocol

from src.diagnostics_contract import DiagnosticMessage, utc_timestamp


class DiagnosticsSource(Protocol):
    source_name: str

    def get_messages(self) -> list[DiagnosticMessage]:
        """Return normalized diagnostic messages for the UI."""


@dataclass
class MockDiagnosticsSource:
    """Mock source that can be switched between normal and fault demos."""

    mode: str = "normal"
    source_name: str = "Mock"

    def set_mode(self, mode: str) -> None:
        if mode not in {"normal", "fault"}:
            raise ValueError(f"Unsupported diagnostics mode: {mode}")
        self.mode = mode

    def get_messages(self) -> list[DiagnosticMessage]:
        if self.mode == "fault":
            return self._fault_messages()
        return self._normal_messages()

    def _normal_messages(self) -> list[DiagnosticMessage]:
        pulse = math.sin(time.monotonic() / 4.0)
        return [
            DiagnosticMessage("board.temperature", "OK", utc_timestamp(0.4), round(42.0 + pulse, 1), "°C"),
            DiagnosticMessage("motor.current", "OK", utc_timestamp(0.5), round(0.8 + pulse * 0.05, 2), "A"),
            DiagnosticMessage("imu.roll", "OK", utc_timestamp(0.2), round(1.2 + pulse * 0.1, 1), "deg"),
            DiagnosticMessage("imu.pitch", "OK", utc_timestamp(0.2), round(-0.4 + pulse * 0.1, 1), "deg"),
            DiagnosticMessage("imu.yaw", "OK", utc_timestamp(0.2), round(12.9 + pulse * 0.2, 1), "deg"),
        ]

    def _fault_messages(self) -> list[DiagnosticMessage]:
        pulse = math.sin(time.monotonic() / 3.0)
        return [
            DiagnosticMessage(
                "board.temperature",
                "FAULT",
                utc_timestamp(0.2),
                round(82.0 + pulse * 0.5, 1),
                "°C",
                "High temperature detected",
            ),
            DiagnosticMessage("motor.current", "OK", utc_timestamp(0.5), 0.8, "A"),
            DiagnosticMessage("imu.roll", "OK", utc_timestamp(0.2), 1.2, "deg"),
            DiagnosticMessage("imu.pitch", "OK", utc_timestamp(0.2), -0.4, "deg"),
            DiagnosticMessage("imu.yaw", "OK", utc_timestamp(0.2), 12.9, "deg"),
            DiagnosticMessage("imu.heartbeat", "STALE", utc_timestamp(5.2), None, None, "Sensor timeout"),
        ]
