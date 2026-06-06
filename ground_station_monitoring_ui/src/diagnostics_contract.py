"""Normalized diagnostics contract used by the monitoring UI.

The UI consumes these objects only. A future ROS 2 subscriber should convert
live diagnostics into this shape before handing them to the dashboard.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from typing import Literal


DiagnosticStatus = Literal["OK", "WARN", "FAULT", "STALE"]


@dataclass(frozen=True)
class DiagnosticMessage:
    signal_name: str
    status: DiagnosticStatus
    timestamp: str
    value: float | str | None
    unit: str | None
    alert_message: str | None = None


def utc_timestamp(seconds_ago: float = 0.0) -> str:
    """Return an ISO-8601 UTC timestamp offset from the current time."""
    now = datetime.now(timezone.utc)
    if seconds_ago:
        now = now - timedelta(seconds=seconds_ago)
    return now.isoformat()


def seconds_since(timestamp: str) -> float:
    """Return age in seconds for an ISO-8601 timestamp."""
    try:
        parsed = datetime.fromisoformat(timestamp)
    except ValueError:
        return 0.0

    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return max(0.0, (datetime.now(timezone.utc) - parsed).total_seconds())


def format_age(timestamp: str) -> str:
    return f"{seconds_since(timestamp):.1f}s ago"
