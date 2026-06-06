# Diagnostics Contract

The ground station UI consumes normalized diagnostic messages. Mock data uses this contract today, and future ROS 2 data should be converted into the same shape before reaching the UI.

## Internal Format

```python
DiagnosticMessage:
    signal_name: str
    status: "OK" | "WARN" | "FAULT" | "STALE"
    timestamp: str
    value: float | str | None
    unit: str | None
    alert_message: str | None
```

## Fields

- `signal_name`: stable name of the monitored topic or signal, such as `board.temperature`, `motor.current`, `imu.roll`, `imu.pitch`, `imu.yaw`, or `imu.heartbeat`.
- `status`: current health state of the signal. It must map cleanly to `OK`, `WARN`, `FAULT`, or `STALE`.
- `timestamp`: time the diagnostic was generated or last updated. ISO-8601 UTC strings are preferred.
- `value`: current reading, if applicable.
- `unit`: unit for the value, such as `°C`, `A`, `deg`, `V`, or empty/null.
- `alert_message`: human-readable message shown in the alerts panel when status is not `OK`.

## Example Normal Diagnostics

```python
[
    DiagnosticMessage("board.temperature", "OK", "2026-06-06T18:30:00.400000+00:00", 42, "°C", None),
    DiagnosticMessage("motor.current", "OK", "2026-06-06T18:30:00.500000+00:00", 0.8, "A", None),
    DiagnosticMessage("imu.roll", "OK", "2026-06-06T18:30:00.200000+00:00", 1.2, "deg", None),
    DiagnosticMessage("imu.pitch", "OK", "2026-06-06T18:30:00.200000+00:00", -0.4, "deg", None),
    DiagnosticMessage("imu.yaw", "OK", "2026-06-06T18:30:00.200000+00:00", 12.9, "deg", None),
]
```

## Example Fault Diagnostics

```python
[
    DiagnosticMessage(
        "board.temperature",
        "FAULT",
        "2026-06-06T18:30:00.200000+00:00",
        82,
        "°C",
        "High temperature detected",
    ),
    DiagnosticMessage("motor.current", "OK", "2026-06-06T18:30:00.500000+00:00", 0.8, "A", None),
    DiagnosticMessage("imu.roll", "OK", "2026-06-06T18:30:00.200000+00:00", 1.2, "deg", None),
    DiagnosticMessage("imu.pitch", "OK", "2026-06-06T18:30:00.200000+00:00", -0.4, "deg", None),
    DiagnosticMessage("imu.yaw", "OK", "2026-06-06T18:30:00.200000+00:00", 12.9, "deg", None),
    DiagnosticMessage(
        "imu.heartbeat",
        "STALE",
        "2026-06-06T18:29:55.200000+00:00",
        None,
        None,
        "Sensor timeout",
    ),
]
```

## Future ROS 2 Integration

The current flow is:

```text
MockDiagnosticsSource -> normalized DiagnosticMessage objects -> UI
```

The future flow should be:

```text
ROS2DiagnosticsSubscriber -> normalized DiagnosticMessage objects -> same UI
```

The preferred future topic is `/diagnostics`. The backend may publish diagnostics using ROS 2 diagnostic-style messages, such as diagnostic array/status messages, or another agreed format.

Whatever live ROS 2 source is used, it should be converted into the normalized `DiagnosticMessage` format before reaching the UI. The UI should not directly depend on ROS message internals.

## Integration Note For Korede / Backend

Please publish each monitored signal with a stable signal name, status level, timestamp, current value if applicable, unit, and alert message if applicable.

The UI expects statuses to map cleanly to `OK`, `WARN`, `FAULT`, or `STALE`. Faults and stale signals should generate visible alerts.

The UI can initially be tested using mock messages, then connected to live ROS 2 diagnostics once available.
