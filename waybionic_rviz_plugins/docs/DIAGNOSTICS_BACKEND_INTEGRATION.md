# Diagnostics Backend Integration

This note explains how a real WayBionic diagnostics backend should replace the temporary publisher in `waybionic_rviz_plugins`. The RViz `DiagnosticsPanel` does not need code changes when the backend is swapped in.

## Topic and Message Type

| Item | Value |
|------|-------|
| Default topic | `/diagnostics` |
| Message type | `diagnostic_msgs/msg/DiagnosticArray` |
| Override at launch | `diagnostics_topic:=<topic>` on `engineer_view.launch.py` |

## Expected Signal Names

The panel and temporary publisher currently use these stable `DiagnosticStatus.name` values:

| Signal | Typical unit | Notes |
|--------|--------------|-------|
| `board.temperature` | `C` | Board or motor-driver temperature |
| `motor.current` | `A` | Motor or bus current |
| `imu.roll` | `deg` | IMU orientation telemetry |
| `imu.pitch` | `deg` | IMU orientation telemetry |
| `imu.yaw` | `deg` | IMU orientation telemetry |
| `imu.heartbeat` | — | Sensor health / timeout indicator |

Additional signals can be added later. New names should stay dotted and stable (`subsystem.metric`).

## Value and Unit Fields

Populate `DiagnosticStatus.values` with `KeyValue` pairs:

- `value` — reading as a string (for example `42.0`)
- `unit` — unit label (for example `C`, `A`, `deg`)

If those keys are missing, the panel falls back to the first non-empty key/value pair.

## Status Level Mapping

| ROS `DiagnosticStatus.level` | Panel display |
|------------------------------|---------------|
| `OK` | OK |
| `WARN` | WARN |
| `ERROR` | FAULT |
| `STALE` | STALE |

Set `DiagnosticStatus.message` for non-OK rows; the panel shows it in the alerts area.

## Timestamps

Use `DiagnosticArray.header.stamp` when the backend knows the measurement time. If unset, the panel uses receive time and may mark rows stale after about five seconds without updates.

## Replacing the Temporary Publisher

1. Stop `temporary_diagnostics_publisher.launch.py` if it is running.
2. Start the real backend node publishing to `/diagnostics` (or another topic passed to the engineer view).
3. Launch live diagnostics:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
```

Launch arguments take precedence over any saved RViz panel settings.

The temporary publisher at `scripts/temporary_diagnostics_publisher.py` is the reference implementation. Modes:

| Mode | Purpose |
|------|---------|
| `normal` | All sample signals OK |
| `fault` | High board temperature + stale `imu.heartbeat` |
| `stale` | All sample signals STALE |
| `cycle` | Rotate through normal, fault, and stale every 5 seconds |

## Acceptance Criteria for a Real Backend

- Publish at least 1 Hz on `/diagnostics` during normal operation.
- Include `value` and `unit` keys for telemetry rows where applicable.
- Publish `imu.heartbeat` (or equivalent) so stale sensor detection can be validated.
- Keep signal names aligned with this document unless the team agrees on a new contract update.

## Related Docs

- `DIAGNOSTICS_CONTRACT.md` — normalized internal model used by the panel
- `PR_NOTES.md` — PR validation commands and review notes
