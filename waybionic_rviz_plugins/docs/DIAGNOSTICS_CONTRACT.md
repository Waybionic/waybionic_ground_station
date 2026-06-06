# Diagnostics Contract

The RViz2 diagnostics panel consumes a normalized internal model. Mock diagnostics use this contract now, and future ROS 2 diagnostics should be converted into the same model before updating the UI.

## Internal C++ Shape

```cpp
enum class DiagnosticStatus
{
  Ok,
  Warn,
  Fault,
  Stale
};

struct DiagnosticMessage
{
  std::string signal_name;
  DiagnosticStatus status;
  rclcpp::Time timestamp;
  std::optional<std::string> value;
  std::optional<std::string> unit;
  std::optional<std::string> alert_message;
};
```

## Fields

- `signal_name`: stable name of the monitored signal, such as `board.temperature`, `motor.current`, `imu.roll`, `imu.pitch`, `imu.yaw`, or `imu.heartbeat`.
- `status`: current health state, normalized to `OK`, `WARN`, `FAULT`, or `STALE`.
- `timestamp`: time the diagnostic was generated or last updated.
- `value`: current reading, if applicable.
- `unit`: unit for the value, such as `C`, `A`, `deg`, `V`, or empty/null.
- `alert_message`: human-readable message shown in the alerts panel when status is not `OK`.

## Example Normal Data

```text
board.temperature | OK | 42.0 | C | 0.4s ago | -
motor.current     | OK | 0.80 | A | 0.5s ago | -
imu.roll          | OK | 1.2  | deg | 0.2s ago | -
imu.pitch         | OK | -0.4 | deg | 0.2s ago | -
imu.yaw           | OK | 12.9 | deg | 0.2s ago | -
```

## Example Fault Data

```text
board.temperature | FAULT | 82.0 | C | 0.2s ago | High temperature detected
imu.heartbeat     | STALE | -    | - | 5.2s ago | Sensor timeout
```

Other rows can remain `OK` while these fault rows generate visible alerts.

## Future ROS 2 Mapping

Preferred future topic:

```text
/diagnostics
```

Recommended source format:

- `diagnostic_msgs/msg/DiagnosticArray`
- or another agreed WayBionic diagnostics message

Mapping guidance:

- ROS diagnostic `OK` maps to `DiagnosticStatus::Ok`.
- ROS diagnostic `WARN` maps to `DiagnosticStatus::Warn`.
- ROS diagnostic `ERROR` maps to `DiagnosticStatus::Fault`.
- Missing or old timestamps should map to `DiagnosticStatus::Stale`.
- Diagnostic key/value pairs should be normalized into `value`, `unit`, and `alert_message` before updating the panel.

The Qt/RViz UI should depend on `DiagnosticMessage`, not raw ROS message internals. This keeps mock data, future backend diagnostics, and any future simulator diagnostics on the same path.

## Backend Note

Please publish each monitored signal with:

- stable signal name
- status level
- timestamp
- current value, if applicable
- unit, if applicable
- alert message, if applicable

Faults and stale signals should generate visible alerts in the engineer panel.

## Integration Checklist (Later)

1. Publish live diagnostics to `/diagnostics` (or agreed topic) with stable signal names.
2. Implement a ROS 2 subscriber that maps incoming messages to `DiagnosticMessage`.
3. Add a panel parameter such as `use_mock_diagnostics:=false` to switch from mock to live data.
4. Verify Normal and Fault scenarios in the engineer RViz panel before operator use.
5. Keep camera and robot visualization integration separate — they do not require changes to the diagnostic contract itself.
