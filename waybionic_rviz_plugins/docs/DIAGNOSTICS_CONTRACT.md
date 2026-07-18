# Diagnostics Contract

The RViz2 diagnostics panel consumes a normalized internal model. Mock diagnostics and live ROS 2 diagnostics both convert into this model before updating the UI.

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

## ROS 2 Diagnostics Mapping

Live mode subscribes to `/diagnostics` by default. The topic can be overridden at launch:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

Message type:

- `diagnostic_msgs/msg/DiagnosticArray`

Mapping guidance:

- `DiagnosticStatus.name` maps to `signal_name`.
- ROS diagnostic `OK` maps to `DiagnosticStatus::Ok`.
- ROS diagnostic `WARN` maps to `DiagnosticStatus::Warn`.
- ROS diagnostic `ERROR` maps to `DiagnosticStatus::Fault`.
- ROS diagnostic `STALE` maps to `DiagnosticStatus::Stale`.
- `DiagnosticStatus.message` maps to `alert_message` when the status is not `OK`.
- `DiagnosticStatus.values` may provide `value` and `unit` keys. If those keys are not present, the first non-empty key/value pair is shown as a value with the key as the unit/label.
- `DiagnosticArray.header.stamp` is used as the row timestamp. If it is unset, receive time is used.
- If no live message has arrived, the panel shows `Live diagnostics mode active` and `Waiting for <topic> messages` instead of mock data.
- If the latest live message is old, the panel marks rows stale.

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

## Source Switching

Launch with mock diagnostics:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=true
```

Launch with live ROS 2 diagnostics:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

Mock mode keeps the `Mock Normal` and `Mock Fault` validation controls enabled. Live mode disables those controls and listens to the configured diagnostics topic.

## Integration Checklist

1. Publish live diagnostics to `/diagnostics` or pass the agreed topic as `diagnostics_topic:=<topic>`.
2. Verify that each `DiagnosticStatus` includes a stable `name`, useful `message`, and value/unit keys where applicable.
3. Launch the engineer view with `use_mock_diagnostics:=false`.
4. Verify mock normal/fault validation states and live backend diagnostics before operator use.
5. Keep robot visualization integration separate; it does not require changes to the diagnostic contract itself.
