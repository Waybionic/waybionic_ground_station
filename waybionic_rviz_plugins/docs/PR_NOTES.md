# PR Notes

## Summary

This branch makes `waybionic_rviz_plugins` a generic WayBionic RViz2 diagnostics foundation package. The default engineer launch opens RViz with the diagnostics panel, mock diagnostics are kept for validation, and live mode is ready to consume ROS 2 diagnostics without binding the panel UI to raw ROS messages.

## Primary Launch Commands

```bash
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
ros2 launch waybionic_rviz_plugins doctor_view.launch.py
```

Live diagnostics mode:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

## What Works

- Engineer RViz view with the `WayBionic Diagnostics` panel.
- Mock normal and fault validation states.
- Alert banner/table rendering from normalized `DiagnosticMessage` rows.
- Live diagnostics source path that subscribes to a `diagnostic_msgs/msg/DiagnosticArray` topic.
- Doctor/surgeon placeholder camera layout with a docked `SurgeonCameraPanel`.
- Both RViz panels are registered under `waybionic_rviz_plugins`: `DiagnosticsPanel` and `SurgeonCameraPanel`.

## Mock Diagnostics Mode

Mock mode is enabled by default through `use_mock_diagnostics:=true`. It uses synthetic values and keeps `Mock Normal` and `Mock Fault` controls enabled for UI validation while backend `/diagnostics` publishing is not ready.

## Live `/diagnostics` Path

`RosDiagnosticsSource` subscribes to `/diagnostics` by default, or another topic passed as `diagnostics_topic:=<topic>`, and converts each status into the internal `DiagnosticMessage` model before the panel renders it. If no backend is publishing yet, the panel stays stable and shows that live diagnostics mode is active while it waits for messages.

Expected mapping:

- `DiagnosticStatus.name` -> `signal_name`
- `OK` -> `OK`
- `WARN` -> `WARN`
- `ERROR` -> `FAULT`
- `STALE` -> `STALE`
- `DiagnosticStatus.message` -> `alert_message` for non-OK rows
- `DiagnosticStatus.values` -> `value` and `unit` when those keys are present
- `DiagnosticArray.header.stamp`, or receive time when unset, -> `timestamp`

## Doctor / Surgeon Camera Placeholder View

`doctor_view.launch.py` opens `config/doctor_camera_view.rviz` with RViz Image displays and the docked `WayBionic Surgeon Camera` placeholder panel. The configured topics are placeholders until Gianna/electrical select the real camera hardware and ROS topics; this package does not claim a camera pipeline exists yet.

The surgeon camera panel is available from RViz through:

```text
Panels -> Add Panel -> waybionic_rviz_plugins -> SurgeonCameraPanel
```

It shows the placeholder topic names and `Waiting for camera feed`; actual image rendering remains in RViz Image displays once camera topics exist.

## Optional AR4 Visualization Helper

Core package dependencies do not include Annin/AR4 packages. The generic launch is `engineer_view.launch.py`; it starts RViz only and does not require AR4 descriptions or passive joint publishers.

Optional AR4 coupling is isolated in:

- `launch/engineer_ar4_demo.launch.py`
- `config/engineer_ar4_demo.rviz`

That helper requires an AR4/Annin workspace with `annin_ar4_description`, `xacro`, `robot_state_publisher`, and `joint_state_publisher` available.

## Testing Performed

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py --show-args
ros2 launch waybionic_rviz_plugins doctor_view.launch.py --show-args
```

Optional AR4 helper, only in a workspace with AR4 dependencies:

```bash
ros2 launch waybionic_rviz_plugins engineer_ar4_demo.launch.py --show-args
```

The doctor view was also launched briefly under a timeout to verify RViz starts and loads the placeholder panel without plugin errors.

## PR Status

PR status could not be confirmed from this environment. `gh` is not available in PowerShell, and the WSL check did not return PR data. Use this file as the PR description source if the PR still needs to be opened.

## Known Limitations

- Meaningful live rows require Korede/backend to publish stable diagnostics data.
- Stale handling is currently a simple five-second freshness check.
- Doctor view is a placeholder layout only; no real camera hardware or pipeline is implied.
- The UI is monitoring-only and does not send motor commands.

## Next Steps

- Confirm final diagnostic signal names and value/unit conventions with backend.
- Connect live camera topic names after hardware selection.
- Add focused runtime validation once a diagnostics publisher is available.
