# WayBionic RViz2 Ground Station UI

This package contains the WayBionic RViz2-native diagnostics plugin and monitoring layouts. It keeps the operator interface inside RViz2 and avoids modifying RViz source code.

## Scope

This is a monitoring-first UI package.

- It does not send motor commands.
- It does not implement robot control.
- It does not implement safety-critical logic.
- It does not implement camera streaming drivers.
- It does not require Annin/AR4 packages for the generic engineer or doctor views.

## Build

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
```

## Views

### Engineer Monitoring View

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
```

This opens RViz2 with `config/engineer_monitoring_view.rviz`. The layout includes a docked `WayBionic Diagnostics` panel and generic RViz displays. It does not launch AR4 robot description packages, passive joint publishers, hardware drivers, or motor command nodes.

Diagnostics source modes:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=true
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

- Mock mode uses `MockDiagnosticsSource` and keeps the `Mock Normal` and `Mock Fault` validation controls working.
- Live mode uses `RosDiagnosticsSource`, subscribes to the configured diagnostics topic, and disables mock controls.

### Doctor / Surgeon Placeholder View

```bash
ros2 launch waybionic_rviz_plugins doctor_view.launch.py
```

This opens RViz2 with `config/doctor_camera_view.rviz`. The layout is intentionally camera-focused and avoids engineering telemetry clutter.

The layout includes RViz Image displays and the docked `WayBionic Surgeon Camera` panel. Current camera topics are placeholders until Gianna/electrical select the actual camera hardware and ROS topics:

- `/camera/camera/color/image_raw`
- `/surgeon/secondary/image_raw` disabled by default

The `SurgeonCameraPanel` is a placeholder/status panel only. It shows the configured topics, waits for a future camera feed, and does not start camera nodes or claim a real camera pipeline exists.

### Optional AR4 Visualization Helper

```bash
ros2 launch waybionic_rviz_plugins engineer_ar4_demo.launch.py
```

This launches passive AR4 robot visualization using `annin_ar4_description`, `xacro`, `robot_state_publisher`, and `joint_state_publisher`, then opens `config/engineer_ar4_demo.rviz`. These are optional visualization dependencies, not core plugin dependencies.

## Architecture

```text
Doctor view:
  doctor_view.launch.py -> doctor_camera_view.rviz -> RViz Image displays
    -> WayBionic Surgeon Camera panel

Generic engineer view:
  engineer_view.launch.py -> engineer_monitoring_view.rviz
    -> WayBionic Diagnostics panel

Optional AR4 visualization helper:
  engineer_ar4_demo.launch.py -> engineer_ar4_demo.rviz
    -> AR4 robot_description + passive joint states

Diagnostics flow:
  DiagnosticsSource
    -> MockDiagnosticsSource -> DiagnosticMessage -> DiagnosticsPanel
    -> RosDiagnosticsSource  -> DiagnosticMessage -> DiagnosticsPanel
```

RViz panel plugins:

```text
Panels -> Add Panel -> waybionic_rviz_plugins
  -> DiagnosticsPanel
  -> SurgeonCameraPanel
```

## Live Data Integration

`RosDiagnosticsSource` subscribes to `/diagnostics` by default, or another topic passed as `diagnostics_topic:=<topic>`, with `diagnostic_msgs/msg/DiagnosticArray`. It maps each ROS diagnostic status into `DiagnosticMessage`, including status level, name, timestamp, value/unit fields, and alert message. See `DIAGNOSTICS_CONTRACT.md` for the exact mapping.

The panel remains monitoring-only in both mock and live modes.

## Package Contents

```text
waybionic_rviz_plugins/
  include/waybionic_rviz_plugins/
    diagnostics_contract.hpp
    diagnostics_panel.hpp
    diagnostics_source.hpp
    mock_diagnostics_source.hpp
    ros_diagnostics_source.hpp
    surgeon_camera_panel.hpp
  src/
    diagnostics_panel.cpp
    mock_diagnostics_source.cpp
    ros_diagnostics_source.cpp
    surgeon_camera_panel.cpp
  config/
    engineer_monitoring_view.rviz
    engineer_ar4_demo.rviz
    doctor_camera_view.rviz
  launch/
    engineer_view.launch.py
    engineer_ar4_demo.launch.py
    doctor_view.launch.py
  docs/
    DIAGNOSTICS_CONTRACT.md
    GROUND_STATION_RVIZ_UI.md
    PR_NOTES.md
```
