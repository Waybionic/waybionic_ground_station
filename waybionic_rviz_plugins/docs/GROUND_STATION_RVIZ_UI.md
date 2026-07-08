# WayBionic RViz2 Ground Station UI

This package contains the WayBionic RViz2-native diagnostics plugin and engineer monitoring layout. It keeps the operator interface inside RViz2 and avoids modifying RViz source code.

This PR is diagnostics-only. Camera/doctor placeholder work was removed and will be handled later as a separate low-latency/VR/camera workflow.

## Scope

This is a monitoring-first UI package.

- It does not send motor commands.
- It does not implement robot control.
- It does not implement safety-critical logic.
- It does not include camera or doctor/surgeon UI.
- It does not require Annin/AR4 packages for the generic engineer view.

## Build

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
colcon test --packages-select waybionic_rviz_plugins
colcon test-result --verbose
```

## Engineer Monitoring View

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

## Temporary Diagnostics Publisher

A temporary demo publisher is included for local `/diagnostics` validation while Korede/backend publishing is unavailable:

```bash
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=cycle
```

Supported modes: `normal`, `fault`, `stale`, `cycle`.

Sample signals: `board.temperature`, `motor.current`, `imu.roll`, `imu.pitch`, `imu.yaw`, `imu.heartbeat`.

## Architecture

```text
Generic engineer view:
  engineer_view.launch.py -> engineer_monitoring_view.rviz
    -> WayBionic Diagnostics panel

Temporary backend demo:
  temporary_diagnostics_publisher.launch.py -> /diagnostics

Diagnostics flow:
  DiagnosticsSource
    -> MockDiagnosticsSource -> DiagnosticMessage -> DiagnosticsPanel
    -> RosDiagnosticsSource  -> DiagnosticMessage -> DiagnosticsPanel
```

RViz panel plugin:

```text
Panels -> Add Panel -> waybionic_rviz_plugins -> DiagnosticsPanel
```

## Live Data Integration

`RosDiagnosticsSource` subscribes to `/diagnostics` by default, or another topic passed as `diagnostics_topic:=<topic>`, with `diagnostic_msgs/msg/DiagnosticArray`. It maps each ROS diagnostic status into `DiagnosticMessage`, including status level, name, timestamp, value/unit fields, and alert message. See `DIAGNOSTICS_CONTRACT.md` for the exact mapping.

The panel remains monitoring-only in both mock and live modes.

## Platform Notes

- Primary validation target is Ubuntu/WSL2 with ROS 2 Jazzy.
- A Mac/RoboStack RViz shutdown crash is not treated as a merge blocker unless it is reproduced on Ubuntu/WSL2.

## Package Contents

```text
waybionic_rviz_plugins/
  include/waybionic_rviz_plugins/
    diagnostics_contract.hpp
    diagnostics_panel.hpp
    diagnostics_source.hpp
    mock_diagnostics_source.hpp
    ros_diagnostics_source.hpp
  src/
    diagnostics_panel.cpp
    mock_diagnostics_source.cpp
    ros_diagnostics_source.cpp
  scripts/
    temporary_diagnostics_publisher.py
  config/
    engineer_monitoring_view.rviz
  launch/
    engineer_view.launch.py
    temporary_diagnostics_publisher.launch.py
  test/
    test_package_metadata.py
  docs/
    DIAGNOSTICS_CONTRACT.md
    GROUND_STATION_RVIZ_UI.md
    PR_NOTES.md
```
