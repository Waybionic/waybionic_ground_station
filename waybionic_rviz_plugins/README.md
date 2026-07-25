# WayBionic RViz2 Diagnostics Plugin

RViz2-native diagnostics and monitoring UI for WayBionic. This package is diagnostics-only: it provides the engineer monitoring panel, mock/live diagnostics switching, and a temporary `/diagnostics` publisher for local validation.

Monitoring-only scope:

- No motor commands from this package.
- No robot control or safety-critical logic.
- No camera or doctor/surgeon UI in this PR (handled separately later).
- Mock diagnostics for validation while backend `/diagnostics` is not ready.

## Quickstart

Use this package from any ROS 2 Jazzy colcon workspace:

```bash
cd <your_ros2_ws>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
```

Run package tests:

```bash
colcon test --packages-select waybionic_rviz_plugins
colcon test-result --verbose
```

## Package Layout

```text
waybionic_rviz_plugins/
  CMakeLists.txt              # Builds the shared RViz plugin library + tests
  package.xml                 # Generic deps only; no Annin/AR4 exec deps
  plugin_description.xml      # Registers DiagnosticsPanel
  scripts/
    temporary_diagnostics_publisher.py
  include/waybionic_rviz_plugins/
    diagnostics_contract.hpp  # Normalized DiagnosticMessage model
    diagnostics_source.hpp    # DiagnosticsSource interface
    diagnostics_panel.hpp     # Engineer monitoring panel
    mock_diagnostics_source.hpp
    ros_diagnostics_source.hpp
  src/
    diagnostics_panel.cpp
    mock_diagnostics_source.cpp
    ros_diagnostics_source.cpp
  config/
    engineer_monitoring_view.rviz   # Generic engineer layout (default)
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

### What each launch/config pair does

| Launch | RViz config | Purpose |
|--------|-------------|---------|
| `engineer_view.launch.py` | `engineer_monitoring_view.rviz` | Generic engineer monitoring |
| `temporary_diagnostics_publisher.launch.py` | — | Temporary `/diagnostics` demo publisher |

Core plugin dependencies are ROS/RViz/Qt only (`rclcpp`, `rviz_common`, `diagnostic_msgs`, etc.).

## RViz Panel

`DiagnosticsPanel` appears under **Panels → Add Panel → `waybionic_rviz_plugins`**.

It provides engineer monitoring: telemetry table, alerts, and mock/live diagnostics switching.

## Engineer View

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
```

Opens `config/engineer_monitoring_view.rviz` with the docked `WayBionic Diagnostics` panel. Does not launch Annin/AR4 packages or robot publishers.

### Mock vs live diagnostics

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=true
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

| Mode | Behavior |
|------|----------|
| Mock (`use_mock_diagnostics:=true`, default) | Uses `MockDiagnosticsSource`; `Mock Normal` / `Mock Fault` controls enabled |
| Live (`use_mock_diagnostics:=false`) | Uses `RosDiagnosticsSource`; subscribes to `diagnostic_msgs/msg/DiagnosticArray`; mock controls disabled |

Live mode with no publisher yet shows a stable waiting state (`Waiting for <topic> messages`) instead of fake mock data.

Launch arguments `use_mock_diagnostics` and `diagnostics_topic` take precedence over any saved RViz panel settings.

### Temporary diagnostics publisher

While Korede/backend publishing is unavailable, use the temporary demo publisher:

```bash
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=fault
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=stale
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=cycle
```

Then open live diagnostics in the engineer panel:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
```

Modes:

| Mode | Behavior |
|------|----------|
| `normal` | OK telemetry for board, motor, and IMU signals |
| `fault` | High board temperature + stale IMU heartbeat |
| `stale` | All sample signals published as STALE |
| `cycle` | Rotates through normal, fault, and stale every 5 seconds |

### Diagnostics architecture

```text
DiagnosticsSource
  MockDiagnosticsSource -> DiagnosticMessage -> DiagnosticsPanel
  RosDiagnosticsSource  -> DiagnosticMessage -> DiagnosticsPanel
```

`RosDiagnosticsSource` maps ROS diagnostic levels and fields into the internal `DiagnosticMessage` model before the Qt panel renders them. See `docs/DIAGNOSTICS_CONTRACT.md` for the full mapping Korede/backend should follow, and `docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` for backend replacement guidance.

## Platform Notes

- Primary validation target is Ubuntu/WSL2 with ROS 2 Jazzy.
- A Mac/RoboStack RViz shutdown crash is not treated as a merge blocker unless it is reproduced on Ubuntu/WSL2.

## Related Docs

- `docs/DIAGNOSTICS_CONTRACT.md` — normalized diagnostic model and ROS mapping
- `docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` — how a real backend replaces the temporary publisher
- `docs/GROUND_STATION_RVIZ_UI.md` — extended architecture notes
- `docs/PR_NOTES.md` — review summary and PR description source

## Follow-Ups

- Validate live `/diagnostics` against Korede/backend once stable publishing is available.
- Camera/doctor low-latency workflow will be handled in a separate PR.
