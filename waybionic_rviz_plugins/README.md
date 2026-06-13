# WayBionic RViz2 Diagnostics Plugin

RViz2-native diagnostics and monitoring UI for WayBionic. This package is a generic foundation plugin: it does not require Annin/AR4 packages for the default engineer or doctor views.

Monitoring-only scope:

- No motor commands from this package.
- No robot control or safety-critical logic.
- No camera driver or camera pipeline implementation.
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
ros2 launch waybionic_rviz_plugins doctor_view.launch.py
```

## Package Layout

```text
waybionic_rviz_plugins/
  CMakeLists.txt              # Builds the shared RViz plugin library
  package.xml                 # Generic deps only; no Annin/AR4 exec deps
  plugin_description.xml      # Registers DiagnosticsPanel + SurgeonCameraPanel
  include/waybionic_rviz_plugins/
    diagnostics_contract.hpp  # Normalized DiagnosticMessage model
    diagnostics_source.hpp    # DiagnosticsSource interface
    diagnostics_panel.hpp     # Engineer monitoring panel
    mock_diagnostics_source.hpp
    ros_diagnostics_source.hpp
    surgeon_camera_panel.hpp  # Doctor/surgeon placeholder panel
  src/
    diagnostics_panel.cpp
    mock_diagnostics_source.cpp
    ros_diagnostics_source.cpp
    surgeon_camera_panel.cpp
  config/
    engineer_monitoring_view.rviz   # Generic engineer layout (default)
    doctor_camera_view.rviz         # Doctor/surgeon placeholder layout
    engineer_ar4_demo.rviz          # Optional AR4 visualization helper
  launch/
    engineer_view.launch.py         # Generic engineer launch (default)
    doctor_view.launch.py           # Doctor/surgeon placeholder launch
    engineer_ar4_demo.launch.py     # Optional AR4 helper only
  docs/
    DIAGNOSTICS_CONTRACT.md         # Backend /diagnostics mapping contract
    GROUND_STATION_RVIZ_UI.md       # Extended architecture notes
    PR_NOTES.md                     # Review/PR summary
```

### What each launch/config pair does

| Launch | RViz config | Purpose |
|--------|-------------|---------|
| `engineer_view.launch.py` | `engineer_monitoring_view.rviz` | Generic engineer monitoring; no AR4 |
| `doctor_view.launch.py` | `doctor_camera_view.rviz` | Surgeon camera placeholder layout |
| `engineer_ar4_demo.launch.py` | `engineer_ar4_demo.rviz` | Optional passive AR4 robot viz |

Core plugin dependencies are ROS/RViz/Qt only (`rclcpp`, `rviz_common`, `diagnostic_msgs`, etc.). AR4/Annin packages are required only for the optional `engineer_ar4_demo` helper.

## RViz Panels

Both panels appear under **Panels → Add Panel → `waybionic_rviz_plugins`**:

| Panel | Role |
|-------|------|
| `DiagnosticsPanel` | Engineer monitoring: telemetry table, alerts, mock/live diagnostics |
| `SurgeonCameraPanel` | Doctor/surgeon placeholder: topic names, waiting state, scope notes |

Saved launch layouts are still the recommended way to open a clean engineer or doctor view. Panels can also be docked manually in one RViz session.

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

Panel settings are also saved in `engineer_monitoring_view.rviz` as `Use Mock Diagnostics` and `Diagnostics Topic`.

### Diagnostics architecture

```text
DiagnosticsSource
  MockDiagnosticsSource -> DiagnosticMessage -> DiagnosticsPanel
  RosDiagnosticsSource  -> DiagnosticMessage -> DiagnosticsPanel
```

`RosDiagnosticsSource` maps ROS diagnostic levels and fields into the internal `DiagnosticMessage` model before the Qt panel renders them. See `docs/DIAGNOSTICS_CONTRACT.md` for the full mapping Korede/backend should follow.

## Doctor / Surgeon Placeholder View

```bash
ros2 launch waybionic_rviz_plugins doctor_view.launch.py
```

Opens `config/doctor_camera_view.rviz` with:

- RViz Image displays for placeholder camera topics
- Docked `WayBionic Surgeon Camera` panel (`SurgeonCameraPanel`)

Placeholder topics until Gianna/electrical select actual camera hardware:

- `/camera/camera/color/image_raw` (primary)
- `/surgeon/secondary/image_raw` (secondary placeholder)

Both Image displays are configured in the RViz layout, but no camera feed will appear until a driver publishes those topics. The surgeon panel shows `Waiting for camera feed` and does not subscribe to images directly.

## Switching Between Views

Engineer and doctor are separate saved layouts. Options:

1. Close one window and launch the other launch file.
2. In RViz: **File → Open Config** and load the other layout from the installed package:
   - Engineer: `.../share/waybionic_rviz_plugins/config/engineer_monitoring_view.rviz`
   - Doctor: `.../share/waybionic_rviz_plugins/config/doctor_camera_view.rviz`
3. Dock `DiagnosticsPanel` or `SurgeonCameraPanel` manually via the Panels menu.

## Optional AR4 Visualization Helper

Not part of the default quickstart. Isolated for prototype review only:

```bash
ros2 launch waybionic_rviz_plugins engineer_ar4_demo.launch.py
ros2 launch waybionic_rviz_plugins engineer_ar4_demo.launch.py ar_model:=mk3 include_gripper:=True
```

Requires an AR4/Annin workspace with `annin_ar4_description`, `xacro`, `robot_state_publisher`, and `joint_state_publisher`. These are not core dependencies of `waybionic_rviz_plugins`.

## Mock-Only vs Live

**Mock-only today (synthetic, for UI validation):**

- `Mock Normal` / `Mock Fault` controls in the engineer panel
- Synthetic telemetry values in mock mode

**Live path (ready for backend):**

- `use_mock_diagnostics:=false`
- `diagnostics_topic:=/diagnostics` (or another agreed topic)
- `RosDiagnosticsSource` in `src/ros_diagnostics_source.cpp`

**Placeholder only (no real pipeline yet):**

- Doctor/surgeon camera topics and `SurgeonCameraPanel` waiting state

## Related Docs

- `docs/DIAGNOSTICS_CONTRACT.md` — normalized diagnostic model and ROS mapping
- `docs/GROUND_STATION_RVIZ_UI.md` — extended architecture and view notes
- `docs/PR_NOTES.md` — review summary, testing, and PR description source

## Follow-Ups

- Test engineer layout against Harold's WayBionic placeholder robot once `origin/rebuild/waybionic-foundation` is merge-ready.
- Connect live camera topic names after hardware selection.
- Validate live `/diagnostics` once Korede/backend publishes stable data.
