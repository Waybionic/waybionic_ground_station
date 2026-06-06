# WayBionic Ground Station UI (RViz2)

RViz2-native ground station interface for WayBionic. This package provides two operator views inside RViz2:

- **Doctor / Surgeon Camera View** — camera feeds only, no engineering telemetry.
- **Engineer View** — robot visualization plus diagnostics, telemetry, live values, and alerts.

This is the V2 direction. The archived standalone PySide6 prototype lives in `ground_station_monitoring_ui/` on earlier branches.

## Scope

Monitoring-first only:

- No motor commands from this package.
- No robot control or safety-critical logic.
- No camera driver implementation in this sprint.
- No RViz source-code modifications.

## Prerequisites

- ROS 2 Jazzy on Ubuntu 24.04 (WSL or native Linux).
- An existing colcon workspace with the AR4 packages built, for example `~/ar4_ws`.
- RViz2 and Qt development packages (installed via `rosdep`).

Tested workflow uses WSL Ubuntu with an `ar4_ws` workspace that already contains `annin_ar4_moveit_config`, `annin_ar4_description`, and related AR4 packages.

## Add This Package To Your Workspace

If this repo is checked out on Windows and you build from WSL, symlink the package into your ROS workspace `src/`:

```bash
ln -s "/mnt/c/Users/<you>/OneDrive/Desktop/Uni Work/Clubs/WayBionic/waybionic_ground_station/waybionic_ground_station/waybionic_rviz_plugins" \
  ~/ar4_ws/src/waybionic_rviz_plugins
```

If you clone the repo directly inside WSL, copy or symlink `waybionic_rviz_plugins/` into `~/ar4_ws/src/` instead.

## Build

From your ROS workspace (same style as the existing AR4 demo):

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
```

After editing files in this package, rebuild with the same `colcon build` command and re-source `install/setup.bash`.

## Run

### Engineer monitoring view

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
```

Optional arguments:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py ar_model:=mk3 include_gripper:=True
```

This launches:

- `robot_state_publisher` and `joint_state_publisher` for passive robot visualization.
- RViz2 with `config/engineer_monitoring_view.rviz`.
- The docked `WayBionic Diagnostics` panel with mock Normal/Fault demo modes.

It does **not** start the hardware driver or send motor commands.

### Doctor / surgeon camera view

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins doctor_view.launch.py
```

This opens `config/doctor_camera_view.rviz` with image displays only.

## Demo Modes (Engineer Panel)

Use the panel buttons:

- **Normal Demo** — all mock signals `OK`, no active alerts.
- **Fault Demo** — high board temperature `FAULT` and IMU heartbeat `STALE`.

The panel refreshes about once per second from the current diagnostic source.

## Future Integration With Live Data

### Diagnostics (backend / Korede)

Today:

```text
MockDiagnosticsSource -> DiagnosticMessage -> DiagnosticsPanel
```

Future:

```text
ROS2 /diagnostics subscriber -> DiagnosticMessage -> same DiagnosticsPanel
```

Preferred topic: `/diagnostics` using `diagnostic_msgs/msg/DiagnosticArray` or an agreed WayBionic format.

Backend should publish stable signal names, status, timestamp, value, unit, and alert message. See `docs/DIAGNOSTICS_CONTRACT.md` for the full normalized contract and mapping guidance.

Integration steps (later):

1. Add a `ROS2DiagnosticsSubscriber` (or equivalent) that converts live messages into `DiagnosticMessage`.
2. Replace or supplement `MockDiagnosticsSource` inside the panel based on a parameter such as `use_mock_diagnostics`.
3. Keep all Qt table and alert rendering on the normalized contract — do not bind the UI directly to ROS message fields.
4. Test with mock data first, then connect live `/diagnostics` once the backend publishes it.

### Surgeon camera feeds

Current placeholder topics in `config/doctor_camera_view.rviz`:

- `/camera/camera/color/image_raw` (primary)
- `/surgeon/secondary/image_raw` (disabled placeholder)

When real camera drivers are available:

1. Confirm the live `sensor_msgs/Image` topic names.
2. Update `config/doctor_camera_view.rviz` Image display topics.
3. Launch `doctor_view.launch.py` while the camera node is running.
4. No panel plugin changes are required for basic camera viewing.

### Robot visualization with live joint states

The engineer launch currently uses `joint_state_publisher` for a passive demo pose. For live hardware or simulation:

1. Stop relying on the passive joint publisher.
2. Launch your existing arm stack (driver, Gazebo, or MoveIt) so `/joint_states` and TF are published.
3. Keep using `engineer_monitoring_view.rviz` or point RViz at the same config via `rviz_config_file`.
4. The diagnostics panel remains independent of arm control.

Example reference for an existing AR4 RViz demo:

```bash
cd ~/ar4_ws
source install/setup.bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

WayBionic engineer monitoring is separate from that launch, but can reuse the same workspace and robot description packages.

## Package Layout

```text
waybionic_rviz_plugins/
  README.md
  CMakeLists.txt
  package.xml
  plugin_description.xml
  include/waybionic_rviz_plugins/
  src/
  config/
  launch/
  docs/
```

## More Documentation

- `docs/GROUND_STATION_RVIZ_UI.md` — view details and architecture notes.
- `docs/DIAGNOSTICS_CONTRACT.md` — normalized diagnostic interface for backend integration.
