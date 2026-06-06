# WayBionic RViz2 Ground Station UI

This package contains the RViz2-native direction for the WayBionic ground station. It keeps the operator interface inside RViz2 and avoids modifying RViz source code.

## Scope

This is a monitoring-first UI package.

- It does not send motor commands.
- It does not implement robot control.
- It does not implement safety-critical logic.
- It does not implement camera streaming drivers.
- It does not replace the existing AR4 driver, MoveIt, Gazebo, or firmware packages.

## Workspace Setup (WSL / Ubuntu)

The expected development environment is ROS 2 Jazzy on Ubuntu 24.04, using an existing colcon workspace such as `~/ar4_ws`.

### 1. Add package to workspace

Symlink from a Windows checkout into WSL:

```bash
ln -s "/mnt/c/Users/<you>/OneDrive/Desktop/Uni Work/Clubs/WayBionic/waybionic_ground_station/waybionic_ground_station/waybionic_rviz_plugins" \
  ~/ar4_ws/src/waybionic_rviz_plugins
```

### 2. Build

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
```

### 3. Rebuild after code changes

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_rviz_plugins --symlink-install
source install/setup.bash
```

## Views

### Doctor / Surgeon Camera View

Launch:

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins doctor_view.launch.py
```

This opens RViz2 with `config/doctor_camera_view.rviz`. The layout is intentionally camera-focused and avoids engineering telemetry clutter.

Current camera topics are placeholders:

- `/camera/camera/color/image_raw`
- `/surgeon/secondary/image_raw` disabled by default

Update the RViz config when the real surgeon camera topics are available.

### Engineer View

Launch:

```bash
cd ~/ar4_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
```

This opens RViz2 with `config/engineer_monitoring_view.rviz`. The layout includes:

- RViz robot visualization using the AR4 description.
- A docked `WayBionic Diagnostics` panel.
- Mock diagnostics with Normal Demo and Fault Demo controls.
- Telemetry/live values and alerts derived from normalized diagnostic messages.

The engineer launch uses `robot_state_publisher` and `joint_state_publisher` for passive visualization. It does not start the hardware driver and does not send motor commands.

Useful launch arguments:

```bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py ar_model:=mk3 include_gripper:=True
```

## Architecture

```text
Doctor view:
  doctor_view.launch.py -> doctor_camera_view.rviz -> RViz Image displays

Engineer view:
  engineer_view.launch.py -> engineer_monitoring_view.rviz
    -> RobotModel / TF visualization
    -> WayBionic Diagnostics panel

Diagnostics flow (today):
  MockDiagnosticsSource -> DiagnosticMessage -> DiagnosticsPanel

Diagnostics flow (future):
  ROS2 /diagnostics subscriber -> DiagnosticMessage -> same DiagnosticsPanel
```

## Future Live Data Integration

### Diagnostics panel

1. Backend publishes to `/diagnostics` (or agreed topic).
2. A future subscriber converts `diagnostic_msgs/msg/DiagnosticArray` into `DiagnosticMessage`.
3. The panel UI stays unchanged and still renders tables/alerts from the normalized contract.
4. Add a launch parameter to switch between mock and live sources during bring-up.

See `DIAGNOSTICS_CONTRACT.md` for field definitions, examples, and ROS mapping guidance.

### Camera integration

1. Start the surgeon camera driver/node.
2. Confirm published `sensor_msgs/Image` topic names.
3. Edit `config/doctor_camera_view.rviz` to point Image displays at live topics.
4. Relaunch `doctor_view.launch.py`.

### Live robot visualization

1. Launch driver, Gazebo, or MoveIt so `/joint_states` and TF are live.
2. Use engineer RViz config without the passive `joint_state_publisher`, or include MoveIt with `rviz_config_file` override.
3. Keep diagnostics monitoring separate from control commands.

## Package Contents

```text
waybionic_rviz_plugins/
  include/waybionic_rviz_plugins/
    diagnostics_contract.hpp
    diagnostics_panel.hpp
    mock_diagnostics_source.hpp
  src/
    diagnostics_panel.cpp
    mock_diagnostics_source.cpp
  config/
    engineer_monitoring_view.rviz
    doctor_camera_view.rviz
  launch/
    engineer_view.launch.py
    doctor_view.launch.py
  docs/
    GROUND_STATION_RVIZ_UI.md
    DIAGNOSTICS_CONTRACT.md
```
