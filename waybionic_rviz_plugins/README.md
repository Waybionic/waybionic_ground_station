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
    diagnostics_recorder.py
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
    test_ros_diagnostics_source.cpp   # Live/mock source handoff stress tests
  docs/
    DIAGNOSTICS_CONTRACT.md
    DIAGNOSTICS_SOURCE_LIFECYCLE.md
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

### Recording a diagnostics session

The recorder saves the original ROS messages in a standard rosbag2 session and
writes `metadata.json` beside the bag. It records `/diagnostics` by default;
additional topics must be named explicitly.

From `~/waybionic_ws`, source the ROS 2 Jazzy and workspace environments first:

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

The installed recorder command is:

```bash
ros2 run waybionic_rviz_plugins diagnostics_recorder.py
```

For a timed recording, run the temporary publisher in one terminal and the
recorder in another:

```bash
# Terminal 1
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py \
  mode:=normal

# Terminal 2
ros2 run waybionic_rviz_plugins diagnostics_recorder.py \
  --duration 30 \
  --output-directory ~/diagnostics-sessions/fault-001 \
  --source-label mock \
  --tested-commit 0a2e9e5
```

For a recording stopped manually, omit `--duration` and press `Ctrl+C` in the
recorder terminal after the session has run:

```bash
ros2 run waybionic_rviz_plugins diagnostics_recorder.py \
  --output-directory ~/diagnostics-sessions/fault-ctrl-c \
  --source-label mock \
  --tested-commit 0a2e9e5
```

The output directory must not already exist. A successful session contains a
`bag/` directory (MCAP files and rosbag2 metadata) and a `metadata.json` file:

```text
<session>/
  bag/
    bag_0.mcap
    metadata.yaml
  metadata.json
```

The JSON records `topic`, `start_time`, `end_time`, `source_label`,
`message_count`, and optional `tested_commit`. The recorder returns `0` only
after finalization and a positive message count. It returns `2` when the output
directory already exists or cannot be created, and returns `1` for missing
dependencies, recorder-child failure, finalization failure, or an empty
recording. A recorder interrupted with `Ctrl+C` finalizes normally when the bag
contains messages. Existing session directories are never overwritten.

Inspect or replay a session in an isolated ROS domain so it cannot interfere
with an active robot or publisher. The following commands were verified with
the Task 5 fixture and ROS domain `71`:

```bash
ROS_DOMAIN_ID=71 ros2 bag info ~/diagnostics-sessions/task5-imu-stall/bag
ROS_DOMAIN_ID=71 ros2 bag play ~/diagnostics-sessions/task5-imu-stall/bag
ROS_DOMAIN_ID=71 ros2 topic echo /diagnostics
```

Run the replay subscriber before `ros2 bag play`. A ROS subscriber received all
17 Task 5 messages, including healthy IMU statuses before and after the stored
gap. The preserved `DiagnosticArray.header.stamp` values can be historical, so
a live consumer may calculate them as stale even when the stored diagnostic
level is `OK`.

Label replayed data as `recorded/mock` when sharing it. Replay does not require
the original diagnostics publisher to be running. Generated bag directories
should remain outside Git; the repository ignores local recording output.

For a complete temporary-publisher validation, record each mode in a separate
new directory. Use a duration long enough for the first rosbag2 startup on the
machine:

```bash
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py mode:=normal
ros2 run waybionic_rviz_plugins diagnostics_recorder.py --duration 30 \
  --output-directory ~/diagnostics-sessions/normal \
  --source-label mock
```

Repeat with `mode:=fault`, `mode:=stale`, and `mode:=cycle`. To preserve a
publisher message gap, stop the publisher with `Ctrl+C`, leave the recorder
running, restart the publisher, and then let the recorder finish. The recorder
does not insert samples during that gap.

Switching between mock and live replaces the active source while a ROS callback may still be running. `docs/DIAGNOSTICS_SOURCE_LIFECYCLE.md` documents the ownership rules that keep that handoff safe and the stress test that guards it.

## Platform Notes

- Recorder and MCAP validation was performed on Ubuntu 24.04 under WSL2 with
  ROS 2 Jazzy.
- The workspace was rebuilt before final validation. The missing `python3-can`
  dependency was installed through rosdep using the apt package provider.
- Final full-workspace validation passed: 237 tests, 0 errors, 0 failures, and
  0 skipped.
- macOS/RoboStack, native Windows, and other hosts are untested for recorder
  behavior and are not claimed as supported by this validation.

## Related Docs

- `docs/DIAGNOSTICS_CONTRACT.md` — normalized diagnostic model and ROS mapping
- `docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` — how a real backend replaces the temporary publisher
- `docs/DIAGNOSTICS_SOURCE_LIFECYCLE.md` — mock/live source ownership and the handoff stress test
- `docs/GROUND_STATION_RVIZ_UI.md` — extended architecture notes
- `docs/PR_NOTES.md` — review summary and PR description source

## Follow-Ups

- Validate live `/diagnostics` against Korede/backend once stable publishing is available.
- Camera/doctor low-latency workflow will be handled in a separate PR.
