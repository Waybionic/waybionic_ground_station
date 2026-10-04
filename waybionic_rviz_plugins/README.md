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

```bash
ros2 run waybionic_rviz_plugins diagnostics_recorder.py \
  --duration 30 \
  --output-directory ~/diagnostics-sessions/fault-001 \
  --source-label mock \
  --tested-commit "$(git rev-parse HEAD)"
```

The output directory must not already exist. Inspect or replay a session in an
isolated ROS domain so it cannot interfere with an active robot or publisher:

```bash
ROS_DOMAIN_ID=42 ros2 bag info ~/diagnostics-sessions/fault-001/bag
ROS_DOMAIN_ID=42 ros2 bag play ~/diagnostics-sessions/fault-001/bag
ROS_DOMAIN_ID=42 ros2 topic echo /diagnostics
```

Label replayed data as `recorded/mock` when sharing it. Replay does not require
the original diagnostics publisher to be running. Generated bag directories
should remain outside Git; the repository ignores local recording output.

### Task 6: Simulated Drive-Health Sessions

Build and source the workspace from its root:

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to waybionic_bringup waybionic_rviz_plugins
source install/setup.bash
```

In terminal 1, start the host and mock drives in an isolated ROS domain with
the legacy timed faults disabled. Use SocketCAN when `vcan0` exists; if it is
unavailable, use the mock-only `udp_multicast` transport shown here. UDP
multicast is not for physical hardware.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch waybionic_bringup can_demo.launch.py simulate_faults:=false transport:=udp_multicast
```

In terminal 2, open the live Engineer View:

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false diagnostics_topic:=/diagnostics
```

For a healthy baseline, run this in terminal 3 to keep commands fresh, then
check `/diagnostics`. Expect `can.bus: Command Age` to be ACTIVE and all six
`can.bus: Joint N Health` rows to be OK with zero tracking error and no faults.
Keep the publisher running while recording the healthy session.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState "{name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

After confirming the six healthy rows, record the baseline in a new directory:

```bash
SESSION=~/diagnostics-sessions/task6-normal-healthy-$(date +%F)
ros2 run waybionic_rviz_plugins diagnostics_recorder.py --duration 15 \
  --output-directory "$SESSION" --source-label mock \
  --tested-commit "$(git rev-parse HEAD)"
ros2 bag info "$SESSION/bag"
```

Check the live rows with `ros2 topic echo /diagnostics --once`. Stop the
healthy command publisher with Ctrl+C before testing a fault. For each fault,
set the mock parameter for joint 1, then publish the listed command at 10 Hz
in terminal 3. Verify the expected state in `/diagnostics` and the Engineer
View before recording. Wait at least 2 seconds for `STALLED` and for the next
diagnostics update for `NOT_RESPONDING`.

| State | Trigger | Command stream |
|-------|---------|----------------|
| `FOLLOWING_ERROR` | `ros2 param set /mock_drives test_health_fault 1:following_error` | `ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState "{name: [joint_1], position: [0.2], velocity: [0.1]}"` |
| `STALLED` | `ros2 param set /mock_drives test_health_fault 1:stalled` | `ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState "{name: [joint_1], position: [0.05], velocity: [0.1]}"` |
| `NOT_RESPONDING` | `ros2 param set /mock_drives test_health_fault 1:not_responding` | `ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState "{name: [joint_1], position: [0.0], velocity: [0.0]}"` |
| `DISABLED` | `ros2 param set /mock_drives test_health_fault 1:disabled` | `ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState "{name: [joint_1], position: [0.0], velocity: [0.0]}"` |

Once the expected state is visible, record that fault in terminal 4. Use a
different output directory for each state; the directory must not exist yet.
After recording, inspect the bag. Then stop the command stream, clear the
fault, send joint 1 back to zero, and confirm it returns to OK before starting
the next case.

```bash
SESSION=~/diagnostics-sessions/task6-following-error-$(date +%F)
ros2 run waybionic_rviz_plugins diagnostics_recorder.py --duration 15 \
  --output-directory "$SESSION" --source-label mock \
  --tested-commit "$(git rev-parse HEAD)"
ros2 bag info "$SESSION/bag"

ros2 param set /mock_drives test_health_fault none
ros2 topic pub --once /joint_commands sensor_msgs/msg/JointState "{name: [joint_1], position: [0.0], velocity: [0.0]}"
```

Use unique session names such as `task6-normal-healthy-$(date +%F)`,
`task6-following-error-$(date +%F)`, `task6-stalled-$(date +%F)`,
`task6-not-responding-$(date +%F)`, and `task6-disabled-$(date +%F)`. The
recorder captures `/diagnostics`; `ros2 bag info` should list that topic as
`diagnostic_msgs/msg/DiagnosticArray` with a nonzero message count. Keep
recordings under `diagnostics-sessions/`, which is ignored by Git.

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

- Primary validation target is Ubuntu/WSL2 with ROS 2 Jazzy.
- A Mac/RoboStack RViz shutdown crash is not treated as a merge blocker unless it is reproduced on Ubuntu/WSL2.

## Related Docs

- `docs/DIAGNOSTICS_CONTRACT.md` — normalized diagnostic model and ROS mapping
- `docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` — how a real backend replaces the temporary publisher
- `docs/DIAGNOSTICS_SOURCE_LIFECYCLE.md` — mock/live source ownership and the handoff stress test
- `docs/GROUND_STATION_RVIZ_UI.md` — extended architecture notes
- `docs/PR_NOTES.md` — review summary and PR description source

## Follow-Ups

- Validate live `/diagnostics` against Korede/backend once stable publishing is available.
- Camera/doctor low-latency workflow will be handled in a separate PR.
