# WayBionic RViz2 Diagnostics Plugin

RViz2-native diagnostics and monitoring UI for WayBionic. This package provides the engineer monitoring panel, mock/live diagnostics switching, and a temporary `/diagnostics` publisher for local validation. Two small operator panels sit beside it, the arm setup guide and joint control (see [Operator panels](#operator-panels)); they only read topics and call services, and never send motion.

It also provides `ground_station_rviz`, the ground station's RViz entry point.
It uses RViz's normal application initialization and then maximizes the window
with Qt, rather than entering RViz's panel-hiding fullscreen mode. It accepts
the normal RViz and ROS arguments and does not send motor commands.

Monitoring scope:

- No motor commands from this package.
- No robot control or safety-critical logic. The operator panels ask the drive node to hold or release a joint, or to zero, over services; the drive node decides and refuses while the arm is enabled.
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

## Operator panels

Both panels are in the ground station's right-hand dock, above the diagnostics panel
(`ground_station_rviz` stacks them; a plain `rviz2` leaves them wherever the layout puts them).

**Arm setup** (`waybionic_rviz_plugins/SetupPanel`) lists the start-up steps the `setup_guide`
node publishes on the latched `/arm/setup` topic (a DiagnosticArray of `setup.<step>` rows:
carrier, drives, controller, zero, ready). Each row's message is what to do next and its values
are `title`, `state` (`ok`, `todo`, `blocked` or `working`), `action` (`zero` or empty) and
`current`. When the guide asks for a zero, the panel offers a Zero button; after the operator
confirms the arm is in its zero pose it calls the drive node's `/sim_arm_drives/zero`
(`std_srvs/Trigger`) and shows the answer. Once every step is done the panel shrinks to one line.

**Joint control** (`waybionic_rviz_plugins/JointControlPanel`) shows a checkbox per joint,
J1 to J5 and the tool. It reads the latched `/arm/joint_control` topic (a DiagnosticArray with
one row per joint: `state` is `enabled`, `disabled` or `no_drive`, plus `drives` and `angle`)
and calls `/arm/<joint>/set_enabled` (`std_srvs/SetBool`) when a box is clicked. A joint that
is off is held by the drive node, which refuses a change while the arm is enabled; the panel
shows that refusal and leaves the box as the topic says. A joint with no drive is shown but
cannot be switched.

Both publishers repeat their message every 0.5 s, so three seconds of silence greys a panel out
as waiting. The parsing of each topic lives in a Qt-free unit (`joint_control_model.*`,
`setup_model.*`) with gtests that build fabricated messages.

## Package Layout

```text
waybionic_rviz_plugins/
  CMakeLists.txt              # Builds the shared RViz plugin library + tests
  package.xml                 # Generic deps only; no Annin/AR4 exec deps
  plugin_description.xml      # Registers DiagnosticsPanel, SetupPanel, JointControlPanel, MotionTestPanel
  scripts/
    temporary_diagnostics_publisher.py
    diagnostics_recorder.py
  include/waybionic_rviz_plugins/
    diagnostics_contract.hpp  # Normalized DiagnosticMessage model
    diagnostics_source.hpp    # DiagnosticsSource interface
    diagnostics_panel.hpp     # Engineer monitoring panel
    joint_control_model.hpp   # Qt-free parsing of /arm/joint_control
    joint_control_panel.hpp   # Joint on/off checkboxes
    mock_diagnostics_source.hpp
    ros_diagnostics_source.hpp
    setup_model.hpp           # Qt-free parsing of /arm/setup
    setup_panel.hpp           # Start-up steps and the Zero button
  src/
    diagnostics_panel.cpp
    joint_control_model.cpp
    joint_control_panel.cpp
    mock_diagnostics_source.cpp
    ros_diagnostics_source.cpp
    setup_model.cpp
    setup_panel.cpp
  config/
    engineer_monitoring_view.rviz   # Generic engineer layout (default)
  launch/
    engineer_view.launch.py
    temporary_diagnostics_publisher.launch.py
  test/
    test_package_metadata.py
    test_joint_control_model.cpp      # /arm/joint_control parsing
    test_ros_diagnostics_source.cpp   # Live/mock source handoff stress tests
    test_setup_model.cpp              # /arm/setup parsing
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
  --tested-commit <commit>
```

For a recording stopped manually, omit `--duration` and press `Ctrl+C` in the
recorder terminal after the session has run:

```bash
ros2 run waybionic_rviz_plugins diagnostics_recorder.py \
  --output-directory ~/diagnostics-sessions/fault-ctrl-c \
  --source-label mock \
  --tested-commit <commit>
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
with an active robot or publisher. These commands use ROS domain `71` and a
generic session path:

```bash
ROS_DOMAIN_ID=71 ros2 bag info ~/diagnostics-sessions/fault-001/bag
ROS_DOMAIN_ID=71 ros2 bag play ~/diagnostics-sessions/fault-001/bag
ROS_DOMAIN_ID=71 ros2 topic echo /diagnostics
```

Run the replay subscriber before `ros2 bag play`. The preserved
`DiagnosticArray.header.stamp` values can be historical, so a live consumer
may calculate them as stale even when the stored diagnostic level is `OK`.

Label replayed data as `recorded/mock` when sharing it. Replay does not require
the original diagnostics publisher to be running. Generated bag directories
should remain outside Git; the repository ignores local recording output.

### Task 7: Record Simulated Arm Drive-Health Sessions

Build and source the workspace from its root:

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to waybionic_bringup waybionic_rviz_plugins
source install/setup.bash
```

In terminal 1, launch the current arm and teleop stack with the in-process
simulated MKS drives. `joy_source:=none` and the manual topics below avoid
requiring an attached controller; this command does not use the legacy
`can_demo.launch.py`/`waybionic_control` mock path.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch waybionic_bringup ground_station.launch.py teleop:=true \
  drive_interface:=sim joy_source:=none launch_rviz:=false \
  use_joint_state_publisher_gui:=false
```

In terminal 2, watch diagnostics. Each simulated drive is initially enabled
and zeroed by the node; wait until all six `drive.<name>` rows show
`state: OK` and level `OK`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic echo /diagnostics
```

For deterministic command-based recordings (simulation only), stop only the
idle `xbox_teleop` process so it does not compete with the test publishers for
`/teleop_enabled` and `/joint_commands`. Keep the launch running; the
`sim_arm_drives` node remains active. Find the exact process ID and stop that
process:

```bash
ps -eo pid,args | grep '[x]box_teleop'
kill <xbox_teleop_pid>
```

In terminal 3, publish a release followed by a latched enable. The
`teleop_enabled` subscription requires transient-local durability. Keep the
enable publisher running; the command gate times out if enable updates stop.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic pub --qos-durability transient_local --once /teleop_enabled \
  std_msgs/msg/Bool "{data: false}"
ros2 topic pub --qos-durability transient_local --rate 10 /teleop_enabled \
  std_msgs/msg/Bool "{data: true}"
```

In terminal 4, publish the full joint command at 10 Hz. The ordered joint
names are the five arm joints followed by the simulated `tool_grip`.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4, joint_5, tool_grip], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Record each session in a unique directory under the ignored
`diagnostics-sessions/` directory at the repository root. For example:

```bash
SESSION=~/waybionic_ws/src/waybionic_ground_station/diagnostics-sessions/task7-normal-healthy-$(date +%Y%m%d-%H%M%S)
ros2 run waybionic_rviz_plugins diagnostics_recorder.py --duration 10 \
  --output-directory "$SESSION" --source-label mock \
  --tested-commit "$(git rev-parse HEAD)"
ros2 bag info "$SESSION/bag"
```

For every fault, check `/diagnostics` before recording and confirm the matching
`drive.base_yaw` `state` and ROS level (`NOT_RESPONDING`: `STALE`; other faults:
`ERROR`). Record each only after it is visible, using a new `SESSION` name and
the same recorder command. The `test_fault` parameter is available only when
`sim_arm_drives` is using `interface:=sim`; its format is
`<drive_name>:<fault>`. Supported values are `stalled`, `not_responding`,
`disabled`, and `none` to clear the active fault.

For `FOLLOWING_ERROR`, leave the fault parameter at `none` and command a
stationary target six degrees away from the current zero pose:

```bash
ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4, joint_5, tool_grip], position: [0.10472, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

After recording, command all joints back to zero and confirm `drive.base_yaw`
returns to `OK`.

For `STALLED`, set the simulated base drive fault and command motion toward a
four-degree target. Wait for the configured two-second stall window and the
next diagnostics report before recording:

```bash
ros2 param set /sim_arm_drives test_fault base_yaw:stalled
ros2 topic pub --rate 10 /joint_commands sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4, joint_5, tool_grip], position: [0.06981, 0.0, 0.0, 0.0, 0.0, 0.0], velocity: [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Clear it with `ros2 param set /sim_arm_drives test_fault none`, allow the
servo to catch up, command zero, and confirm `OK`.

For `NOT_RESPONDING`, keep the zero-pose command and teleop enable streams
running so the health row naturally ages past its reply timeout:

```bash
ros2 param set /sim_arm_drives test_fault base_yaw:not_responding
```

After recording, clear the fault. Stop the enable and command publishers,
publish `/teleop_enabled` false with transient-local durability, wait at least
one second for command quiet, then zero and re-arm the drives:

```bash
ros2 param set /sim_arm_drives test_fault none
ros2 topic pub --qos-durability transient_local --once /teleop_enabled \
  std_msgs/msg/Bool "{data: false}"
ros2 service call /sim_arm_drives/zero std_srvs/srv/Trigger "{}"
```

Wait for `OK`, then repeat the release/enable and joint-command publisher
steps before continuing.

For `DISABLED`, selecting the fault safely disarms teleop before releasing the
simulated base servo:

```bash
ros2 param set /sim_arm_drives test_fault base_yaw:disabled
```

Record once the row reports `DISABLED` / `ERROR`. Clear with
`ros2 param set /sim_arm_drives test_fault none`, publish teleop false once
with transient-local durability and then resume the true enable publisher,
command zero, and confirm `OK`.

Name directories `task7-normal-healthy-<timestamp>`,
`task7-following-error-<timestamp>`, `task7-stalled-<timestamp>`,
`task7-not-responding-<timestamp>`, and `task7-disabled-<timestamp>`.
`ros2 bag info "$SESSION/bag"` should show `/diagnostics` as
`diagnostic_msgs/msg/DiagnosticArray` and a nonzero message count. The session
folders and MCAP files remain outside Git tracking under the ignored
`diagnostics-sessions/` directory.

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

Validated on Ubuntu 24.04 under WSL2 with ROS 2 Jazzy. macOS/RoboStack and
native Windows are untested.

## Related Docs

- `docs/DIAGNOSTICS_CONTRACT.md` — normalized diagnostic model and ROS mapping
- `docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` — how a real backend replaces the temporary publisher
- `docs/DIAGNOSTICS_SOURCE_LIFECYCLE.md` — mock/live source ownership and the handoff stress test
- `docs/GROUND_STATION_RVIZ_UI.md` — extended architecture notes
- `docs/PR_NOTES.md` — review summary and PR description source

## Follow-Ups

- Validate live `/diagnostics` against Korede/backend once stable publishing is available.
- Camera/doctor low-latency workflow will be handled in a separate PR.
