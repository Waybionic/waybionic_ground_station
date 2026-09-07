# Integration Smoke Test - Stable Baseline

Use this checklist to validate the stable baseline across active pull requests.
The primary validation target is Ubuntu or WSL2 with ROS 2 Jazzy. GUI checks
require a working graphical display.

## 1. Setup and full build/test

From the workspace root, install dependencies, build all packages, and run the
automated tests:

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
colcon test
colcon test-result --all --verbose
```

Expected result:

- `colcon build` completes successfully and creates `install/setup.bash`.
- `colcon test` completes without unexpected failures.
- `colcon test-result --all --verbose` reports passing package tests.
- The automated ground-station launch test runs headlessly; it checks process
	startup and allowed exit codes, not visible RViz output or topic contents.

Window/topic:

- No GUI window or diagnostics topic is expected from these commands.

Common failure:

- A missing `install/setup.bash` usually means the build failed or the current
	terminal was not sourced after building. Missing dependencies usually mean
	the `rosdep install` command was skipped.

## 2. Ground station launch

In a terminal with the workspace sourced, run:

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_bringup ground_station.launch.py
```

Expected result:

- RViz2 opens with the unified WayBionic configuration.
- The placeholder robot is visible with `base_link` as the fixed frame.
- The Joint State Publisher GUI opens in a separate window; moving its slider
	moves the placeholder arm.
- The `WayBionic Diagnostics` panel is visible and starts in mock mode.

Window/topic:

- Expected windows are RViz2 and Joint State Publisher GUI.
- No live `/diagnostics` publisher is started by default. Mock diagnostics are
	rendered inside the RViz panel.

Common failure:

- No window usually indicates that RViz or the Joint State Publisher GUI cannot
	access the graphical display. A missing model or RViz config causes the launch
	to fail before the nodes start.

## 3. Mock diagnostics

The engineer view can be tested independently of the robot visualization:

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
	use_mock_diagnostics:=true
```

Expected result:

- One RViz2 engineer-monitoring window opens.
- The docked `WayBionic Diagnostics` panel shows board temperature, motor
	current, and IMU telemetry rows.
- `Mock Normal` and `Mock Fault` controls are enabled.
- Normal mode shows healthy sample values. Fault mode shows high board
	temperature and a stale IMU heartbeat.

Window/topic:

- The window uses `engineer_monitoring_view.rviz`.
- No ROS diagnostics topic is required in mock mode.

Common failure:

- If the engineer view opens but the panel is missing, the RViz plugin may not
	have built or the workspace overlay may not be sourced.

## 4. Live diagnostics with temporary publisher

Open two terminals. Source ROS 2 and the workspace in both terminals.

### Terminal 1: start the publisher

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py
	mode:=cycle
```

Expected result:

- The terminal reports that `DiagnosticArray` messages are being published to
	`/diagnostics` at 2 Hz.
- `cycle` rotates through `normal`, `fault`, and `stale` every five seconds.

### Terminal 2: open the live engineer view

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_rviz_plugins engineer_view.launch.py
	use_mock_diagnostics:=false
```

Expected result:

- The engineer RViz2 window opens and live rows appear after the first message.
- Mock controls are disabled because the panel is subscribed to live data.
- Normal mode shows OK telemetry, fault mode shows high
	`board.temperature` and stale `imu.heartbeat`, and stale mode shows stale
	sample signals.

Verify the topic from either sourced terminal:

```bash
ros2 topic list
ros2 topic info /diagnostics
ros2 topic echo /diagnostics
```

Expected topic result:

- `/diagnostics` is listed with type
	`diagnostic_msgs/msg/DiagnosticArray`.
- `ros2 topic echo /diagnostics` prints changing diagnostic status messages.

Common failure:

- A waiting panel means the publisher is not running, the topic names differ,
	or one terminal was sourced from a different workspace/environment.
- `display.launch.py` only provides the older robot visualization and does not
	start the diagnostics panel or temporary publisher.
- The publisher accepts only `normal`, `fault`, `stale`, and `cycle` modes.

To test one fixed state instead of cycling, replace `mode:=cycle` with
`mode:=normal`, `mode:=fault`, or `mode:=stale`.