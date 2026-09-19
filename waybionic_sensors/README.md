# waybionic_sensors

IMU publishing and sensor health for the WayBionic ground station.

The package publishes `sensor_msgs/msg/Imu` and reports IMU health on
`/diagnostics` so the merged `waybionic_rviz_plugins` DiagnosticsPanel can show a
live `imu.heartbeat`. It runs entirely on a mock source today, and defines the
boundary a real driver will plug into once electrical confirms the sensor.

## Quickstart

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select waybionic_sensors --symlink-install
source install/setup.bash

ros2 launch waybionic_sensors imu_publisher.launch.py
```

Check the output:

```bash
ros2 topic hz /waybionic/imu/data_raw
ros2 topic echo /waybionic/imu/data_raw --once
ros2 topic echo /diagnostics --once
```

## RViz walkthrough

```bash
ros2 launch waybionic_sensors imu_demo.launch.py
```

This enables the synthetic orientation and the rotating demo TF so there is
something to look at. Both are off in `imu_publisher.launch.py`.

The demo config uses `rviz_imu_plugin/Imu` (Jazzy does not ship
`rviz_default_plugins/Imu`) and points the orientation display at
`/waybionic/imu/data_demo`, not `/waybionic/imu/data_raw`. `rosdep install`
from this package pulls `rviz_imu_plugin` automatically.

## Heartbeat in the diagnostics panel

```bash
# Terminal 1
ros2 launch waybionic_sensors imu_publisher.launch.py

# Terminal 2
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=false
```

The panel shows `imu.heartbeat` as OK with an age in seconds. To watch it go
stale without unplugging anything:

```bash
ros2 launch waybionic_sensors imu_publisher.launch.py mock_stall_after_sec:=5.0
```

The mock stops after five seconds. Once the sample age passes
`stale_timeout_sec`, `imu.heartbeat`, `imu.rate`, `imu.angular_velocity`, and
`imu.linear_acceleration` all report STALE. The last gyro and accel magnitudes
remain visible so the panel does not look like the sensor is still healthy.

## Raw vs demo data, in plain English

An IMU is a small sensor that measures two things:

- **How fast it is spinning** (angular velocity, rad/s)
- **How it is accelerating**, including gravity (linear acceleration, m/s^2)

It does **not** automatically know which way the robot is facing. Estimating
that facing direction is a separate step called fusion. Until a real fusion
source exists, this package keeps the two kinds of data on different topics so
nobody mixes them up:

| Topic | What it is | Default | Who should use it |
|-------|------------|---------|-------------------|
| `/waybionic/imu/data_raw` | Gyro + accelerometer measurements only. No facing direction. | Always on | Downstream code, diagnostics, a future fusion node |
| `/waybionic/imu/data_demo` | The same measurements **plus a made-up facing direction** so RViz can show a spinning box | Off, unless you run `imu_demo.launch.py` | Humans looking at RViz. Never control or localisation |

The RViz IMU display subscribes to `data_demo`, because that is the only topic
with an orientation to draw. `data_raw` marks orientation as unavailable
(`orientation_covariance[0] = -1`).

If we do not yet know how noisy the sensor is, raw gyro and accel covariance
stays all zeros. In ROS that means **unknown**, not "perfectly certain." Fake
noise numbers stay on the demo topic only, until electrical supplies a
datasheet or calibration value.

Full details, units, covariance conventions, and the parameter list are in
`docs/IMU_CONTRACT.md`.

## Package layout

```text
waybionic_sensors/
  waybionic_sensors/
    imu_reading.py         # Hardware-independent sample type: the boundary contract
    mock_source.py         # Synthetic sample generation, no ROS types
    hardware_reader.py     # Driver interface plus an unimplemented stub
    imu_messages.py        # sensor_msgs/Imu and TF construction, covariance rules
    imu_diagnostics.py     # DiagnosticArray construction, heartbeat and freshness
    imu_publisher_node.py  # ROS node that only wires the above together
  launch/
    imu_publisher.launch.py
    imu_demo.launch.py
  config/
    imu_demo.rviz
  docs/
    IMU_CONTRACT.md
    HARDWARE_INTERFACE.md
    PR_NOTES.md
  test/
```

Each stage is separately testable: sample generation, message construction,
diagnostics, and the hardware boundary have no dependency on one another.

## Hardware status

No physical IMU driver exists yet. `hardware_reader.py` defines the interface
and deliberately implements no serial protocol. Sensor model, transport,
mounting, calibration, and noise values stay pending until Electrical answers
the questions in `docs/HARDWARE_INTERFACE.md`.

Running with `use_mock:=false` is still meaningful: no samples are published and
`imu.heartbeat` reports STALE, which is what a missing sensor should look like.

**There is currently no physical IMU driver.** Do not treat mock or demo output
as a real sensor.

## Runtime handoff

For the integration runner (Malik). Source the workspace overlay first.
There is no physical IMU driver.

Verification record (not a physical-sensor claim):

- Historical runtime verification of the handoff commands: commit `e317df4`
  (Ubuntu 24.04.4 LTS / ROS 2 Jazzy / Python 3.12.3 / WSL2). Strict
  `rosdep install --from-paths . --ignore-src -y` (no `-r`, no skip keys);
  `ros-jazzy-rviz-imu-plugin` present; IMU suite 96 passed there, then 98
  after docs-guard tests.
- Merged PR #11 head: `4022337540209b8f2c4f1ce988f31537b8bd9a41` (merge
  commit `dbd4ff0bb5915b34a03794afdf978c625a8557c4` on `main`). Yassin
  approved; CI green; full workspace 139 tests, including 98 IMU tests.
  Physical IMU behavior remains unverified because no physical driver exists.
- Reader-validation follow-up: this branch, verified after the implementation
  with strict rosdep (no `-r`), full workspace build, **125** IMU tests and
  **166** workspace tests passing, plus mock / stall / unconfigured-live
  runtime checks. Same launch commands and PR #11 topic/frame/covariance/
  stall/lifecycle semantics. New coverage is the reader failure policy in
  `docs/IMU_CONTRACT.md`. Physical IMU behavior is still unverified.

Shutdown: Ctrl+C on the launch process. The node calls `stop()` on the
reader, then destroys itself. Mock and unconfigured live mode have no extra
processes.

### Raw

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_sensors imu_publisher.launch.py
```

| | |
|--|--|
| Topics | `/waybionic/imu/data_raw` (`sensor_msgs/msg/Imu`), `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`). `/waybionic/imu/data_demo` is absent. |
| Frames | `header.frame_id` = `imu_link`. No demo TF. |
| Orientation | Unavailable: identity quaternion placeholder, `orientation_covariance[0] = -1`. |
| Covariance | Gyro and accel 3x3 all zeros (ROS unknown). No datasheet stddev is configured. |
| Diagnostics | `imu.heartbeat`, `imu.rate`, `imu.angular_velocity`, `imu.linear_acceleration` all OK while streaming. |
| Check | `ros2 topic echo /waybionic/imu/data_raw --once` |

### Demo

```bash
ros2 launch waybionic_sensors imu_demo.launch.py
```

Headless (no GUI): add `launch_rviz:=false`.

| | |
|--|--|
| Topics | Raw as above, plus `/waybionic/imu/data_demo` (`sensor_msgs/msg/Imu`) and `/tf`. |
| Frames | IMU messages: `imu_link`. Demo TF parent: `base_link`. RViz fixed frame: `base_link`. |
| RViz | Same launch starts `rviz2 -d` `share/waybionic_sensors/config/imu_demo.rviz`. Display class `rviz_imu_plugin/Imu` named "IMU orientation (demo)", topic `/waybionic/imu/data_demo`. Expect a red box / axes wobbling on the grid. Raw semantics stay unchanged (`orientation_covariance[0] = -1` on `data_raw`). |
| Diagnostics | Same four rows, OK while the mock streams. |

### Stall / mock

```bash
ros2 launch waybionic_sensors imu_publisher.launch.py mock_stall_after_sec:=5.0
```

Faster bench check: `mock_stall_after_sec:=1.0 stale_timeout_sec:=0.5`.

| | |
|--|--|
| Publication | Mock stops producing samples after the delay and stays stopped (latched). Last gyro/accel magnitudes remain on the diagnostic rows. |
| Diagnostics | After `stale_timeout_sec`, all four rows go STALE (level 3). Heartbeat age is `now - last sample stamp` (source freshness, not a rewritten clock). |
| Recovery | Stop the launch (Ctrl+C) and start the default publisher again. All four rows return to OK and `data_raw` resumes. Restart is required; the latch does not un-stall in-process. |

Unconfigured live mode (no fake samples):

```bash
ros2 launch waybionic_sensors imu_publisher.launch.py use_mock:=false
```

Zero IMU samples. `imu.heartbeat` is STALE.

## Tests

```bash
colcon test --packages-select waybionic_sensors
colcon test-result --all --verbose
```

Run the suite after building; do not assume a fixed count from an older
commit. This follow-up: **125 tests, 0 failures** for `waybionic_sensors` and
**166 tests, 0 failures** for the full workspace (Ubuntu 24.04.4 / ROS 2 Jazzy
/ WSL2). PR #11 had merged with 98 IMU tests / 139 workspace tests; the added
coverage is reader validation and recovery (`None`, non-finite/malformed data,
out-of-order timestamps, `read()` exceptions, and rejected input that must not
refresh diagnostics). Coverage still includes message semantics and covariance,
mock generation and stalling, diagnostics levels and units, the hardware
boundary, package structure, flake8/pep257, and a runtime suite that spins the
node to check timestamps, frame IDs, rate, demo defaults, and the heartbeat
transitioning from OK to STALE.

There is no physical IMU driver and no Hamnah recording in this verification.

## Related docs

- `docs/IMU_CONTRACT.md` — topics, units, covariance, timestamps, and parameters
- `docs/HARDWARE_INTERFACE.md` — questions for electrical (owner / OPEN status) and how to add a driver
- `docs/PR_NOTES.md` — review notes, design rationale, and runtime evidence
- README **Runtime handoff** — commands for Malik to run raw / demo / stall without reading the code
- `waybionic_rviz_plugins/docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` — the diagnostics contract this package publishes against
