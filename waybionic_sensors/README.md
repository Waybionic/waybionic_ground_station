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
rosdep install --from-paths src --ignore-src -r -y
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

The mock stops after five seconds, and the row turns STALE once the sample age
passes `stale_timeout_sec`.

## What is measured and what is generated

The raw topic carries gyroscope and accelerometer data only. It marks
orientation unavailable with `orientation_covariance[0] = -1`, because an
accelerometer and a gyroscope cannot observe absolute heading. The synthetic
orientation lives on its own `data_demo` topic and is off by default, so nothing
can mistake it for a measurement.

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
and deliberately implements no serial protocol, because the sensor model,
transport, and packet format are unconfirmed. The open questions for electrical
are tracked in `docs/HARDWARE_INTERFACE.md`.

Running with `use_mock:=false` is still meaningful: no samples are published and
`imu.heartbeat` reports STALE, which is what a missing sensor should look like.

## Tests

```bash
colcon test --packages-select waybionic_sensors
colcon test-result --all --verbose
```

Coverage spans message semantics and covariance, mock generation and stalling,
diagnostics levels and units, the hardware boundary, package structure, and a
runtime suite that spins the node to check timestamps, frame IDs, rate, demo
defaults, and the heartbeat transitioning from OK to STALE.

## Related docs

- `docs/IMU_CONTRACT.md` — topics, units, covariance, and parameters
- `docs/HARDWARE_INTERFACE.md` — questions for electrical and how to add a driver
- `docs/PR_NOTES.md` — review notes, design rationale, and runtime evidence
- `waybionic_rviz_plugins/docs/DIAGNOSTICS_BACKEND_INTEGRATION.md` — the diagnostics contract this package publishes against
