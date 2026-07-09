# IMU Integration PR Notes

Follow-up PR after diagnostics PR #2 merges. Branch: `feature/imu-rviz-integration`.

## Summary

Adds `waybionic_sensors`, a WayBionic-owned ROS 2 package that publishes accelerometer/gyroscope data as `sensor_msgs/msg/Imu` for benchtop development without hardware.

## Topic and Frame

| Item | Value |
|------|-------|
| Topic | `/waybionic/imu/data_raw` |
| Message | `sensor_msgs/msg/Imu` |
| Frame | `imu_link` |
| Parent TF | `base_link` |

## Launch Commands

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_sensors --symlink-install
source install/setup.bash
ros2 launch waybionic_sensors imu_publisher.launch.py
ros2 launch waybionic_sensors imu_demo.launch.py
```

Inspect topic:

```bash
ros2 topic echo /waybionic/imu/data_raw --once
ros2 topic hz /waybionic/imu/data_raw
```

## Modes

- `use_mock:=true` (default) — sinusoidal orientation, angular velocity, and acceleration
- `use_mock:=false` — reserved for live serial/hardware; stub until hardware path is wired

## Testing Performed (Ubuntu 24.04 / ROS 2 Jazzy / WSL2)

| Check | Result |
|-------|--------|
| `colcon build --packages-select waybionic_sensors --symlink-install` | Pass |
| `colcon test --packages-select waybionic_sensors` | Pass |
| `ros2 topic hz /waybionic/imu/data_raw` | Pass (~50 Hz) |
| `imu_demo.launch.py` TF visualization | Pass — `base_link` + `imu_link` in RViz |

ROS 2 Jazzy does not ship `rviz_default_plugins/Imu`; the demo uses TF frame visualization instead.

## Merge Recommendation

Ready to open as a separate PR against `main` **after PR #2 is merged**. Do not stack on the diagnostics PR branch for review.

## Out of Scope (bonus for later)

- Live serial/hardware IMU driver
- Publishing `imu.heartbeat` through `/diagnostics` for the diagnostics panel
