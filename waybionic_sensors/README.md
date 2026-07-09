# WayBionic Sensors

ROS 2 sensor publishers for the WayBionic ground station. Initial scope: accelerometer/gyroscope data as `sensor_msgs/msg/Imu`.

## Topic and Frame

| Item | Value |
|------|-------|
| Topic | `/waybionic/imu/data_raw` |
| Message | `sensor_msgs/msg/Imu` |
| Frame | `imu_link` |
| Parent TF | `base_link` |

## Quickstart (mock mode)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_sensors --symlink-install
source install/setup.bash
ros2 launch waybionic_sensors imu_publisher.launch.py
```

## RViz demo

```bash
ros2 launch waybionic_sensors imu_demo.launch.py
```

This starts the mock IMU publisher and opens RViz with a TF tree. ROS 2 Jazzy does not ship `rviz_default_plugins/Imu`, so orientation is verified through the `imu_link` frame relative to `base_link`.

## Inspect the topic

```bash
ros2 topic echo /waybionic/imu/data_raw
ros2 topic hz /waybionic/imu/data_raw
```

## Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_mock` | `true` | Publish synthetic changing IMU values |
| `topic` | `/waybionic/imu/data_raw` | Output topic |
| `frame_id` | `imu_link` | IMU frame |
| `parent_frame_id` | `base_link` | TF parent frame |
| `publish_rate_hz` | `50.0` | Publish rate |
| `serial_port` | `""` | Reserved for live hardware path |

Live hardware mode (`use_mock:=false`) is stubbed until the serial driver is wired. Mock mode is the supported development path for now.

## Follow-ups

- Wire live serial/hardware reader when the physical IMU is available.
- Optional: publish `imu.heartbeat` through `/diagnostics` for integration with `waybionic_rviz_plugins`.
