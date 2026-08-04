# IMU Message Contract

What `waybionic_sensors` publishes, in what units, and which parts are measured
versus generated for display.

## Topics

| Topic | Type | Default | Meaning |
|-------|------|---------|---------|
| `/waybionic/imu/data_raw` | `sensor_msgs/msg/Imu` | always on | Gyroscope and accelerometer only. Orientation marked unavailable. |
| `/waybionic/imu/data_demo` | `sensor_msgs/msg/Imu` | off | Synthetic orientation for visualisation. Not a measurement. |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | always on | Sensor health including `imu.heartbeat`. |
| `/tf` | `tf2_msgs/msg/TFMessage` | off | Rotating demo transform for `imu_link`. |

## Units and axes

REP-103 throughout: right-handed, x forward, y left, z up.

| Field | Unit |
|-------|------|
| `angular_velocity` | rad/s |
| `linear_acceleration` | m/s^2, including gravity |
| `orientation` | unit quaternion, demo topic only |
| Covariance diagonals | squared units of the field above |

A level, stationary sensor reports approximately `+9.80665` m/s^2 on
`linear_acceleration.z`, matching the `sensor_msgs/msg/Imu` convention that
acceleration is not gravity compensated.

## Raw versus fused orientation

An accelerometer and a gyroscope cannot observe absolute heading. Publishing a
generated quaternion on the raw topic would let any consumer, including a future
localisation node, treat invented data as a measurement.

The raw topic therefore always sets:

```
orientation             = (0, 0, 0, 1)   # placeholder, not a measurement
orientation_covariance[0] = -1.0         # "orientation not available"
```

`orientation_covariance[0] = -1` is the standard `sensor_msgs/msg/Imu` signal for
absent orientation, and well-behaved consumers check it before using the field.

When a real fusion source exists, either on-chip or from a filter node, it should
publish on a separate fused topic with a genuine covariance. Only then should
`imu.roll`, `imu.pitch`, and `imu.yaw` appear in `/diagnostics`.

The demo topic exists so RViz has something to show. It is off by default, named
`data_demo` rather than `data`, and documented here as generated.

## Covariance

Each covariance is a row-major 3x3 matrix carrying `stddev^2` on the diagonal.
Off-diagonal terms are zero because the mock models the axes as uncorrelated,
which is a stated assumption rather than a missing value.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `angular_velocity_stddev` | `0.01` | Assumed gyroscope noise, rad/s |
| `linear_acceleration_stddev` | `0.05` | Assumed accelerometer noise, m/s^2 |
| `orientation_stddev` | `0.05` | Assumed demo orientation noise, rad |

These are placeholders chosen to be plausible for a consumer MEMS IMU. They are
parameters precisely so they can be replaced with datasheet or bench values
without touching code. See `HARDWARE_INTERFACE.md` question 14.

## Diagnostics

Names, `value`/`unit` keys, and level mapping follow
`waybionic_rviz_plugins/docs/DIAGNOSTICS_BACKEND_INTEGRATION.md`, so the merged
DiagnosticsPanel renders these without IMU-specific code.

| Signal | Unit | Levels | Meaning |
|--------|------|--------|---------|
| `imu.heartbeat` | `s` | OK, STALE | Age of the newest sample. STALE past `stale_timeout_sec`, or when none ever arrived. |
| `imu.rate` | `Hz` | OK, WARN, STALE | Measured publish rate. WARN below 80% of the configured rate. |
| `imu.angular_velocity` | `rad/s` | OK | Gyroscope vector magnitude. |
| `imu.linear_acceleration` | `m/s^2` | OK | Accelerometer vector magnitude, including gravity. |

Published at `diagnostics_rate_hz`, default 2 Hz, which satisfies the
at-least-1-Hz requirement in issue #4.

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `use_mock` | `true` | Mock samples instead of hardware |
| `topic` | `/waybionic/imu/data_raw` | Raw output topic |
| `demo_orientation_topic` | `/waybionic/imu/data_demo` | Demo output topic |
| `diagnostics_topic` | `/diagnostics` | Health output topic |
| `frame_id` | `imu_link` | Measurement frame |
| `parent_frame_id` | `base_link` | Parent for the demo TF |
| `publish_rate_hz` | `50.0` | Sample rate |
| `diagnostics_rate_hz` | `2.0` | Health report rate |
| `stale_timeout_sec` | `1.0` | Heartbeat staleness threshold |
| `publish_demo_orientation` | `false` | Enable the demo topic |
| `publish_demo_tf` | `false` | Enable the demo TF |
| `angular_velocity_stddev` | `0.01` | Gyroscope noise assumption |
| `linear_acceleration_stddev` | `0.05` | Accelerometer noise assumption |
| `orientation_stddev` | `0.05` | Demo orientation noise assumption |
| `mock_stall_after_sec` | `0.0` | Stop the mock to demonstrate stale, 0 disables |
| `serial_port` | `''` | Reserved for the future driver |
