# IMU PR Notes

Review notes for the standalone `waybionic_sensors` pull request.

## Scope

This branch adds one package, `waybionic_sensors`, on top of current `main`. It
touches nothing else: no diagnostics panel changes, no CI or foundation changes.

The earlier `feature/imu-rviz-integration` branch was not rebased or merged
forward. It predated the merged foundation, so replaying it would have reverted
CI, `CONTRIBUTING.md`, and other files that landed in the meantime. Only the
`waybionic_sensors` directory was carried across, and the publisher was then
rewritten against the review feedback.

## PR #11 review fixes

Requested changes on the open IMU PR, without expanding scope:

| Request | Fix |
|---------|-----|
| Jazzy has no `rviz_default_plugins/Imu` | `imu_demo.rviz` uses `rviz_imu_plugin/Imu`; `package.xml` and `robostack.yaml` declare the dependency so `rosdep install` / macOS setup pull it |
| Display subscribed to `data_raw` | Orientation display now subscribes to `/waybionic/imu/data_demo` |
| Stale gyro/accel left OK | When samples age out, `imu.angular_velocity` and `imu.linear_acceleration` report STALE (last values still shown). Rate also goes STALE, not just WARN |
| No regression for the stall path | `test_stale_stall_marks_heartbeat_rate_and_telemetry` plus unit tests on the diagnostics builder |
| Placeholder stddev implied confidence | Raw/live gyro and accel covariances default to all-zero (ROS unknown). Positive `*_stddev` is opt-in for datasheet/calibration. Synthetic orientation covariance stays on the demo topic only |

## What changed relative to the old IMU branch

| Old behaviour | Problem | Now |
|---------------|---------|-----|
| Synthetic quaternion published on `data_raw` | Presented generated data as a measurement | Raw topic sets `orientation_covariance[0] = -1`; synthetic orientation moved to `/waybionic/imu/data_demo`, off by default |
| Rotating TF always broadcast | Implied the sensor knows its own attitude | `publish_demo_tf`, default false; enabled only by `imu_demo.launch.py` |
| Covariances all zero | Zero was misread as "perfectly certain" | Raw gyro/accel now stay unknown (all-zero) until a datasheet stddev is supplied; demo orientation covariance is synthetic and demo-only |
| No `/diagnostics` output | Panel could not show IMU health | `imu.heartbeat` plus rate and telemetry rows at 2 Hz |
| One 120-line node doing everything | Serial work would have to be bolted into the publisher | Six modules: reading type, mock source, hardware boundary, message builder, diagnostics builder, node |
| Three metadata tests | No behavioural coverage | 92 tests including a runtime suite that spins the node |
| `serial_port` parameter with no reader | Suggested a driver existed | Documented boundary plus a stub that makes the missing driver visible in diagnostics |

## Raw versus fused orientation

An accelerometer and a gyroscope cannot observe absolute heading. Publishing a
generated quaternion on the raw topic would let a future localisation or fusion
node consume invented data as if it were measured.

`/waybionic/imu/data_raw` therefore always sets `orientation_covariance[0] = -1`,
the standard `sensor_msgs/msg/Imu` marker for absent orientation, and leaves the
quaternion at identity as a placeholder. The synthetic orientation lives on
`/waybionic/imu/data_demo`, is off by default, and is named so it cannot be
mistaken for the real thing.

`imu.roll`, `imu.pitch`, and `imu.yaw` from the backend integration doc are
deliberately **not** published yet, for the same reason. They should appear when
a real fusion source exists.

## Covariance

`sensor_msgs/Imu` treats an all-zero matrix as unknown, not as perfect
certainty. Raw gyroscope and accelerometer covariances therefore default to
all zeros. A positive `angular_velocity_stddev` or
`linear_acceleration_stddev` fills `stddev^2` on the diagonal once electrical
supplies a datasheet or calibration value.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `angular_velocity_stddev` | `0.0` | rad/s; 0 = unknown |
| `linear_acceleration_stddev` | `0.0` | m/s^2; 0 = unknown |
| `orientation_stddev` | `0.05` | rad, demo topic only |

Tracked as question 14 in `docs/HARDWARE_INTERFACE.md`.

## Module boundaries

```text
mock_source.py  ─┐
                 ├─> ImuReading ─┬─> imu_messages.py   -> sensor_msgs/Imu, TF
hardware_reader.py ─┘            └─> imu_diagnostics.py -> DiagnosticArray
                                        imu_publisher_node.py wires them
```

`imu_reading.py` is the contract between sample producers and consumers. A real
driver implements `ImuHardwareReader` and returns `ImuReading` values; message
construction, covariance, diagnostics, and TF need no changes.

Two structural tests enforce this: the node must not construct `Imu()` or
`DiagnosticStatus` itself.

## Hardware handoff

No serial protocol is implemented, because the sensor model, transport, and
packet format are unconfirmed. `docs/HARDWARE_INTERFACE.md` holds 18 numbered
questions for electrical across sensor, transport, data format, and integration,
plus the list of known unknowns and the recipe for adding the driver later.

A structural test asserts no invented protocol crept in (`import serial`,
`baudrate`, `struct.unpack`).

Live mode is still meaningful today: with `use_mock:=false` the node publishes no
samples and `imu.heartbeat` reports STALE, which is the correct depiction of an
absent sensor.

## Runtime evidence (Ubuntu 24.04 / ROS 2 Jazzy / WSL2)

`ros2 topic hz /waybionic/imu/data_raw`:

```
average rate: 49.988
	min: 0.019s max: 0.021s std dev: 0.00030s window: 51
```

`ros2 topic echo /waybionic/imu/data_raw --once`:

```yaml
header:
  stamp: {sec: 1785808547, nanosec: 917613821}
  frame_id: imu_link
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: {x: -0.0736..., y: -0.0383..., z: -0.1473...}
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: {x: -0.0368..., y: -0.0153..., z: 9.80665}
linear_acceleration_covariance: [0.0, ...]
```

`ros2 topic hz /diagnostics` gives `average rate: 2.000`, comfortably above the
1 Hz requirement.

Heartbeat while streaming:

```
name: imu.heartbeat
message: IMU streaming from mock generator
values: [{key: value, value: '0.00'}, {key: unit, value: s}]
```

Heartbeat after `mock_stall_after_sec:=3.0`:

```
name: imu.heartbeat
message: No IMU sample for 6.64 s (timeout 1.00 s)
values: [{key: value, value: '6.64'}, {key: unit, value: s}]
```

Heartbeat with `use_mock:=false`:

```
name: imu.heartbeat
message: No IMU samples received from unconfigured IMU driver; awaiting sensor
         model, transport and packet format from electrical
values: [{key: value, value: never}, {key: unit, value: s}]
```

`/waybionic/imu/data_demo` is absent from `ros2 topic list` on a default launch,
confirming the demo output is off unless asked for.

## Tests

```bash
colcon test --packages-select waybionic_sensors
colcon test-result --all --verbose
```

92 tests, 0 failures.

| Suite | Count | Covers |
|-------|-------|--------|
| `test_imu_messages.py` | 17 | Frame, timestamp, orientation-unavailable marker, unknown vs datasheet covariance, demo message, demo TF |
| `test_imu_diagnostics.py` | 19 | Heartbeat OK/STALE, custom timeout, never-received, age units, rate WARN/STALE, telemetry OK then STALE, absence of roll/pitch/yaw |
| `test_imu_publisher_node.py` | 16 | Runtime: rate, timestamps, frame IDs, demo defaults, unknown covariance, heartbeat OK then STALE, stall marks all four signals, live mode without hardware |
| `test_mock_source.py` | 13 | Determinism, gravity, amplitude bounds, stall behaviour, quaternion normalisation |
| `test_hardware_reader.py` | 9 | Interface surface, stub behaviour, a custom reader satisfying the boundary |
| `test_package_metadata.py` | 16 | Module separation, node delegation, launch defaults, `rviz_imu_plugin` on `data_demo`, docs, entry point, no invented protocol |
| `test_flake8.py`, `test_pep257.py` | 2 | Style and docstrings |

## Known limitations

- No physical IMU driver. Blocked on `docs/HARDWARE_INTERFACE.md`.
- Covariance values on the raw topic stay unknown until electrical answers
  question 14. `orientation_stddev` is a demo-topic-only placeholder.
- The demo orientation and demo TF are visualisation aids, not estimates.
- The `base_link` to `imu_link` offset in the demo TF is a placeholder 0.1 m, not
  a mounting claim.
