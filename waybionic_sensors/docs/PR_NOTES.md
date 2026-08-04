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

## What changed relative to the old IMU branch

| Old behaviour | Problem | Now |
|---------------|---------|-----|
| Synthetic quaternion published on `data_raw` | Presented generated data as a measurement | Raw topic sets `orientation_covariance[0] = -1`; synthetic orientation moved to `/waybionic/imu/data_demo`, off by default |
| Rotating TF always broadcast | Implied the sensor knows its own attitude | `publish_demo_tf`, default false; enabled only by `imu_demo.launch.py` |
| All covariances left at zero | Zero means "perfectly certain" to a consumer | Diagonal covariances from parameterised standard deviations, documented as placeholders |
| No `/diagnostics` output | Panel could not show IMU health | `imu.heartbeat` plus rate and telemetry rows at 2 Hz |
| One 120-line node doing everything | Serial work would have to be bolted into the publisher | Six modules: reading type, mock source, hardware boundary, message builder, diagnostics builder, node |
| Three metadata tests | No behavioural coverage | 83 tests including a runtime suite that spins the node |
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

Row-major 3x3 with `stddev^2` on the diagonal. Off-diagonals are zero because
the mock models the axes as uncorrelated, which is a stated assumption rather
than an unknown left blank.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `angular_velocity_stddev` | `0.01` | rad/s |
| `linear_acceleration_stddev` | `0.05` | m/s^2 |
| `orientation_stddev` | `0.05` | rad, demo topic only |

Plausible consumer-MEMS placeholders, exposed as parameters so datasheet values
can replace them without a code change. Tracked as question 14 in
`docs/HARDWARE_INTERFACE.md`.

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
angular_velocity_covariance: [0.0001, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0001]
linear_acceleration: {x: -0.0368..., y: -0.0153..., z: 9.80665}
linear_acceleration_covariance: [0.0025, ...]
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

83 tests, 0 failures.

| Suite | Count | Covers |
|-------|-------|--------|
| `test_imu_messages.py` | 15 | Frame, timestamp, orientation-unavailable marker, covariance diagonals and cross terms, demo message, demo TF |
| `test_imu_diagnostics.py` | 16 | Heartbeat OK/STALE, custom timeout, never-received, age units, rate WARN, telemetry units, absence of roll/pitch/yaw |
| `test_imu_publisher_node.py` | 14 | Runtime: rate follows the parameter, monotonic timestamps, frame IDs, demo defaults off, demo TF on request, heartbeat OK then STALE, live mode without hardware |
| `test_mock_source.py` | 13 | Determinism, gravity, amplitude bounds, stall behaviour, quaternion normalisation |
| `test_hardware_reader.py` | 9 | Interface surface, stub behaviour, a custom reader satisfying the boundary |
| `test_package_metadata.py` | 14 | Module separation, node delegation, launch defaults, docs, entry point, no invented protocol |
| `test_flake8.py`, `test_pep257.py` | 2 | Style and docstrings |

## Known limitations

- No physical IMU driver. Blocked on `docs/HARDWARE_INTERFACE.md`.
- Covariance values are documented placeholders, not measured noise.
- The demo orientation and demo TF are visualisation aids, not estimates.
- The `base_link` to `imu_link` offset in the demo TF is a placeholder 0.1 m, not
  a mounting claim.
