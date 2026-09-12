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
| No regression for the stall path | `test_stale_stall_marks_heartbeat_rate_and_telemetry` plus unit tests on the diagnostics builder. Mock stall latches so a later earlier-timestamp read cannot unstall |
| Placeholder stddev implied confidence | Raw/live gyro and accel covariances default to all-zero (ROS unknown). Positive `*_stddev` is opt-in for datasheet/calibration. Synthetic orientation covariance stays on the demo topic only |
| Docs should be beginner-readable | README and `IMU_CONTRACT.md` explain raw vs demo in plain English before the ROS field names |
| `ament_python` has no Jazzy rosdep key | Removed `<buildtool_depend>ament_python</buildtool_depend>`; retained `<build_type>ament_python</build_type>`. Strict `rosdep install` no longer needs `-r` or `--skip-keys ament_python` for this package |

## What changed relative to the old IMU branch

| Old behaviour | Problem | Now |
|---------------|---------|-----|
| Synthetic quaternion published on `data_raw` | Presented generated data as a measurement | Raw topic sets `orientation_covariance[0] = -1`; synthetic orientation moved to `/waybionic/imu/data_demo`, off by default |
| Rotating TF always broadcast | Implied the sensor knows its own attitude | `publish_demo_tf`, default false; enabled only by `imu_demo.launch.py` |
| Covariances all zero | Zero was misread as "perfectly certain" | Raw gyro/accel now stay unknown (all-zero) until a datasheet stddev is supplied; demo orientation covariance is synthetic and demo-only |
| No `/diagnostics` output | Panel could not show IMU health | `imu.heartbeat` plus rate and telemetry rows at 2 Hz |
| One 120-line node doing everything | Serial work would have to be bolted into the publisher | Six modules: reading type, mock source, hardware boundary, message builder, diagnostics builder, node |
| Three metadata tests | No behavioural coverage | 96 tests including a runtime suite that spins the node |
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

```text
average rate: 50.005
	min: 0.019s max: 0.021s std dev: 0.00026s window: 52
```

`ros2 topic echo /waybionic/imu/data_raw --once`:

```yaml
header:
  stamp: {sec: 1789120170, nanosec: 981566836}
  frame_id: imu_link
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: {x: -0.0568..., y: 0.0479..., z: -0.1136...}
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: {x: -0.0284..., y: 0.0192..., z: 9.80665}
linear_acceleration_covariance: [0.0, ...]
```

`ros2 topic hz /diagnostics` is 2 Hz. While streaming, heartbeat/rate/gyro/accel are OK.

Default launch topic list has `/waybionic/imu/data_raw` and `/diagnostics` only.
`imu_demo.launch.py launch_rviz:=false` adds `/waybionic/imu/data_demo` and `/tf`.
Demo orientation covariance is the placeholder `0.05^2 = 0.0025`; gyro/accel stay unknown.

Heartbeat after `mock_stall_after_sec:=2.0` (timeout 1.00 s):

```text
name: imu.heartbeat
message: No IMU sample for 4.14 s (timeout 1.00 s)
values: [{key: value, value: '4.14'}, {key: unit, value: s}]
name: imu.rate
message: Publishing at 0.0 Hz; IMU samples are stale
name: imu.angular_velocity
message: Gyroscope magnitude (stale)
name: imu.linear_acceleration
message: Accelerometer magnitude, including gravity (stale)
```

Heartbeat with `use_mock:=false`:

```text
name: imu.heartbeat
message: No IMU samples received from unconfigured IMU driver; awaiting sensor
         model, transport and packet format from electrical
values: [{key: value, value: never}, {key: unit, value: s}]
```

## Tests

```bash
colcon test --packages-select waybionic_sensors
colcon test-result --all --verbose
```

96 tests, 0 failures.

Full workspace on Ubuntu 24.04 / ROS 2 Jazzy / WSL2:

```bash
colcon build --symlink-install
colcon test
colcon test-result --all --verbose
```

4 packages finished. **137 tests, 0 errors, 0 failures, 0 skipped.**

| Suite | Count | Covers |
|-------|-------|--------|
| `test_imu_messages.py` | 17 | Frame, timestamp, orientation-unavailable marker, unknown vs datasheet covariance, demo message, demo TF |
| `test_imu_diagnostics.py` | 19 | Heartbeat OK/STALE, custom timeout, never-received, age units, rate WARN/STALE, telemetry OK then STALE, absence of roll/pitch/yaw |
| `test_imu_publisher_node.py` | 16 | Runtime: rate, timestamps, frame IDs, demo defaults, unknown covariance, heartbeat OK then STALE, stall marks all four signals, live mode without hardware |
| `test_mock_source.py` | 14 | Determinism, gravity, amplitude bounds, stall latch, quaternion normalisation |
| `test_hardware_reader.py` | 9 | Interface surface, stub behaviour, a custom reader satisfying the boundary |
| `test_package_metadata.py` | 19 | Module separation, node delegation, launch defaults, `rviz_imu_plugin` on `data_demo`, no ament_python rosdep, raw vs demo docs, hardware lifecycle, entry point, no invented protocol |
| `test_flake8.py`, `test_pep257.py` | 2 | Style and docstrings |

## Known limitations

- No physical IMU driver. Sensor model, transport, mounting, calibration, and
  noise values stay pending until Electrical answers
  `docs/HARDWARE_INTERFACE.md`.
- Covariance values on the raw topic stay unknown until electrical answers
  question 14. `orientation_stddev` is a demo-topic-only placeholder.
- The demo orientation and demo TF are visualisation aids, not estimates.
- The `base_link` to `imu_link` offset in the demo TF is a placeholder 0.1 m, not
  a mounting claim.

## Verification (Ubuntu 24.04 / ROS 2 Jazzy / WSL2)

Standard setup, no `-r` and no `--skip-keys`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths . --ignore-src -y
#All required rosdeps installed successfully
dpkg -s ros-jazzy-rviz-imu-plugin   # install ok; class rviz_imu_plugin/Imu
colcon build --symlink-install      # 4 packages finished
colcon test && colcon test-result --all
# Summary: 137 tests, 0 errors, 0 failures, 0 skipped
#   waybionic_sensors: 96 passed
```

Launch checks from that overlay:

| Command | Result |
|---------|--------|
| `ros2 launch waybionic_sensors imu_publisher.launch.py` | 151 raw msgs; `data_demo` absent; `orientation_covariance[0] = -1`; gyro/accel covariance all-zero; heartbeat/rate/gyro/accel OK |
| `ros2 launch waybionic_sensors imu_demo.launch.py launch_rviz:=false` | raw + 150 demo msgs; demo orientation covariance usable |
| `ros2 launch waybionic_sensors imu_demo.launch.py` | RViz started (`OpenGl version: 4.5`); config class `rviz_imu_plugin/Imu` on `/waybionic/imu/data_demo`; no plugin load error |
| `mock_stall_after_sec:=1.0 stale_timeout_sec:=0.5` | heartbeat, rate, angular_velocity, linear_acceleration all STALE (3); last magnitudes still shown |
| restart default publisher | all four rows recovered to OK |
| `use_mock:=false` | 0 sensor samples; heartbeat STALE |

### Environment notes (not part of the standard setup)

- A non-interactive WSL user session cannot type a sudo password. The first
  `rosdep install --from-paths . --ignore-src -y` therefore stopped on
  `sudo: a password is required`. Installing `ros-jazzy-rviz-imu-plugin` as
  root after `apt-get update` (the previous apt candidate 404'd on a stale
  index) made the same rosdep command exit 0 with no `-r` or skip key.
- Building the checkout under a Windows path that contains a space (`Uni Work`)
  makes `xacro` split the URDF argument in `waybionic_bringup`'s launch test.
  Full-workspace evidence above used a copy at `/home/khuzaymah/pr11_ws`.
