# Arm Hardware and Motion Architecture

## Goal

Use the same four-joint arm model in RViz for both simulation and a connected
Arduino, while allowing the motion test to command the physical arm and report
failures instead of only animating a simulated pose.

The opt-in old-arm model is `waybionic_old_arm.urdf.xacro` with these joints:

- `base_yaw`
- `shoulder`
- `elbow`
- `wrist_roll`

## Current State

```text
Simulation: MotionTestNode -- /old_arm/joint_states --> robot_state_publisher --> RViz

Hardware: RViz MotionTestPanel -- /old_arm_motion_test/command --> Arduino bridge
      Arduino bridge -- /old_arm/joint_states --> robot_state_publisher --> RViz
```

The simulation motion test publishes generated positions for visualization.
The Arduino bridge accepts the existing `RUN`, `HOME`, and `STOP` commands,
translates them to the serial protocol, and publishes an estimated joint state
from the accepted command timeline. It does not provide measured servo
feedback.

There is no `ros2_control` hardware plugin, trajectory controller, or motion
action in the workspace yet; those remain the longer-term control interface.

The archived handoff provides a usable first transport contract:

- Arduino Uno R4 WiFi at 115200 baud over USB serial.
- Servo signals: D1 `base_yaw`, D2 `shoulder`, D3 `elbow`, D4 `wrist_roll`.
- Hold switch: D7 to ground using `INPUT_PULLUP`.
- Commands: `ID`, `MOVE,s1,s2,s3,s4,durationMs`,
  `JOG,servo,delta,durationMs`, and `HOLD`.
- Responses: `READY,IK4,1`, `OK,MOVE`, `OK,JOG`, `OK,ARRIVED`, `OK,HOLD`,
  `OK,SWITCH HOLD`, and `ERROR,...`.

The firmware interpolates commands internally every 20 ms and has software
limits, but it does not stream measured servo angles. Its `OK,ARRIVED` response
means the command timeline completed, not that the mechanism was measured at
the target.

## Target Runtime Architecture

```text
                         command path
RViz MotionTestPanel or motion_test
              |
              | FollowJointTrajectory action
              v
joint_trajectory_controller
              |
              v
waybionic_hardware (ros2_control SystemInterface)
              |
              | serial/USB protocol
              v
Arduino + servo hardware
              |
              | command status, health, errors
              v
waybionic_hardware
              |
              +--> /joint_states --> robot_state_publisher --> RViz
              |
              +--> /diagnostics --> DiagnosticsPanel
```

The current Arduino firmware has no encoder or potentiometer feedback. It
reports command acceptance and arrival, but not measured servo positions. In
the first physical integration, the host bridge can publish an estimated
`/joint_states` pose from the accepted command timeline. RViz will then show
the commanded pose, not verified mechanical position. When real feedback is
added, measured state should replace that estimate.

## Runtime Modes

### Simulation

```text
motion_test simulation publisher --> /joint_states --> robot_state_publisher
```

This preserves the existing visual motion test. It must not run at the same
time as the physical hardware state publisher.

### Physical Hardware

```text
RViz MotionTestPanel --> /old_arm_motion_test/command --> waybionic_hardware
waybionic_hardware --> serial/USB --> Arduino
Arduino command status --> estimated /joint_states --> robot_state_publisher --> RViz
```

`old_arm.launch.py` selects exactly one mode with a
`hardware_mode` argument whose values are `simulation` and `arduino`.

## ROS Interfaces

These are the current and planned interfaces between packages:

| Purpose | Current interface | Planned interface |
| --- | --- | --- |
| Joint position estimate | `/old_arm/joint_states`, `sensor_msgs/msg/JointState` | Measured `/joint_states` feedback |
| Motion command | `/old_arm_motion_test/command`, `std_msgs/msg/String` | `FollowJointTrajectory` action |
| Hardware and safety state | `/diagnostics`, `diagnostic_msgs/msg/DiagnosticArray` | Controller/action feedback |

The motion test should eventually become an action client. Actions provide
acceptance, feedback, completion, cancellation, and failure results, which the
current `RUN`/`HOME`/`STOP` string topic cannot provide.

The existing string topic is currently used by both the RViz panel and the
Arduino bridge as the first integration interface.

The first bridge implementation is available as `waybionic_hardware` and
accepts the existing `/old_arm_motion_test/command` topic. It translates
`RUN`, `HOME`, and `STOP` into the firmware protocol, publishes the estimated
pose, and publishes connection/command status on `/diagnostics`.

The initial and HOME pose is the calibrated upright pose: physical servo
angles `[90.0, 35.0, 151.5, 27.5]`, corresponding to model joint angles
`[0, 90, 0, 0]` degrees. The manual joint-state stepper is opt-in so it does
not overwrite this upright motion-test state by publishing zero positions.

## Package Responsibilities

### `waybionic_description`

- Own the production URDF and joint limits.
- Add a `ros2_control` block for the four actuated joints.
- Keep the physical-to-model calibration in one documented place.

### New `waybionic_hardware`

- Own the Arduino transport and packet protocol.
- Convert model radians to calibrated servo commands.
- Convert accepted command state to model radians until real sensors exist.
- Replace estimated state with measured state when encoders or potentiometers
   are added.
- Publish hardware diagnostics and connection/watchdog faults.
- Refuse motion when disconnected, unhandshaken, faulted, stale, out of range,
  or stopped.

The current implementation is this small serial bridge. A full
`ros2_control` plugin remains a future replacement that should preserve the
RViz and motion-test interfaces.

### `waybionic_motion_test`

- Generate safe, bounded test trajectories.
- Publish bounded simulation trajectories.
- Keep physical command sequencing behind the bridge's `HOME` and `RUN` gates.
- Verify Arduino acknowledgment and arrival reports in the bridge.
- Add action and measured-feedback verification when those interfaces exist.
- Keep a simulation implementation for development without hardware.

It must not publish synthetic `/joint_states` in physical mode.

### `waybionic_rviz_plugins`

- Keep the existing `RobotModel` visualization.
- Keep the current `RUN`/`HOME`/`STOP` panel for the bridge adapter.
- Show the movement-test controls and diagnostics state.
- Update the panel to use the action interface when the controller is added.
- Keep emergency stop separate from normal test completion.

### `waybionic_bringup`

- Select simulation or Arduino mode.
- Start `robot_state_publisher` in both modes.
- Start the simulation publisher only in simulation mode.
- Start hardware and controllers only in Arduino mode.
- Prevent both state publishers from running together.

## Safety Rules

1. Start in a verified HOME pose before running a sequence.
2. Enforce URDF position and velocity limits before sending commands.
3. Require a hardware heartbeat and stop on a stale heartbeat.
4. Stop on serial disconnect, malformed feedback, over-current, or an
   Arduino-reported fault.
5. Treat the stop command as cancellation plus a hardware stop request; do not
   merely stop publishing messages.
6. A latched bridge fault ignores subsequent `READY` messages and can only be
   cleared by restarting the bridge; inspect the reported fault before doing so.
7. Require an explicit physical-mode launch argument so a development launch
   cannot move the arm accidentally.
8. Keep the first physical test at low speed with one joint at a time before
   running synchronized trajectories.

## Incremental Implementation Plan

1. **Define the contract:** confirm Arduino transport, feedback availability,
   servo calibration, joint limits, and the Arduino packet format.
2. **Separate modes:** add launch arguments and prevent the current simulated
   `/joint_states` publisher from starting in physical mode.
3. **Build the transport:** add `waybionic_hardware` with connection state,
   heartbeat, command conversion, command-state estimation, and diagnostics.
4. **Add standard control:** connect the transport to `ros2_control` and load a
   joint state broadcaster plus trajectory controller.
5. **Convert the test:** send bounded trajectories through the action and
   verify Arduino arrival status against each target. Add measured-feedback
   verification when sensors become available.
6. **Upgrade the panel:** display action and hardware state, and expose stop
   only when the controller reports the arm is connected.
7. **Validate progressively:** test conversion math, then a fake serial device,
   then RViz with recorded feedback, and only then the physical arm.

## Current Bringup Commands

Simulation remains the default old-arm mode:

```bash
ros2 launch waybionic_bringup old_arm.launch.py \
   hardware_mode:=simulation \
   use_joint_state_publisher_gui:=false \
   start_motion_test:=true
```

Arduino mode requires an explicit serial port:

```bash
ros2 launch waybionic_bringup old_arm.launch.py \
   hardware_mode:=arduino \
   arduino_port:=/dev/ttyACM0 \
   arduino_baud:=115200 \
   arduino_dry_run:=false \
   use_joint_state_publisher_gui:=false
```

The bridge can be exercised without an Arduino:

```bash
ros2 launch waybionic_bringup old_arm.launch.py \
   hardware_mode:=arduino \
   arduino_dry_run:=true \
   launch_rviz:=false \
   use_joint_state_publisher_gui:=false
```

The first physical test should use the external servo-power switch, confirm
the arm is supported and clear, then use `HOME` before `RUN`. The current
bridge intentionally does not claim measured position feedback.

## Current Validation

- Simulation movement is validated without an Arduino.
- The bridge has dry-run coverage and serial-bridge safety tests.
- `HOME` is required before `RUN`; `STOP`, disconnects, malformed responses,
   watchdog expiry, and latched faults stop or refuse motion.
- The Arduino command estimate is published on `/old_arm/joint_states`.
- Isolated launch tests and the full workspace test suite pass.
- Physical hardware validation requires an approved powered-arm test.

## Decisions Still Needed

- Whether future feedback will be servo command echo, potentiometer/encoder
   measurement, or both. The current firmware provides command status only.
- Whether the external servo-power switch is the formal emergency stop.
- Physical emergency-stop wiring and how its state reaches ROS.
- Whether the arm is controlled by hobby servos or motor drivers with a
  lower-level controller.