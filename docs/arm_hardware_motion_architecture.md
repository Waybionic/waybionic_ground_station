# Arm Hardware and Motion Architecture

## Goal

Use the same four-joint arm model in RViz for both simulation and a connected
Arduino, while allowing the motion test to command the physical arm and report
failures instead of only animating a simulated pose.

The current model is `old_arm_prototype.urdf` with these joints:

- `base_yaw`
- `shoulder`
- `elbow`
- `wrist_roll`

## Current State

```text
MotionTestNode -- /joint_states --> robot_state_publisher --> RViz RobotModel
       ^
       |
RViz MotionTestPanel -- String RUN/HOME/STOP
```

The motion test currently publishes generated positions directly to
`/joint_states`. That is suitable for visualization, but it is not a hardware
control path: it represents commanded positions as if they were measured
positions.

There is currently no Arduino transport, encoder feedback path,
`ros2_control` hardware plugin, trajectory controller, or motion action in the
workspace.

The archived handoff provides a usable first transport contract:

- Arduino Uno R4 WiFi at 115200 baud over USB serial.
- Servo signals: D1 `base_yaw`, D2 `shoulder`, D3 `elbow`, D4 `wrist_roll`.
- Hold switch: D7 to ground using `INPUT_PULLUP`.
- Commands: `ID`, `MOVE,s1,s2,s3,s4,durationMs`, `JOG,servo,delta,durationMs`,
  and `HOLD`.
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
motion_test/action client --> trajectory controller --> Arduino
Arduino command status --> estimated /joint_states --> robot_state_publisher --> RViz
```

The launch file should select exactly one mode, for example with a
`hardware_mode` argument whose values are `simulation` and `arduino`.

## ROS Interfaces

These are the proposed stable interfaces between packages:

| Purpose | Interface |
| --- | --- |
| Joint position estimate or measurement | `/joint_states`, `sensor_msgs/msg/JointState` |
| Motion command | `/joint_trajectory_controller/follow_joint_trajectory`, `control_msgs/action/FollowJointTrajectory` |
| Optional lower-level command stream | `/joint_trajectory_controller/joint_trajectory`, `trajectory_msgs/msg/JointTrajectory` |
| Hardware and safety state | `/diagnostics`, `diagnostic_msgs/msg/DiagnosticArray` |

The motion test should become an action client. Actions provide acceptance,
feedback, completion, cancellation, and failure results, which the current
`RUN`/`HOME`/`STOP` string topic cannot provide.

The existing string topic can remain temporarily as a simulation-only adapter
while the action path is introduced.

The first bridge implementation is available as `waybionic_hardware` and
accepts the existing `/old_arm_motion_test/command` topic. It translates
`RUN`, `HOME`, and `STOP` into the firmware protocol, publishes the estimated
pose, and publishes connection/command status on `/diagnostics`.

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
- Refuse motion when disconnected, stale, out of range, or stopped.

The first implementation can use a small serial bridge if a full
`ros2_control` plugin is not yet ready. The public ROS interfaces should still
match the target design so the bridge can later be replaced without changing
RViz or the motion test.

### `waybionic_motion_test`

- Generate safe, bounded test trajectories.
- Send trajectories through the controller/action interface.
- Verify Arduino acknowledgment and arrival reports for each target.
- Verify measured feedback against each target once sensors are available.
- Abort on action failure, stale feedback, limit violation, or timeout.
- Keep a simulation implementation for development without hardware.

It must not publish synthetic `/joint_states` in physical mode.

### `waybionic_rviz_plugins`

- Keep the existing `RobotModel` visualization.
- Update `MotionTestPanel` to use the action interface.
- Show connection, current joint values, target, progress, and failure reason.
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
6. Require an explicit physical-mode launch argument so a development launch
   cannot move the arm accidentally.
7. Keep the first physical test at low speed with one joint at a time before
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

Simulation remains the default:

```text
ros2 launch waybionic_bringup ground_station.launch.py
```

Arduino mode requires an explicit serial port:

```text
ros2 launch waybionic_bringup ground_station.launch.py \
   hardware_mode:=arduino arduino_port:=/dev/ttyACM0
```

The bridge can be exercised without an Arduino:

```text
ros2 launch waybionic_bringup ground_station.launch.py \
   hardware_mode:=arduino arduino_dry_run:=true launch_rviz:=false
```

The first physical test should use the external servo-power switch, confirm
the arm is supported and clear, then use `HOME` before `RUN`. The current
bridge intentionally does not claim measured position feedback.

## Definition of Done

- RViz follows the Arduino command estimate in the first physical version and
   measured feedback once sensors are added.
- The motion test moves the physical arm through a bounded sequence.
- A disconnected or faulted Arduino prevents motion and is visible in the
  panel and `/diagnostics`.
- A stop request cancels the active command and leaves the arm in a known
  state.
- Simulation still works without an Arduino.
- No node other than the selected state source publishes `/joint_states`.

## Decisions Still Needed

- Whether future feedback will be servo command echo, potentiometer/encoder
   measurement, or both. The current firmware provides command status only.
- Whether the external servo-power switch is the formal emergency stop.
- Physical emergency-stop wiring and how its state reaches ROS.
- Whether the arm is controlled by hobby servos or motor drivers with a
  lower-level controller.