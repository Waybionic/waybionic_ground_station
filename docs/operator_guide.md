# Operator Guide

How to drive the arm with a controller, run the self-running demo, show the
doctor-view camera and connect the real arm. Set up and build the ground station
first, using [BuildInstructions.md](../BuildInstructions.md).

**Safety.** Everything here runs on simulated drives unless you choose a real drive
bus with `drive_interface`. Powered tests need a lead present, low speeds and the
hardware emergency stop within reach. The checks below have been tested with
simulated drives only, so they are not a safety case for the powered arm.

## Contents

- [Controllers](#controllers)
- [Self-running demo](#self-running-demo)
- [Doctor-view camera](#doctor-view-camera)
- [Real arm: MKS drives over CAN](#real-arm-mks-drives-over-can)

## Controllers

`teleop:=true` drives the arm with an Xbox or PS5 DualSense controller through
simulated CAN drives. Nothing reaches hardware unless you choose a drive bus, as
described in [Real arm](#real-arm-mks-drives-over-can). The controller mapping is in
`waybionic_teleop/config/xbox_teleop.yaml`, and the joint-to-drive map (MKS
SERVO42D/57D CAN IDs, the 26:1 and 3:1 gear ratios and the wrist differential) is
in `waybionic_teleop/config/arm_drives.yaml`.

### Windows: Xbox controller

1. Connect the controller, then start the ground station with teleop from the
   repository root:

   ```powershell
   docker compose run --rm --build --service-ports wslg-teleop
   ```

2. In a second PowerShell window, start the controller bridge from the repository
   root. It needs only Python 3, with no extra packages:

   ```powershell
   cd waybionic_teleop
   python -m waybionic_teleop.xinput_bridge
   ```

The bridge sends the controller state to the container on `127.0.0.1:47300/udp`.
Press **Ctrl+C** in each window to stop.

### Windows: PS5 DualSense

1. Pair the controller in Windows Bluetooth settings or connect it by USB, then
   start the same `wslg-teleop` command as for the Xbox controller.
2. In a second PowerShell window, use Python 3.13 or older, install Pygame once, and
   start the SDL bridge:

   ```powershell
   python -m pip install pygame==2.6.1
   cd waybionic_teleop
   python -m waybionic_teleop.sdl_bridge
   ```

Wait for `Controller ready` before pressing Options. The bridge waits for every
control to be released when the controller connects or reconnects.

- **Rumble over Bluetooth:** restart the bridge with
  `python -m waybionic_teleop.sdl_bridge --bluetooth-rumble`. This uses SDL
  extended reports, which can stop other DirectInput apps from using the controller
  until you turn it off and on. USB rumble doesn't need this option.
- **Several controllers:** the bridge uses the first one it recognizes; pick another
  with `--index 1`.
- **Not detected over Bluetooth:** try USB, and tell a lead which connection worked.

Run only one Windows bridge at a time. The
[self-running demo](#self-running-demo) opens the Xbox bridge, so close its window
before starting the SDL bridge.

### Linux

- **Native ROS:** with the controller plugged in, run
  `ros2 launch waybionic_bringup ground_station.launch.py teleop:=true`.
- **Docker:** plug the controller in by USB, then run
  `./scripts/linux-gui.sh --controller`. The container gets only the game
  controllers listed as `/dev/input/by-id/*-event-joystick`, not your keyboard or
  mouse. After unplugging or reconnecting the controller, stop the container and
  start it again. Bluetooth controllers aren't listed there, so use USB.

### Controls

| Input | Action |
| --- | --- |
| Start (Xbox Menu button, three lines) | Enable or disable; the arm starts disabled |
| B | Stop and hold the current pose |
| Y | Switch group: base (joints 1-3), upper (joints 2-5), Cartesian (tool tip) or incision |
| Base group: left stick | Base yaw (left/right) and shoulder (up/down) |
| Base group: right stick up/down | Elbow |
| Upper group: left stick | Shoulder (left/right) and elbow (up/down) |
| Upper group: right stick | Wrist roll (left/right) and wrist pitch (up/down) |
| Cartesian group: left stick | Tool tip left/right and forward/back, in straight lines |
| Cartesian group: right stick | Tool roll (left/right) and tool tip up/down |
| Cartesian group: LB (hold) | Move along one axis only: the stick direction pushed furthest |
| Cartesian group: D-pad left/right | Tilt the tool about its tip; the tip stays still |
| Incision group: left stick up/down | Insert or withdraw the tool along its own axis |
| Incision group: right stick up/down | Tilt the tool about the incision point |
| Incision group: right stick left/right | Tool roll |
| RT / LT | Close / open the placeholder end effector |
| D-pad up/down | Speed: 10, 25, 50 or 100% of 60 deg/s, or of 50 mm/s and 30 deg/s of tilt in the Cartesian and incision groups |
| A (hold) | Return to the zero pose (not in the incision group) |

On a PS5 DualSense, **Options** replaces Start, **Circle** replaces B, **Triangle**
replaces Y, **Cross** replaces A, and the triggers are L2/R2. The sticks, D-pad and
shoulder buttons are in the same places.

**Check a new controller** with each row above, and check that it rumbles when a
move stops at a limit. Then turn it off while teleop is enabled: the arm must hold
its pose and show disabled, and after reconnecting, teleop must stay disabled until
you release the controls and press Start again.

The RViz camera follows the tool as the arm moves. Drag to orbit and scroll to
zoom, or add `follow_camera:=false` to the launch command for a fixed view.

### How teleop behaves

**Starting and stopping.**

- Start is refused until the sticks are centred and the triggers and motion buttons
  are released. After Y changes group with motion held, the new group waits until
  the controls are neutral.
- Stick and trigger movement inside the 15% deadzone is ignored, so a trigger that
  doesn't fully return doesn't move the tool.
- When the bridge reports that the controller disconnected, every control is
  released, so the arm slows to a stop. After 0.5 s without controller input, teleop
  disables and holds the arm. Reconnect and press Start to continue.

**Limits.** When a move stops at a limit, the controller rumbles briefly, at most
once a second, and the RViz label turns red and names the cause, for example
`STOPPED-joint_3` or `OUT-OF-REACH`. If the drives stop the arm while teleop is
still enabled, for example after rejecting a target or losing encoder feedback, the
label shows `DRIVES-STOPPED` and the drives' reason; press B, then Start, to
continue. Rumble works with a controller plugged into the ground station and
through either Windows bridge.

**The drives.**

- The simulated drives accept complete, finite joint commands only after receiving
  a valid URDF and a fresh teleop enable, which teleop repeats on every update. If
  the enable stops for 0.5 s, or the host pauses past the 500 ms drive heartbeat,
  the drives stop, and teleop must be disabled and Start pressed again.
- A zero-velocity command, including B and the controller timeout, sends each
  moving drive an MKS F5 frame with zero speed and acceleration. An out-of-range
  encoder target stops every drive rather than only part of the arm.
- Drive speed is capped at 600 RPM (`max_rpm`); the MKS manual warns against
  immediate software stops above 1000 RPM. Each drive also runs no faster than 1.5
  times its commanded speed plus 1 RPM, and teleop never advances its targets by
  more than two updates at once, so after a pause the drives don't catch up faster
  than the teleop speed limits.
- The diagnostics panel shows the command gate, the teleop state, each joint, each
  drive's last CAN frame and the simulated bus load.

**The Cartesian group** computes its next setpoint with the same limit-aware solver
used for the current tool-tip pose. Diagonal moves are limited to the same 50 mm/s
as moves along one axis. The simulated drives are given speeds that make them arrive
together. Tilt moves the shoulder, elbow and wrist around the tool tip. Roll spins
about the tool axis; a downward-pointing blade keeps its heading in this model.

**The incision group** is for working through a keyhole. The incision point is
where the tip was when you selected the group, shown as a pink dot in RViz. The tool
always passes through that point, whether you insert, withdraw or tilt it, and the
return to the zero pose is off so the tool is never dragged sideways through the
incision. With five joints, the tool can only tilt in the arm's vertical plane.

- Y doesn't leave the incision group while the tool is inserted past the incision
  point; withdraw it first.
- Selecting the group again keeps the incision point while the tool axis still
  passes within 2 mm of it; otherwise it sets a new one at the tip.
- If the arm is re-enabled with the tool axis within 2 mm of the incision point,
  the point moves onto the axis instead of the arm moving. Further away, the group
  stops rather than pull the tool back: press Y, withdraw the tool in another group,
  then select the incision group again.

### Joint limits and collision checks

The arm URDF's joint limits are where its CAD parts touch, found by turning each
joint from the upright pose and checking the exact meshes
(`scripts/cad/exact_limits.py`):

| Joint | Limit, either way | Where it stops |
| --- | --- | --- |
| `joint_1` base yaw | 360 degrees | Nothing touches; the cabling isn't modelled |
| `joint_2` shoulder | 109.65 degrees | The straight arm meets the ground under the base |
| `joint_3` elbow | 148.93 degrees | The forearm meets the shoulder's motor mount |
| `joint_4` wrist pitch | 94.67 degrees | The tool mount meets the cap on the forearm, at the worst wrist roll |
| `joint_5` wrist roll | 360 degrees | Nothing touches; the cabling isn't modelled |

Don't power the motors until Mechanical confirms each joint's signed travel from
upright zero on the built arm, including any cable or gear stop.

Every group stops a move where any part of the arm would go below the table or into
the arm's own base and shoulder.

- Against the table, each link's lowest point comes from the convex hull of its CAD
  parts (`waybionic_hull` in the URDF), so the arm stops where its real parts reach
  the table.
- Against the base and shoulder, the checks use a box around each CAD part, or a
  cylinder where a part, or one side of it, is round about a joint axis: the base's
  lid, the tool mount and the lower half of the wrist housing. You can see them by
  ticking **Collision Enabled** on RViz's RobotModel display.
- Those shapes are too coarse where the forearm folds against the shoulder or the
  base, so there the elbow follows tables of its limits against the shoulder angle,
  measured from the CAD (`waybionic_fold` in the URDF, from
  `scripts/cad/joint_clearance.py`). The base's table holds at every yaw.
- The tool's box is a placeholder around the RViz jaws, which reach 30 mm past
  `tool_link`, until the real tool is modelled; a longer tool needs a longer box.
- The joint sliders use the same check. `joint_state_guard` moves the arm toward the
  sliders' pose, checking at least every half degree of each joint, stops at the
  last pose before any part would touch, and moves the sliders back to it.
- When a move stops, the diagnostics panel names the part, for example
  `At limit: forearm_link: table`.
- If the arm stands on a raised mount, set `table_height` in
  `waybionic_teleop/config/xbox_teleop.yaml` to the table's height above the bottom
  of the base. `collision_clearance` in the same file keeps a gap to the table, and
  `self_collision_clearance` to the base and shoulder. Both are 0, so the arm stops
  where its parts touch; raise them for a margin on the real arm.
- If the arm is already touching something, the only moves allowed are ones that
  back out of it. A move that presses any contact further in is refused, even if it
  would ease another contact at the same time.
- A `robot_description` without collision shapes for the base, shoulder, upper arm,
  forearm, wrist, wrist roll or tool leaves teleop disabled, and the diagnostics
  panel names the missing links.
- Reloading the description while teleop is enabled disables teleop and holds the
  arm, and the drives stop, so release Start and press it again.

After any change to the CAD, the limits or the collision shapes, run
`scripts/cad/check_collision_model.py`. It tests about 300,000 poses that the checks
call clear (random poses, poses with a joint at a limit, and the poses where moves
stop) against the exact CAD meshes, and fails if two parts or the tool's box
intersect, or a part goes below the ground.

### If the controller stops responding on Windows

If the arm stops responding or keeps moving after you release the sticks, and the
bridge's axis values stop changing, Windows has stopped updating the controller. The
bridge keeps sending the last state it read, so the controller can't stop the arm,
not even with B.

1. Press **Ctrl+C** in the bridge window. With no controller input, teleop disables
   and holds the arm after 0.5 s.
2. Turn the controller off and on, or unplug and replug it.
3. Start the bridge again and press Start.

Real drives need a stop that doesn't depend on the controller.

## Self-running demo

`autoplay:=true` plays a scripted demo as soon as the ground station is ready, and
again whenever the controller has been left alone for 30 seconds. The arm lifts,
draws a square and a vertical line with the tool tip, and tilts the tool about its
tip, with the path and a caption in RViz. Touching any button, stick or trigger
stops the demo and leaves teleop disabled; press Start to drive the arm yourself.
The demo runs only with simulated drives.

- **Windows:** double-click `scripts\windows-demo.cmd`. It starts Docker Desktop if
  needed, opens the controller bridge in its own window if Python 3 is installed,
  and starts the ground station. Press **Ctrl+C** in its window to stop.
- **Linux:**
  `ros2 launch waybionic_bringup ground_station.launch.py teleop:=true autoplay:=true`

## Doctor-view camera

Until the doctor's camera is chosen, a USB webcam stands in for it. `camera_bridge`
captures the webcam on the computer it's plugged into and sends JPEG frames to the
ground station, each stamped with its capture time. The ground station publishes
them on `/doctor_view/left/image_raw/compressed` with
`/doctor_view/left/camera_info`. The diagnostics panel's `camera.doctor_view` row
shows the frame rate and the delay from capture to arrival: mean, minimum, maximum
and jitter.

### Windows

1. Install OpenCV for the bridge with `python -m pip install opencv-python`. Use
   Python 3.13 or newer if you can: older versions read the clock in 15.6 ms steps
   on Windows, which adds up to 15.6 ms to the measured delay.
2. Start the ground station with the camera port, from the repository root:

   ```powershell
   docker compose run --rm --build -p 127.0.0.1:47310:47310 wslg ros2 launch waybionic_bringup ground_station.launch.py camera:=true camera_bind:=0.0.0.0
   ```

3. In a second PowerShell window, start the bridge from the repository root:

   ```powershell
   cd waybionic_camera
   python -m waybionic_camera.camera_bridge
   ```

4. Tick **Doctor View** in RViz's Displays panel.

### Linux

Run `ros2 launch waybionic_bringup ground_station.launch.py camera:=true`, then,
from `waybionic_camera`, run `python3 -m waybionic_camera.camera_bridge`. The
bridge needs `python3-opencv`.

### Bridge options

- `--camera 1` uses another camera.
- `--width 1280 --height 720` changes the resolution.
- On Windows, `--backend dshow` often opens a webcam faster.

### Measuring the delay

- **What the panel measures:** from when OpenCV hands the bridge a frame to when it
  reaches ROS. It leaves out the time inside the camera and on the display.
- **Full delay:** point the camera at a millisecond stopwatch on the same screen as
  RViz, and compare the live stopwatch with the one in the Doctor View.
- **Every frame:** add `camera_log:=` with a CSV path to log each frame's capture
  time, arrival time and delay. `frame` counts the frames that arrived, so a dropped
  frame shows as a longer gap in `captured_s`. With native ROS, any path works, such
  as `camera_log:=/tmp/camera.csv`. In Docker, `--rm` deletes the container's files
  when it stops, so log to a mounted Windows folder. This writes `camera.csv` to
  `camera-logs` in your user folder:

  ```powershell
  docker compose run --rm --build -p 127.0.0.1:47310:47310 -v "$HOME\camera-logs:/camera-logs" wslg ros2 launch waybionic_bringup ground_station.launch.py camera:=true camera_bind:=0.0.0.0 camera_log:=/camera-logs/camera.csv
  ```

- **Clocks:** the bridge stamps frames with the camera computer's clock, so the two
  clocks must agree. Docker Desktop keeps them within a few milliseconds. If the
  delay looks negative, or keeps growing after the PC sleeps, restart Docker
  Desktop.

## Real arm: MKS drives over CAN

The controller plugs into the computer, not the Arduino. The computer runs teleop,
the Cartesian moves, the joint limits and collision checks, the RViz twin and the
monitor, then sends MKS frames through the carrier board: a UNO R4 WiFi with a
TJA1051T transceiver running `waybionic_can/arduino/carrier_bridge`. The carrier
only passes frames between USB and the CAN bus, and reports its own status.

```text
controller -> computer (ROS 2) -> USB, 1 Mbaud -> carrier -> CAN, 1 Mbit/s -> 5 MKS drives
```

`waybionic_teleop/config/arm_drives.yaml` maps the joints to the drives: CAN IDs 1
to 5 (base yaw, shoulder, elbow, wrist left, wrist right), 26:1 cycloidal reducers
on the first three joints and 3:1 belts on the two wrist motors. Its `tool` drive
(CAN ID 6) is a placeholder with no hardware behind it, and the host waits for every
drive in the list, so **remove `tool` from `drives`** before driving the real arm.
On a real bus, every drive joint needs a URDF limit before the host holds or moves
anything (`unmodeled_joints` applies to simulation only).

**Before the drives are first powered:** Mechanical confirms each joint's signed
travel from the zero pose against the URDF limits, including any cable or gear stop,
and someone shows that the E-stop cuts power to the drives. Start with the motors
unloaded, and keep the E-stop within reach.

The computer running ROS needs the carrier or a USB CAN adapter. Docker Desktop on
Windows and macOS can't reach USB devices, so use native ROS on macOS
([RoboStack](../BuildInstructions.md#macos-native-ros-with-robostack)) or Linux, or
Docker on Linux with `--network host` for a SocketCAN adapter.

### On a Mac

Do the [macOS setup](../BuildInstructions.md#macos-native-ros-with-robostack) first,
then run these from the repository root.

1. **Flash the carrier** once, and again after carrier firmware changes. Plug in only
   the carrier, so its port is the only `/dev/cu.usbmodem*`, or set
   `WAYBIONIC_CARRIER=/dev/cu.usbmodem...`.

   ```bash
   brew install arduino-cli
   ./scripts/macos.sh flash
   ```

2. **Set up each drive once,** alone on the bus: give it its CAN ID (they all ship
   as 1), then switch it to 1 Mbit/s. The carrier keeps its first bit rate until it
   resets, so replug it, connect every drive, and check they all answer.

   ```bash
   ./scripts/macos.sh drives set-id 1 3            # for example the elbow
   ./scripts/macos.sh drives set-bitrate 3 1000000
   # after every drive is switched: replug the carrier
   ./scripts/macos.sh drives --bitrate 1000000 scan
   ```

3. **Start the ground station.** With `tool` removed from `drives` in
   `arm_drives.yaml`, put the arm in the zero pose (upright, as RViz shows it before
   anything moves), and run the command below. A controller plugged into the Mac, by
   USB or Bluetooth, drives the arm and its twin together.

   ```bash
   ./scripts/macos.sh arm
   ```

4. **Zero the drives** in a second Terminal, with teleop still disabled, then press
   Start on the controller:

   ```bash
   ./scripts/macos.sh run ros2 service call /sim_arm_drives/zero std_srvs/srv/Trigger
   ```

5. **Check each joint** at the slowest speed in the `upper` group. If a drive turns
   the wrong way, negate its `factors` in `arm_drives.yaml` and rebuild. Then move
   each joint a known angle, such as 10 degrees in RViz, and measure it on the arm; a
   different angle means its `gear_ratio` is wrong.

### Remote operator

The controller can stay with an operator on another computer. On the operator's
Windows computer, point the controller bridge at the Mac's address:

```powershell
cd waybionic_teleop
python -m waybionic_teleop.xinput_bridge --host 192.168.1.20
```

On the Mac, listen on the network:

```bash
./scripts/macos.sh arm joy_source:=udp joy_udp_bind:=0.0.0.0
```

The receiver takes controller packets from any computer that can reach its port, so
do this only on a private network.

### On Linux

With the carrier, set up the drives as in step 2 above, with `mks_setup`, then
launch:

```bash
ros2 run waybionic_teleop mks_setup --channel /dev/ttyACM0 scan
ros2 launch waybionic_bringup ground_station.launch.py teleop:=true drive_interface:=slcan drive_channel:=/dev/ttyACM0
```

With a SocketCAN adapter (candleLight firmware):

```bash
sudo ip link set can0 up type can bitrate 1000000
ros2 launch waybionic_bringup ground_station.launch.py teleop:=true drive_interface:=socketcan drive_channel:=can0
```

**Without hardware,** `mks_drive_sim` answers on a CAN interface the way the drives
do, so the host can be tested end to end over virtual CAN:

```bash
sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
ros2 run waybionic_teleop mks_drive_sim --interface socketcan --channel vcan0
```

In a second terminal, launch with `drive_interface:=socketcan drive_channel:=vcan0`
and zero as below. CI runs `waybionic_teleop/test/test_can_drives.py` over `vcan0`.

### Zeroing and stops

The encoders count from where the drives were powered on, so the host publishes no
joint states and moves nothing until the arm is zeroed. Put the arm in the zero pose
(the pose RViz shows before anything moves), leave teleop disabled, and run:

```bash
ros2 service call /sim_arm_drives/zero std_srvs/srv/Trigger
```

Then press Start. Real drives take commands through the same gate as the simulated
ones. Every drive stops, and the arm must be zeroed again, when:

- a drive stops answering, for example because the E-stop cut its power or the
  carrier was unplugged;
- a drive's encoder count jumps further than `max_rpm` allows between two readings,
  as after a brief power loss;
- a drive reports a failed move (as when its stall protection releases the motor)
  or an end-limit stop;
- a drive stays more than `following_error_counts` from its target for
  `following_error_ticks` ticks. Both are provisional values in `arm_drives.yaml`.

Once the carrier's E-stop sense is wired, pressing the E-stop also stops every drive
at once, and Start is refused until it's released. If the joint commands stop for
0.5 s, every drive stops where it is, and if the host stops, the drives' heartbeat
stops them.

The carrier answers a frame it can't pass on yet with BELL. The host sends a refused
move, stop or setup frame again on the next tick, and a refused zero while the zero
service waits. A refused stop frame is sent again on every tick, and Start is
refused until it has gone out. After any stop, disable teleop and press Start
again.

### Engineering monitor

With `teleop:=true`, the diagnostics panel shows live rows:

| Row | Shows |
| --- | --- |
| `controller.link` | Remote controller only: whether the bridge, the controller and its input are live, plus packet rate, loss, jitter, longest gap and network round trip; warns past 2% loss, a 100 ms gap or a 50 ms round trip |
| `teleop.input` | Whether controller input is arriving; teleop disables itself after 0.5 s without it |
| `arm.command_gate` | Whether the drives take teleop's commands, or why they stopped |
| `can.carrier` | The carrier's USB link (reopened every second if unplugged), E-stop and supply (once wired), CAN controller errors, and frames the carrier refused |
| `can.bus` | Frames per second and bus load |
| `drive.*` | Each drive's reply state and encoder, and its health: `OK`, `FOLLOWING_ERROR`, `STALLED`, `NOT_RESPONDING` or `DISABLED`, with its following error and reply age |
| `arm.*` | Each joint against its target, and whether the arm is zeroed |

To try each drive fault in simulation, see
[recording drive health sessions](../waybionic_rviz_plugins/README.md#task-7-record-simulated-arm-drive-health-sessions)
in the diagnostics guide.
