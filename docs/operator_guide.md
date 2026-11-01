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
| Base group: left stick | Base yaw (left/right; right turns joint 1 positive, counter-clockwise from above) and shoulder (up/down) |
| Base group: right stick up/down | Elbow (up lifts the elbow, joint 3 negative) |
| Upper group: left stick | Shoulder (left/right) and elbow (up/down; up lifts the elbow, joint 3 negative, as in the base group) |
| Upper group: right stick | Wrist roll (left/right; right turns joint 5 positive, counter-clockwise from above at the zero pose) and wrist pitch (up/down; up turns joint 4 negative) |
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

The current arm URDF has provisional joint limits, and the collision checks model
each link as a box. Don't change a wrist bound or power the motors until Mechanical
identifies the URDF joint and measures its signed travel from upright zero,
including any cable or gear stop.

Every group stops a move before any part of the arm comes within 10 mm of the table
or folds into the arm's own base. You can see the boxes by ticking
**Collision Enabled** on RViz's RobotModel display. The tool's box is a placeholder
around the RViz jaws, which reach 30 mm past `tool_link`, until the real tool is
modelled; a longer tool needs a longer box.

- When a move stops, the diagnostics panel names the part, for example
  `At limit: forearm_link: table`.
- If the arm stands on a raised mount, set `table_height` in
  `waybionic_teleop/config/xbox_teleop.yaml` to the table's height above the bottom
  of the base. `collision_clearance` in the same file sets the 10 mm margin.
- If the arm is already touching something, the only moves allowed are ones that
  back out of it. A move that presses any contact further in is refused, even if it
  would ease another contact at the same time.
- A `robot_description` without a collision box for the base, shoulder, upper arm,
  forearm, wrist, wrist roll or tool leaves teleop disabled, and the diagnostics
  panel names the missing links.
- Reloading the description while teleop is enabled disables teleop and holds the
  arm, and the drives stop, so release Start and press it again.

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
monitor, then sends MKS frames through the carrier board: a UNO R4 with an MCP2515 CAN
module running `waybionic_can/arduino/carrier_bridge_mcp2515`. The carrier only passes
frames between USB and the CAN bus, and reports its own status.

```text
controller -> computer (ROS 2) -> USB -> carrier -> CAN, 500 kbit/s -> 5 MKS drives
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
([RoboStack](../BuildInstructions.md#macos-native-ros-with-robostack)), the
[carrier relay on Windows](#on-windows) or Linux, or Docker on Linux with
`--network host` for a SocketCAN adapter.

### On a Mac

Do the [macOS setup](../BuildInstructions.md#macos-native-ros-with-robostack) first,
then run these from the repository root.

1. **Flash the carrier** once, and again after carrier firmware changes. Plug in only
   the carrier, so its port is the only `/dev/cu.usbmodem*`, or set
   `WAYBIONIC_CARRIER=/dev/cu.usbmodem...`. This uploads `carrier_bridge_mcp2515`,
   for the arm's MCP2515 module; `./scripts/macos.sh flash carrier_bridge` is only for
   a carrier using the R4's own CAN with a TJA1051T. The wrong sketch leaves every
   drive silent.

   ```bash
   brew install arduino-cli
   ./scripts/macos.sh flash
   ```

2. **Set up each drive once,** alone on the bus: give it its CAN ID (they all ship
   as 1). Leave them at the factory 500 kbit/s, the most the arm's MCP2515 carrier
   runs. Then connect every drive and check they all answer.

   ```bash
   ./scripts/macos.sh drives set-id 1 3            # for example the elbow
   ./scripts/macos.sh drives scan --ids 1 2 3 4 5
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

   `arm` saves the zero in `~/.waybionic/drive_zero.yaml`. While the drives stay
   powered, the next `arm` keeps it, so the arm needs no zero pose and no service
   call: wait for `Every drive kept its zero`, then press Start. After the drives lose
   power, zero again. See [Zeroing and stops](#zeroing-and-stops).

5. **Check each joint** at the slowest speed in the `upper` group. If a drive turns
   the wrong way, negate its `factors` in `arm_drives.yaml` and rebuild. Then move
   each joint a known angle, such as 10 degrees in RViz, and measure it on the arm; a
   different angle means its `gear_ratio` is wrong.

### Starting by double-click (Mac)

After the [macOS setup](../BuildInstructions.md#macos-native-ros-with-robostack), install
the app once from the repository root:

```bash
./scripts/macos.sh app
```

This puts **WayBionic Ground Station** in `~/Applications`; drag it to the Dock. The
copy in `packaging/macos/` also opens in place. Then, to start the arm:

1. Plug in the carrier, the controller (USB or Bluetooth) and power the drives.
2. Double-click the app. The first time, it offers to run the setup in Terminal, which
   takes 30 to 60 minutes; open the app again when that finishes.
3. If the carrier is not plugged in, a dialog says so: plug it in and choose *Try again*,
   or choose *Without the arm* to run the simulation. With several USB serial devices
   connected, it asks which one is the carrier.
4. It then starts the station exactly as `./scripts/macos.sh arm` does (the arm profile
   and the saved zero included), with RViz filling the screen. The Arm setup panel there
   walks through what is left: the controller, the zero pose, Start.

Closing the RViz window, or quitting the app from the Dock, stops the station the way
Ctrl+C does, so each drive gets its stop first. The station's output goes to
`~/Library/Logs/WayBionic/` (the last 10 sessions are kept), and if it stops unexpectedly
the app offers to open the log. The app is not signed: if you download the repository as
a zip instead of cloning it, macOS asks you to confirm opening it once (right-click, then
Open).

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

### On Windows

Docker Desktop can't open COM ports, so a small program on Windows, `carrier_relay`,
passes the carrier's port through `127.0.0.1:47320`, and the drive node in the container
connects to it as `socket://host.docker.internal:47320`. Besides
[Docker Desktop with WSLg](../BuildInstructions.md#windows-wslg), as for the other Windows
demos, it needs Python 3 on Windows and pyserial:

```powershell
python -m pip install pyserial
```

Flash the carrier first, as in step 1 of [On a Mac](#on-a-mac) (from a Mac, or with the
Arduino IDE).

**In one step:** plug in only the carrier, connect an Xbox controller and double-click
`scripts\windows-arm.cmd`. It starts Docker Desktop, the carrier bridge and the controller
bridge, then the ground station with `waybionic_teleop/config/arm_profile_bench.yaml`
(J1 to J5 driven, no tool). The first start builds the image, which takes a while. The Arm
setup panel in RViz then walks through what is left: put the arm in its zero pose, press
**Zero the arm**, then press Start on the controller. The zero is not kept between runs on
Windows, so do this every time. Closing RViz, or Ctrl+C in the script's window, stops the
ground station first, so each drive gets its stop, then the bridges.

**Step by step,** from the repository root. The bridge serves one connection at a time, so
let each command finish before the next. If `docker` is not recognized, see
[its troubleshooting](../BuildInstructions.md#docker-is-not-recognized-on-windows).

1. Start the bridge in its own PowerShell window and leave it running. It looks for the
   Arduino's COM port each time it opens the carrier, so plugging the carrier in after it
   starts, or replugging it under another COM number, is fine; with several Arduinos plugged
   in, pass `--port COM5`. Wait for `Carrier open; waiting for the ground station`: the
   drive node stops at start-up if nothing is listening.

   ```powershell
   cd waybionic_teleop
   python -m waybionic_teleop.carrier_relay
   ```

2. Scan the drives from a second window:

   ```powershell
   docker compose run --rm --build wslg-arm ros2 run waybionic_teleop mks_setup --channel socket://host.docker.internal:47320 scan --ids 1 2 3 4 5
   ```

3. Start the controller bridge in a third window, then the ground station:

   ```powershell
   cd waybionic_teleop
   python -m waybionic_teleop.xinput_bridge
   ```

   ```powershell
   docker compose run --rm --build --service-ports wslg-arm
   ```

Stopping the bridge or unplugging the carrier closes the connection, so the drive node
loses the carrier as it does when the USB cable is pulled on a Mac: every drive stops and
the arm must be zeroed again. The bridge listens only on `127.0.0.1`; keep it local,
because anything that connects to it can drive the arm.

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
(the pose RViz shows before anything moves), leave teleop disabled, and press **Zero the
arm** in the Arm setup panel (see [Start-up guide](#start-up-guide)), or run:

```bash
ros2 service call /sim_arm_drives/zero std_srvs/srv/Trigger
```

Then press Start. Real drives take commands through the same gate as the simulated
ones. Every drive stops, and the arm must be zeroed again, when:

- a drive stops answering, for example because the E-stop cut its power or the
  carrier was unplugged (unless the zero is saved and the drive kept it, as below);
- a drive's encoder count jumps further than `max_rpm` allows between two readings,
  as after a brief power loss;
- a drive reports a failed move (as when its stall protection releases the motor)
  or an end-limit stop;
- a drive stays more than `following_error_counts` from its target for
  `following_error_ticks` ticks. Both are provisional values in `arm_drives.yaml`.

**A saved zero.** With `drive_zero_state:=<file>` (`./scripts/macos.sh arm` uses
`~/.waybionic/drive_zero.yaml`), each zero is saved to the file once every drive has
reported it. A restart of the station, or a drive that answers again after it went
quiet, keeps the zero of each drive that shows it didn't lose power since. MKS firmware
V1.0.9 reports that in two reads, checked on the bench on all four drives:

- The zero status (3Bh) says whether the drive was zeroed since power-on. Power-on
  clears it, and the host's setup frames don't.
- The raw encoder (35h) counts from power-on, and a zero leaves it alone. So it minus
  the zeroed count (31h) stays the same from one zero until the drive loses power. A
  zero made elsewhere, such as by `mks_setup`, changes it.

A drive whose status shows no zero, or whose offset is more than 16 counts from the
saved one, isn't zeroed, and the log says why. A new zero, a failed move and a
following error delete the file.

A drive that turns more than `following_error_counts` after its stop, for
`following_error_ticks` ticks, also stops every drive, with teleop enabled or not. That
drive gets another stop. The arm keeps its zero, since that drive's encoder still counts
true.

When the stick reverses a joint, its drive stops and settles (typically 17 ms, at most
about 40 ms on the bench) before it turns back. MKS firmware V1.0.9 keeps a target sent
while the drive is reversing, and now and then one sent mid-stroke, and runs back to it, up
to 12°, at the next stop. So right behind every stop of a running move, including a
Start or B press, the drive also gets a 60 rpm move to where it's to stay. That's the end
of its last move, at most two ticks of travel ahead. The move is sent again every tick
for 0.25 s. On the bench this kept every stop within 0.1° of base travel. See
[F5h on firmware V1.0.9](../waybionic_can/arduino/README.md#f5h-on-firmware-v109).

Once the carrier's E-stop sense is wired, pressing the E-stop also stops every drive
at once, and Start is refused until it's released. If the joint commands stop for
0.5 s, every drive stops where it is, and if the host stops, the drives' heartbeat
stops them.

The carrier answers a frame it can't pass on yet with BELL. The host sends a refused
move, stop or setup frame again on the next tick, and a refused zero while the zero
service waits. A refused stop frame is sent again on every tick, and Start is
refused until it has gone out. After any stop, disable teleop and press Start
again.

### Start-up guide

The **Arm setup** panel (top of RViz's right-hand dock) lists what is left to do before the
arm can be operated, one thing at a time. The `setup_guide` node builds the list from the
same diagnostics the station already publishes, and also says each change in the terminal.

| Step | Done when | If it isn't |
| --- | --- | --- |
| Carrier | the carrier reports on USB and its E-stop is not pressed | plug in its USB-C cable; if it is plugged in but silent, replug it or flash it |
| Drives | every drive that is switched on answers | the panel names the drive and its CAN ID: check its power and wiring, or switch its joint off (below) |
| Controller | `/joy` input is arriving | connect an Xbox controller (USB or Bluetooth) and press a button |
| Zero | every drive that is switched on is zeroed or kept its saved zero | put the arm in its zero pose and press **Zero the arm** (it asks you to confirm) |
| Ready | teleop is enabled | press Start (the Menu button) on the controller |

The controller needs no other step: the guide shows it as connected once input arrives. Once
every step is done, including Start, and the arm has stayed ready for four seconds, the panel
closes so the other panels get its room. It opens again when something needs you: a step
before Ready is to do or blocked, such as the controller dropping out, a drive losing its zero
or the carrier being unplugged, or Ready is blocked because the drives stopped the arm. A step
that is only being checked does not open it, and neither does stopping teleop (B or Start),
which leaves only Start to press. The panel's close button, or Panels > SetupPanel, shows or
hides it by hand, and it is not hidden again until the arm next becomes ready.

If the guide itself is not running, the panel says so. A station started without the arm, such
as the plain `wslg` viewer, has no guide: the arm launchers (`scripts\windows-arm.cmd`,
`./scripts/macos.sh arm`) start it with `teleop:=true`. Nothing in the panel moves the arm.

### Switching joints off

The **Joint control** panel, below Arm setup, has a checkbox for each joint (J1 to J5 and the
tool). Unchecking one switches that joint off:

- The drive stays powered and holds where it is. It is not released, so the joint does not
  fall or move by itself.
- The sticks and triggers that were mapped to it do nothing, so the controller no longer
  vibrates when you push a stick that needs a joint you don't want. The Cartesian and
  incision groups need every joint, so they are left out of the group cycle while any joint
  is off or unfitted.
- A drive whose joints are all off may be faulty or missing: it no longer stops the arm,
  keeps the zero service from succeeding or keeps the arm from being enabled. The two wrist
  drives move both wrist joints together, so switching off only J4 or only J5 still needs both
  drives, which then hold that joint; switch off both to ignore a wrist drive.

A change is only taken while teleop is stopped and no joint command has arrived for a second,
as for zeroing. Otherwise the panel shows why, and the box stays as it was. Turning a joint
back on needs its drive answering and zeroed before the arm can be enabled again, so after
fitting or repairing a drive, put the arm in the zero pose and zero it.

The state is not saved: after a restart every fitted joint is on. A joint that has no drive
at all, such as the tool on the bench arm, is shown as *no drive* and cannot be switched
here. It comes from the arm profile (`~/.waybionic/arm_profile.yaml` on a Mac,
`waybionic_teleop/config/arm_profile_bench.yaml` for the Windows launcher):

```yaml
# Bench arm: J1 to J5 have a drive (CAN IDs 1 to 5), and there is no tool.
fitted_drives: [base_yaw, shoulder, elbow, wrist_left, wrist_right]
locked_joints: {tool_grip: 0.0}
```

To fit another drive later, give it its CAN ID (`./scripts/macos.sh drives set-id 1 6` with
only that drive on the bus), add it to `fitted_drives`, delete its joint from
`locked_joints`, restart the station and zero. Its joint then has a checkbox like the
others. A drive that isn't wired is left out of `fitted_drives` and its joints are locked, as
the shoulder's were before it was wired: `locked_joints: {joint_2: 0.0, tool_grip: 0.0}`.

Without the panel, from a terminal:

```bash
ros2 service call /arm/joint_3/set_enabled std_srvs/srv/SetBool "{data: false}"
ros2 topic echo /arm/joint_control
```

### Powered hand-guiding

Powered hand-guiding is not enabled yet. A stopped drive holds position; disabling
teleop does not release it or turn a push into a movement command.

The intended workflow is an explicitly selected Hand Guide mode: a deliberate push
starts bounded assistance, then release and stillness for a provisional two seconds
capture the new holding position. This changes the holding target, not the calibrated
zero. Stop must leave the mode disarmed rather than let another push restart it.

The existing-hardware approach is being evaluated with passive encoder recordings
before adding assistance. Encoder displacement and position error are not force
measurements: gravity, friction, settling and externally applied pressure can all
affect them. Initial base-pressure recordings showed a small, direction-dependent
response that did not consistently separate from untouched encoder jitter.

The drive's `0.00err` display is motor-shaft angular error, not by itself a fault code.
CAN command `39h` reads signed 32-bit position error: 51200 units per motor turn
(not the encoder's 16384 counts per turn). To read it without changing motor settings,
first stop the ground station so only one process owns the carrier, then run:

```bash
./scripts/macos.sh drives read-error 3
```

This reads the elbow (CAN ID 3). It reports motor degrees, not joint degrees or force;
the display and CAN readback should be compared before using the signal for hand-guiding.
Motor-generated tracking error must not be mistaken for continued hand pressure.

An elbow press-hold-release recording received all 1424 requested `39h` replies.
Using the manual's scale, it captured brief excursions of +1.73 and -4.25 motor
degrees, with near-zero error between them and after release. The operator also
observed `Err` returning toward zero while still pressing. Thus zero error does not
prove pressure was released, and an opposite-sign transient must not automatically
trigger assistance in the opposite direction. These observations do not validate an
active hand-guiding controller.

For the configured MKS `SR_vFOC` mode, the V1.0.9 manual describes `Ma` as the maximum
operating current, and its configuration readback is not measured load current.
`Hold Ma` does not apply in vFOC mode (manual sections 3.2-3.4 and 5.1.10).
Reducing current or changing drive PID gains is not a validated hand-guiding shortcut,
particularly for joints supporting the arm's weight.

Gravity compensation like Seeed's reBot-DevArm (a torque feed-forward from the arm's
model, so the arm floats under the hand) needs torque-controlled actuators. The MKS
drives accept only position and speed commands and report no current or torque, so
it is not possible on this arm without different motors or a force/torque sensor.

Bench test of a sensorless workaround, base yaw (CAN ID 1, no gravity load), firmware
V1.0.9, factory vFOC gains Kp=220, Ki=100, Kd=270, Kv=320 on every drive: writing Ki=0
or Ki=1 with `96h` returns "set success", but `00 96 00` still reads Ki=100, also after
disabling and re-enabling the drive. Held pushes decayed back to near-zero `39h` error
within 1-3 seconds with and without the write, so the position error cannot stand in
for a torque sensor. The drives were restored to Ki=100 and read back.

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
| `arm.*` | Each joint against its target, and whether the arm is zeroed or its saved zero is being checked |

To try each drive fault in simulation, see
[recording drive health sessions](../waybionic_rviz_plugins/README.md#task-7-record-simulated-arm-drive-health-sessions)
in the diagnostics guide.
