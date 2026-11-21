# Architecture diagram specification (for SVG)

**Purpose:** one poster-scale system diagram of the **implemented** ground-station software. This file is the technical brief. The editable drawing is [figures/waybionic_software_architecture.svg](figures/waybionic_software_architecture.svg), with a print PNG beside it.

The figure keeps the WayBionic purple, lavender, pink, and mint palette from the supplied artwork. The hex values in the table below were an earlier colour-blind proposal for the brief. Topology follows this spec: one drive node, collision checks inside teleoperation, video and diagnostics off the motion path, simulation as a caption rather than a second pipeline.

**Commit:** `cf3e887` (`main`). If a connection is not listed, it is not confirmed.

---

## 1. Poster layout

Landscape, roughly **3:1 width:height** inside the software column (or full-width under the four text blocks). Left = operator, right = arm. Four horizontal bands:

1. **Top band (video):** camera → TCP → Doctor View. Cool teal. Dashed.
2. **Middle-upper (commands):** controller → teleop → collision/limits → drive node → carrier → CAN → MKS → arm. Solid dark arrows, left → right.
3. **Middle-lower (motion feedback):** encoder replies returning into the drive node, then `/joint_states` into teleop and RViz. Solid lighter arrows, right → left.
4. **Bottom band (diagnostics):** thin collectors from teleop, drive node, controller link, and camera into the diagnostics panel. Dotted, down/in.

Keep **Ground station** as one rounded rectangle so the laptop/ROS computer is visually one machine. Nested columns inside it, left to right: Teleoperation | Collision & joint limits | Drive node, with Diagnostics as a full-height strip on the inner-right of that rectangle (or a dock along the bottom of it). RViz (arm twin + Doctor View + panel) can sit as a “display” attached to the top of the ground-station box.

Suggested colours (colour-blind safe):

| Kind | Colour | Stroke |
| --- | --- | --- |
| Command | `#1B4F72` | 2.5 pt solid |
| Feedback (encoders / joints) | `#1E8449` | 2 pt solid |
| Video | `#117A65` | 2 pt dashed |
| Diagnostics | `#6C3483` | 1.5 pt dotted |
| Ground-station fill | `#F4F6F7` | `#2C3E50` 1.5 pt |
| Hardware fill | `#FDEBD0` | `#B9770E` |
| Simulation-only badge | small grey caption, not a second pipeline |

---

## 2. Components (every box to draw)

### Operator side

| ID | Label on poster | Notes |
| --- | --- | --- |
| `CTRL` | Game controller | Xbox or DualSense. One box. |
| `BRIDGE` | Host controller bridge | Draw **only** as a small optional stack under CTRL with caption “Windows: UDP 47300 (`WBJY`)”. Omit on a Linux-only simplified cut, but the full poster should show it, because Docker-on-Windows is how the team demos. Linux native path is USB HID into `game_controller_node` and **skips** this box. |

### Ground station (one parent box: “Ground station (ROS 2)”)

| ID | Label | Parent |
| --- | --- | --- |
| `JOY` | Controller input | `joy_udp_receiver` **or** `game_controller_node`. Caption: `/joy` |
| `TELEOP` | Teleoperation | `xbox_teleop` / `ArmTeleop`. Caption: groups base · upper · cartesian · incision, 120 Hz |
| `SAFETY` | Collision checking and joint limits | Same process as teleop (`ArmCollision`, `ArmKinematics.jog`, URDF limits). Draw as a nested gate **inside or immediately after** TELEOP, not as a separate ROS node. |
| `DRIVE` | Drive node | `sim_arm_drives`. Caption: F5h commands, 31h polls, 120 Hz |
| `RVIZ` | RViz | Robot twin + markers (incision point, jaws, status label) |
| `DOCVIEW` | Doctor View | RViz Image display, **off until enabled** |
| `DIAG` | Diagnostics panel | “WayBionic Engineering Monitor” |

Autoplay (`autoplay` node) is optional and **simulation-only**. Do not put it on the main command path. A footnote is enough: “Idle demo injects `/joy` when `autoplay:=true`.”

### Hardware (parent box: “Arm electronics”)

| ID | Label | Notes |
| --- | --- | --- |
| `CARRIER` | Carrier board | UNO R4 WiFi + TJA1051T, `carrier_bridge`. Caption: USB SLCAN, 1 Mbaud serial / 1 Mbit/s CAN |
| `CAN` | CAN bus | 1 Mbit/s, 11-bit standard frames |
| `MKS` | MKS motor drives | Five boxes or one stacked label: IDs 1–5, SERVO42D/57D |
| `ARM` | Robotic arm | Visual mesh; not a sixth motor |
| `CAM` | Doctor-view camera | USB webcam stand-in. Caption: “procedure camera not selected” |

Do **not** draw: Gemini 2, tool-board IDs 7–10 as connected, wired e-stop as a closed circuit, `waybionic_control` mock host, old-arm Arduino servos.

Small caption under MKS: “Simulation uses the same frames on an in-process bus (`drive_interface:=sim`).”

---

## 3. Connections (direction is mandatory)

Legend: `A --> B` command/control, `A <-- B` feedback, `A ==> B` video, `A ..> B` diagnostics.

### Command path (dark blue, left → right)

| From | To | Technology / topic | File evidence |
| --- | --- | --- | --- |
| `CTRL` | `BRIDGE` | USB / Bluetooth HID | `xinput_bridge.py`, `sdl_bridge.py` |
| `BRIDGE` | `JOY` | UDP `127.0.0.1:47300`, magic `WBJY` | `gamepad.py`, `joy_udp_receiver.py` |
| `CTRL` | `JOY` | USB HID via `game_controller_node` (Linux native; **not** through BRIDGE) | `ground_station.launch.py` |
| `JOY` | `TELEOP` | `/joy` (`sensor_msgs/Joy`) | `xbox_teleop_node.py` `on_joy` |
| `TELEOP` | `SAFETY` | in-process, every tick | `teleop.py` `move` / `avoid_collisions` / `jog` / `incise` |
| `SAFETY` | `DRIVE` | `/joint_commands` (`JointState` name/position/velocity) and `/teleop_enabled` (`Bool`) | `xbox_teleop_node.py`, `sim_arm_drives_node.py` |
| `DRIVE` | `CARRIER` | USB serial SLCAN, 1 Mbaud, `tIII#…` plus empty-line ack | `slcan_bus.py`, `carrier_bridge.ino` |
| `DRIVE` | `MKS` | In simulation: in-process `SimulatedBus` (no carrier). Draw as a bypass labelled `drive_interface:=sim`. | `sim_arm_drives_node.py` |
| `CARRIER` | `CAN` | CAN 1 Mbit/s | `carrier_bridge.ino` |
| `CAN` | `MKS` | Std 11-bit, ID = motor ID, F5h move / F3h enable / 92h zero / 98h heartbeat / 31h poll | `mks_can.py`, `arm_drives.yaml` |
| `MKS` | `ARM` | Gearboxes 26:1 (yaw/shoulder/elbow), 3:1 wrist belts, wrist differential | `arm_drives.yaml` |

### Feedback path (green, right → left)

| From | To | Technology / topic | File evidence |
| --- | --- | --- | --- |
| `MKS` | `CAN` | 31h encoder replies, F5h run status | `mks_can.py` `parse` |
| `CAN` | `CARRIER` | CAN RX | `carrier_bridge.ino` `pollCan` |
| `CARRIER` | `DRIVE` | SLCAN `t` lines + host-only status ID `0x7F0` | `slcan_bus.py`, `carrier.py` |
| `DRIVE` | `TELEOP` | `/joint_states` | `sim_arm_drives_node.py` `tick`, `xbox_teleop_node.py` `on_joint_states` |
| `DRIVE` | `RVIZ` | `/joint_states` → `robot_state_publisher` → TF / RobotModel | `ground_station.launch.py` |
| `TELEOP` | `RVIZ` | `/waybionic/teleop/markers` (jaws, status, pink incision point) | `xbox_teleop_node.py` `publish_markers` |
| `TELEOP` | `BRIDGE` / `CTRL` | `joy/set_feedback` → UDP `WBRM` rumble | `xbox_teleop_node.py`, `joy_udp_receiver.py` |
| `DRIVE` | `JOY` (quality) | not a motion path; ping `WBPG`/`WBPO` is **controller-link**, not motor feedback | `joy_udp_receiver.py` |

### Video path (teal dashed)

| From | To | Technology / topic | File evidence |
| --- | --- | --- | --- |
| `CAM` | camera_bridge | OpenCV capture (MJPG 1080p30 default) | `camera_bridge.py` |
| camera_bridge | `camera_receiver` | TCP port **47310**, stamped JPEG | `frames.py`, `camera_receiver.py` |
| `camera_receiver` | `DOCVIEW` | `/doctor_view/left/image_raw/compressed` + `/doctor_view/left/camera_info` | `camera_receiver.py`, `waybionic_unified.rviz` |
| `camera_receiver` | `DIAG` | via `latency_monitor` → `/diagnostics` `camera.doctor_view` | `latency_monitor.py` |

Place `camera_bridge` **outside** the ground-station box (it often runs on Windows Python, not in ROS). `camera_receiver` sits **inside** the ground-station box.

### Diagnostics path (purple dotted)

All of these publish `diagnostic_msgs/DiagnosticArray` on `/diagnostics`. Draw a bus, not a spider of equal-weight arrows.

| From | Row names | File |
| --- | --- | --- |
| `JOY` (`joy_udp_receiver`) | `controller.link` | `joy_udp_receiver.py` |
| `TELEOP` | `teleop.input`, `teleop.state`, `teleop.group`, `teleop.speed`, `teleop.tool` | `xbox_teleop_node.py` `report` |
| `DRIVE` | `can.bus`, `can.carrier` (slcan only), `arm.command_gate`, `arm.zero`, `arm.joint_*`, `drive.*` | `sim_arm_drives_node.py` `report` |
| `latency_monitor` | `camera.doctor_view` | `latency_monitor.py` |
| `/diagnostics` | `DIAG` (RViz plugin) | `diagnostics_panel.cpp`, `DIAGNOSTICS_CONTRACT.md` |

Secondary diagnostic use: TELEOP subscribes to `/diagnostics` to read `arm.command_gate` (rumble on a drive-side stop). Draw a short purple tick from DIAG/bus back into TELEOP labelled `arm.command_gate`.

---

## 4. What not to imply

- No arrow from Doctor View into teleop (the camera does not close a control loop).
- No e-stop loop unless captioned **“status unimplemented until Electrical wires it”**.
- No tool-board motors on the CAN trunk as connected devices.
- No “safe for surgery” badge on SAFETY.
- Simulation and hardware share DRIVE; they are not two different planners. Show one drive node with two possible right-hand edges (sim bus vs carrier).

---

## 5. Mini sequence (optional inset)

If there is room, a 6-step numbered strip under the diagram:

1. Controller sample (`/joy`)
2. Group mapping and speed ramp
3. Joint-limit + collision check
4. Synchronized F5h setpoints
5. Encoder 31h → `/joint_states`
6. RViz twin + diagnostics update

---

## 6. Suggested SVG grouping / `id`s

```
svg
  g#legend
  g#operator        CTRL, BRIDGE
  g#ground-station  JOY, TELEOP, SAFETY, DRIVE, RVIZ, DOCVIEW, DIAG, camera_receiver
  g#hardware        CARRIER, CAN, MKS, ARM
  g#camera          CAM, camera_bridge
  g#arrows-command
  g#arrows-feedback
  g#arrows-video
  g#arrows-diag
```

Font: a geometric sans (Inter / IBM Plex Sans). Box labels 14–16 pt at print size; captions 9–11 pt. No 3D isometric; flat 2D boxes, 8 px corner radius.

---

## 7. One-line data-flow summary (for the designer)

Operator commands flow left to right through software safety checks, then to motors; encoder and status flow right to left; camera video is an independent top path into Doctor View; diagnostics is a monitor, not a controller.
