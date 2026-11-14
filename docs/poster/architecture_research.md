# Ground-station architecture research (software poster)

**Scope:** how the current `main` ground station turns a game controller into simulated or physical MKS motion, and how the operator sees the arm, camera, and diagnostics.

**Commit audited:** `cf3e887` (origin/main, 2026-10-24, PR #53).
**Branch for this work:** `docs/poster-software-khuzaymah`.
**Not in scope for the poster arm:** the older four-joint Arduino servo path (`old_arm.launch.py`, `waybionic_hardware`). That stack is documented in `docs/arm_hardware_motion_architecture.md` and is a separate model.

Every statement below is taken from source on this commit, from a merged pull request, or is marked **unconfirmed**.

---

## 1. What the ground station is

`waybionic_bringup/launch/ground_station.launch.py` is the one-step bring-up. With `teleop:=true` it starts:

| Node | Package | Role |
| --- | --- | --- |
| `robot_state_publisher` | `robot_state_publisher` | Broadcasts TF from `/robot_description` and `/joint_states` |
| `joy` (`game_controller_node`) | `joy` | Local USB/Bluetooth controller → `/joy` (`joy_source:=device`) |
| `joy_udp_receiver` | `waybionic_teleop` | Host UDP bridge → `/joy` (`joy_source:=udp`) |
| `autoplay` | `waybionic_teleop` | Scripted demo; only with `autoplay:=true` and simulated drives |
| `xbox_teleop` | `waybionic_teleop` | Controller → `/joint_commands` |
| `sim_arm_drives` | `waybionic_teleop` | `/joint_commands` → MKS CAN frames → `/joint_states` |
| `camera_follower.py` | `waybionic_bringup` | Keeps the RViz camera near the tool |
| `camera_receiver` + `latency_monitor` | `waybionic_camera` | Doctor-view feed (`camera:=true`) |
| `rviz2` | `rviz2` | Robot model, teleop markers, diagnostics panel, Doctor View |

Default model: `waybionic_description/urdf/waybionic_arm.urdf`.
Default RViz layout: `waybionic_bringup/rviz/waybionic_unified.rviz`.

`drive_interface` defaults to `sim`. Nothing reaches hardware unless it is set to `slcan` (WayBionic carrier) or another python-can interface such as `socketcan`.

---

## 2. Controller → ground station

Three input paths exist. All publish the same `sensor_msgs/Joy` layout defined in `waybionic_teleop/waybionic_teleop/gamepad.py` (SDL `game_controller_node` axis/button order).

### 2.1 Local controller (Linux / native ROS)

`joy_source:=device` starts `game_controller_node`. The controller is plugged into the computer that runs ROS. Confirmed by `ground_station.launch.py` and `docs/operator_guide.md`.

Linux Docker passthrough is `./scripts/linux-gui.sh --controller` (PR #38). The PR author did not feel a controller on Linux; Karis later confirmed DualSense rumble on Windows (PR #26).

### 2.2 Windows host bridge (Docker Desktop / WSL)

Docker Desktop cannot see a Windows controller, so a host process sends 64-byte `WBJY` UDP packets to port **47300**:

- Xbox: `python -m waybionic_teleop.xinput_bridge` (`xinput_bridge.py`)
- PS5 DualSense: `python -m waybionic_teleop.sdl_bridge` (`sdl_bridge.py`, PR #26)

`joy_udp_receiver.py` validates packets, publishes `/joy`, and reports `controller.link` on `/diagnostics`. It polls every **1 ms**. The Xbox bridge default send rate is **250 Hz** (PR #45).

Link monitor (PR #45): packet loss, arrival jitter, longest gap, and ping/pong round trip (`WBPG` / `WBPO`). It warns above **2% loss**, a **100 ms** gap, or a **50 ms** round trip.

Rumble returns on the same UDP socket as `WBRM` packets (`joy/set_feedback` → receiver → bridge). Confirmed in `joy_udp_receiver.py` and PR #38.

### 2.3 Autoplay

With `autoplay:=true`, the controller topic is remapped to `/joy_operator`. `autoplay_node.py` passes operator packets through, or plays `demo_script.py` after **30 s** idle. Autoplay is refused unless `teleop:=true` and `drive_interface:=sim` (`check_files_exist` in the launch file).

---

## 3. Teleoperation pipeline and movement groups

Logic lives in `waybionic_teleop/waybionic_teleop/teleop.py` (`ArmTeleop`). The ROS wrapper is `xbox_teleop_node.py`. Mapping and speeds are `waybionic_teleop/config/xbox_teleop.yaml`.

Control rate: **120 Hz**. Input timeout: **0.5 s**. Deadzone: **15%**. Speed levels: **10 / 25 / 50 / 100%**.

Y cycles four groups:

| Group | Mode | Sticks | What it moves |
| --- | --- | --- | --- |
| `base` | `joint` | left x/y, right y | `joint_1` yaw, `joint_2` shoulder, `joint_3` elbow |
| `upper` | `joint` | left x/y, right y/x | shoulder, elbow, `joint_4` wrist pitch, `joint_5` roll |
| `cartesian` | `cartesian` | left y/x, right y/x | tip x/y/z in `base_link`, tool roll; D-pad tilts about the tip |
| `incision` | `incision` | left y, right y/x | insert along the tool axis, tilt about the port, roll |

Closed-form kinematics (`kinematics.py`, `ArmKinematics`) assume a yaw joint, three parallel pitch joints, and a tool roll through the tip. Cartesian and incision groups are omitted if the URDF does not match that layout.

Start enables only from a measured pose with sticks centred. B stops and holds. A homes every group except incision. RT/LT close and open the placeholder `tool_grip` joint (not a real tool).

When a move is blocked, teleop publishes one **0.25 s** rumble at intensity **0.6** on `joy/set_feedback` (at most once per second) and turns the RViz label red with a one-word cause (`STOPPED-joint_3`, `OUT-OF-REACH`, `DRIVES-STOPPED`, …). PR #38.

### 3.1 Incision group (PR #35)

Selecting `incision` stores the current tip as the incision point (pink RViz sphere, `waybionic/teleop/markers`). Each tick aims at the point on the **new** tool axis at the **new** depth, rather than integrating velocity, so the port does not drift off the axis.

- Left stick up/down: insert / withdraw along the tool axis (50 mm/s at 100%).
- Right stick up/down: tilt about the incision point (30 deg/s at 100%).
- Right stick left/right: roll.
- Homing is disabled (A would drag the tool sideways through the port).
- Y will not leave the group while the tip is inserted past the port (`INCISION_HELD`).
- If the axis misses the point by more than **2 mm**, the group stops (`INCISION_LOST`) rather than pulling the tool back.
- Five joints keep the tool axis in the arm's vertical plane; sideways tilt is not available.

Verified only in simulation. See `waybionic_teleop/test/test_incision.py`.

---

## 4. Collision checks and joint limits

### Joint limits

`joint_limits()` in `kinematics.py` reads URDF `limit` tags. On current `main`:

| Joint | URDF range | Notes |
| --- | --- | --- |
| `joint_1` | ±180° | provisional |
| `joint_2`–`joint_5` | ±90° | provisional |
| `tool_grip` | 0–1 | placeholder, from teleop YAML, not the URDF |

These are **not** CAD contact limits. Open PR #44 (`feature/cad-joint-limits`) opens `joint_3` to 147° and stops other joints just short of CAD contact; it is **not merged**. Open PR #47 adds a workspace keep-in box; it is **not merged**. Do not describe either as current behaviour.

Joint-space groups stop at a limit without pulling an already-outside target back (`ArmTeleop.move`). Cartesian and incision moves go through `ArmKinematics.jog`, which shrinks the **whole** Cartesian step so the tip stays on the commanded line when a joint would exceed `max_speed` (60 deg/s) or a URDF limit. Unreachable steps are tagged `reach`.

The drive node rejects a `/joint_commands` target outside the URDF range and stops every drive (`SimArmDrives.on_command`).

### Collision boxes (PR #37)

`collision.py` (`ArmCollision`) reads URDF collision boxes. Required links: `base_link`, `shoulder_link`, `upper_arm_link`, `forearm_link`, wrist/tool folding links. Missing any of them **disables teleop** with an ERROR naming the link.

Each step is checked in the base frame:

- Moving links must stay `collision_clearance` (**10 mm**) above the table (`table_height`, default 0).
- Folding links must not occupy the base or shoulder boxes grown by that clearance.

A step is refused if **any single contact** deepens, even if total penetration shrinks. Backing out is allowed. The diagnostic names the contact, for example `forearm_link: table`. Reloading `robot_description` while enabled disables teleop and sends an empty stop; the drive node latches speed-zero F5 frames.

Boxes were derived from committed meshes, not a live CAD export (PR #37). Hardware pinout, gearing and travel remain provisional in that PR. **No powered drive was exercised.**

Open PR #47's workspace box is a planned extra keep-in volume, not present on `main`.

---

## 5. Drive node, carrier, CAN, MKS motors

### Drive node

`sim_arm_drives_node.py` keeps the name `sim_arm_drives` for branch compatibility. It is the **only** path the current arm uses, in simulation and on a real bus (PR #39 review: the older `waybionic_control` host is not on this path).

Per tick (120 Hz):

1. Gate: valid URDF, every drive zeroed and setup-confirmed, fresh encoder replies, fresh `teleop_enabled`, carrier e-stop not pressed.
2. `DriveMap.synchronized` converts the next-period joint setpoint to per-drive encoder axis and rpm, scaling all speeds together at `max_rpm` **600**.
3. MKS **F5h** absolute-axis frames go out; **31h** encoder polls follow.
4. Replies update `/joint_states`. Health, bus load and the command gate go to `/diagnostics`.

Frame codec: `mks_can.py` (MKS SERVO42D/57D CAN manual V1.0.9 subset). Counts: **16384 / rev**. Wrist left/right motors mix `joint_4` and `joint_5` (bevel differential) in `arm_drives.yaml`.

| Drive | CAN ID | Gear | Joints |
| --- | --- | --- | --- |
| `base_yaw` | 1 | 26:1 cycloidal | `joint_1` |
| `shoulder` | 2 | 26:1 | `joint_2` |
| `elbow` | 3 | 26:1 | `joint_3` |
| `wrist_left` | 4 | 3:1 belt | `joint_4`, `joint_5` (−roll) |
| `wrist_right` | 5 | 3:1 belt | `joint_4`, `joint_5` (+roll) |
| `tool` | 6 | 1:1 | `tool_grip` placeholder |

The operator guide requires **removing `tool` from `drives`** before a real arm run; the host waits for every listed drive. Tool-board motors (IDs 7–10, PR #32) are **not** in this map.

Simulation zeroes at start-up. Real drives publish no `/joint_states` and move nothing until `~/zero` (`std_srvs/Trigger`, MKS **92h**), refused while commands are still arriving.

Stops:

- F5h with speed 0 (immediate if acc 0).
- Drive heartbeat **98h**, 500 ms, if the host goes quiet.
- Command publisher silence **0.5 s**.
- Lost / jumped encoder, run-failed or end-limit reply, following-error latch (provisional 4096 counts × 12 ticks).
- Carrier e-stop status, once Electrical wires it (currently unknown).
- F7h broadcast emergency stop exists in `mks_can.emergency_stop` and `mks_setup stop`; the control loop's routine stop is still F5h.

Encoder feedback **is** used: `/joint_states` is decoded from 31h replies; teleop starts from measured pose; following-error and drive-health compare target vs encoder in **joint degrees**.

### Carrier board (PR #40 firmware, PR #45 host)

Hardware: UNO R4 WiFi + TJA1051T, sketch `waybionic_can/arduino/carrier_bridge/carrier_bridge.ino`. USB serial **1 Mbaud**, CAN **1 Mbit/s**, independent.

The carrier speaks a Lawicel SLCAN subset (`SlcanBridge`). It queues up to **16** frames, expires a target after **10 ms**, and answers each empty line with CR, or BELL then CR if the frame was refused. Status frame **0x7F0** is host-only and never enters the CAN bus (`carrier.py`). E-stop and supply flags stay unknown until Electrical specifies wiring (`carrier_bridge.ino` TODO).

Host transport: `slcan_bus.py`. One write per tick; refused frames are retried; no more than 16 lines unanswered (carrier answer FIFO is 32). USB drop → reopen every 1 s. Shutdown writes stops, waits up to 0.5 s, retries refused stops, then sends `C`.

**Hardware status of this path:** PRs #31, #37, #40 and #45 all state that **no powered MKS drive / UNO R4 / TJA1051T motion was exercised** in those reviews. `waybionic_can/docs/HARDWARE_BRINGUP_CHECKLIST.md` is still unchecked. Open PR #51 is a Windows USB relay for the same carrier; not merged. Do not claim a completed physical-arm demonstration.

`waybionic_control` (`can_host.py`, `mock_drives.py`) is the earlier CAN-control foundation (PR #15). The live arm does not use it.

---

## 6. Doctor-view camera (PR #36)

Until a surgical camera is chosen, a USB webcam or a video file stands in.

```
camera_bridge (OpenCV, no ROS) --TCP 47310--> camera_receiver --ROS--> RViz Doctor View
                                         \-> latency_monitor -> /diagnostics
```

- `camera_bridge.py`: MJPG capture default **1920×1080 @ 30 fps**, JPEG quality 80, capture timestamp in the frame header. Runs on the computer the camera is plugged into (covers Windows, where Docker cannot reach USB cameras).
- `camera_receiver.py`: publishes `/doctor_view/left/image_raw/compressed` and `/doctor_view/left/camera_info`, frame id `doctor_view_left_optical_frame`.
- `latency_monitor.py`: `camera.doctor_view` row with fps and capture-to-arrival delay (mean / min / max / jitter) over **5 s**. Warns below **25 fps** or above **100 ms**. That delay **excludes** time inside the camera and on the display.
- RViz Image display **Doctor View** is **off by default** in `waybionic_unified.rviz`. Enable it in the Displays panel. Launch with `camera:=true`.

PR #33 (Gianna, camera/controller research, Orbbec Gemini 2 recommendation) is **still open**. Do not treat Gemini 2 as the implemented camera.

PR #36 was **not** tested with a physical webcam or with RViz's image panel on screen. The reported 2 ms delay used a **test video** from Windows into Docker.

---

## 7. Diagnostics panel (PR #2, live data from later PRs)

RViz plugin `waybionic_rviz_plugins/DiagnosticsPanel` (`diagnostics_panel.cpp`). Title: **WayBionic Engineering Monitor**.

It consumes `diagnostic_msgs/DiagnosticArray` on `/diagnostics` (contract: `waybionic_rviz_plugins/docs/DIAGNOSTICS_CONTRACT.md`). Mock mode is for UI validation; with `teleop:=true` or `camera:=true` the launch file forces **live** mode (`use_mock_diagnostics` ANDed with NOT teleop/demo/camera).

The panel shows:

- Overall state: NORMAL / FAULT / WAITING FOR LIVE DIAGNOSTICS
- Source, ROS connection, heartbeat, “No motor commands sent from this RViz panel”
- Table: Signal, Status, Value, Unit, Last Updated, Message
- Current alerts for any non-OK row

Rows the current stack actually publishes (operator guide + source):

| Row | Publisher | Content |
| --- | --- | --- |
| `controller.link` | `joy_udp_receiver` | Bridge/controller/input state; Hz, loss, jitter, gap, RTT |
| `teleop.input` | `xbox_teleop` | `/joy` rate; STALE after 0.5 s |
| `teleop.state` / `.group` / `.speed` / `.tool` | `xbox_teleop` | Enable, group description, speed, placeholder grip |
| `arm.command_gate` | `sim_arm_drives` | enabled/stopped and why |
| `arm.zero`, `arm.joint_*` | `sim_arm_drives` | Zeroing; joint vs target (deg) |
| `can.bus` | `sim_arm_drives` | frames/s, worst-case load, interface label |
| `can.carrier` | `sim_arm_drives` | Only `interface:=slcan`: USB, e-stop, supply, refusals |
| `drive.<name>` | `sim_arm_drives` | Encoder rev, last CAN frame, health (`OK`, `FOLLOWING_ERROR`, `STALLED`, `NOT_RESPONDING`, `DISABLED`) with joint-degree error and reply age (PR #39) |
| `camera.doctor_view` | `latency_monitor` | fps and delay (only with `camera:=true`) |

IMU rows (`waybionic_sensors`) are a separate optional launch, not part of `ground_station.launch.py` teleop.

Health thresholds in `arm_drives.yaml` are marked **provisional**: 5° following error, <1° motion over 2 s for stall, 0.5 s reply timeout.

---

## 8. End-to-end path (implemented)

```
Gamepad
  -> USB HID (Linux) or UDP/WBJY (Windows bridge)
  -> /joy
  -> ArmTeleop (groups, speeds, URDF limits, ArmCollision, ArmKinematics)
  -> /joint_commands  +  /teleop_enabled
  -> SimArmDrives (gate, DriveMap, mks_can)
  -> SimulatedBus  OR  SlcanBus -> carrier USB -> CAN 1 Mbit/s -> MKS SERVO42D/57D
  -> 31h encoder replies
  -> /joint_states
  -> robot_state_publisher + xbox_teleop (measured pose, RViz twin)
```

Feedback that is implemented: encoder → joint states → teleop enable-from-measured; drive health; bus load; controller link; camera delay. Feedback that is **not** implemented on this path: measured tool-board joints, wired e-stop/supply, physical camera pose, clinical video.

---

## 9. Implemented vs planned vs hardware-tested

| Feature | On `main`? | Evidence environment |
| --- | --- | --- |
| Four teleop groups including incision | Yes | Simulation + unit/launch tests |
| Collision boxes + per-contact refusal | Yes | Simulation; boxes from meshes |
| CAD-accurate joint limits (#44) | No (open PR) | — |
| Workspace keep-in box (#47) | No (open PR) | — |
| Simulated MKS drives, zeroing, heartbeat | Yes | In-process sim + vcan CI |
| Carrier SLCAN + status 0x7F0 | Yes in software | Unit tests, slcan_sim, firmware compile |
| Powered 5-drive motion through the carrier | **Not evidenced** | Checklist blank; PRs say not run |
| Doctor-view TCP pipeline | Yes | Test video Windows→Docker; no webcam in #36 |
| Gemini 2 / surgical camera | No | Open PR #33 |
| Diagnostics panel + drive health | Yes | Simulation; mock UI controls remain visible |
| PS5 DualSense on Windows | Yes | Physical controller, USB and Bluetooth (PR #26) |
| Tool board IDs 7–10 on the arm map | Firmware exists; **not** in `arm_drives.yaml` | Not run on hardware (PR #32) |
| Old-arm Arduino servos | Separate launch | Not the poster arm |

---

## 10. Merged PRs used for this research

| PR | Title | Why it matters |
| --- | --- | --- |
| [#2](https://github.com/Waybionic/waybionic_ground_station/pull/2) | RViz diagnostics plugin | Panel UI |
| [#24](https://github.com/Waybionic/waybionic_ground_station/pull/24) / [#28](https://github.com/Waybionic/waybionic_ground_station/pull/28) | Simulated Xbox teleop | Controller → simulated drives |
| [#26](https://github.com/Waybionic/waybionic_ground_station/pull/26) | PS5 DualSense | Second Windows controller |
| [#27](https://github.com/Waybionic/waybionic_ground_station/pull/27) | Cartesian teleop | Straight-line / tilt-about-tip |
| [#31](https://github.com/Waybionic/waybionic_ground_station/pull/31) | Real MKS host + vcan CI | `drive_interface`, zeroing |
| [#32](https://github.com/Waybionic/waybionic_ground_station/pull/32) | Tool-board firmware | IDs 7–10, not on the arm map |
| [#34](https://github.com/Waybionic/waybionic_ground_station/pull/34) | Autoplay demo | Scripted Cartesian demo |
| [#35](https://github.com/Waybionic/waybionic_ground_station/pull/35) | Incision group | Port-constrained motion |
| [#36](https://github.com/Waybionic/waybionic_ground_station/pull/36) | Doctor-view camera | TCP feed + delay monitor |
| [#37](https://github.com/Waybionic/waybionic_ground_station/pull/37) | Collision boxes | Table/base checks |
| [#38](https://github.com/Waybionic/waybionic_ground_station/pull/38) | Rumble + Linux passthrough | Limit feedback |
| [#39](https://github.com/Waybionic/waybionic_ground_station/pull/39) | Drive/controller health | `drive.*` health states |
| [#40](https://github.com/Waybionic/waybionic_ground_station/pull/40) | UNO R4 CAN + SLCAN | Carrier firmware (Khuzaymah) |
| [#45](https://github.com/Waybionic/waybionic_ground_station/pull/45) | Carrier host integration | `SlcanBus`, gear ratios, link quality |
| [#53](https://github.com/Waybionic/waybionic_ground_station/pull/53) | README / operator guide | Operator-facing architecture text |

Open, do not cite as shipped: [#33](https://github.com/Waybionic/waybionic_ground_station/pull/33), [#44](https://github.com/Waybionic/waybionic_ground_station/pull/44), [#47](https://github.com/Waybionic/waybionic_ground_station/pull/47), [#51](https://github.com/Waybionic/waybionic_ground_station/pull/51).

---

## 11. Primary source files

```
waybionic_bringup/launch/ground_station.launch.py
waybionic_bringup/rviz/waybionic_unified.rviz
waybionic_teleop/config/xbox_teleop.yaml
waybionic_teleop/config/arm_drives.yaml
waybionic_teleop/waybionic_teleop/xbox_teleop_node.py
waybionic_teleop/waybionic_teleop/teleop.py
waybionic_teleop/waybionic_teleop/kinematics.py
waybionic_teleop/waybionic_teleop/collision.py
waybionic_teleop/waybionic_teleop/sim_arm_drives_node.py
waybionic_teleop/waybionic_teleop/drive_map.py
waybionic_teleop/waybionic_teleop/mks_can.py
waybionic_teleop/waybionic_teleop/slcan_bus.py
waybionic_teleop/waybionic_teleop/carrier.py
waybionic_teleop/waybionic_teleop/joy_udp_receiver.py
waybionic_teleop/waybionic_teleop/xinput_bridge.py
waybionic_teleop/waybionic_teleop/sdl_bridge.py
waybionic_can/arduino/carrier_bridge/carrier_bridge.ino
waybionic_camera/waybionic_camera/camera_bridge.py
waybionic_camera/waybionic_camera/camera_receiver.py
waybionic_camera/waybionic_camera/latency_monitor.py
waybionic_rviz_plugins/src/diagnostics_panel.cpp
waybionic_description/urdf/waybionic_arm.urdf
docs/operator_guide.md
```
