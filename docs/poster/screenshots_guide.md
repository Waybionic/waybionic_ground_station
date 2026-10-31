# Screenshot guide

The poster needs three **real** software images:

1. RViz with the simulated robotic arm
2. Ground-station diagnostics panel
3. Doctor-view camera feed

Do not generate composites, do not label simulation as hardware, and do not edit production code to pretty-print a capture.

---

## What is already in `screenshots/`

Captured 2026-10-26 from `main` at `cf3e887`, native ROS 2 Jazzy in WSL2 Ubuntu 24.04 (not Docker). Launch:

```bash
ros2 launch waybionic_bringup ground_station.launch.py \
  teleop:=true joy_source:=none autoplay:=true
```

| File | What it is | What it is not |
| --- | --- | --- |
| `rviz_simulated_arm.png` | Crop of the RViz RobotModel during the Cartesian autoplay demo (`STRAIGHT LINES`, group CARTESIAN, 50%, ENABLED). Simulated SolidWorks-exported arm. | Not a photograph of the physical arm. |
| `diagnostics_panel.png` | Live **WayBionic Engineering Monitor** on `/diagnostics` in the same session: NORMAL, `teleop.input` 120 Hz, `teleop.state` enabled, `teleop.group` cartesian, no alerts. | The table is **scrolled to the teleop rows**. Drive, CAN, and command-gate rows exist below the fold. Mock Normal / Mock Fault buttons are visible even in live mode (that is the real UI). |
| `rviz_ground_station_overview.png` | Full RViz window (arm + panel + empty Doctor View). Useful as a layout reference. | Doctor View shows **No Image** because `camera:=true` was not set. |

**Doctor-view feed: not captured.** This machine has no USB webcam, `ros-jazzy-compressed-image-transport` is not installed in WSL (apt needs sudo), and generating a labelled test-pattern video would still not be the RViz Doctor View the poster asks for. See §3.

---

## Common prerequisites

Pick **one** environment. Docker Desktop on Windows is the documented demo path; native Ubuntu/WSL is what produced the files above.

### A. Windows + Docker (usual club laptop)

1. Docker Desktop running, WSL2 backend, repo at the workspace root.
2. First image: `docker compose --progress plain build wslg` (see `BuildInstructions.md`). The `wslg` service uses the Windows GPU (`GALLIUM_DRIVER=d3d12`).
3. Docker **is** required for this path. Native ROS on Windows is not supported.

### B. Native ROS 2 Jazzy (Ubuntu 24.04 / WSL2)

1. ROS Jazzy desktop (`rviz2`, `robot_state_publisher`, `joint_state_publisher_gui`, `joy`, `xacro`).
2. `python3-serial`, `python3-can` if you will talk to a carrier (not needed for simulated screenshots).
3. `ros-jazzy-compressed-image-transport` **for Doctor View in RViz**.
4. Docker is **not** required.
5. GUI: WSLg (`DISPLAY=:0`) or a Linux desktop.

Build (native):

```bash
source /opt/ros/jazzy/setup.bash
cd /path/to/overlay_ws   # do not colcon-build inside OneDrive if it is flaky
# copy or bind the repo into src/
colcon build --symlink-install
source install/setup.bash
```

---

## 1. RViz with the simulated arm

**Goal:** CAD arm, grey grid, teleop status label. Simulation only.

### Docker / Windows

```powershell
docker compose run --rm --build --service-ports wslg-teleop
```

In a second window, if you want a moving arm from a controller:

```powershell
cd waybionic_teleop
python -m waybionic_teleop.xinput_bridge
```

Or, for a self-moving demo with no controller (clean poster motion):

```powershell
docker compose run --rm --build --service-ports wslg-demo
```

(`wslg-demo` is `teleop:=true autoplay:=true` with simulated drives. Autoplay lifts the arm, draws a square, and tilts about the tip. It will not run with a real `drive_interface`.)

### Native ROS

```bash
ros2 launch waybionic_bringup ground_station.launch.py teleop:=true autoplay:=true joy_source:=none
```

**Expected on screen**

- RViz with RobotModel, TF, MarkerArray.
- After a few seconds, autoplay enables teleop: label `CARTESIAN` / `50%` / `ENABLED`, caption `STRAIGHT LINES` then `TIP FIXED`.
- Diagnostics panel on the left (live).
- Doctor View collapsed / “No Image” unless camera is on.
- Terminal: `Simulated drives: base_yaw=CAN 1, … (no hardware)`.

**Capture**

- Maximize RViz. Hide the Displays/Views docks if you want only the arm (`rviz_simulated_arm.png` style).
- Wait until the arm is **not** at the upright zero pose.
- Print-quality: window ≥ 1920×1080. Do not include a browser or other apps in frame.
- Filename: `screenshots/rviz_simulated_arm.png`.
- Caption: “RViz twin of the WayBionic arm, simulated drives, Cartesian teleop demo.”

**Limitations**

- Follow-camera may crop oddly; `follow_camera:=false` gives a fixed view.
- Autoplay can enter the incision group and raise `INCISION_HELD` (FAULT styling). Prefer a Cartesian-demo frame for the poster.
- WSLg may prefix the title bar `[WARN:COPY MODE]`; that is a compositor warning, not an arm fault.

---

## 2. Diagnostics panel

**Goal:** Engineering Monitor with live teleop/drive rows, not Mock Fault.

Same launch as §1 (`teleop:=true` forces live diagnostics).

To fill **drive** rows as well as teleop rows, enlarge the panel (drag the dock) or float it: right-click the `DiagnosticsPanel` title and undock, then stretch until `can.bus`, `arm.command_gate`, and `drive.base_yaw` are visible.

**Expected rows (simulated teleop, no camera, no UDP bridge)**

- `teleop.input`, `teleop.state`, `teleop.group`, `teleop.speed`, `teleop.tool`
- `can.bus` (label `simulated MKS bus`)
- `arm.command_gate`, `arm.joint_*`
- `drive.base_yaw` … `drive.tool` with health `OK` when idle

`controller.link` appears only with `joy_source:=udp`. `can.carrier` only with `drive_interface:=slcan`. `camera.doctor_view` only with `camera:=true`.

**Capture**

- Crop to the panel; include the NORMAL/FAULT banner.
- Filename: `screenshots/diagnostics_panel.png`.
- Caption: “Live `/diagnostics` in RViz (simulated teleop).” Say if Mock buttons are in frame.

Optional fault shot (simulation only):

```bash
ros2 param set /sim_arm_drives test_fault shoulder:stalled
```

Do **not** present a mock-fault screenshot as a hardware failure.

---

## 3. Doctor-view camera feed

**Pending. Do not fill this slot with a generated image.**

This assignment does not install camera packages or keep troubleshooting the feed. `figures/software_section.svg` leaves a labeled placeholder. A team lead needs to provide a real RViz Doctor View screenshot, or an approved capture from the test-video path, before that placeholder is replaced.

The steps below are for that person. They were not run for this deliverable.

### Windows (documented in `docs/operator_guide.md`)

1. `python -m pip install opencv-python` (Python 3.13+ preferred; older Windows clocks step 15.6 ms).
2. Ground station with the camera port published:

```powershell
docker compose run --rm --build -p 127.0.0.1:47310:47310 wslg ros2 launch waybionic_bringup ground_station.launch.py camera:=true camera_bind:=0.0.0.0
```

3. Bridge, from `waybionic_camera`:

```powershell
python -m waybionic_camera.camera_bridge
```

Use `--camera 1` for a second device, or pass a video file path as `--camera`. `--width 1280 --height 720` matches the PR #36 measurement.

4. In RViz Displays, tick **Doctor View**.

5. Combine with teleop if you want arm + feed in one window:

```text
teleop:=true camera:=true camera_bind:=0.0.0.0 joy_source:=udp
```

and publish **both** UDP 47300 and TCP 47310.

### Native Linux / WSL

```bash
sudo apt-get install -y ros-jazzy-compressed-image-transport python3-opencv
ros2 launch waybionic_bringup ground_station.launch.py camera:=true
# other terminal, from waybionic_camera:
python3 -m waybionic_camera.camera_bridge
```

WSL USB cameras need `usbipd` from Windows. If there is no camera, a video file is a documented stand-in — caption it as such.

**Expected**

- Doctor View shows the JPEG stream.
- `camera.doctor_view` reports fps and delay; warns below 25 fps or above 100 ms.
- Disconnecting the bridge blacks the view and raises a camera alert; reconnect resumes (behaviour described in open PR #33, consistent with `camera_bridge.py`).

**Capture**

- Filename: `screenshots/doctor_view.png`.
- Caption must state **webcam**, **video file**, or **procedure camera**, and resolution.
- Do not crop out the “Doctor View” display name.
- Do not use a still from a surgical video library unless the team owns it.

**If RViz stays on “No Image”**

- `camera:=true` not set, or Doctor View display still disabled.
- Missing `compressed_image_transport`.
- Bridge connected to the wrong host/port (Docker needs `camera_bind:=0.0.0.0` and published 47310).
- Image topic is compressed; the display must use that topic (`/doctor_view/left/image_raw/compressed`).

---

## What to ask team leads for

| Need | Who | Why |
| --- | --- | --- |
| USB webcam or a 1080p/720p test recording, plus a capture of RViz Doctor View with the display enabled | Software lead / Gianna (PR #33) | This is the missing poster asset. PR #36 never screenshotted RViz’s image panel. |
| Confirm whether to caption the feed “USB webcam stand-in” vs a named camera | Biomedical + software | Gemini 2 is research-only (open PR #33). |
| Optional: native Linux box with `compressed_image_transport` already installed | Anyone on Ubuntu | Avoids the WSL apt/sudo and USB forwarding issues. |
| Do **not** request a powered-arm photo for these three slots | — | The brief is software screenshots. A hardware photo would need a separate caption and a completed bring-up checklist. |

---

## Clean-capture checklist

- [ ] Terminal in the shot does not contain personal paths you care about (WSL RViz titles include the overlay path).
- [ ] No other team’s chat, email, or browser.
- [ ] Simulated vs hardware stated in the filename or caption.
- [ ] PNG, not a lossy phone photo of the screen.
- [ ] Production YAML/URDF unchanged.
