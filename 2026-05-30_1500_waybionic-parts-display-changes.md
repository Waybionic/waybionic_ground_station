# Waybionic Parts Display — Changes & How to Run
**Date:** 2026-05-30 | **Time:** 3:00 PM MDT

---

## What Was Changed and Why

### Background

The `FULL-ARM_redownload-march8` folder contained the full CAD design for the waybionic
robot arm in SolidWorks format (`.SLDPRT`, `.SLDASM`). SolidWorks files are proprietary
and cannot be read without the software, but the folder also contained three `.glb` files
— these are glTF binary 3D models exported specifically for web/software use. The file
`website-arm.glb` holds the complete arm assembly with every named part embedded inside it.

The goal was to get those custom parts visible inside the existing RViz2 ground station GUI.

---

## Files Changed or Added

### 1. `annin_ar4_description/meshes/waybionic/` *(new folder)*

**What it is:** 71 STL mesh files extracted from `website-arm.glb` using the Python
`trimesh` library.

**How it was made:**
- `trimesh` loaded the `.glb` scene, which contains 69 named geometry nodes (one per part).
- Each node was exported to its own STL file with its world-frame position already baked in,
  so all parts sit in the correct relative positions to each other.
- `StepperHolder.STL` from the `3-key-to-diff` subfolder was also copied in.
- `full_arm_assembly.stl` is a single merged mesh of the entire arm (all parts combined).

**Why STL:** RViz2 natively reads STL files (and DAE/Collada). The `.glb` format is not
supported by RViz directly.

---

### 2. `annin_ar4_driver/scripts/waybionic_parts_display.py` *(new file)*

**What it is:** A ROS2 Python node that reads every STL from the `meshes/waybionic/`
folder and publishes them as a `visualization_msgs/MarkerArray` on the `/waybionic_parts`
topic.

**Key behaviours:**
- Uses **Transient Local** (latched) QoS so RViz receives the markers even if it starts
  after this node.
- Assigns a colour to each part based on its sub-assembly group:

  | Colour | Sub-assembly group | Example parts |
  |--------|-------------------|---------------|
  | Blue | Base | `Base_Updated`, `Base_Funnel`, `lid` |
  | Green | Shoulder / Elbow | `shoulder_elbow_housing`, `90_degree_elbow`, `baseToShoulder` |
  | Red | Differential | `DifferentialHousing`, `CapForDifferentialV2`, `keyed_shaft` |
  | Orange | Drive / Steppers | `Stepper`, `StepperHolderV2`, `carrier`, `2WireSlipRing` |
  | Purple | Gears & Belts | `spur_gear`, `GT2`, `pulley`, `timing_belt`, `outer_ring` |
  | Cyan | Joints | `3rd_Joint_Pulley`, `3rd_Joint`, `bearing` |
  | Grey | Everything else | structural parts, pipe, PVC |

- Applies a centering offset so the assembly appears at the RViz world origin (0, 0, 0)
  instead of floating at the GLB's original coordinate (~1.5 m off-centre).

---

### 3. `annin_ar4_driver/CMakeLists.txt` *(modified)*

Added `waybionic_parts_display.py` to the `install(PROGRAMS ...)` block so the build
system copies the script into `install/annin_ar4_driver/lib/annin_ar4_driver/` where ROS2
can find and launch it.

---

### 4. `annin_ar4_driver/package.xml` *(modified)*

Added three `<exec_depend>` entries:
- `rclpy` — the Python ROS2 client library
- `visualization_msgs` — the message package for `Marker` and `MarkerArray`
- `ament_index_python` — lets the node find the mesh install path at runtime

These were not there before because the package was C++-only.

---

### 5. `annin_ar4_moveit_config/rviz/moveit.rviz` *(modified)*

Added a **WayBionic Parts** display entry at the top of the Displays list:

```yaml
- Class: rviz_default_plugins/MarkerArray
  Enabled: true
  Name: WayBionic Parts
  Topic: /waybionic_parts
  Durability Policy: Transient Local
```

This means every time you open RViz via either launch file, the waybionic parts display
is already configured and active — you do not need to add it manually.

---

### 6. `annin_ar4_moveit_config/launch/moveit.launch.py` *(modified)*
### 7. `annin_ar4_moveit_config/launch/demo.launch.py` *(modified)*

Both launch files now start `waybionic_parts_display.py` as a node alongside RViz and
MoveIt. This ensures the markers are published before RViz finishes loading.

---

## How to Run and See the Arm

### Step 1 — Open a terminal and source the workspace

```bash
cd ~/waybionic_ground_station
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

> **Always source both** — `opt/ros/jazzy` gives you base ROS2, `install/setup.bash`
> gives you the local packages including the new meshes and node.

---

### Step 2 — Launch the demo (no real hardware needed)

```bash
ros2 launch annin_ar4_moveit_config demo.launch.py
```

This starts:
- `move_group` — MoveIt motion planning server
- `robot_state_publisher` — broadcasts the AR4 URDF joint transforms
- `ros2_control_node` + controller spawners — fake hardware for simulation
- `rviz2` — the 3D visualiser, pre-loaded with `moveit.rviz`
- **`waybionic_parts_display`** *(new)* — publishes the 69 waybionic part markers

---

### Step 3 — What you will see in RViz

When RViz opens you will see two things overlaid in the 3D viewport:

1. **Yellow AR4 robot model** — the existing URDF robot, driven by MoveIt/ros2_control.
   You can drag the interactive markers (coloured spheres/arrows on the end effector) to
   plan and execute motions.

2. **Colour-coded waybionic parts** — the full custom arm assembly from your CAD files,
   shown as static meshes centred at the world origin. Each sub-assembly is a different
   colour (see table above).

In the **Displays panel** on the left you will see:

```
[✓] WayBionic Parts         ← toggle the whole custom arm on/off
      Namespaces
        [✓] waybionic_parts  ← toggle individual namespace
[✓] MotionPlanning           ← the existing MoveIt robot
[✓] Grid
```

You can click the checkbox next to **WayBionic Parts** to hide the custom arm and work
with just the motion planning view, or turn it back on to compare the design against the
running robot model.

---

### Step 4 — Navigating the 3D view

| Action | How |
|--------|-----|
| Orbit / rotate | Left-click and drag |
| Pan | Middle-click and drag (or Shift + left-click drag) |
| Zoom | Scroll wheel |
| Reset camera to focus | Press **F** with the cursor over the 3D viewport |

If the waybionic parts appear separate from the yellow AR4 model it is because the
custom arm has different link geometry and origins than the stock AR4 URDF. The parts
are at the correct scale (metres) and in the correct positions relative to **each other**.

---

### Alternative — launch with real hardware

If the arm is physically connected:

```bash
# Terminal 1
ros2 launch annin_ar4_driver driver.launch.py calibrate:=True ar_model:=mk3

# Terminal 2
ros2 launch annin_ar4_moveit_config moveit.launch.py
```

The `moveit.launch.py` also starts `waybionic_parts_display`, so the custom parts will
appear alongside the live robot.

---

### Rebuilding after any source changes

If you edit the Python node or any other source file, rebuild before launching:

```bash
cd ~/waybionic_ground_station
colcon build --packages-select annin_ar4_description annin_ar4_driver annin_ar4_moveit_config
source install/setup.bash
```
