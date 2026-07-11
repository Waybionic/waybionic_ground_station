# Waybionic Checklist

This checklist provides a streamlined setup process for club members working with the Waybionic model. Follow the steps below to prepare the required files, place them in the correct folders, build the workspace, and launch the model successfully.

---

## 1. Prepare the Files

If the URDF and STL files are packaged in a ZIP folder, unzip them before continuing.

Place the files in the following folders:

| File Type | Destination Folder |
|-----------|-------------------|
| URDF file | `waybionic_ground_station/waybionic_description/urdf` |
| STL files | `waybionic_ground_station/waybionic_description/meshes` |

---

## 2. Build and Launch the Model

Run all commands from the **workspace root** (the folder that contains `waybionic_bringup/` and `waybionic_description/`).

```bash
# 1. Source ROS
source /opt/ros/jazzy/setup.bash

# 2. Build the packages
colcon build --packages-select waybionic_description waybionic_bringup

# 3. Source the install setup
source install/setup.bash

# 4. Launch the model (pick ONE — see "Which launch file?" below)
ros2 launch waybionic_bringup display.launch.py
```

> **You must rebuild after ANY change to a URDF, launch file, or mesh.** These
> are `ament_cmake` packages that *copy* files into `install/` at build time, so
> edits in the source tree are invisible to `ros2 launch` until you rebuild. If
> a brand-new launch file reports "file not found", you simply haven't rebuilt
> yet.

### Clean rebuild (when packages were added or removed)

If packages were **deleted or renamed** (e.g. after a merge), the stale copies
linger in `build/`, `install/`, and `log/`. Do a clean rebuild so ROS doesn't
keep seeing removed packages:

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

### Which launch file?

| Command | Shows |
|---------|-------|
| `ros2 launch waybionic_bringup display.launch.py` | The working model (`waybionic_placeholder.urdf`, 50 parts) in its assembled layout. |
| `ros2 launch waybionic_bringup assembled.launch.py` | The curated assembled arm (`waybionic_assembled.urdf`, 36 unique parts). |
| `ros2 launch waybionic_bringup test_all.launch.py` | **Every** mesh in the package (`test_all_meshes.urdf`, 90 parts) spread out on a grid — used to confirm all meshes load. |

All three also start the **Joint State Publisher GUI** and open RViz with the
saved `waybionic.rviz` config.

### RViz Configuration

> **Note:** If you launch via one of the launch files above, RViz auto-loads the saved `waybionic.rviz` config (Fixed Frame, RobotModel display, and Description Topic are already set), so you normally **don't** need the steps below. **However, if the model doesn't appear** (empty view, or you started RViz with bare `rviz2`), configure it manually as follows:

Once launched, configure RViz in the **main window**:

1. In **Global Options** (left panel), set **Fixed Frame** to `base_link` (or `world`)
2. Click **Add** (bottom left), select **RobotModel**, then click **Add**
3. Expand **RobotModel** settings and set **Description Topic** to `robot_description`
4. Click **Add** again, select **TF**, then click **Add**
5. Use the **Joint State Publisher GUI** (small window) to move the arm with the slider

---

## 3. Verify All Parts Loaded

Don't try to count parts by eye in the 3D view — parts range from a ~30 cm
housing to a few-millimeter screw, so small ones are nearly invisible. Use these
instead.

```bash
# Count parts a URDF DEFINES (run from the workspace root, no launch needed)
grep '<joint name=' waybionic_description/urdf/test_all_meshes.urdf | grep -vc world_to_base

# Count parts actually RENDERING (run in a 2nd terminal while a launch is active)
# Result = number of parts + 1 (the world->base anchor)
ros2 topic echo /tf_static --once | grep -c child_frame_id

# Validate a URDF's structure (links form a valid tree) BEFORE launching
check_urdf install/waybionic_description/share/waybionic_description/urdf/test_all_meshes.urdf

# Optional: visual frame tree as a PDF
ros2 run tf2_tools view_frames
```

> `check_urdf` confirms the **structure** (all links/joints parse). A part whose
> STL is missing still parses and still counts on `/tf_static` — it just renders
> invisibly. To confirm the **meshes exist on disk**, check that every file
> referenced under `meshes/` is actually present.

---

## 4. URDF Files — what each one shows

All models live in `waybionic_description/urdf/`. The meshes they reference live
in `waybionic_description/meshes/` and are authored in millimetres, so every
`<mesh>` uses `scale="0.001 0.001 0.001"` to convert to metres.

| File | Parts | What it shows |
|------|-------|---------------|
| `waybionic_assembled.urdf` | 36 | **The real assembled arm.** Each STL is pre-positioned in its own coordinates (all joint origins `0 0 0`), so the parts render as the complete arm in its true shape. Loaded by `assembled.launch.py`. |
| `waybionic_placeholder.urdf` | 50 | **The working full model** — same assembled-coordinate meshes as above but the full 50 parts (with duplicates) instead of the trimmed 36. Looks like the assembled arm. Loaded by `display.launch.py`. *(This was renamed from `waybionic.urdf` during the foundation merge.)* |
| `test_all_meshes.urdf` | 90 | **Diagnostic "contact sheet."** Renders every mesh in the package, each as its own fixed link, laid out in a 10-column grid (0.5 m spacing) so nothing overlaps. Does **not** look like an arm — it's a flat field of all parts. Used to confirm all meshes load. Loaded by `test_all.launch.py`. |

### Backups (`.bak` — not loaded by any launch file)

| File | Parts | What it is |
|------|-------|------------|
| `test_all_meshes.preGrid.urdf.bak` | 90 | Snapshot of `test_all_meshes.urdf` **before** the grid layout — all parts stacked at the origin (one overlapping blob). Restore point. |
| `waybionic_49parts.urdf.bak` | 49 | Earlier 49-part revision of the working model. |
| `waybionic_placeholder.urdf.bak` | 3 | The only **movable** model: a primitive-shape (box + cylinders + sphere) 3-DOF arm with `revolute` joints you can drive from the joint sliders. The early lightweight stand-in before the real meshes existed. |

> **Key distinction:** every active model uses `fixed` joints (static — parts
> can't move), because the STLs carry only geometry, not joint/connection
> information. Only `waybionic_placeholder.urdf.bak` has real `revolute` joints.
> To make the mesh model articulate, joints (axis + limits + pivot) must be
> authored in the URDF — they cannot be recovered from the STL files.

---

## 5. Final Check

Before launching, confirm the following:

- [ ] All files are in the correct directories
- [ ] The workspace has been built without errors
- [ ] The launch command runs successfully
- [ ] The Waybionic model displays as expected in RViz
- [ ] Part count from `/tf_static` matches the count defined in the URDF
