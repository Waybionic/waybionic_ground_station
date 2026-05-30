# GUI Customisation Guide — Waybionic Ground Station
**Date:** 2026-05-30 | **Time:** 3:00 PM MDT

This document explains every lever you can pull to change how the RViz GUI looks
and behaves. Changes are grouped by what you want to achieve.

---

## 1. Changing Part Colours

**File to edit:**
`annin_ar4_driver/scripts/waybionic_parts_display.py`

Find the `_GROUPS` list near the top of the file:

```python
_GROUPS = [
    ('base',         (0.25, 0.45, 0.85, 0.85), ['base_updated', 'base_funnel', ...]),
    ('shoulder',     (0.20, 0.80, 0.40, 0.85), ['shoulder', 'elbow', ...]),
    ('differential', (0.85, 0.20, 0.20, 0.85), ['differential', 'diff', ...]),
    ('drive',        (0.90, 0.55, 0.10, 0.85), ['stepper', 'carrier', ...]),
    ('gears_belts',  (0.60, 0.20, 0.85, 0.85), ['spur_gear', 'gt2', ...]),
    ('joints',       (0.15, 0.75, 0.85, 0.85), ['3rd_joint', 'joint', ...]),
    ('structure',    (0.65, 0.65, 0.65, 0.85), []),
]
```

Each row is: `('group name', (R, G, B, Alpha), ['keyword', ...])`

- **R, G, B** are floats from `0.0` (dark) to `1.0` (full brightness).
- **Alpha** is transparency: `1.0` = fully solid, `0.0` = invisible, `0.5` = half-transparent.

### Example — make the differential bright yellow, fully solid

```python
('differential', (1.0, 1.0, 0.0, 1.0), ['differential', 'diff', 'keyed_shaft', 'key_to_diff', 'bevel', 'capfor']),
```

### Example — make structure parts invisible (hide them)

```python
('structure', (0.65, 0.65, 0.65, 0.0), []),
```

After editing, rebuild and re-source:

```bash
cd ~/waybionic_ground_station
colcon build --packages-select annin_ar4_driver
source install/setup.bash
```

Then restart the launch.

---

## 2. Moving the Assembly (Position & Rotation)

**File to edit:**
`annin_ar4_driver/scripts/waybionic_parts_display.py`

### Changing the position

Find this line near the top:

```python
_CENTER_OFFSET = (-1.539, -1.134, -0.903)
```

These three numbers are the `(X, Y, Z)` offset in **metres** applied to every marker.
They currently shift the GLB's world-frame coordinates so the assembly sits at the RViz
world origin (0, 0, 0).

| To move the arm... | Change... |
|--------------------|-----------|
| Further from you (forward) | increase X |
| Closer to you (backward) | decrease X |
| Left | increase Y |
| Right | decrease Y |
| Up | increase Z |
| Down | decrease Z |

**Example — move the arm 0.5 m to the right and 0.2 m up:**

```python
_CENTER_OFFSET = (-1.539 - 0.5, -1.134, -0.903 + 0.2)
# which simplifies to:
_CENTER_OFFSET = (-2.039, -1.134, -0.703)
```

### Changing the rotation

Inside the `for idx, filename in enumerate(stl_files):` loop, find:

```python
m.pose = Pose(
    position=Point(x=_CENTER_OFFSET[0], y=_CENTER_OFFSET[1], z=_CENTER_OFFSET[2]),
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
)
```

The `orientation` field uses a quaternion. Common rotations:

| Rotation | x | y | z | w |
|----------|---|---|---|---|
| No rotation (default) | 0.0 | 0.0 | 0.0 | 1.0 |
| 90° around Z (yaw left) | 0.0 | 0.0 | 0.707 | 0.707 |
| 90° around X (pitch forward) | 0.707 | 0.0 | 0.0 | 0.707 |
| 180° around Z (flip) | 0.0 | 0.0 | 1.0 | 0.0 |

**Example — rotate the whole assembly 90° around the Z axis:**

```python
orientation=Quaternion(x=0.0, y=0.0, z=0.707, w=0.707),
```

Rebuild and re-source after any change to this file.

---

## 3. Changing Which Parts Are Visible

You have two ways to do this:

### Option A — Toggle in RViz at runtime (no rebuild needed)

In the **Displays** panel on the left of RViz:

1. Expand **WayBionic Parts**
2. Expand **Namespaces**
3. Uncheck `waybionic_parts` to hide all parts at once, or check/uncheck individual
   namespaces if you split parts into separate namespaces (see Section 6).

### Option B — Remove a part permanently from the node

In `waybionic_parts_display.py`, add a skip list before the marker loop:

```python
_SKIP_PARTS = {
    'full_arm_assembly.stl',      # merged mesh — usually too heavy to show alongside parts
    'arduino-uno-rough-1.stl',    # rough placeholder model
}

# Inside the loop, add:
if filename in _SKIP_PARTS:
    continue
```

Rebuild and re-source after editing.

---

## 4. Changing the AR4 Robot Model Colour

The yellow robot model comes from the URDF `<material>` tag.

**File to edit:**
`annin_ar4_description/urdf/ar_macro.xacro`

Every `<link>` block has this inside its `<visual>` section:

```xml
<material name="">
  <color rgba="1 1 0 1"/>
</material>
```

`rgba` is `Red Green Blue Alpha` from 0 to 1.

**Common colours:**

| Colour | rgba value |
|--------|-----------|
| Yellow (current) | `1 1 0 1` |
| White | `1 1 1 1` |
| Grey | `0.5 0.5 0.5 1` |
| Blue | `0.2 0.4 0.9 1` |
| Semi-transparent grey | `0.5 0.5 0.5 0.4` |

Change all six links (`base_link`, `link_1` through `link_6`) to the same value, or give
each link a different colour to distinguish joints.

After editing the URDF, rebuild:

```bash
colcon build --packages-select annin_ar4_description
source install/setup.bash
```

---

## 5. Changing the RViz Background Colour

**File to edit:**
`annin_ar4_moveit_config/rviz/moveit.rviz`

Find:

```yaml
Global Options:
  Background Color: 48; 48; 48
```

The format is `R; G; B` using values from 0 to 255.

| Colour | Value |
|--------|-------|
| Dark grey (current) | `48; 48; 48` |
| Black | `0; 0; 0` |
| White | `255; 255; 255` |
| Navy blue | `15; 25; 60` |

This file is read directly by RViz — no rebuild needed. Just restart the launch.

---

## 6. Splitting Parts Into Separate Namespaces (Advanced Toggling)

By default all 69 parts share the namespace `waybionic_parts`, so you can only toggle
them all at once in RViz.

To toggle sub-assemblies individually, assign each part a namespace based on its colour
group. Edit `waybionic_parts_display.py`:

Find:

```python
m.ns = 'waybionic_parts'
```

Replace with:

```python
m.ns = _group_name_for(filename)
```

And add this helper function above `_color_for`:

```python
def _group_name_for(filename: str) -> str:
    key = filename.lower().replace('-', '_').replace(' ', '_')
    for name, _, keywords in _GROUPS:
        if any(kw in key for kw in keywords):
            return name
    return 'structure'
```

After rebuilding, the RViz **Namespaces** section under **WayBionic Parts** will show
separate checkboxes for `base`, `shoulder`, `differential`, `drive`, `gears_belts`,
`joints`, and `structure` — each independently toggleable.

---

## 7. Adding a New Display Panel to RViz

**File to edit:**
`annin_ar4_moveit_config/rviz/moveit.rviz`

Open the file and find the `Displays:` list. Each item under it is one panel. Add new
panels by inserting a new block. Common useful panels:

### TF (coordinate frames)

```yaml
- Class: rviz_default_plugins/TF
  Enabled: false
  Name: TF
  Value: true
```

Set `Enabled: true` to show all the joint coordinate frames as coloured arrows.

### Robot Model (second copy, useful for ghost/target pose)

```yaml
- Class: rviz_default_plugins/RobotModel
  Enabled: false
  Name: Ghost Robot
  Robot Description: robot_description
  Visual Enabled: true
  Collision Enabled: false
  Alpha: 0.3
```

### Camera / Image feed

```yaml
- Class: rviz_default_plugins/Image
  Enabled: false
  Name: Camera Feed
  Topic:
    Value: /camera/image_raw
```

No rebuild needed for `.rviz` changes — just restart the launch.

---

## 8. Changing the Default Camera View

**File to edit:**
`annin_ar4_moveit_config/rviz/moveit.rviz`

Find the `Views:` section near the bottom:

```yaml
Views:
  Current:
    Class: rviz_default_plugins/Orbit
    Distance: 1.7173622846603394
    Focal Point:
      X: 0.013721015304327011
      Y: -0.02422581985592842
      Z: 0.12349729984998703
    Pitch: 0.5153981447219849
    Yaw: 5.413585186004639
```

| Field | What it does |
|-------|-------------|
| `Distance` | How far the camera is from the focal point (metres). Increase to zoom out. |
| `Focal Point X/Y/Z` | The 3D point the camera orbits around. Set to `0 0 0` to focus on the world origin. |
| `Pitch` | Camera tilt up/down in radians. `0` = level, `1.57` = looking straight down. |
| `Yaw` | Camera rotation left/right in radians. |

**Example — a good top-down angled view of the arm:**

```yaml
Distance: 2.5
Focal Point:
  X: 0.0
  Y: 0.0
  Z: 0.4
Pitch: 1.1
Yaw: 0.8
```

No rebuild needed — just restart the launch.

---

## 9. Changing the Grid

**File to edit:**
`annin_ar4_moveit_config/rviz/moveit.rviz`

Find the Grid display block:

```yaml
- Alpha: 0.5
  Cell Size: 1
  Class: rviz_default_plugins/Grid
  Color: 160; 160; 164
  Enabled: true
  Plane Cell Count: 10
```

| Field | What it does |
|-------|-------------|
| `Alpha` | Grid opacity (0.0–1.0) |
| `Cell Size` | Size of each grid square in metres |
| `Color` | Grid line colour as `R; G; B` (0–255) |
| `Plane Cell Count` | Number of cells per side (10 = 10×10 grid) |
| `Enabled` | Set to `false` to hide the grid entirely |

---

## 10. Rebuild & Re-source Reference

| What you changed | Rebuild needed? | Command |
|-----------------|-----------------|---------|
| `waybionic_parts_display.py` | Yes | `colcon build --packages-select annin_ar4_driver` |
| `ar_macro.xacro` (URDF colours) | Yes | `colcon build --packages-select annin_ar4_description` |
| `moveit.rviz` | No | Just restart the launch |
| `moveit.launch.py` / `demo.launch.py` | No | Just restart the launch |
| Added new STL to `meshes/waybionic/` | Yes | `colcon build --packages-select annin_ar4_description` |

Always run after any rebuild:

```bash
source install/setup.bash
```
