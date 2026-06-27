# waybionic_description / urdf

URDF (Unified Robot Description Format) files describing the Waybionic arm and
its individual parts. These are consumed by `robot_state_publisher`, which reads
a URDF, publishes the `/robot_description` topic, and broadcasts a TF transform
for every joint so the model can be rendered in RViz.

Mesh files referenced here live in `../meshes/` and are pulled in via
`package://waybionic_description/meshes/<file>.stl`. All meshes are authored in
millimetres, so every `<mesh>` uses `scale="0.001 0.001 0.001"` to convert to
metres (ROS works in metres).

## Files

| File | Active? | Parts | Purpose |
|------|---------|-------|---------|
| `waybionic_assembled.urdf` | **yes** | 36 | The real assembled arm. Each STL is pre-positioned in its own coordinates, so all joint origins are `0 0 0` — the assembly offsets are baked into the meshes. Loaded by `waybionic_bringup/assembled.launch.py`. |
| `waybionic.urdf` | yes | 50 | Working model with 50 named parts (`baseToShoulder_01`, …). General-purpose description. |
| `test_all_meshes.urdf` | yes | 90 | Diagnostic file: renders **every** mesh in the package, each as its own fixed link off `base_link`. Parts are laid out in a 10-column grid (0.5 m spacing) so they don't overlap — used to confirm all meshes load. |
| `test_all_meshes.preGrid.urdf.bak` | backup | 90 | Snapshot of `test_all_meshes.urdf` *before* the grid layout, i.e. all parts at the origin (`0 0 0`). Restore point. |
| `waybionic_49parts.urdf.bak` | backup | 49 | Earlier 49-part revision of the model. Not loaded by any launch file. |
| `waybionic_placeholder.urdf.bak` | backup | 3 | Minimal placeholder arm (3 parts, includes `revolute` joints). Backup of an early simplified model. |

> `.bak` files are kept for reference/restore only — no launch file loads them.
> To use one, copy it to a `.urdf` name and rebuild the package.

## Basic URDF syntax

A URDF is XML describing a tree of **links** (rigid bodies) connected by
**joints**. Every file here follows the same shape:

```xml
<?xml version="1.0"?>
<robot name="waybionic">

  <!-- Frames: 'world' is the fixed reference, 'base_link' is the robot root -->
  <link name="world"/>
  <link name="base_link"/>
  <joint name="world_to_base" type="fixed">
    <parent link="world"/><child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- A part: a link holds the geometry -->
  <link name="my_part">
    <visual>                          <!-- what you see in RViz -->
      <geometry>
        <mesh filename="package://waybionic_description/meshes/my_part.stl"
              scale="0.001 0.001 0.001"/>
      </geometry>
    </visual>
    <collision>                       <!-- shape used for collision checking -->
      <geometry>
        <mesh filename="package://waybionic_description/meshes/my_part.stl"
              scale="0.001 0.001 0.001"/>
      </geometry>
    </collision>
  </link>

  <!-- A joint connects the part to its parent and places it in space -->
  <joint name="j_my_part" type="fixed">
    <parent link="base_link"/><child link="my_part"/>
    <origin xyz="0.5 0.0 0" rpy="0 0 0"/>
  </joint>

</robot>
```

### Key elements

- **`<link>`** — a rigid body. May contain `<visual>` (render geometry),
  `<collision>` (collision geometry), and `<inertial>` (mass/inertia, omitted
  in these files). A link with no children is just a coordinate frame
  (e.g. `world`, `base_link`).
- **`<joint>`** — connects a `<parent>` link to a `<child>` link. Every link
  except the root must be the child of exactly one joint (the tree must have no
  cycles and a single root).
  - **`type="fixed"`** — rigidly attached, no motion. All display/diagnostic
    parts here use this.
  - **`type="revolute"`** — rotates about an `<axis>` between `<limit>` bounds
    (used in the placeholder model).
  - Other types: `continuous` (revolute with no limit), `prismatic` (slides).
- **`<origin xyz="x y z" rpy="roll pitch yaw"/>`** — pose of the child relative
  to the parent. `xyz` is in metres; `rpy` is in radians. This is how a part is
  positioned — e.g. the grid layout in `test_all_meshes.urdf` works purely by
  setting each joint's `xyz`.
- **`<mesh filename="package://<pkg>/<path>" scale="..."/>`** — references an STL
  by ROS package path. `scale="0.001 0.001 0.001"` converts mm meshes to metres.

## Quick checks

```bash
# Count parts (joints excluding the world->base anchor)
grep '<joint name=' <file>.urdf | grep -vc world_to_base

# List every mesh referenced
grep -o 'meshes/[^"]*' <file>.urdf | sort -u

# Validate the XML
python3 -c "import xml.dom.minidom; xml.dom.minidom.parse('<file>.urdf')"

# Full URDF validation (links form a valid tree, etc.)
check_urdf <file>.urdf
```

## Loading in RViz

These files are installed into `share/waybionic_description/urdf/` at build
time, so after editing any URDF (or adding a new mesh) you must rebuild:

```bash
colcon build --packages-select waybionic_description
source install/setup.bash
```

Then launch one of the `waybionic_bringup` launch files
(`assembled.launch.py`, `display.launch.py`, `test_all.launch.py`), or load a
URDF directly with `robot_state_publisher` + `rviz2`. In RViz set the
**Fixed Frame** to `world` and add a **RobotModel** display.
