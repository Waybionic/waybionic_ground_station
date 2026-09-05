# Waybionic Model Import & Validation Runbook

How to import, run, and validate a robot model in this workspace without editing
the launch files. Run every command from the **workspace root** — the folder
containing `waybionic_bringup/` and `waybionic_description/`.

## Models in this package

Both live in `waybionic_description/urdf/`:

| File | Role | Meshes |
|------|------|--------|
| `full_arm_mar24.urdf` | **Default.** The real arm — a 5-link serial chain `base_link → shoulder → elbow → forearm → wrist` with articulated (revolute/continuous) joints. | 5 STLs in `meshes/` |
| `waybionic_placeholder.urdf` | Fallback / test asset. A primitive box + cylinder on one revolute joint. | **None** — pure URDF primitives, always loads |

The real arm's meshes are the only files kept in `waybionic_description/meshes/`:
`base_link.STL`, `shoulder.STL`, `elbow.STL`, `forearm.STL`, `wrist.STL`.

## 1. Import files

- **URDF/Xacro** (`.urdf`, `.xacro`) → `waybionic_description/urdf/`
- **Meshes** (`.stl`, `.dae`) → `waybionic_description/meshes/`

Inside the URDF, reference meshes with the ROS package path, e.g.
`<mesh filename="package://waybionic_description/meshes/base_link.STL"/>`.

## 2. Build

These are `ament_cmake` packages that *copy* files into `install/` at build
time, so **rebuild after any change** to a URDF, mesh, or launch file — edits in
the source tree are invisible to `ros2 launch` until you do.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select waybionic_description waybionic_bringup
source install/setup.bash
```

If packages were renamed/removed (e.g. after a merge), do a clean rebuild so
stale copies don't linger: `rm -rf build install log && colcon build`.

## 3. Run

`display.launch.py` defaults to the real arm and opens RViz (pre-configured with
`waybionic.rviz`) plus the Joint State Publisher GUI for driving the joints.

```bash
# Real arm (default)
ros2 launch waybionic_bringup display.launch.py

# Placeholder (fallback / test) — needs no meshes
ros2 launch waybionic_bringup display.launch.py \
  model:=$(ros2 pkg prefix waybionic_description --share)/urdf/waybionic_placeholder.urdf

# Any other model — no need to edit the launch file
ros2 launch waybionic_bringup display.launch.py \
  model:=$(ros2 pkg prefix waybionic_description --share)/urdf/YOUR_FILE.urdf
```

The `model` argument accepts a plain `.urdf` (read directly) or a `.xacro`
(expanded via `xacro`). If a model doesn't appear, errors print in the terminal.

## 4. Test & validate

Run these from the workspace root after building. Steps 4.1–4.4 are automated
(no GUI); 4.5 is the manual RViz/joint check. Expected results below are from the
last verified run.

### 4.1 Structural check — `check_urdf`

Needs `liburdfdom-tools` (`sudo apt install liburdfdom-tools`).

```bash
check_urdf install/waybionic_description/share/waybionic_description/urdf/full_arm_mar24.urdf
check_urdf install/waybionic_description/share/waybionic_description/urdf/waybionic_placeholder.urdf
```

**Expect:** `Successfully Parsed XML` and, for the arm, **`root Link: world`** with
the chain `world → base_link → shoulder → elbow → forearm → wrist`. The `world`
root is what stops KDL from ignoring `base_link`'s inertia — if the root prints as
`base_link`, the massless `world` root link is missing.

### 4.2 Build + unit tests

```bash
colcon build                 # or select the description, bringup, MoveIt, and RViz packages
colcon test
colcon test-result --all
```

**Expect:** build finishes with no errors; `colcon test-result` ends with
`0 errors, 0 failures` (last run: **52 tests, 0 failures** across the workspace).

### 4.3 KDL root-inertia check (headless)

Confirms the "root link has inertia — KDL ignores it" warning is gone.

```bash
if ! rsp_prefix="$(ros2 pkg prefix robot_state_publisher 2>&1)"; then
  printf '%s\n' "$rsp_prefix"
  echo "ERROR — robot_state_publisher package is unavailable"
  exit 1
fi
rsp_executable="$rsp_prefix/lib/robot_state_publisher/robot_state_publisher"
if [ ! -x "$rsp_executable" ]; then
  echo "ERROR — robot_state_publisher executable is missing"
  exit 1
fi

kdl_log="$(mktemp)"
"$rsp_executable" \
  install/waybionic_description/share/waybionic_description/urdf/full_arm_mar24.urdf \
  >"$kdl_log" 2>&1 &
kdl_pid=$!
sleep 5
if kill -0 "$kdl_pid" 2>/dev/null; then
  kdl_was_running=true
  kill -INT "$kdl_pid" 2>/dev/null || kdl_was_running=false
else
  kdl_was_running=false
fi
if wait "$kdl_pid" 2>/dev/null; then
  kdl_status=0
else
  kdl_status=$?
fi
kdl_output="$(cat "$kdl_log")"
rm -f "$kdl_log"
printf '%s\n' "$kdl_output"
if [ "$kdl_was_running" != true ] \
  || { [ "$kdl_status" -ne 0 ] && [ "$kdl_status" -ne 130 ]; }; then
  echo "ERROR — robot_state_publisher exited unexpectedly (status $kdl_status)"
  exit 1
elif ! printf '%s\n' "$kdl_output" | grep -q 'Robot initialized'; then
  echo "ERROR — robot_state_publisher did not initialize within 5 seconds"
  exit 1
elif printf '%s\n' "$kdl_output" | grep -qiE 'root link.*inertia|KDL.*inertia'; then
  echo "ERROR — KDL root-inertia warning found"
  exit 1
else
  echo "OK — no KDL root-inertia warning"
fi
```

**Expect:** `OK — no KDL root-inertia warning` and `Robot initialized`.

### 4.4 Part & mesh audit (simulation running in another terminal)

Don't count parts by eye — they range from a ~30 cm housing to a few-mm screw.

```bash
if ! robot_description="$(
  ros2 param get /robot_state_publisher robot_description 2>&1
)"; then
  printf '%s\n' "$robot_description"
  echo "ERROR — could not read the live robot_description parameter"
  exit 1
fi

# Robot links in the LIVE model loaded by RViz, excluding only the world frame
printf '%s\n' "$robot_description" \
  | grep -oE '<link name="[^"]+"' | sed -E 's/<link name="//;s/"//' \
  | grep -vE '^world$' | wc -l

# Confirm every referenced mesh exists in the installed package RViz is using
if ! description_share="$(ros2 pkg prefix --share waybionic_description 2>&1)"; then
  printf '%s\n' "$description_share"
  echo "ERROR — installed waybionic_description package is unavailable"
  exit 1
fi
mesh_root="$description_share/meshes"
mesh_names="$(
  printf '%s\n' "$robot_description" \
    | grep -oE 'meshes/[^"]+\.STL' | sed 's#meshes/##' | sort -u
)"
if [ -z "$mesh_names" ]; then
  echo "ERROR — live robot_description does not reference any STL meshes"
  exit 1
fi
missing_mesh=false
while IFS= read -r m; do
  if [ -f "$mesh_root/$m" ]; then
    echo "OK   $m"
  else
    echo "MISS $m"
    missing_mesh=true
  fi
done <<< "$mesh_names"
[ "$missing_mesh" = false ] || exit 1
```

A missing mesh still parses and still counts as a link — it just renders
invisibly — so check disk presence separately.

### 4.5 Joint check (RViz + Joint State Publisher GUI)

```bash
ros2 launch waybionic_bringup display.launch.py
```

Drive the bounded revolute sliders through their full ranges and move continuous
`joint3` through representative positive and negative angles. Confirm each link
rotates about the intended axis. Movable joints in `full_arm_mar24.urdf` (all
axis `[0 0 1]`, placeholder limits `effort=100 velocity=1`):

| Joint | Type | Moves | Range | Notes / known limitations |
|-------|------|-------|-------|---------------------------|
| `joint1` | revolute | `base_link → shoulder` | ±3.14 rad | limits are exporter defaults, not real RoM |
| `joint2` | revolute | `shoulder → elbow` | ±3.14 rad | limits are exporter defaults, not real RoM |
| `joint3` | continuous | `elbow → forearm` | unbounded | `continuous` = no limit; bound it if the real joint is limited |
| `joint4` | revolute | `forearm → wrist` | ±3.14 rad | wrist is a **differential** (pitch+roll) modeled as one joint — may need 2 |

`world_to_base` is `fixed` (not movable). Record any joint that rotates the wrong
way (bad `<axis>`) or exceeds its true range **by exact joint name**.

---

*Model provenance:* `full_arm_mar24.urdf` was exported from the
`full-arm-mar24.SLDASM` SolidWorks assembly via the `sw2urdf` exporter. Joint
axes and limits are authored in the URDF (they can't be recovered from STLs).
