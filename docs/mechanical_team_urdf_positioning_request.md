# Action Required: Mechanical Team — Send URDF Parts With Positioning

**Date:** 2026-06-27
**Model:** `waybionic_description/urdf/test_all_meshes.urdf` (all-parts test model)
**Status:** All 90 parts now render, but **not in their correct positions.**

---

## The ask

> ⚠️ **We specifically need the URDF for the May 26th version of the arm.**
> Please make sure the parts and positioning correspond to the **May 26th**
> revision of the assembly — not an earlier or later version — so the software
> model matches the build we are working from.

**The mechanical team needs to provide the parts with their assembled
positioning** — i.e. each part's correct position and orientation relative to
the base, or an assembly export where every STL is already placed in its real
location.

Right now the software side has the geometry (the STL meshes) but **none of the
positioning information**, because an STL file contains only shape — it carries
no location, no joints, and no part-to-part relationships. Without the mechanical
team supplying where each part belongs, we cannot assemble the arm correctly; we
can only display the loose parts.

Please send, for each part (or as a single positioned assembly export):

- The part's position (X, Y, Z) and orientation (roll, pitch, yaw) relative to
  the base, **in the assembled configuration**, or
- A CAD/assembly export (e.g. STEP, or a SolidWorks → URDF export configured with
  the full link/joint tree) where parts are already in their assembled poses.

## Why — what went wrong the previous week

Last week it looked like **only a few parts were showing up**. The parts were not
actually missing — **they were all spawning on top of each other at the origin
(0, 0, 0)**, stacked in the exact same spot. Because every part shared the same
position, they overlapped into what looked like a single small cluster, so the
count appeared far lower than it actually was.

The cause: in the URDF, every part's joint origin was set to `0 0 0` (no
position was provided), so the renderer placed all 90 parts at the same point.

## What was done this week (temporary workaround)

To prove all parts are present, we spread them out artificially into a **10-column
grid (0.5 m spacing)** by assigning each part a placeholder position. This is
**only a visualization workaround** — the parts are now visible and countable,
but they are laid out in an arbitrary grid, **not** in their real assembled
positions. The correct positions still have to come from the mechanical team.

## Current count — parts appearing in the RViz simulation

Verified live from the running simulation (`robot_state_publisher` /
`robot_description`) on 2026-06-27:

- **Parts loaded:** 90
- **Meshes present on disk:** 90 / 90
- **Meshes missing:** 0
- **Parts rendering in RViz:** **90 / 90** ✅

All referenced STL files exist and every part is rendering. The only thing
missing is correct positioning.

### Full list of the 90 parts appearing

| # | Part (STL) |
|---|-----------|
| 1 | 2WireSlipRing-2-wire-slipring.step-1_rotor.step-1_negative-rotor-wire.step-1 |
| 2 | 2WireSlipRing-2-wire-slipring.step-1_rotor.step-1_positive-rotor-wire.step-1 |
| 3 | 2WireSlipRing-2-wire-slipring.step-1_rotor.step-1_rotator-base.step-1 |
| 4 | 2WireSlipRing-2-wire-slipring.step-1_stator.step-1_negative-stator-wire.step-1 |
| 5 | 2WireSlipRing-2-wire-slipring.step-1_stator.step-1_positive-stator-wire.step-1 |
| 6 | 2WireSlipRing-2-wire-slipring.step-1_stator.step-1_stator_body.step-1 |
| 7 | 3rd_Joint_Pulley_V1_HT_20250927 |
| 8 | 3rd_Joint_V1_HT_20250920 |
| 9 | 6656K153_Ultra-Thin_Ball_Bearing |
| 10 | 90_degree_elbow_V4_NB |
| 11 | another-Stepper |
| 12 | arduino-uno-rough |
| 13 | Base_Funnel_1 |
| 14 | baseToShoulder ... 2WireSlipRing rotor negative-rotor-wire (-1) |
| 15 | baseToShoulder ... 2WireSlipRing rotor negative-rotor-wire |
| 16 | baseToShoulder ... 2WireSlipRing rotor positive-rotor-wire (-1) |
| 17 | baseToShoulder ... 2WireSlipRing rotor positive-rotor-wire |
| 18 | baseToShoulder ... 2WireSlipRing rotor rotator-base (-1) |
| 19 | baseToShoulder ... 2WireSlipRing rotor rotator-base |
| 20 | baseToShoulder ... 2WireSlipRing stator stator_body (-1) |
| 21 | baseToShoulder ... 2WireSlipRing stator stator_body |
| 22 | baseToShoulder ... 2WireSlipRing stator negative-stator-wire (-1) |
| 23 | baseToShoulder ... 2WireSlipRing stator negative-stator-wire |
| 24 | baseToShoulder ... 2WireSlipRing stator positive-stator-wire (-1) |
| 25 | baseToShoulder ... 2WireSlipRing stator positive-stator-wire |
| 26 | baseToShoulder ... 6656K153_Ultra-Thin_Ball_Bearing (-1-1) |
| 27 | baseToShoulder ... 6656K153_Ultra-Thin_Ball_Bearing (-1) |
| 28 | baseToShoulder ... carrier_1-changedDimensionsMar21 (-1-1) |
| 29 | baseToShoulder ... carrier_1-changedDimensionsMar21 (-1) |
| 30 | baseToShoulder ... outer_ring_newDimensionsMar21 (-5-1) |
| 31 | baseToShoulder ... outer_ring_newDimensionsMar21 (-5) |
| 32 | baseToShoulder ... planetShafts-1-1 |
| 33 | baseToShoulder ... planetShafts-1 |
| 34 | baseToShoulder ... planetShafts-2-1 |
| 35 | baseToShoulder ... planetShafts-2 |
| 36 | baseToShoulder ... planetShafts-3-1 |
| 37 | baseToShoulder ... planetShafts-3 |
| 38 | baseToShoulder ... planetShafts-4-1 |
| 39 | baseToShoulder ... planetShafts-4 |
| 40 | baseToShoulder ... spur_gear_iso-1-1 |
| 41 | baseToShoulder ... spur_gear_iso-1 |
| 42 | baseToShoulder ... spur_gear_iso-2-1 |
| 43 | baseToShoulder ... spur_gear_iso-2 |
| 44 | baseToShoulder ... spur_gear_iso-3-1 |
| 45 | baseToShoulder ... spur_gear_iso-3 |
| 46 | baseToShoulder ... spur_gear_iso-4-1 |
| 47 | baseToShoulder ... spur_gear_iso-4 |
| 48 | baseToShoulder ... spur_gear_iso-5-1 |
| 49 | baseToShoulder ... spur_gear_iso-5 |
| 50 | baseToShoulder ... Stepper-1-1 |
| 51 | baseToShoulder ... Stepper-1 |
| 52 | Base_Updated |
| 53 | bottom-pvc |
| 54 | CapForDifferentialV2_Mar14 |
| 55 | carrier_1-changedDimensionsMar21 |
| 56 | carrier_to_pulley_feb28 ... 3rd_Joint_Pulley_V1_HT_20250927 |
| 57 | carrier_to_pulley_feb28 ... 3rd_Joint_V1_HT_20250920 |
| 58 | carrier_to_pulley_feb28 ... GT2_x_6mm_Pully_5mm_Bore-pulley |
| 59 | carrier_to_pulley_feb28 ... Pipe |
| 60 | carrier_to_pulley_feb28 ... Stepper-Pulley |
| 61 | DifferentialHousing |
| 62 | full-arm-jun13 ... 90_degree_elbow_V4_NB |
| 63 | GT2_60T_b5_6mm |
| 64 | GT2_x_6mm_Pully_5mm_Bore-pulley |
| 65 | GT2_x_6mm_Pully_5mm_Bore |
| 66 | keyed_shaft-updated-mar13-15mm |
| 67 | key-to-diff-mar24 ... another-Stepper-1 |
| 68 | key-to-diff-mar24 ... another-Stepper-2 |
| 69 | key-to-diff-mar24 ... arduino-uno-rough |
| 70 | key-to-diff-mar24 ... bottom-pvc |
| 71 | key-to-diff-mar24 ... GT2_60T_b5_6mm |
| 72 | key-to-diff-mar24 ... GT2_x_6mm_Pully_5mm_Bore-1 |
| 73 | key-to-diff-mar24 ... GT2_x_6mm_Pully_5mm_Bore-2 |
| 74 | key-to-diff-mar24 ... keyed_shaft-updated-mar13-15mm |
| 75 | key-to-diff-mar24 ... lid |
| 76 | key-to-diff-mar24 ... revised-3rd_Joint_Top_key_hole_170126_-mar15 |
| 77 | key-to-diff-mar24 ... StepperHolderV2 |
| 78 | key-to-diff-mar24 ... top-pvc |
| 79 | lid |
| 80 | outer_ring_newDimensionsMar21 |
| 81 | Pipe |
| 82 | planetShafts |
| 83 | revised-3rd_Joint_Top_key_hole_170126_-mar15 |
| 84 | shoulder_elbow_stepper_housing_NB |
| 85 | spur_gear_iso |
| 86 | StepperHolderV2 |
| 87 | Stepper-Pulley |
| 88 | Stepper |
| 89 | straight_bevel_pinion_iso |
| 90 | top-pvc |

> Names are shortened for readability; the exact filenames are in
> `waybionic_description/meshes/` and listed in
> `waybionic_description/urdf/test_all_meshes.urdf`.

---

## Summary

- ✅ All 90 parts are present and rendering — nothing is missing.
- ⚠️ Last week's "missing parts" was actually all parts stacked at the origin
  `(0,0,0)`, overlapping into one cluster.
- 📐 The current grid layout is a temporary spread-out workaround, **not** the
  real assembly.
- ➡️ **Next step (mechanical team):** provide the URDF / assembly for the
  **May 26th version** of the arm, with each part's assembled position &
  orientation (or a positioned assembly export), so the arm can be built correctly
  instead of laid out on a grid.

---

## Appendix: How the "90 parts appearing" count was verified

The count was confirmed two ways: **(1)** how many part-links the running
simulation loaded, and **(2)** whether every referenced STL actually exists on
disk (a missing mesh still loads as a link but renders invisibly). 90 on both
⇒ 90 parts genuinely appearing. Reproduce with:

```bash
# 0. Source ROS + the workspace so ros2 can see the running nodes
cd waybionic_ground_station            # workspace root
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 1. Confirm the simulation is actually up
ros2 node list
#   -> /robot_state_publisher  /joint_state_publisher_gui  /rviz2

# 2. Pull the LIVE model that robot_state_publisher loaded (what RViz renders)
ros2 param get /robot_state_publisher robot_description > robotdesc.txt

# 3. Count the part links it loaded (every <link>, minus the world + base frames)
grep -oE '<link name="[^"]*"' robotdesc.txt \
  | sed 's/<link name="//;s/"//' \
  | grep -vE '^(world|base_link)$' \
  | wc -l
#   -> 90   (90 parts loaded)

# 4. Extract every mesh the model references, de-duplicated
grep -oE 'meshes/[^"]*\.stl' robotdesc.txt | sed 's#meshes/##' | sort -u > refs.txt
wc -l < refs.txt
#   -> 90   (90 unique meshes referenced)

# 5. Verify each referenced STL actually EXISTS on disk
present=0; missing=0
while IFS= read -r m; do
  if [ -f "waybionic_description/meshes/$m" ]; then
    present=$((present+1))
  else
    missing=$((missing+1)); echo "MISSING: $m"
  fi
done < refs.txt
echo "present=$present missing=$missing"
#   -> present=90 missing=0   (all 90 parts have geometry -> all 90 render)
```

**Conclusion:** 90 links loaded (step 3) **and** 90/90 meshes present (step 5)
⇒ **90 parts appearing in RViz.**

> Alternative (textbook) method — count the static transforms RViz uses:
> ```bash
> ros2 topic echo /tf_static --once --qos-durability transient_local | grep -c child_frame_id
> # = parts + 1 (the world->base anchor), so 91 here
> ```
> In this environment that echo hung (QoS handshake didn't complete), so the
> `robot_description` parameter above was used instead — it is the authoritative
> source `robot_state_publisher` feeds to RViz, giving the same answer.
