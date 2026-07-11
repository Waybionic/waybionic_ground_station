# How the WayBionic Arm URDF Was Exported from SolidWorks

**Date:** 2026-07-11
**Source assembly:** `full-arm-mar24.SLDASM` (March 24 revision)
**Source folder:** `FULL-ARM_redownload-march8/` (complete native CAD — `.SLDASM` + all `.SLDPRT` parts)
**Tool:** SolidWorks to URDF Exporter (`sw2urdf`), v1.6.x — `Tools → Export as URDF`

---

## Why this was needed

An STL file contains **only geometry** — no origin, no joints, no part-to-part
relationships. Loading the 90 loose STLs gave us the shapes but no positioning,
so every part stacked at the origin `(0,0,0)`. The positioning and joint
information lives only in the **SolidWorks assembly** (its mates), so the URDF
had to be generated from the native `.SLDASM`, not from the STLs.

See also: [mechanical_team_urdf_positioning_request.md](./mechanical_team_urdf_positioning_request.md)

## Prerequisites

- **Windows** machine (SolidWorks does not run on Linux).
- **SolidWorks** with the **`sw2urdf`** add-in installed
  (`Tools → Add-Ins…` → tick "SOLIDWORKS to URDF Exporter").
- The **complete assembly folder** open — `full-arm-mar24.SLDASM` together with
  every referenced `.SLDPRT`. Opening the `.SLDASM` without its parts produces
  red-⊗ "unresolved component" errors and a broken export.

## A prior failed export (what NOT to do)

An earlier attempt (2026-06-27) produced an **empty** package:
`full-arm-mar24-…/full-arm-mar24/` had **1 link, 0 joints, and no meshes**.
Cause: the exporter was run with only the default `base_link` and **no tree
built** before hitting export. The export log also showed
`Found 0 CoordSys features` — no reference coordinate systems/axes were defined
in the CAD. Lesson: **you must build the full link/joint tree before exporting.**

---

## The link / joint tree

The arm is modeled as a serial chain of **base + 4 moving segments**, matching the
four numbered subassembly folders in the source
(`1-bottom-to-elbow → 2-carrier-to-pulley → 3-key-to-diff → 4-Differential`):

```
base_link                                    [fixed base]
  └─(joint1: revolute)──► shoulder
       └─(joint2: revolute)──► elbow
            └─(joint3: revolute)──► forearm
                 └─(joint4: revolute)──► wrist   [differential]
```

### Component-to-link mapping (all 17 top-level components)

| Link | Parent | Joint type | SolidWorks components |
|---|---|---|---|
| `base_link` | — | (root) | `Base_Funnel (1)-2`, `Bridget - Base V2-1`, `pad-Jun-1` |
| `shoulder` | `base_link` | revolute | `baseToShoulder-newDim-Mar24-1`, `shoulder_elbow+stepper_housing_NB-1` |
| `elbow` | `shoulder` | revolute | `baseToShoulder-newDim-Mar24-2`, `90_degree_elbow_V4_NB-1`, `carrier_to_pulley_feb28-1` |
| `forearm` | `elbow` | revolute | `key-to-diff-mar24-1` |
| `wrist` | `forearm` | revolute | `DifferentialHousing-3`, `CapForDifferentialV2_Mar14-1`, `straight bevel pinion_iso-1/-2/-3`, `GT2 60T b5 (6mm)-1/-2`, `BioMedRod-1` |

**Modeling principle:** one link = a set of parts that move together rigidly;
one joint = where two links rotate relative to each other. Gears, pulleys, and
housings on the same segment are grouped into that segment's link rather than
modeled as separate joints.

---

## Export procedure (step by step)

1. Open `full-arm-mar24.SLDASM` from `FULL-ARM_redownload-march8/` (full part set).
2. Rebuild and clear any errors: **`Ctrl+Q`**, confirm no red-⊗ components, then **save**.
3. **`Tools → Export as URDF`** to open the exporter panel.
4. Build the tree **top-down**, one link at a time:
   - **`base_link`**: click the components box → select the 3 base parts →
     **Global Origin = Automatically Generate** → set **child count = 1**.
   - For each child (`shoulder`, `elbow`, `forearm`, `wrist`): click it in the
     tree → set the **name** → select its components → **Joint Type = `revolute`**
     → set the **Axis** by selecting the rotating shaft's edge/reference axis →
     set its **child count = 1** (the last link, `wrist`, gets `0`).
5. **Preview and Export** → choose an output folder. This writes a ROS package:
   `urdf/<name>.urdf`, a `meshes/` folder of positioned STLs, `config/`, and
   `launch/`. The tree is also saved back into the `.SLDASM` (and as a `.csv`) so
   it can be reloaded via **Load Configuration** on future re-exports.

## Bringing the export into the ROS workspace

Copy the exported `urdf/` and `meshes/` into
[waybionic_description/](../waybionic_description/), then on the Linux side:

```bash
cd /home/richard/waybionic_ground_station/waybionic_ground_station
colcon build --packages-select waybionic_description waybionic_bringup
source install/setup.bash
ros2 launch waybionic_bringup assembled.launch.py
```

Drag the Joint State Publisher GUI sliders to articulate the revolute joints.

---

## Open items / caveats to verify with the mechanical team

1. **Two `baseToShoulder` subassemblies** (`-1`, `-2`): assigned one to `shoulder`
   and one to `elbow` here — confirm which physical pivot each represents.
2. **The wrist is a differential** (bevel pinions): a differential typically
   produces **two** motions (pitch + roll), so `wrist` may need **2 joints**, not
   one. A single revolute is used as a first pass to get it rendering.
3. **Joint axes** are chosen manually during export (the CAD had no reference
   coordinate systems). If a joint rotates the wrong way in RViz, re-run the
   exporter and correct that joint's axis.
4. **Revolute limits** (lower/upper angle) were not defined by the CAD and should
   be set to the arm's real range of motion.
