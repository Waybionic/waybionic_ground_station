# waybionic_moveit_config

MoveIt 2 configuration for the WayBionic arm (`full_arm_mar24.urdf`).

## Quickstart

Clone the repository once and use that directory as the colcon workspace:

```bash
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd waybionic_ground_station
```

### Ubuntu 24.04 (ROS 2 Jazzy)

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch waybionic_moveit_config demo.launch.py
```

### macOS (Apple Silicon)

Use the repository helper so the RoboStack environment and Bash workspace
overlay are applied correctly. Run `./scripts/macos.sh setup` once to create the
environment, then use:

```bash
./scripts/macos.sh build
./scripts/macos.sh run ros2 launch waybionic_moveit_config demo.launch.py
```

RViz opens with the MotionPlanning display already configured for the `arm`
group. Use the **Planning** tab: pick a start/goal state (or a named pose),
press **Plan**, then **Execute**.

To run the automatic Cartesian demonstration at startup, append
`auto_demo:=true` to the launch command. For example, on Ubuntu:

```bash
ros2 launch waybionic_moveit_config demo.launch.py auto_demo:=true
```

The arm first moves to its ready pose, then uses MoveIt's `/compute_ik`
service to move the wrist along X, Y, and Z. RViz shows a red X axis, green Y
axis, blue Z axis, and a yellow target. Click **Replay XYZ Demo** in the IK Demo
panel to run it again. For manual IK, drag a colored goal-state arrow and click
**Plan & Execute**. MotionPlanning uses 50% of the model's velocity and
acceleration limits by default; those sliders can still be adjusted in RViz.

Headless (no RViz), useful for testing:

```bash
ros2 launch waybionic_moveit_config demo.launch.py use_rviz:=false
```

On macOS, prefix the Ubuntu `ros2 launch` examples above with
`./scripts/macos.sh run`.

## Important: this arm has 4 DOF

`joint1`, `joint2`, `joint3` and `joint4` are all revolute — four degrees of
freedom total. **A 4-DOF arm cannot reach an arbitrary 6-DOF pose.**

Consequences you need to know about:

- IK is configured **position-only** (`position_only_ik: true` in
  `config/kinematics.yaml`). Goals are matched on XYZ; end-effector orientation
  is whatever the arm happens to produce.
- The RViz config sets `MoveIt_Allow_Approximate_IK: true`. Without it, dragging
  the interactive marker almost never finds a solution.
- Planning in **joint space** (the Joints tab, or named poses) is fully reliable
  and is the recommended workflow for this arm.
- The RViz config also sets `MoveIt_Use_Constraint_Aware_IK: true`, so goal
  states that put the arm through itself are rejected instead of displayed.

## Joint limits are UNVERIFIED

**Mechanical has not supplied travel, velocity or effort data for any joint.**

The SolidWorks exporter emitted `lower="-3.14" upper="3.14" effort="100"
velocity="1"` identically for all four joints, and `joint3` as `continuous`
(unbounded). Those were placeholders, not measurements — no real mechanism has
four joints with matching symmetric travel and matching dynamics.

Current state of the model:

- `joint3` is now `revolute` rather than `continuous`. The slip ring that could
  justify endless rotation is in the `baseToShoulder` assembly (the base), not
  at the belt-driven elbow-to-forearm joint.
- All four joints are limited to ±1.5708 rad (±90°). This is a deliberately
  conservative interim value, chosen because it is a strict subset of the
  exporter's original ±3.14 and therefore cannot permit any motion the model did
  not already permit. **It is not a measurement.**
- `effort` and `velocity` are unchanged exporter defaults and remain unverified.

**Do not treat the workspace RViz displays as actually reachable**, and do not
drive hardware from these numbers. To replace them, mechanical needs to supply:

1. Per-joint travel for `joint1`–`joint4` in degrees, min and max **separately**
   — real limits are rarely symmetric.
2. Whether `joint3` is genuinely bounded, and by what.
3. Max velocity (rad/s) and effort/torque (N·m) per joint.
4. Any joint *pairs* that collide before reaching their individual limits.

## What runs

| Component | Purpose |
|---|---|
| `robot_state_publisher` | Publishes TF from the URDF |
| `ros2_control_node` | Mock hardware (`mock_components/GenericSystem`) |
| `joint_state_broadcaster` | Publishes `/joint_states` |
| `arm_controller` | `JointTrajectoryController`, executes planned paths |
| `move_group` | Planning, IK, collision checking |
| `ik_xyz_demo` | Runs and replays the Cartesian XYZ demonstration |
| `rviz2` | MotionPlanning UI |

No physical hardware is needed. The mock system echoes commands back as state,
so **Execute** animates the arm in RViz.

## Layout

```text
srdf/waybionic.srdf                  # Planning group, named poses, collision matrix
config/kinematics.yaml               # KDL, position-only IK
config/joint_limits.yaml             # Velocity/acceleration limits
config/ompl_planning.yaml            # Focused OMPL RRTConnect configuration
config/moveit_controllers.yaml       # move_group -> ros2_control handoff
config/ros2_controllers.yaml         # controller_manager + JTC
urdf/waybionic_moveit.urdf.xacro     # Includes base URDF, adds ros2_control
launch/demo.launch.py                # Brings up everything
rviz/moveit.rviz                     # MotionPlanning preconfigured for group "arm"
```

The xacro in `urdf/` includes the shared robot description and layers
`<ros2_control>` on top. High-resolution STL files are visual-only; lightweight
boxes and cylinders provide portable, fast collision checking.

## Known limitations

- **The collision matrix only disables adjacent link pairs.** It was written by
  hand, not sampled by the MoveIt Setup Assistant. The primitive collision
  envelopes test clean at the demo poses, but production hardware should still
  regenerate the matrix across the complete workspace with:
  ```bash
  ros2 launch moveit_setup_assistant setup_assistant.launch.py
  ```
  Load `urdf/waybionic_moveit.urdf.xacro`, then use the Self-Collisions pane.
- **No end effector is defined** — there is no gripper in the URDF.
- `No 3D sensor plugin(s) defined for octomap updates` is logged as an ERROR at
  startup. It is harmless: there is no depth camera in this setup.

  
