# waybionic_moveit_config

MoveIt 2 configuration for the WayBionic arm (`full_arm_mar24.urdf`).

## Quickstart

```bash
cd ~/waybionic_ground_station/waybionic_ground_station
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch waybionic_moveit_config demo.launch.py
```

RViz opens with the MotionPlanning display already configured for the `arm`
group. Use the **Planning** tab: pick a start/goal state (or a named pose),
press **Plan**, then **Execute**.

For an automatic Cartesian demonstration, run:

```bash
./scripts/macos.sh run ros2 launch waybionic_moveit_config demo.launch.py auto_demo:=true
```

The arm first moves to its ready pose, then uses MoveIt's `/compute_ik`
service to move the wrist along X, Y, and Z. RViz shows a red X axis, green Y
axis, blue Z axis, and a yellow target. After the sequence, use the goal-state
interactive marker for manual IK: drag a colored arrow, then click **Plan &
Execute**.

Headless (no RViz), useful for testing:

```bash
ros2 launch waybionic_moveit_config demo.launch.py use_rviz:=false
```

## Important: this arm has 4 DOF

`joint1`, `joint2`, `joint4` are revolute and `joint3` is continuous — four
degrees of freedom total. **A 4-DOF arm cannot reach an arbitrary 6-DOF pose.**

Consequences you need to know about:

- IK is configured **position-only** (`position_only_ik: true` in
  `config/kinematics.yaml`). Goals are matched on XYZ; end-effector orientation
  is whatever the arm happens to produce.
- The RViz config sets `MoveIt_Allow_Approximate_IK: true`. Without it, dragging
  the interactive marker almost never finds a solution.
- Planning in **joint space** (the Joints tab, or named poses) is fully reliable
  and is the recommended workflow for this arm.

## What runs

| Component | Purpose |
|---|---|
| `robot_state_publisher` | Publishes TF from the URDF |
| `ros2_control_node` | Mock hardware (`mock_components/GenericSystem`) |
| `joint_state_broadcaster` | Publishes `/joint_states` |
| `arm_controller` | `JointTrajectoryController`, executes planned paths |
| `move_group` | Planning, IK, collision checking |
| `rviz2` | MotionPlanning UI |

No physical hardware is needed. The mock system echoes commands back as state,
so **Execute** animates the arm in RViz.

## Layout

```text
srdf/waybionic.srdf                  # Planning group, named poses, collision matrix
config/kinematics.yaml               # KDL, position-only IK
config/joint_limits.yaml             # Velocity/acceleration limits
config/ompl_planning.yaml            # OMPL planners (RRTConnect default)
config/moveit_controllers.yaml       # move_group -> ros2_control handoff
config/ros2_controllers.yaml         # controller_manager + JTC
urdf/waybionic_moveit.urdf.xacro     # Includes base URDF, adds ros2_control
launch/demo.launch.py                # Brings up everything
rviz/moveit.rviz                     # MotionPlanning preconfigured for group "arm"
```

`waybionic_description` is **not modified**. The xacro in `urdf/` includes the
existing URDF unchanged and layers `<ros2_control>` on top.

## Known limitations

- **The collision matrix only disables adjacent link pairs.** It was written by
  hand, not sampled by the MoveIt Setup Assistant. If planning fails immediately
  with "start state in collision", non-adjacent links (likely `base_link` against
  the arm links, given the SolidWorks export offsets) may be overlapping in the
  meshes. Regenerate a proper matrix with:
  ```bash
  ros2 launch moveit_setup_assistant setup_assistant.launch.py
  ```
  Load `urdf/waybionic_moveit.urdf.xacro`, then use the Self-Collisions pane.
- **Collision geometry is the full visual STL** for every link. It works, but
  the demo strips those placeholder collision meshes before loading MoveIt.
  The visual STL model is unchanged. Add simplified convex collision meshes
  before using this configuration for collision-aware planning.
- **No end effector is defined** — there is no gripper in the URDF.
- `No 3D sensor plugin(s) defined for octomap updates` is logged as an ERROR at
  startup. It is harmless: there is no depth camera in this setup.
