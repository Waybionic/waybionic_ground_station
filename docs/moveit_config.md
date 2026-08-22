# Waybionic MoveIt Config

What the `waybionic_moveit_config` package is, why it was added as a separate
package, and how to run it. Run every command from the **workspace root** — the
folder containing `waybionic_bringup/` and `waybionic_description/`.

## Why this package exists

Adding a **MotionPlanning** display to the RViz started by
`waybionic_bringup display.launch.py` fails with:

```
[rviz2] Could not find parameter robot_description_semantic and did not receive
        robot_description_semantic via std_msgs::msg::String subscription within
        10.000000 seconds.
Error:  Could not parse the SRDF XML File. Error=XML_ERROR_EMPTY_DOCUMENT
[rviz2.moveit.ros.rdf_loader]: Unable to parse SRDF
[rviz2.moveit.ros.planning_scene_monitor]: Robot model not loaded
```

That is one failure cascading, not three. MoveIt needs **two** descriptions of
the robot, and the workspace only produced one:

| Description | Answers | Where it comes from |
|-------------|---------|---------------------|
| **URDF** (`robot_description`) | *What is this physically?* Links, joints, meshes, masses. | `waybionic_description` — already existed |
| **SRDF** (`robot_description_semantic`) | *What does it mean?* Which joints form a planning group, named poses, which link pairs may touch. | **Was missing.** Now `waybionic_moveit_config` |

`display.launch.py` starts `robot_state_publisher`, `joint_state_publisher_gui`
and `rviz2` — no `move_group`, and no SRDF. The MotionPlanning display waits 10
seconds for a semantic description that nothing publishes, then gives up. The
empty-document parse error is the consequence of it receiving an empty string.

## Why a separate package

`display.launch.py` has a job already: show the model and jog it with sliders.
It is light, starts fast, and needs no planner. Adding MoveIt to it would make
every quick model check pay for a planning stack.

So MoveIt lives beside it. Two launch files, two purposes:

| Launch | Use it for |
|--------|------------|
| `ros2 launch waybionic_bringup display.launch.py` | Look at the model, jog joints with sliders. **Unchanged.** |
| `ros2 launch waybionic_moveit_config demo.launch.py` | Motion planning, IK, collision checking, trajectory execution. |

**No existing file was modified.** `waybionic_moveit_config/urdf/waybionic_moveit.urdf.xacro`
includes `full_arm_mar24.urdf` unchanged and layers a `<ros2_control>` block on
top, so `waybionic_description` did not have to change.

## 1. Build

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --packages-select waybionic_moveit_config --symlink-install
source install/setup.bash
```

## 2. Launch

```bash
ros2 launch waybionic_moveit_config demo.launch.py
```

RViz opens with MotionPlanning **already configured** for the `arm` group — do
not add the display by hand, that is what caused the original error.

Wait for the green `You can start planning now!` line; that is the ready signal.

To plan and move the arm, in the MotionPlanning panel:

1. **Planning** tab
2. Set **Goal State** to `ready` (named poses `home` and `ready` are defined)
3. **Plan** — the trajectory preview animates
4. **Execute** — the arm follows it

Headless, for checking the stack comes up clean:

```bash
ros2 launch waybionic_moveit_config demo.launch.py use_rviz:=false
```

`Ctrl+C` in the launch terminal shuts everything down.

## 3. What runs

Six nodes, started in this order:

| # | Node | Purpose |
|---|------|---------|
| 1 | `robot_state_publisher` | Expands the xacro and broadcasts TF |
| 2 | `ros2_control_node` | Mock hardware (`mock_components/GenericSystem`) |
| 3 | `joint_state_broadcaster` | Publishes `/joint_states` |
| 4 | `arm_controller` | `JointTrajectoryController`, executes planned paths |
| 5 | `move_group` | Planning, IK, collision checking |
| 6 | `rviz2` | MotionPlanning UI |

The mock hardware echoes position commands back as measured state, so
**no physical arm is required** — Execute animates the model in RViz.

> **Do not run `display.launch.py` at the same time.** Both publish
> `/joint_states` (one via the slider GUI, one via the broadcaster) and they
> will fight over the topic.

## 4. Important: the arm has 4 DOF

`joint1`, `joint2`, `joint4` are revolute and `joint3` is continuous — four
degrees of freedom. Placing an end effector freely in space requires six.
**Four is not enough, and no configuration file can change that.**

Two settings follow from this and are deliberate:

| Setting | Where | Why |
|---------|-------|-----|
| `position_only_ik: true` | `config/kinematics.yaml` | Match XYZ only, accept whatever orientation results. Without it KDL tries to satisfy all six numbers and reports no solution nearly every time. |
| `MoveIt_Allow_Approximate_IK: true` | `rviz/moveit.rviz` | Lets the interactive marker settle for the nearest reachable pose instead of refusing to move. |

**Plan in joint space** — the Joints tab, or the named poses. That path is exact
and reliable. Dragging the orange interactive marker works but is approximate by
construction; that is the arm's geometry, not a misconfiguration.

## 5. Package layout

```text
waybionic_moveit_config/
  srdf/waybionic.srdf                # Planning group, named poses, collision matrix
  urdf/waybionic_moveit.urdf.xacro   # Includes base URDF unchanged, adds ros2_control
  config/
    kinematics.yaml                  # KDL solver, position-only IK
    joint_limits.yaml                # Velocity / acceleration limits
    ompl_planning.yaml               # OMPL planners, RRTConnect default
    moveit_controllers.yaml          # move_group -> ros2_control handoff
    ros2_controllers.yaml            # controller_manager + JointTrajectoryController
  launch/demo.launch.py              # Starts all six nodes
  rviz/moveit.rviz                   # MotionPlanning preconfigured for group "arm"
  package.xml / CMakeLists.txt
  README.md
```

The SRDF declares:

- Planning group `arm` — chain `base_link` → `wrist`
- Named poses `home` (all zeros) and `ready`
- A fixed virtual joint anchoring the URDF root link `world`
- Four `disable_collisions` entries, one per adjacent link pair

## 6. Verification

The stack was run headless and queried over the same service interfaces RViz
uses:

| Check | Result |
|-------|--------|
| Package builds | clean |
| Xacro expands, base URDF intact | 6 links, 5 joints, 1 `ros2_control` block |
| Robot model loads in `move_group` | `full-arm-mar24-urdf-and-meshes` |
| KDL binds all four joints | `Joint weights for group 'arm': 1 1 1 1` |
| Controllers activate | broadcaster + `arm_controller` |
| Home pose collision-free (`/check_state_validity`) | `valid=True` |
| Joint-space plan (`/plan_kinematic_path`) | `val=1` (SUCCESS), 26 ms |

The last two matter most: `valid=True` confirms the hand-written collision rules
are not wrongly reporting self-collision, and `val=1` is MoveIt's success code
for a real four-joint plan.

Reproduce the plan check with the stack running:

```bash
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity \
  "{group_name: 'arm', robot_state: {joint_state: {name: ['joint1','joint2','joint3','joint4'], position: [0.0,0.0,0.0,0.0]}, is_diff: true}}"
```

## 7. Deliberate deviations from defaults

- **Controller rate is 50 Hz, not 100.** WSL2 cannot grant the controller
  manager realtime scheduling, so 100 Hz produced continuous
  `Overrun detected` warnings. Mock hardware does not need the headroom.
- **`robot_description` is not passed as a parameter to `ros2_control_node`.**
  On Jazzy the controller manager reads it from the topic. Note that the
  `ResourceManager has already loaded a urdf` warning this was meant to silence
  turns out to be inherent and still appears; the topic-based form is still the
  documented one, so it was kept.

## 8. Known limitations

- **The collision matrix is hand-written**, not sampled. Only the four adjacent
  link pairs are disabled, where the MoveIt Setup Assistant would sample
  thousands of random poses. It tests clean at `home` and planning succeeds, but
  a pose deep in the workspace could report a false collision. Regenerate with:
  ```bash
  ros2 launch moveit_setup_assistant setup_assistant.launch.py
  ```
  Load `urdf/waybionic_moveit.urdf.xacro`, then use the Self-Collisions pane.
- **Collision geometry is the full visual STL** for every link. Correct, but
  convex hulls or primitives would be considerably faster.
- **No end effector is defined** — there is no gripper in the URDF.

## 9. Log lines that look alarming but are not

| Line | Meaning |
|------|---------|
| `No 3D sensor plugin(s) defined for octomap updates` | Printed as a red ERROR. Only means there is no depth camera. Harmless. |
| `Could not enable FIFO RT scheduling policy` | WSL2 cannot grant realtime scheduling. Harmless. |
| `ResourceManager has already loaded a urdf` | The description arrived twice. Harmless. |
