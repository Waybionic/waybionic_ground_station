# WayBionic MoveIt demo

The canonical setup and operating guide lives with the package:
[`waybionic_moveit_config/README.md`](../waybionic_moveit_config/README.md).

Use the MoveIt launch when you need planning, position-only IK, collision
checking, or mock trajectory execution:

```bash
ros2 launch waybionic_moveit_config demo.launch.py
```

The lighter `waybionic_bringup/display.launch.py` only displays the robot and
jogs individual joints. Do not run both launches together because they publish
competing joint states.

The MoveIt demo supplies the semantic robot description, mock ros2_control
hardware, controllers, planner, RViz MotionPlanning UI, and an XYZ IK replay
service. Click **Replay XYZ Demo** in RViz, or start the launch with
`auto_demo:=true`. The arm has four degrees of freedom, so its IK intentionally
solves position (XYZ) rather than an arbitrary six-degree-of-freedom pose.

Visuals use the imported STL files. Collision checking uses conservative boxes
and cylinders so both macOS and Ubuntu avoid loading high-resolution meshes into
FCL. See the package README for limitations and test commands.
