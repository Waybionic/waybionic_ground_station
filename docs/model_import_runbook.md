# Waybionic Model Import Runbook

This guide is for importing and testing real URDF and mechanical mesh exports (STLs) without breaking the clean ROS 2 foundation or editing Python launch files.

## 1. Where to put the files
- **Meshes (.stl, .dae):** Place all 3D mesh files into `waybionic_description/meshes/`.
- **URDF/Xacro (.urdf, .xacro):** Place your exported robot description file into `waybionic_description/urdf/`.

*Important: Inside the URDF, ensure the mesh paths use the standard ROS package syntax. Example:*
`<mesh filename="package://waybionic_description/meshes/base_link.stl"/>`

## 2. Rebuild the Workspace
Any time new files are added, rebuild the foundation so CMake can install them to the ROS 2 share directory. 
From the root of your workspace (`~/waybionic_ws`):
```
colcon build --packages-select waybionic_description
source install/setup.bash
```

## 3. Test the model
Don't edit `display.launch.py` to test the model. Instead, pass the path to the new URDF using the `model:=` argument.
From the root of your workspace, run:
```
ros2 launch waybionic_bringup display.launch.py model:=$(ros2 pkg prefix waybionic_description --share)/urdf/YOUR_NEW_FILE.urdf
```
If parsed correctly, RViz will automatically open and display the model. If there are issues, errors will print in the terminal.