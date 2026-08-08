# WayBionic Ground Station

ROS2 workspace for the WayBionic robotic ground station. 

Currently, this repository contains the clean foundation and placeholder robot model for the ground station rebuild.

## Packages

- **`waybionic_description`**
  Hardware description, URDF/Xacro files, and meshes. (Currently using a geometric placeholder model until mechanical exports are finalized).
- **`waybionic_bringup`**
  Launch files and RViz configurations to bring up the robot state and visualization.

## macOS (Apple Silicon)

Install Miniforge once, then clone and set up the native RoboStack environment:

```bash
brew install --cask miniforge
git clone https://github.com/Waybionic/waybionic_ground_station.git && cd waybionic_ground_station && ./scripts/macos.sh setup
```

Launch the ground station visualization:

```bash
./scripts/macos.sh launch
```

## Building and Launching

Please refer to [BuildInstructions.md](./BuildInstructions.md) for complete instructions on how to build the workspace and launch the ground station visualization.
