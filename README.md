# WayBionic Ground Station

ROS2 workspace for the WayBionic robotic ground station. 

Currently, this repository contains the clean foundation and placeholder robot model for the ground station rebuild.

## Packages

- **`waybionic_description`**
  Hardware description, URDF/Xacro files, and meshes. (Currently using a geometric placeholder model until mechanical exports are finalized).
- **`waybionic_bringup`**
  Launch files and RViz configurations to bring up the robot state and visualization.
- **`waybionic_rviz_plugins`**
  Engineer diagnostics panel, mock/live diagnostics sources, and a temporary diagnostics publisher.

## Docker Development and Tests

Docker is the default development and headless build/test path. See
[BuildInstructions.md](./BuildInstructions.md#docker-setup-default) for first-time
Windows, macOS, or Linux setup; no host ROS installation is required for this path.

From the repository root:

```console
docker build --progress=plain --target test --file docker/Dockerfile --tag waybionic-ground-station:jazzy .
```

For editing, open the repository with VS Code's **Dev Containers: Reopen in Container**.
Local development and CI use the same Dockerfile, with CI jobs for x86-64 and ARM64.
The default container is headless. Windows users can run the
[RViz demo through WSLg](./BuildInstructions.md#windows-wslg-demo); native GUI
options are also documented below.

## macOS (Apple Silicon)

For native RViz, install Miniforge once, then clone and set up the RoboStack environment:

```bash
brew install --cask miniforge
git clone https://github.com/Waybionic/waybionic_ground_station.git && cd waybionic_ground_station && ./scripts/macos.sh setup
```

Launch the ground station visualization:

```bash
./scripts/macos.sh launch
```

## Environment Setup, Build, and Launch

Use [BuildInstructions.md](./BuildInstructions.md) for Docker-first setup and the
separate native Ubuntu/WSL and macOS RViz instructions. Team access and contribution
workflow are covered in [CONTRIBUTING.md](./CONTRIBUTING.md).
