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

## Docker Compose Development and Tests

Docker Compose is the default development and headless build/test path. See
[BuildInstructions.md](./BuildInstructions.md#docker-setup-default) for first-time
Windows, macOS, or Linux setup; no host ROS installation is required for this path.

From the repository root:

```console
docker compose --progress plain build test
```

Start the headless demo with `docker compose up --build`. Stop it with **Ctrl+C**,
then remove the container with `docker compose down`.

For editing, open the repository with VS Code's **Dev Containers: Reopen in Container**.
Local development and CI use [compose.yaml](./compose.yaml) and the same
Dockerfile, with CI jobs for x86-64 and ARM64.
The default container is headless. Linux users can launch RViz with
`./scripts/linux-gui.sh` (see [Linux GUI setup](./BuildInstructions.md#linux-gui-demo)).
Windows users can run the
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
