# WayBionic Ground Station

The ROS 2 Jazzy workspace for the WayBionic surgical robot arm. It shows the
five-joint arm in RViz as a digital twin, drives it with an Xbox or PS5 controller,
checks joint limits and collisions, runs the arm's MKS drives over CAN (simulated by
default), and shows the doctor-view camera and live diagnostics.

This is a student research prototype, not a medical device. Everything runs in
simulation unless you choose the real drives, and powered tests need a lead
present.

## Quick start

[Install Docker](BuildInstructions.md#1-install-docker), then build the workspace
and run its tests:

```console
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd waybionic_ground_station
docker compose --progress plain build test
```

Then open the arm in RViz:

| Platform | Command |
| --- | --- |
| Windows 11 (PowerShell) | `docker compose run --rm --build wslg` |
| Linux | `./scripts/linux-gui.sh --build` |
| macOS (Apple Silicon) | `./scripts/macos.sh setup`, then `./scripts/macos.sh launch` ([setup first](BuildInstructions.md#macos-native-ros-with-robostack)) |

[BuildInstructions.md](BuildInstructions.md) has every step for each platform.

## Documentation

| Document | Covers |
| --- | --- |
| [BuildInstructions.md](BuildInstructions.md) | Installing, building, testing and opening RViz on Windows, macOS and Linux |
| [Operator guide](docs/operator_guide.md) | Controllers, the self-running demo, the doctor-view camera and the real arm |
| [Launch options](waybionic_bringup/README.md) | Every argument of `ground_station.launch.py` |
| [Diagnostics](waybionic_rviz_plugins/README.md) | The engineering monitor panel |
| [CAN and the carrier board](waybionic_can/README.md) | The MKS protocol, the carrier firmware and CAN simulation |
| [CONTRIBUTING.md](CONTRIBUTING.md) | Branches, commits, pull requests and reviews |

## Repository layout

| Folder | Contents |
| --- | --- |
| `waybionic_description` | The arm's URDF and meshes, generated from the SolidWorks assembly by `scripts/cad/`, with joint limits where the CAD parts touch. Regenerate it after CAD changes rather than editing it. |
| `waybionic_bringup` | `ground_station.launch.py`, the RViz layouts and the launch tests |
| `waybionic_teleop` | Controller teleop, the collision checks, and the MKS drive node, simulated or real |
| `waybionic_can` | The MKS CAN protocol, CAN simulation and the UNO R4 carrier firmware |
| `waybionic_camera` | The doctor-view camera bridge and its delay monitoring |
| `waybionic_rviz_plugins` | The engineering monitor panel for RViz |
| `waybionic_sensors` | IMU publishing and sensor health |
| `waybionic_control`, `waybionic_diagnostics` | The earlier CAN-FD host and the diagnostics command-line tool |
| `waybionic_hardware`, `waybionic_motion_test` | The old four-servo arm's Arduino bridge and motion tests |
| `firmware/tool_board` | Firmware for the biomedical tool's motor board |
| `scripts` | `macos.sh`, `linux-gui.sh`, `windows-demo.cmd` and the CAD tools |
| `docker`, `compose.yaml`, `.devcontainer` | The container image, its services and the VS Code Dev Container |

## Contributing

Read [CONTRIBUTING.md](CONTRIBUTING.md) before your first pull request. Every change
goes through a reviewed pull request into `main`, and CI builds and tests it on
x86-64 and ARM64.
