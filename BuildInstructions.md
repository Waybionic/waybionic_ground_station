Basic placeholder robot with a base box and moveable cylinder arm.

## Setup workspace and Clone repo (*Skip if repo already cloned*)
- Run these commands inside the terminal in Ubuntu:
```
mkdir -p ~/waybionic_ws/src
cd ~/waybionic_ws/src
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd ~/waybionic_ws
```

## Build and Launch
- Run these commands from the root of your workspace (`~/waybionic_ws`) to install dependencies and build the foundation:
```
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
- Launch ground station using:
```
ros2 launch waybionic_bringup ground_station.launch.py
```
- **RViz** and **Joint State Publisher GUI** (separate small window) will pop up after the last command
- RViz opens pre-configured with `base_link` fixed frame, `RobotModel`, and `TF` displays already loaded
- Move the slider in the Joint State Publisher GUI (small window) to move the arm

## macOS (Apple Silicon)

The workspace runs natively through RoboStack. Docker and XQuartz are not required.
Intel macOS is not currently verified.

### Prerequisites

Install the Xcode command-line tools, Git, and Miniforge:

```bash
xcode-select --install
brew install git
brew install --cask miniforge
```

Reopen the terminal if `mamba` or `conda` is not immediately available.

### Setup and launch

For a new clone, run:

```bash
git clone https://github.com/Waybionic/waybionic_ground_station.git && cd waybionic_ground_station && ./scripts/macos.sh setup
```

For an existing clone, run `./scripts/macos.sh setup` from the repository root.
The command creates or updates the `waybionic_robostack` environment and builds
the workspace.

Launch RViz and Joint State Publisher GUI:

```bash
./scripts/macos.sh launch
```

Other useful commands:

```bash
./scripts/macos.sh build                 # rebuild the workspace
./scripts/macos.sh run ros2 topic list   # run any overlaid ROS command
```

After pulling repository changes, update and rebuild with:

```bash
git pull && ./scripts/macos.sh setup
```

If RViz reports a missing workspace package, rerun `./scripts/macos.sh build`.
Do not source `install/setup.bash` directly from zsh; the helper handles the
workspace overlay through Bash.
