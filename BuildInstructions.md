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

The workspace runs natively through RoboStack. Docker and XQuartz are not
required. Intel macOS is not verified.

### First-time setup

1. Install the prerequisites:

   ```bash
   xcode-select --install
   brew install git
   brew install --cask miniforge
   ```

   If Homebrew is missing, install it from [brew.sh](https://brew.sh/) first.
   If Xcode reports that its tools are already installed, continue.

2. Close Terminal, open a new Terminal window, and verify Miniforge:

   ```bash
   mamba --version
   ```

3. Clone the repository:

   ```bash
   mkdir -p ~/waybionic
   cd ~/waybionic
   git clone https://github.com/Waybionic/waybionic_ground_station.git
   cd waybionic_ground_station
   ```

   For an existing clone, skip the clone commands and change to that
   repository's root directory.

4. Create the RoboStack environment and build the workspace:

   ```bash
   ./scripts/macos.sh setup
   ```

   Wait for `Setup complete` before continuing.

### Launch

From the repository root, run:

```bash
./scripts/macos.sh launch
```

Keep this Terminal window open. Within a few seconds:

- The RViz splash screen is replaced by the main window.
- `DiagnosticsPanel` displays **WayBionic Engineering Monitor** and
  **Current State: NORMAL**.
- Joint State Publisher displays the `base_to_arm` slider.

To stop the application, return to the launch Terminal and press
<kbd>Control</kbd>+<kbd>C</kbd>.

Always use `scripts/macos.sh`. It selects the macOS SDK and Cyclone DDS and
loads the workspace correctly. Do not source `install/setup.bash` from zsh or
replace the helper with direct `colcon` or `ros2 launch` commands.

### Verify ROS nodes

While the application is running, open a second Terminal, change to the
repository root, and run:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./scripts/macos.sh run ros2 node list
```

The output must include:

```text
/joint_state_publisher
/robot_state_publisher
/rviz2
```

### Update or rebuild

After pulling repository changes:

```bash
git pull
./scripts/macos.sh setup
```

To rebuild without updating the environment:

```bash
./scripts/macos.sh build
```

### Troubleshooting

Run these commands from the repository root. After applying a fix, use the
single command in the **Launch** section.

#### `mamba` is not found

Close and reopen Terminal. If `mamba --version` still fails, reinstall
Miniforge and reopen Terminal again:

```bash
brew reinstall --cask miniforge
```

#### Setup cannot solve the environment or reports missing ROS tools

Use this for `Could not solve for environment specs`, `colcon: not found`,
`xacro: not found`, or a missing Joint State Publisher.

First confirm that `waybionic_robostack` appears in:

```bash
mamba env list
```

If it exists, repair and rebuild it:

```bash
mamba install --yes --name waybionic_robostack --freeze-installed \
  --channel conda-forge --channel robostack-jazzy \
  colcon-common-extensions ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui
./scripts/macos.sh build
```

If the environment does not exist, rerun the first-time setup command instead.

#### CMake reports a missing OpenGL framework header

If the error names
`/System/Library/Frameworks/OpenGL.framework/Headers`, update and rebuild:

```bash
git pull
./scripts/macos.sh build
```

#### RViz remains on `Initializing`

Stop the application with <kbd>Control</kbd>+<kbd>C</kbd>, remove any Fast DDS
override, and use the launch command above:

```bash
unset RMW_IMPLEMENTATION
```

#### `DiagnosticsPanel` reports `_PyExc_RuntimeError`

Stop the application, clean the plugin's CMake cache, and rebuild it:

```bash
./scripts/macos.sh run colcon build \
  --packages-select waybionic_rviz_plugins \
  --cmake-clean-cache --symlink-install
```

The panel should display **WayBionic Engineering Monitor** after the next
launch.
