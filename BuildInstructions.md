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
required. Intel macOS is not currently verified.

Run every command below separately. If a command fails, stop and use the
matching troubleshooting section before continuing.

### Step 1: Install the prerequisites

1. Open Terminal.
2. Install the Xcode command-line tools:

   ```bash
   xcode-select --install
   ```

   If macOS reports that the tools are already installed, continue to the next
   step.

3. Verify that Homebrew is installed:

   ```bash
   brew --version
   ```

   If the command is not found, install Homebrew from
   [brew.sh](https://brew.sh/), reopen Terminal, and run the verification
   command again.

4. Install Git:

   ```bash
   brew install git
   ```

5. Install Miniforge:

   ```bash
   brew install --cask miniforge
   ```

6. Close Terminal and open a new Terminal window.
7. Verify the required commands:

   ```bash
   git --version
   ```

   ```bash
   mamba --version
   ```

   ```bash
   xcrun --show-sdk-path
   ```

   Do not continue until all three commands succeed.

### Step 2: Clone or open the repository

For a new clone, run:

```bash
mkdir -p ~/waybionic
```

```bash
cd ~/waybionic
```

```bash
git clone https://github.com/Waybionic/waybionic_ground_station.git
```

```bash
cd waybionic_ground_station
```

For an existing clone, open Terminal and change to its repository root. For
the default location above, run:

```bash
cd ~/waybionic/waybionic_ground_station
```

All remaining commands must be run from this repository root.

### Step 3: Create the RoboStack environment and build

Run:

```bash
./scripts/macos.sh setup
```

Wait for all workspace packages to finish building. A successful run ends
with:

```text
Setup complete. Launch with: ./scripts/macos.sh launch
```

Do not source `install/setup.bash` directly from zsh. The helper activates the
RoboStack environment and workspace overlay through Bash.

### Step 4: Launch the ground station

Run:

```bash
./scripts/macos.sh launch
```

Keep this Terminal window open while using the application. The helper selects
the active macOS SDK for builds and Cyclone DDS for launches. Do not replace
this command with a direct `colcon` or `ros2 launch` invocation.

### Step 5: Verify the application

Within a few seconds:

1. The RViz splash screen is replaced by the main RViz window.
2. `DiagnosticsPanel` displays **WayBionic Engineering Monitor**.
3. The monitor displays **Current State: NORMAL**.
4. Joint State Publisher displays the `base_to_arm` slider.

Open a second Terminal window and run:

```bash
cd ~/waybionic/waybionic_ground_station
```

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./scripts/macos.sh run ros2 node list
```

The output must include:

```text
/joint_state_publisher
/robot_state_publisher
/rviz2
```

### Step 6: Close the application

Return to the Terminal window running the launch command and press
<kbd>Control</kbd>+<kbd>C</kbd>. Confirm that both RViz and Joint State
Publisher close.

### Step 7: Update an existing clone

Change to the repository root:

```bash
cd ~/waybionic/waybionic_ground_station
```

Pull the latest changes:

```bash
git pull
```

Update the environment and rebuild:

```bash
./scripts/macos.sh setup
```

Launch again:

```bash
./scripts/macos.sh launch
```

### Useful commands

Rebuild the workspace without updating the environment:

```bash
./scripts/macos.sh build
```

List ROS topics while the ground station is running:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./scripts/macos.sh run ros2 topic list
```

### macOS troubleshooting

Run the commands in this section from the repository root.

#### `mamba` or `conda` is not found

1. Close Terminal and open a new Terminal window.
2. Run:

   ```bash
   mamba --version
   ```

3. If the command is still missing, reinstall Miniforge:

   ```bash
   brew install --cask miniforge
   ```

4. Close Terminal, reopen it, and restart at Step 2.

#### Setup cannot solve the environment or reports missing ROS tools

This recovery applies when setup prints
`Could not solve for environment specs`, `colcon: not found`, or
`xacro: not found`, or reports a missing Joint State Publisher.

1. Confirm that the environment already exists:

   ```bash
   mamba env list
   ```

2. If the output does not contain `waybionic_robostack`, return to Step 3 and
   rerun setup; do not run the repair command below against a new environment.
3. If the environment exists, repair it without upgrading its working ROS
   packages:

   ```bash
   mamba install --yes --name waybionic_robostack --freeze-installed \
     --channel conda-forge --channel robostack-jazzy \
     colcon-common-extensions ros-jazzy-xacro \
     ros-jazzy-joint-state-publisher-gui
   ```

4. Rebuild:

   ```bash
   ./scripts/macos.sh build
   ```

5. Launch:

   ```bash
   ./scripts/macos.sh launch
   ```

#### CMake reports a missing OpenGL framework header

This error names
`/System/Library/Frameworks/OpenGL.framework/Headers`. The current helper
resolves the installed SDK path with `xcrun`.

1. Pull the latest changes:

   ```bash
   git pull
   ```

2. Rebuild:

   ```bash
   ./scripts/macos.sh build
   ```

3. Launch:

   ```bash
   ./scripts/macos.sh launch
   ```

#### RViz remains on `Initializing`

A shell override may be forcing Fast DDS instead of Cyclone DDS.

1. Stop the launch with <kbd>Control</kbd>+<kbd>C</kbd>.
2. Remove the override:

   ```bash
   unset RMW_IMPLEMENTATION
   ```

3. Launch through the helper:

   ```bash
   ./scripts/macos.sh launch
   ```

#### `DiagnosticsPanel` reports `_PyExc_RuntimeError`

The error also names `libwaybionic_rviz_plugins.dylib`.

1. Stop the launch with <kbd>Control</kbd>+<kbd>C</kbd>.
2. Clean the plugin's CMake cache and rebuild only that package:

   ```bash
   ./scripts/macos.sh run colcon build \
     --packages-select waybionic_rviz_plugins \
     --cmake-clean-cache --symlink-install
   ```

3. Launch:

   ```bash
   ./scripts/macos.sh launch
   ```

4. Confirm that `DiagnosticsPanel` displays **WayBionic Engineering Monitor**
   instead of the loader error.
