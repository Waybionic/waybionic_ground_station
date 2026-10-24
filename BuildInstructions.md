# Setup, Build and Run

This guide gets the ground station building, passing its tests and showing the
arm in RViz. The ground station runs on **Ubuntu 24.04 with ROS 2 Jazzy**. The
easiest way to get that on any computer is Docker: ROS and every dependency live
inside the container, so you don't install ROS yourself.

Most people only need the [Docker setup](#docker-setup). Use a native setup only
for RViz on macOS, or to run ROS outside a container.

| You want to | Windows 11 | Linux | macOS (Apple Silicon) |
| --- | --- | --- | --- |
| Build and run the tests | [Docker](#docker-setup) | [Docker](#docker-setup) | [Docker](#docker-setup) |
| See the arm in RViz | [WSLg](#windows-wslg) | [GUI script](#linux-gui-script) | [RoboStack](#macos-native-ros-with-robostack) |
| Edit the code | [VS Code Dev Container](#6-develop-in-vs-code) | [VS Code Dev Container](#6-develop-in-vs-code) | [VS Code Dev Container](#6-develop-in-vs-code) |
| Run ROS outside Docker | [Ubuntu in WSL](#ubuntu-2404-or-wsl-native-ros-2) | [Ubuntu 24.04](#ubuntu-2404-or-wsl-native-ros-2) | [RoboStack](#macos-native-ros-with-robostack) |

Run commands one at a time, and stop at the first error. Never share passwords or
access tokens in chat or issues.

## Contents

- [Docker setup](#docker-setup)
- [macOS: native ROS with RoboStack](#macos-native-ros-with-robostack)
- [Ubuntu 24.04 or WSL: native ROS 2](#ubuntu-2404-or-wsl-native-ros-2)
- [Next steps](#next-steps)
- [Troubleshooting](#troubleshooting)
- [For maintainers: images and CI](#for-maintainers-images-and-ci)

## Docker setup

### 1. Install Docker

**Windows 11**

1. If WSL isn't installed yet, open **PowerShell as Administrator** and run the
   command below, then restart Windows when asked. Docker Desktop needs WSL 2.1.5
   or later; `wsl --version` shows yours.

   ```powershell
   wsl --install --no-distribution
   ```

2. Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
   with the WSL 2 backend and Linux containers, then start Docker Desktop.
3. Close and reopen PowerShell and VS Code, so they find the `docker` command.

**macOS**

Install [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
for your chip, then start it. Apple Silicon uses ARM64 containers; don't force
x86-64 emulation.

**Linux**

Install [Docker Engine](https://docs.docker.com/engine/install/) and the
[Compose plugin](https://docs.docker.com/compose/install/linux/) for your
distribution, and follow its steps for running Docker as your user. Never make the
Docker socket world-writable.

**Check Docker** in any terminal:

```console
docker version
docker compose version
```

Both must succeed, and `docker version` must show a **Server** section. If it
doesn't, Docker isn't running yet.

### 2. Get the code

Install [Git](https://git-scm.com/downloads), then:

```console
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd waybionic_ground_station
```

Run every command below from this folder, the repository root.

### 3. Build and run the tests

```console
docker compose --progress plain build test
```

This builds every package inside the container and runs all of the tests. A
failing test fails the build. The first build downloads ROS, so its time depends
mostly on your network (about 5 minutes on a fast connection). Later builds reuse
the download and take a few minutes.

**Expected result:** near the end, `Summary: ... 0 errors, 0 failures` and the
build finishes without an error. The 11 CAN tests that need a Linux `vcan`
interface are skipped here; CI runs them separately. A cache error near the start
is harmless; see [Messages you can ignore](#messages-you-can-ignore).

The image holds a copy of the source, so build again after you change files.

### 4. Open the arm in RViz

#### Windows: WSLg

With Docker Desktop running, open **Windows PowerShell** (not a terminal inside a
container), go to the repository root, and run:

```powershell
docker compose run --rm --build wslg
```

RViz and the Joint State Publisher window open. Move the `joint_1` to `joint_5`
sliders to move the arm; it stops where it would hit itself or the ground, and the
sliders jump back to where it stopped. Use the diagnostics panel's mock controls to
try normal and fault states. Everything is simulated. Press **Ctrl+C** in PowerShell
to stop.

To sweep each joint in turn instead, add the launch command:

```powershell
docker compose run --rm --build wslg ros2 launch waybionic_bringup ground_station.launch.py demo_mode:=true
```

If RViz doesn't open or stays black, see
[Troubleshooting](#rviz-does-not-open-or-is-black-on-windows).

#### Linux: GUI script

You need a desktop session with X11, or Wayland with XWayland, and `xauth`
(`sudo apt install xauth` on Ubuntu, `sudo dnf install xorg-x11-xauth` on
Fedora). Docker Desktop and remote Docker engines aren't supported here, because
the script uses your local display. From the repository root, as your normal user:

```bash
./scripts/linux-gui.sh --build
```

RViz and the Joint State Publisher window open. The script builds the workspace
from your checkout each time it starts, so after the first run you can leave out
`--build`. Use `--build` again only after changing package dependencies. Press
**Ctrl+C** to stop.

| Command | Does |
| --- | --- |
| `./scripts/linux-gui.sh` | Build the workspace and open RViz |
| `./scripts/linux-gui.sh --build` | Rebuild the image first |
| `./scripts/linux-gui.sh -d` | Run in the background |
| `./scripts/linux-gui.sh --stop` | Stop the background container |
| `./scripts/linux-gui.sh --controller` | Start teleop with the USB controllers plugged in now |

Restart the script after editing C++ code or launch files.

#### macOS

Docker can't open RViz on macOS. Use the
[RoboStack setup](#macos-native-ros-with-robostack) for RViz.

### 5. Try the headless demo

This runs the ground station with simulated diagnostics and no windows:

```console
docker compose up --build demo
```

In a second terminal, read one diagnostics message:

```console
docker compose exec demo /entrypoint.sh ros2 topic echo /diagnostics --once
```

**Expected result:** a timestamped message with temperature, current and IMU demo
values. Press **Ctrl+C** in the first terminal, then remove the container:

```console
docker compose down
```

### 6. Develop in VS Code

1. Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).
2. Open the repository folder in VS Code, with Docker running.
3. Run **Dev Containers: Reopen in Container** from the Command Palette. The first
   start builds the workspace.

Terminals open in `/waybionic_ws` with ROS sourced. Your edits stay in your checkout
on the host. After editing, build and test inside the container:

```bash
cd /waybionic_ws
colcon build --symlink-install
source install/setup.bash
colcon test --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

After changing a package's dependencies, run **Dev Containers: Rebuild Container**.
A new package needs no Dockerfile change. Before opening a PR, run the build in
[step 3](#3-build-and-run-the-tests).

Without VS Code, open the same environment with
`docker compose run --rm development bash` and use the commands above. Build
outputs there are deleted when you exit.

## macOS: native ROS with RoboStack

On Apple Silicon, the workspace runs natively through RoboStack, with RViz. Docker
and XQuartz aren't needed. Intel Macs are untested.

### 1. Install the prerequisites

Install [Homebrew](https://brew.sh/) if you don't have it, then:

```bash
xcode-select --install
brew install git
brew install --cask miniforge
conda init "$(basename "$SHELL")"
```

If Xcode reports that its tools are already installed, continue. Close Terminal,
open a new window, and check Miniforge:

```bash
mamba --version
```

### 2. Get the code

```bash
mkdir -p ~/waybionic
cd ~/waybionic
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd waybionic_ground_station
```

### 3. Create the environment and build

```bash
./scripts/macos.sh setup
```

Wait for `Setup complete`.

### 4. Open the arm in RViz

From the repository root:

```bash
./scripts/macos.sh launch
```

Keep this Terminal open. Within a few seconds, RViz opens with the **WayBionic
Engineering Monitor** showing **Current State: NORMAL**, and the Joint State
Publisher shows the `joint_1` to `joint_5` sliders. The arm stops where it would hit
itself or the ground. Press **Control+C** to stop.

To check the ROS nodes, run this in a second Terminal from the repository root,
while the first one is running:

```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./scripts/macos.sh run ros2 node list
```

The list must include `/joint_state_guard`, `/joint_state_publisher`,
`/robot_state_publisher` and `/rviz2`.

Always use `scripts/macos.sh`: it selects the macOS SDK and Cyclone DDS and loads
the workspace. Don't source `install/setup.bash` from zsh, or run `colcon` or
`ros2 launch` directly. Extra arguments to `launch` go to the launch file, for
example `./scripts/macos.sh launch demo_mode:=true`.

### 5. Update

After pulling changes:

```bash
git pull
./scripts/macos.sh setup
```

To rebuild without updating the environment, run `./scripts/macos.sh build`.

### macOS troubleshooting

Run these from the repository root, then launch again.

**`mamba` is not found.** Close and reopen Terminal. If it still fails, reinstall
Miniforge with `brew reinstall --cask miniforge` and reopen Terminal.

**Setup can't solve the environment, or reports `colcon: not found`,
`xacro: not found` or a missing Joint State Publisher.** Check that
`waybionic_robostack` is listed by `mamba env list`. If it is, repair it:

```bash
mamba install --yes --name waybionic_robostack --freeze-installed \
  --channel conda-forge --channel robostack-jazzy \
  colcon-common-extensions ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher-gui
./scripts/macos.sh build
```

If it isn't listed, run `./scripts/macos.sh setup` again.

**CMake reports a missing header under
`/System/Library/Frameworks/OpenGL.framework/Headers`.** Run `git pull`, then
`./scripts/macos.sh build`.

**RViz stays on `Initializing`.** Stop it, clear any Fast DDS override with
`unset RMW_IMPLEMENTATION`, and launch again.

**The diagnostics panel reports `_PyExc_RuntimeError`.** Rebuild the panel with a
clean CMake cache:

```bash
./scripts/macos.sh run colcon build \
  --packages-select waybionic_rviz_plugins \
  --cmake-clean-cache --symlink-install \
  --cmake-args "-DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-$(xcrun --show-sdk-path)}"
```

## Ubuntu 24.04 or WSL: native ROS 2

Use this to run ROS and RViz outside a container, for example for the real arm on
Linux. It isn't needed for Docker development. On Windows, start at step 1; on
Ubuntu 24.04, start at step 2.

### 1. Windows: install Ubuntu 24.04 in WSL

Open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-24.04
```

Use exactly `Ubuntu-24.04`: ROS 2 Jazzy needs Ubuntu 24.04, not the newest Ubuntu.
Restart Windows when asked, then open **Ubuntu 24.04** from the Start menu and
create your Linux user. In a normal PowerShell window, check:

```powershell
wsl --list --verbose
```

The `Ubuntu-24.04` row must show `VERSION` **2**. Run every remaining command in
the Ubuntu terminal, not in PowerShell.

### 2. Check the Ubuntu version and locale

```bash
cat /etc/os-release
locale
```

You need `VERSION_ID="24.04"` and a UTF-8 locale. If the locale isn't UTF-8, run:

```bash
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 3. Add the ROS 2 package repository

```bash
sudo apt update
sudo apt install software-properties-common curl python3
sudo add-apt-repository universe
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 4. Install ROS 2 Jazzy and the build tools

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop git build-essential python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

`ros-jazzy-desktop` includes RViz. Run `sudo rosdep init` only once per Ubuntu
installation; if it says the configuration already exists, just run
`rosdep update`, without `sudo`.

### 5. Get the code

Keep the workspace in your Linux home folder, not under `/mnt/c`:

```bash
mkdir -p ~/waybionic_ws/src
cd ~/waybionic_ws/src
git clone https://github.com/Waybionic/waybionic_ground_station.git
```

If `~/waybionic_ws/src/waybionic_ground_station` already exists, skip the clone.

### 6. Build, test and launch

```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

**Expected result:** `colcon test-result` reports `0 errors, 0 failures`.

Then open the arm in RViz:

```bash
ros2 launch waybionic_bringup ground_station.launch.py
```

RViz and the Joint State Publisher window open; move the sliders to move the arm,
which stops where it would hit itself or the ground.
Press **Ctrl+C** to stop.

In every new Ubuntu terminal, source both environments before running ROS
commands:

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
```

## Next steps

- **Drive the arm with a controller, show the doctor-view camera, or connect the
  real arm:** see the [operator guide](docs/operator_guide.md).
- **Change what the ground station starts:** the
  [launch options](waybionic_bringup/README.md) list every argument of
  `ground_station.launch.py`.
- **Read the diagnostics panel:** see [waybionic_rviz_plugins](waybionic_rviz_plugins/README.md).
- **Contribute:** read [CONTRIBUTING.md](CONTRIBUTING.md) before your first pull
  request.

## Troubleshooting

### `docker` is not recognized on Windows

Close and reopen PowerShell after installing Docker Desktop. If it still isn't
found, run Docker by its full path, for example:

```powershell
& "$env:LOCALAPPDATA\Programs\DockerDesktop\resources\bin\docker.exe" compose version
```

A build that reports `docker-credential-desktop` not found has the same cause.

### Messages you can ignore

- `ERROR importing cache manifest from ghcr.io/waybionic/...` or
  `failed to configure registry cache importer ... 401 Unauthorized` near the start
  of a build. Docker couldn't read the shared image cache, so it builds everything
  on your computer instead.
- `XDG_RUNTIME_DIR not set` and `Stereo is NOT SUPPORTED` when RViz starts.
- After Ctrl+C, the joint window exiting with code `-2`, and Docker suggesting you
  debug a Compose error.

### RViz does not open or is black on Windows

- Run the command in **Windows PowerShell**. The prompt must start with `PS`, not
  `docker-desktop:~#`, and not a prompt inside a container.
- To render on the CPU instead of the GPU, run this, then the `wslg` command
  again:

  ```powershell
  $env:WAYBIONIC_GL_DRIVER = 'llvmpipe'
  ```

- If Docker reports that the WSLg display socket is missing, restart Docker
  Desktop and WSL (`wsl --shutdown`) rather than creating the folder yourself.

### The Linux GUI script reports a missing display or cookie

Run it from a terminal in your logged-in desktop session, without `sudo`, and
check that `DISPLAY` and `XAUTHORITY` point to that session. A Wayland session
needs XWayland. The script renders on the CPU, which can be slow for complex
scenes. Only one GUI container can run per checkout; stop it with `--stop` before
starting another.

## For maintainers: images and CI

[CI](.github/workflows/ros2_build_test.yml) runs the same `test` build on x86-64
and ARM64 Linux for every pull request. A passing build doesn't check RViz
windows, cameras, USB devices, GPU rendering or networking with the arm.

After a push to main passes on both architectures, CI publishes a multi-platform
image tagged with the commit's short hash, and also as
`ghcr.io/waybionic/waybionic_ground_station:jazzy` if the commit is still the
newest on main. Pull requests build with read-only tokens and don't publish.

Local builds reuse that image's layers when they can read it. While the GHCR
package is private, builds print the cache error listed under
[Messages you can ignore](#messages-you-can-ignore), and running main
without building needs `docker login ghcr.io`. To run main's image instead of
building:

```console
docker compose pull demo
docker compose up demo
```

The Ubuntu and ROS base image is pinned by digest, but apt packages are resolved
when that layer is rebuilt, so builds on different days aren't frozen. To
reproduce a problem, keep the image and record its digest:

```console
docker buildx imagetools inspect ghcr.io/waybionic/waybionic_ground_station:jazzy
```

The containers are headless except for the GUI services. Don't add privileged
containers or broad display permissions to make a build work; the WSLg service
mounts only WSL's GPU device.
