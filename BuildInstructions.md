# Ground Station Setup, Build, and Launch

The ground station shows the five-joint WayBionic arm, exported from the
mechanical team's SolidWorks assembly. The primary development target is
**Ubuntu 24.04 (Noble) with ROS 2 Jazzy**.

Use **Docker Compose for development and headless build/test checks**. ROS and its build
tools live inside the container; you do not need a host ROS installation for
this path. Linux users can [launch the GUI in Docker](#linux-gui-demo), and Windows
users can [view the demo through WSLg](#windows-wslg-demo) without a second ROS
installation. Other graphical RViz options are the
[native Ubuntu/WSL setup](#native-ubuntu-setup-for-rviz) or
[macOS RoboStack setup](#macos-apple-silicon).

Run commands one at a time and stop at the first error. Do not share passwords
or access tokens in chat or issues.

## Docker Setup (Default)

### 1. Install Docker for your platform

**Windows 11:** if WSL is not installed, open **PowerShell as Administrator** and
run:

```powershell
wsl --install --no-distribution
```

Restart Windows when requested. Then check WSL in a normal PowerShell terminal:

```powershell
wsl --version
```

Docker Desktop requires WSL 2.1.5 or later. Follow the
[Microsoft WSL commands](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)
if an existing WSL installation needs updating. The Docker-only path does not
require a separate Ubuntu distribution. Enabling WSL needs administrator rights;
do not try to bypass an elevation or virtualization error.

Install [Docker Desktop for Windows](https://docs.docker.com/desktop/setup/install/windows-install/)
using its per-user installation and WSL2 backend, then start Docker Desktop.
Review its license terms locally. Use Linux containers, not Windows containers.

After installation, close and reopen existing terminal and VS Code windows so
they pick up Docker's updated `PATH`. Otherwise, PowerShell may report that
`docker` is not recognized, or a build may report that `docker-credential-desktop`
cannot be found even when Docker Desktop is running.

**macOS:** install [Docker Desktop for Mac](https://docs.docker.com/desktop/setup/install/mac-install/)
for your processor type, then start it. Apple Silicon uses Linux ARM64 containers;
Intel Macs use Linux x86-64 containers. Do not force x86-64 emulation on Apple Silicon.

**Linux:** install [Docker Engine](https://docs.docker.com/engine/install/)
using the instructions for your distribution. Configure Docker access according
to that guide, and install the
[Compose plugin](https://docs.docker.com/compose/install/linux/); never make the
Docker socket world-writable.

On every platform, verify that Docker can reach its engine:

```console
docker version
docker compose version
```

Docker Desktop includes Compose. The `docker compose version` command must succeed.
Both **Client** and **Server** information must appear. A missing Server section
or daemon connection error must be resolved before building.

### 2. Open the repository and build

Use your existing repository clone. New members need [Git](https://git-scm.com/downloads)
and repository access, then can run:

```console
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd waybionic_ground_station
```

From the **repository root**, run this same command in PowerShell, Bash, or zsh:

```console
docker compose --progress plain build test
```

The image installs package dependencies, builds all workspace packages, and runs
their headless tests. A build or test failure fails the Docker build. The first
build downloads ROS and Qt dependencies; later builds can reuse cached layers.
Rebuild after source changes because this image contains a snapshot of the source.

[CI](.github/workflows/ros2_build_test.yml) runs this Compose test build on native
x86-64 and ARM64 Linux runners. A passing container build does not verify RViz
windows, camera access, USB devices, GPU acceleration, or networking with a robot.

#### Headless Demo

Start the ground station with simulated diagnostics and no GUI:

```console
docker compose up --build demo
```

In a second host terminal, read one message from that container:

```console
docker compose exec demo /entrypoint.sh ros2 topic echo /diagnostics --once
```

Expect a timestamped array containing temperature, current, and IMU demo values.
Press **Ctrl+C** in the first terminal to stop the demo, then run
`docker compose down` to remove its container and network.
Plain `docker compose up --build` also starts only the headless demo; development,
test, Linux GUI, and WSLg services are opt-in profiles or can be targeted by service name.
ROS discovery is limited to localhost inside each service. Connecting to a robot or
other containers requires separate network configuration and validation.

### 3. Develop in VS Code

Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers),
open the repository folder in VS Code, and run **Dev Containers: Reopen in Container**
from the Command Palette. Docker must already be running.

The checked-in [configuration](.devcontainer/devcontainer.json) uses the
`development` service in [compose.yaml](compose.yaml), built from the
`development` stage of the same [Dockerfile](docker/Dockerfile). It mounts your
source checkout, uses a non-root Linux user, and builds the workspace on creation.
Terminals start in `/waybionic_ws` with ROS sourced. After editing, run inside the
Dev Container:

```bash
cd /waybionic_ws
colcon build --symlink-install
source install/setup.bash
colcon test --return-code-on-test-failure --event-handlers console_direct+
colcon test-result --verbose
```

Source edits persist in your host checkout. Build outputs stay inside the
container, outside the mounted checkout, and may be discarded when it is rebuilt.
Copied source in build/test images remains root-owned; only `build`, `install`,
and `log` are writable in those workspaces. These output directories link to
the container user's home so Dev Container UID updates work without `sudo`.
The development source bind mount stays editable.
The Compose source mount uses a shared SELinux label for Fedora and other
SELinux hosts. VS Code adjusts the container user's UID to match your Linux user.
After changing package dependencies, run **Dev Containers: Rebuild Container**.
The Dockerfile finds every package's `package.xml`, so a new package needs no
Dockerfile change. Run the clean Docker test command above before a PR.

For a development shell without VS Code, run `docker compose run --rm development bash`.
Inside it, use the build and test commands above (including sourcing the install setup
after building). Build outputs in this temporary container are removed when you exit.

### Reproducibility and GUI boundaries

The ROS/Ubuntu base is pinned by multi-platform image digest, and local development
and CI share the dependency recipe. Ubuntu and ROS package repositories are still
resolved when that layer is rebuilt, so builds on different dates are not a fully
frozen dependency lock. Keep the built image when reproducing a problem; record
its identifier with:

```console
docker image inspect waybionic-ground-station:jazzy --format '{{.Id}}'
```

CI currently builds and tests images; it does not publish a shared development
image. Sharing an immutable, verified project image is a follow-up once the
container build has been validated. Base-image updates must pass both CI
architectures before adoption.

The default container configuration is **headless**. For RViz, use the Linux GUI or
Windows WSLg demo below, or a native path. Do not add privileged containers or broad
display-server permissions just to get the build working; the WSLg demo mounts only
WSL's paravirtual GPU device.

## Linux GUI Demo

Use Docker Engine on the Linux desktop with X11 or Wayland with XWayland enabled.
The launcher was checked on Fedora KDE Wayland with XWayland: RViz initialized
OpenGL 4.5 and the Joint State Publisher GUI loaded the robot description.
No host ROS installation is needed. Docker Desktop's VM and remote Docker daemons
are not supported by this launcher because it mounts the local display socket.
Install `xauth` if missing: `sudo dnf install xorg-x11-xauth` on Fedora, or
`sudo apt install xauth` on Ubuntu.

From a desktop terminal, as your normal user with Docker access, run:

```bash
./scripts/linux-gui.sh
```

The launcher mounts this checkout into the container and runs an incremental
`colcon build --symlink-install` before opening the UI. Source changes are picked
up the next time you start the launcher, without rebuilding the Docker image.
For the first image build or after changing package dependencies, use `--build`.
Docker reuses cached image layers where possible. Restart the GUI after editing
C++ code or launch files; changes are not automatically loaded into a running RViz.

```bash
./scripts/linux-gui.sh --build       # Build image, build workspace, and launch
./scripts/linux-gui.sh -d            # Build workspace and run detached
./scripts/linux-gui.sh --build -d    # Build image and run detached
./scripts/linux-gui.sh --stop        # Stop this checkout's GUI container
```

RViz and the Joint State Publisher GUI should open on your desktop. Press **Ctrl+C**
in foreground mode, or use `--stop` in detached mode, to stop and remove the container.
Detached windows remain open after you close the terminal. Only one launcher container
per checkout and host user can run at a time; stop it before launching again.
The temporary host authentication file is deleted after Compose returns, including
in detached mode; the running container retains access through its existing file mount.
Use the launcher each time so the authentication file is recreated for your session.

The service uses Qt's X11 backend (through XWayland on Wayland), a read-only display
socket and cookie, and software OpenGL rendering. It does not require GPU devices or
`xhost` changes. On SELinux hosts, label isolation is disabled for this GUI service
to allow access to the existing desktop socket without relabeling it.

If the launcher reports a missing display or cookie, run it from your logged-in
desktop terminal without `sudo` and check that `DISPLAY` and `XAUTHORITY` refer to
that session. A Wayland session must provide XWayland. Software rendering can be
slower for complex scenes; GPU acceleration can be configured separately if needed.

## Windows WSLg Demo

The Compose service preserves the display settings used by the Windows 11 demo
with Docker Desktop's WSL2 backend.
It uses the image built in [Docker setup](#docker-setup-default) and WSL's existing
graphics service; a separate Ubuntu distribution or host ROS install is not needed.

With Docker Desktop running, open **Windows PowerShell** from the Start menu.
The prompt should start with `PS`, not `docker-desktop:~#`. Do not run this in
Docker Desktop's internal WSL distribution or inside the Dev Container. If an
unfinished command leaves you at a `>` prompt, press **Ctrl+C** to cancel it.

From the **repository root**, run the following as **one line**. It uses the
per-user Docker installation from the setup above directly, so it also works in
a terminal with an outdated `PATH`:

```powershell
& "$env:LOCALAPPDATA\Programs\DockerDesktop\resources\bin\docker.exe" compose run --rm --build wslg
```

RViz and the Joint State Publisher GUI open as separate windows. Move the
`joint_1` to `joint_5` sliders to move the arm, and use the diagnostics panel's
mock controls to try normal and fault states. This demo uses simulated data, not
a physical arm. Press **Ctrl+C** in PowerShell to stop it.

To sweep each joint in turn instead, put the launch command after `wslg`:

```powershell
& "$env:LOCALAPPDATA\Programs\DockerDesktop\resources\bin\docker.exe" compose run --rm --build wslg ros2 launch waybionic_bringup ground_station.launch.py demo_mode:=true
```

The display socket path is specific to Docker Desktop's WSL2 backend. RViz renders on
the Windows GPU through WSL's D3D12 driver, using the read-only `/dev/dxg` device and
`/usr/lib/wsl` libraries that Docker Desktop already provides. If RViz fails to open
or shows a black view, start it with CPU rendering instead:
`$env:WAYBIONIC_GL_DRIVER = 'llvmpipe'` before the command above. If the socket mount
is unavailable, stop and check Docker/WSL rather than creating an empty replacement
directory.

Qt may report a default `XDG_RUNTIME_DIR`, and RViz may report that stereo is not
supported; neither prevented this demo from starting. On Ctrl+C, the joint GUI
can report exit code `-2`, indicating the requested SIGINT interruption.

## Xbox Controller (Simulated Arm)

`teleop:=true` drives the arm with an Xbox controller through simulated CAN drives;
nothing is sent to hardware. The controller mapping is in
`waybionic_teleop/config/xbox_teleop.yaml`, and the placeholder joint-to-drive map
(MKS SERVO42D/57D CAN IDs, gear ratios and the wrist differential) is in
`waybionic_teleop/config/arm_drives.yaml`.

**Windows (Docker):** connect the controller, then start the teleop version of the
[WSLg demo](#windows-wslg-demo) from the repository root:

```powershell
& "$env:LOCALAPPDATA\Programs\DockerDesktop\resources\bin\docker.exe" compose run --rm --build --service-ports wslg-teleop
```

In a second PowerShell window, start the controller bridge from the repository root.
It needs only Python 3 on Windows, with no extra packages:

```powershell
cd waybionic_teleop
python -m waybionic_teleop.xinput_bridge
```

The bridge sends the controller state to the container on `127.0.0.1:47300/udp`.
Press **Ctrl+C** in each window to stop.

**Linux (native ROS):** with the controller plugged in, run
`ros2 launch waybionic_bringup ground_station.launch.py teleop:=true`.

| Input | Action |
| --- | --- |
| Start (Xbox Menu button, three lines) | Enable or disable; the arm starts disabled |
| B | Stop and hold the current pose |
| Y | Switch group: base (joints 1-3) or upper (joints 2-5) |
| Base group: left stick | Base yaw (left/right) and shoulder (up/down) |
| Base group: right stick up/down | Elbow |
| Upper group: left stick | Shoulder (left/right) and elbow (up/down) |
| Upper group: right stick | Wrist roll (left/right) and wrist pitch (up/down) |
| RT / LT | Close / open the placeholder end effector |
| D-pad up/down | Speed: 10, 25, 50 or 100% of 60 deg/s |
| A (hold) | Return to the zero pose |

Start is refused until the sticks are centred and the triggers released. The
diagnostics panel shows the teleop state, each joint, each drive's last CAN frame
and the simulated bus load.

The RViz camera follows the tool as the arm moves; drag to orbit and scroll to zoom
as usual, or add `follow_camera:=false` to the launch command for a fixed view.

If the arm stops responding and the bridge's axis values stop changing while you
move the sticks, Windows has stopped updating the controller. Turn the controller
off and on (or unplug and replug it), then press Start again; the bridge
reconnects by itself.

## Native Ubuntu Setup for RViz

Only follow this section if you need ROS and RViz outside the container. It is
not required for Docker-only development or tests.

Windows users start at step 1. Native Ubuntu users start at step 2. Apple Silicon
users should use the [macOS instructions](#macos-apple-silicon) below instead.
If Ubuntu, ROS 2 Jazzy, and the build tools are already configured, go to
[workspace setup](#setup-workspace-and-clone-repo).

Run commands one at a time. If a command fails, stop and record the command and
error before continuing. Installation, a required restart, and Linux password
prompts must be completed locally; do not share passwords in chat or issues.

### 1. Windows: install WSL2 and Ubuntu 24.04

On Windows 11, open **PowerShell as Administrator** and run:

```powershell
wsl --install -d Ubuntu-24.04
```

Use the explicit `Ubuntu-24.04` distribution name, not an unversioned Ubuntu
install: a newer Ubuntu release is not the target for ROS 2 Jazzy. The install
command enables the required Windows features and uses WSL2 by default.
Restart Windows when requested before proceeding.

After the restart, open Ubuntu 24.04 from the Start menu, or run this in a normal
Windows PowerShell terminal:

```powershell
wsl -d Ubuntu-24.04
```

Complete Ubuntu's first-run Linux username and password prompts. Password input
does not display characters. In a separate Windows PowerShell terminal, check:

```powershell
wsl --list --verbose
```

The `Ubuntu-24.04` row must show `VERSION` **2**. Stop here if installation fails,
the distribution is missing, or the version is not 2. Do not run the Ubuntu
commands below in PowerShell.

### 2. Ubuntu: check the version and locale

Run this and all remaining Linux commands **inside Ubuntu's Bash terminal**:

```bash
cat /etc/os-release
locale
```

Confirm `VERSION_ID="24.04"` and a UTF-8 locale. If a valid UTF-8 locale is already
configured, no locale changes are needed. Otherwise run:

```bash
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale
```

### 3. Ubuntu: configure the ROS package repository

Enable Ubuntu Universe and install the tools needed to fetch the official ROS
repository configuration:

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl python3
```

Fetch and install `ros2-apt-source`. The `noble` suffix below is specifically for
the Ubuntu 24.04 version checked in step 2. The release response is parsed as JSON.

```bash
ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json, sys; print(json.load(sys.stdin)["tag_name"])')
curl -fL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

### 4. Ubuntu: install ROS 2 and initialize the build tools

Update Ubuntu packages, then install ROS 2 Jazzy Desktop, which includes RViz:

```bash
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

Do not also install `ros-jazzy-ros-base`: Desktop already includes the base ROS
functionality. The AR4, Gazebo, and MoveIt demo commands in the older ROS README
are separate learning exercises, not steps for building this repository.

Install Git, the C++ build toolchain, colcon, and rosdep:

```bash
sudo apt install git build-essential python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

Run `sudo rosdep init` only once per Ubuntu installation. If it reports that its
configuration already exists, use `rosdep update`; do not delete the existing
configuration. Run `rosdep update` as your normal Linux user, without `sudo`.

Check the environment before moving on:

```bash
printenv ROS_DISTRO
command -v ros2 colcon rosdep git g++
```

The distribution must be `jazzy`, and each command must resolve to a path.

Installation references: [Microsoft WSL installation](https://learn.microsoft.com/en-us/windows/wsl/install)
and [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

## Setup workspace and Clone repo

For the native RViz path, run these commands inside Ubuntu. Keep the workspace in the Linux home directory
(`~/waybionic_ws`), not under `/mnt/c`. An existing Windows clone does not replace
this Linux workspace; keep any uncommitted Windows work intact. Choose the path
below that matches your native workspace.

### New clone

Run this only when `~/waybionic_ws/src/waybionic_ground_station` does not exist:

```bash
mkdir -p ~/waybionic_ws/src
cd ~/waybionic_ws/src
git clone https://github.com/Waybionic/waybionic_ground_station.git
cd ~/waybionic_ws
```

### Existing workspace

If `~/waybionic_ws/src/waybionic_ground_station` already exists, use it without
cloning again. Leave any local changes intact and continue from the workspace root:

```bash
cd ~/waybionic_ws
```

## Build and Launch
- For native Ubuntu/WSL, run these commands from the root of your workspace (`~/waybionic_ws`) to install dependencies and build the foundation:
```bash
cd ~/waybionic_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```
- Validate the full workspace before launching:
```bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
bash src/waybionic_ground_station/scripts/check_waybionic_ws.sh
```
- Launch ground station using:
```bash
ros2 launch waybionic_bringup ground_station.launch.py
```
- **RViz** and **Joint State Publisher GUI** (separate small window) will pop up after the last command
- RViz opens pre-configured with `base_link` fixed frame, `RobotModel`, and `TF` displays already loaded
- Move the `joint_1` to `joint_5` sliders in the Joint State Publisher GUI (small window) to move the arm: base yaw, shoulder, elbow, wrist pitch, and wrist roll
- Joint demo: `ros2 launch waybionic_bringup ground_station.launch.py demo_mode:=true` sweeps each joint in turn (simulated, no hardware) and shows a pass/fail row per joint in the diagnostics panel; `demo_speed:=60` changes the speed in degrees per second

For mock and live diagnostics checks, continue with the
[diagnostics guide](waybionic_rviz_plugins/README.md). For team access, branches,
and pull requests, read [CONTRIBUTING.md](CONTRIBUTING.md).

### Every new Ubuntu terminal

After the first successful build, source both environments before running ROS
commands. Rebuilding is only necessary after changes that require it.

```bash
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
```

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
