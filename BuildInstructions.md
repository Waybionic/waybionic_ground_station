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
colcon build --packages-select waybionic_description waybionic_bringup
source install/setup.bash
```
- Launch using 


- **RViz** and **Joint State Publisher GUI** (separate small window) will pop up after the last command
- RViz opens pre-configured with `base_link` fixed frame, `RobotModel`, and `TF` displays already loaded
- Move the slider in the Joint State Publisher GUI (small window) to move the arm
