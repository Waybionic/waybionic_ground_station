Basic placeholder robot with a base box and moveable cylinder arm.

How to build:
- Run these commands in your workspace:
```
colcon build
source install/setup.bash
ros2 launch waybionic_bringup display.launch.py
```
- **RViz** and **Joint State Publisher GUI** (separate small window) will pop up after the last command
- RViz will automaticallyload the `base_link` fixed frame and the `RobotModel` and `TF` displays
- Move the slider in the Joint State Publisher GUI (small window) to move the arm
