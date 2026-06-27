Basic placeholder robot with a base box and moveable cylinder arm.

How to build:
- Run these commands in your workspace:
```
colcon build --packages-select waybionic_description waybionic_bringup
source install/setup.bash
ros2 launch waybionic_bringup display.launch.py
```
- RViz will pop up after the last command with two windows (one big one small)
- In RViz (big window):
  - In **Global Options** (left panel), next to **Fixed Frame**, type `base_link`
  - At the bottom left, click **Add**, scroll to and select **RobotModel**, click **Add**
  - Click the small triangle next to **RobotModel** to expand its settings, look for **Description Topic** setting, type `robot_description` next to it
  - Click **Add** again, scroll to and select **TF**, click **Add**
  - Use the Joint State Publisher GUI (small window) to move the arm by moving the slider
