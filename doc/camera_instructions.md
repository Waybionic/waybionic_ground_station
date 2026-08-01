```bash
cd ~/<your_ros2_ws>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_camera_tools
source install/setup.bash
ros2 launch waybionic_camera_tools camera_latency_monitor.launch.py
```
It should result in a text based latency monitoring system 

> Will probably change this format... it is very messy at the moment
