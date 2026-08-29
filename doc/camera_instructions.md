```bash
cd ~/<your_ros2_ws>
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select waybionic_camera_tools
source install/setup.bash
ros2 launch waybionic_camera_tools camera_latency_monitor.launch.py
```
It should result in a text based latency monitoring system with no camera connected

> Note: Doesn't work with any sort of camera yet. It just shows the base workings of a text-based latency monitor and how it would be structured. 

> For now, it only works with one camera (monocular) but further implementation can be done for a stereo based camera.
