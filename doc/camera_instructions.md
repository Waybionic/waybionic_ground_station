## Camera Latency Program

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

## Linking Camera with WSL (windows)

> In terminal

 ```
  usbipd list
 ```

 *Take note of the BusID of the camera you wish to use

 ```
 usbipd bind --busid $BUSID
 ```
 > If this doesn't work/camera is "busy", add --force at the end (webcams often have this issue)

 ```
 usbipd attach --wsl --busid $BUSID
 ```
> Make sure wsl is open in another terminal tab
