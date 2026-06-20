# Waybionic Checklist

This checklist provides a streamlined setup process for club members working with the Waybionic model. Follow the steps below to prepare the required files, place them in the correct folders, build the workspace, and launch the model successfully.

---

## 1. Prepare the Files

If the URDF and STL files are packaged in a ZIP folder, unzip them before continuing.

Place the files in the following folders:

| File Type | Destination Folder |
|-----------|-------------------|
| URDF file | `waybionic_ground_station/waybionic_description/urdf` |
| STL files | `waybionic_ground_station/waybionic_description/meshes` |

---

## 2. Build and Launch the Model

Run the following commands **in order** to build the required packages and launch the model:

```bash
# 1. Source ROS
source /opt/ros/jazzy/setup.bash

# 2. Build the packages
colcon build --packages-select waybionic_description waybionic_bringup

# 3. Source the install setup
source install/setup.bash

# 4. Launch the model
ros2 launch waybionic_bringup display.launch.py
```

### RViz Configuration

> **Note:** If you launch via `display.launch.py` or `test_all.launch.py`, RViz auto-loads the saved `waybionic.rviz` config (Fixed Frame, RobotModel display, and Description Topic are already set), so you normally **don't** need the steps below. **However, if the model doesn't appear** (empty view, or you started RViz with bare `rviz2`), configure it manually as follows:

Once launched, configure RViz in the **main window**:

1. In **Global Options** (left panel), set **Fixed Frame** to `base_link`
2. Click **Add** (bottom left), select **RobotModel**, then click **Add**
3. Expand **RobotModel** settings and set **Description Topic** to `robot_description`
4. Click **Add** again, select **TF**, then click **Add**
5. Use the **Joint State Publisher GUI** (small window) to move the arm with the slider

---

## 3. Final Check

Before launching, confirm the following:

- [ ] All files are in the correct directories
- [ ] The workspace has been built without errors
- [ ] The launch command runs successfully
- [ ] The Waybionic model displays as expected in RViz