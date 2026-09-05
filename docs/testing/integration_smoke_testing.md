# Smoke Testing Document - Baseline
This document contains instructions for running the stable baseline main launch. Specifically use this document to run a smoke testing checklist across active PRs.
## Full build/test
As usual, run robot visualization and ground station launch
```
source /opt/ros/jazzy/setup.bash
source ~/waybionic_ws/install/setup.bash
ros2 launch waybionic_bringup display.launch.py
```
Expect RViz and Joint State Publisher GUI opens
## Mock diagnostics
Test that mock mode runs as expected. [TODO: document expected behaviour]
```
ros2 launch waybionic_rviz_plugins engineer_view.launch.py use_mock_diagnostics:=true
```
RViz engineer monitor view should pop up, and the option to switch between Fault and Normal states under the "Waybionic Diagnostics" panel.
## Live diagnostics with temporary publisher
Open two terminals and source setup files (refer to "full build/test").

Terminal 1:
```
ros2 launch waybionic_rviz_plugins temporary_diagnostics_publisher.launch.py
```