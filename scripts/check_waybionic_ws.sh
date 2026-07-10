#!/bin/bash
# Fail immediately if any command fails
set -e

echo "=== Waybionic Workspace Sanity Check ==="

echo "1. Sourcing ROS Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "2. Checking dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y

echo "3. Building Waybionic packages..."
colcon build --packages-select waybionic_description waybionic_bringup

echo "4. Sourcing workspace..."
source install/setup.bash

echo "5. Parsing URDF for syntax errors..."
# This silently parses the URDF to catch XML errors without launching a GUI
xacro src/waybionic_ground_station/waybionic_description/urdf/waybionic_placeholder.urdf > /dev/null

echo "=== ✅ All checks passed! The foundation is clean and ready. ==="
