#!/bin/bash
set -e

# Dynamically find the workspace root (assuming script is in ws/src/repo/scripts)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
WS_ROOT="$( cd "$SCRIPT_DIR/../../.." >/dev/null 2>&1 && pwd )"

echo "=== Waybionic Workspace Sanity Check ==="
echo "Moving to workspace root: $WS_ROOT"
cd "$WS_ROOT"

echo "1. Sourcing ROS Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "2. Checking dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y

echo "3. Building Waybionic packages..."
colcon build --packages-select waybionic_description waybionic_bringup

echo "4. Sourcing workspace..."
source install/setup.bash

echo "5. Parsing URDF for syntax errors..."
# Find the URDF dynamically regardless of the clone folder name
URDF_PATH=$(find src -name "waybionic_placeholder.urdf" | head -n 1)
xacro $URDF_PATH > /dev/null

echo "=== ✅ All checks passed! The foundation is clean and ready. ==="
