#!/usr/bin/env bash
# Runs inside the waybionic wslg container: UDP joint-state receiver + ground station.
# Note: do NOT use `set -u` — /opt/ros/jazzy/setup.bash reads unbound AMENT_* vars.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /waybionic_ws/install/setup.bash
export PYTHONPATH="/embedded:${PYTHONPATH:-}"

VIZ_PORT="${VIZ_PORT:-47301}"

python3 -m waybionic_embedded_bridge.udp_joint_state_receiver --ros-args -p "port:=${VIZ_PORT}" &
RECV_PID=$!
cleanup() { kill "$RECV_PID" 2>/dev/null || true; }
trap cleanup EXIT

echo "[embedded] UDP joint-state receiver on :${VIZ_PORT}"
exec ros2 launch waybionic_bringup ground_station.launch.py \
  use_joint_state_publisher_gui:=false \
  start_temporary_diagnostics_publisher:=true
