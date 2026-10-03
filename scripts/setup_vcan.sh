#!/bin/bash
set -e

echo "=== Setting up Virtual CAN interface (vcan0) ==="

# Load the virtual CAN kernel module
sudo modprobe vcan

# Create the vcan0 link (ignore error if it already exists)
sudo ip link add dev vcan0 type vcan 2>/dev/null || true

# Bring the interface up
sudo ip link set up vcan0

echo "✅ vcan0 is up and running!"
echo "You can monitor traffic by running: candump vcan0"
