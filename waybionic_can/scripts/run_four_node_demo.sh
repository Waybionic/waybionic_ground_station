#!/bin/bash
# Start three simulated receivers in the background and an interactive gateway on one CAN
# interface. Receiver logs go to the same terminal, prefixed with their node ID.
#
#   ros2 run waybionic_can run_four_node_demo.sh [iface]
#   BIN_DIR=build/waybionic_can waybionic_can/scripts/run_four_node_demo.sh vcan0
set -euo pipefail

IFACE="${1:-vcan0}"
BIN_DIR="${BIN_DIR:-$(cd "$(dirname "$0")" && pwd)}"
if [[ ! -x "$BIN_DIR/receiver" ]] && command -v ros2 >/dev/null; then
  BIN_DIR="$(ros2 pkg prefix waybionic_can)/lib/waybionic_can"
fi
if [[ ! -x "$BIN_DIR/receiver" || ! -x "$BIN_DIR/gateway" ]]; then
  echo "gateway/receiver not found in $BIN_DIR; build first or set BIN_DIR" >&2
  exit 1
fi
if ! ip link show "$IFACE" >/dev/null 2>&1; then
  echo "$IFACE does not exist; run scripts/setup_vcan.sh" >&2
  exit 1
fi

pids=()
trap 'kill "${pids[@]}" 2>/dev/null; wait' EXIT
for id in 1 2 3; do
  "$BIN_DIR/receiver" --id "$id" --iface "$IFACE" &
  pids+=("$!")
done
sleep 0.2

echo "Try: mode 2 / enable 2 / move 2 16384 600 2 / read 2 / status / quit"
"$BIN_DIR/gateway" --iface "$IFACE" --nodes 1,2,3
