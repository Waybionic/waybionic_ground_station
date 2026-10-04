#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_NAME="waybionic_robostack"
ENV_FILE="$ROOT/robostack.yaml"
MANAGER=""
ENV_EXISTS=false

fail() {
  echo "error: $*" >&2
  exit 1
}

usage() {
  cat <<'EOF'
Usage: ./scripts/macos.sh COMMAND [ARGS...]

Commands:
  setup    Create/update the RoboStack environment and build the workspace
  build    Build the workspace
  launch   Launch the ground station visualization
  flash    Compile and upload carrier_bridge to the UNO R4 WiFi (needs arduino-cli)
  drives   Set up the MKS drives through the carrier, e.g. drives scan (see mks_setup)
  arm      Drive the real arm: controller on this Mac, carrier on USB
  run      Run any command inside the ROS workspace

The carrier is found as the only /dev/cu.usbmodem* port; set WAYBIONIC_CARRIER to choose.
EOF
}

check_host() {
  [[ "$(uname -s)" == "Darwin" ]] || fail "this helper is for macOS"
  if [[ "$(uname -m)" != "arm64" ]]; then
    echo "warning: this setup is tested on Apple Silicon; Intel macOS is unverified" >&2
  fi
}

select_manager() {
  local candidate

  if [[ -n "${CONDA_EXE:-}" && -x "$CONDA_EXE" ]] \
    && "$CONDA_EXE" env list 2>/dev/null \
      | awk '{print $1}' \
      | grep -qx "$ENV_NAME"; then
    MANAGER="$CONDA_EXE"
    ENV_EXISTS=true
    return
  fi

  for candidate in mamba conda; do
    if command -v "$candidate" >/dev/null 2>&1 \
      && "$candidate" env list 2>/dev/null \
        | awk '{print $1}' \
        | grep -qx "$ENV_NAME"; then
      MANAGER="$candidate"
      ENV_EXISTS=true
      return
    fi
  done

  for candidate in mamba conda; do
    if command -v "$candidate" >/dev/null 2>&1; then
      MANAGER="$candidate"
      return
    fi
  done

  fail "install Miniforge first: brew install --cask miniforge"
}

prepare() {
  check_host
  select_manager
  if [[ -z "${CONDA_BUILD_SYSROOT:-}" ]]; then
    CONDA_BUILD_SYSROOT="$(xcrun --show-sdk-path)"
    export CONDA_BUILD_SYSROOT
  fi
}

setup_environment() {
  if [[ "$ENV_EXISTS" == true ]]; then
    echo "Updating $ENV_NAME with $MANAGER..."
    if [[ "$(basename "$MANAGER")" == "mamba" ]]; then
      "$MANAGER" env update --yes --name "$ENV_NAME" --file "$ENV_FILE" --prune
    else
      "$MANAGER" env update --name "$ENV_NAME" --file "$ENV_FILE" --prune
    fi
  else
    echo "Creating $ENV_NAME with $MANAGER..."
    "$MANAGER" env create --yes --file "$ENV_FILE"
    ENV_EXISTS=true
  fi
}

ensure_environment() {
  if [[ "$ENV_EXISTS" != true ]]; then
    fail "$ENV_NAME is missing; run ./scripts/macos.sh setup"
  fi
}

run_environment() {
  if [[ "$(basename "$MANAGER")" == "mamba" ]]; then
    "$MANAGER" run --attach "" -n "$ENV_NAME" "$@"
  else
    "$MANAGER" run --no-capture-output -n "$ENV_NAME" "$@"
  fi
}

build_workspace() {
  ensure_environment
  echo "Building workspace..."
  (
    cd "$ROOT"
    run_environment colcon build --symlink-install \
      --cmake-args "-DCMAKE_OSX_SYSROOT=$CONDA_BUILD_SYSROOT"
  )
}

ensure_workspace() {
  ensure_environment
  if [[ ! -f "$ROOT/install/setup.bash" ]]; then
    build_workspace
  fi
}

run_workspace() {
  ensure_workspace
  # shellcheck disable=SC2016
  run_environment bash -c '
    cd "$1" || { echo "error: cannot enter workspace $1" >&2; exit 1; }
    . install/setup.bash || {
      echo "error: failed to source install/setup.bash; run ./scripts/macos.sh build" >&2
      exit 1
    }
    shift
    exec "$@"
  ' _ "$ROOT" "$@"
}

find_carrier() {
  local ports

  if [[ -n "${WAYBIONIC_CARRIER:-}" ]]; then
    [[ -e "$WAYBIONIC_CARRIER" ]] || fail "WAYBIONIC_CARRIER=$WAYBIONIC_CARRIER does not exist"
    CARRIER="$WAYBIONIC_CARRIER"
    return
  fi
  shopt -s nullglob
  ports=(/dev/cu.usbmodem*)
  shopt -u nullglob
  case "${#ports[@]}" in
    0) fail "no /dev/cu.usbmodem* port: plug in the carrier (UNO R4 WiFi)" ;;
    1) CARRIER="${ports[0]}" ;;
    *) fail "several ports (${ports[*]}); set WAYBIONIC_CARRIER to the carrier's" ;;
  esac
}

flash_carrier() {
  local fqbn="arduino:renesas_uno:unor4wifi"
  local sketch="$ROOT/waybionic_can/arduino/carrier_bridge"
  local library="$ROOT/waybionic_can/arduino/libraries/WaybionicCan"

  command -v arduino-cli >/dev/null 2>&1 \
    || fail "install arduino-cli first: brew install arduino-cli"
  find_carrier
  arduino-cli core update-index
  arduino-cli core install arduino:renesas_uno
  arduino-cli compile --fqbn "$fqbn" --library "$library" "$sketch"
  arduino-cli upload --fqbn "$fqbn" --port "$CARRIER" "$sketch"
  echo "Flashed carrier_bridge to $CARRIER"
}

command_name="${1:-help}"
if [[ $# -gt 0 ]]; then
  shift
fi

case "$command_name" in
  help|-h|--help)
    usage
    ;;
  setup)
    prepare
    setup_environment
    build_workspace
    echo
    echo "Setup complete. Launch with: ./scripts/macos.sh launch"
    ;;
  build)
    prepare
    build_workspace
    ;;
  launch)
    prepare
    run_workspace env RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
      ros2 launch waybionic_bringup ground_station.launch.py "$@"
    ;;
  flash)
    check_host
    flash_carrier
    ;;
  drives)
    [[ $# -gt 0 ]] || fail "drives needs an mks_setup command, such as scan"
    prepare
    find_carrier
    run_workspace ros2 run waybionic_teleop mks_setup --channel "$CARRIER" "$@"
    ;;
  arm)
    prepare
    find_carrier
    echo "Carrier: $CARRIER"
    run_workspace env RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
      ros2 launch waybionic_bringup ground_station.launch.py teleop:=true \
      joy_source:=device drive_interface:=slcan drive_channel:="$CARRIER" "$@"
    ;;
  run)
    [[ $# -gt 0 ]] || fail "run requires a command"
    prepare
    run_workspace "$@"
    ;;
  *)
    usage >&2
    fail "unknown command: $command_name"
    ;;
esac