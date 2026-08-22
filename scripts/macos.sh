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
  run      Run any command inside the ROS workspace
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
    run_environment colcon build --symlink-install
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
    run_workspace ros2 launch waybionic_bringup ground_station.launch.py "$@"
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