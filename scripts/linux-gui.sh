#!/usr/bin/env bash
set -euo pipefail

# Write an error message to standard error and exit with status 1.
fail() {
  printf '%s\n' "$*" >&2
  exit 1
}

build=false
detach=false
stop=false
while (($#)); do
  case "$1" in
    --build) build=true ;;
    -d|--detach) detach=true ;;
    --stop) stop=true ;;
    -h|--help)
      printf '%s\n' \
        'Usage: linux-gui.sh [--build] [-d|--detach] [-- COMMAND ARG...]' \
        '       linux-gui.sh --stop' \
        'Rebuilds the ROS workspace from live source on launch. Use --build after dependency changes.' \
        'Detached windows remain open after the terminal closes; --stop removes the container.'
      exit 0 ;;
    --) shift; break ;;
    -*) fail "Unknown option: $1 (see --help)." ;;
    *) break ;;
  esac
  shift
done

[[ $(uname -s) == Linux ]] || fail "Run this launcher on a Linux desktop."
command -v docker >/dev/null || fail "Install Docker Engine and the Compose plugin first."
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
repo_id=$(printf '%s' "$repo_root" | cksum)
container_name="waybionic-linux-gui-${UID}-${repo_id%% *}"
if $stop; then
  if $build || $detach || (($#)); then
    fail "Use --stop on its own."
  fi
  # Also works from a terminal without a display session.
  exec docker stop "$container_name"
fi

command -v xauth >/dev/null || fail "Install xauth (Fedora: sudo dnf install xorg-x11-xauth; Ubuntu: sudo apt install xauth)."
[[ ${DISPLAY:-} =~ ^:[0-9]+(\.[0-9]+)?$ ]] || fail "Run from a local desktop terminal with DISPLAY set (Wayland requires XWayland)."
display_number=${DISPLAY#:}
display_number=${display_number%%.*}
[[ -S /tmp/.X11-unix/X${display_number} ]] || fail "The X11/XWayland display socket is missing."
docker compose version >/dev/null

compose=(docker compose -f "$repo_root/compose.yaml")
if ! $build; then
  image_name=$("${compose[@]}" config --images linux-gui)
  docker image inspect "$image_name" >/dev/null 2>&1 || fail "The GUI image is unavailable. Run ./scripts/linux-gui.sh --build first (and check Docker access)."
fi
# Keep credentials outside the build context. Once mounted, Linux keeps the file
# available to the container even after the host pathname is removed on exit.
# This also cleans up immediately after a detached launch, without a background job.
auth_dir=$(mktemp -d /tmp/waybionic-xauth.XXXXXXXX)
trap 'rm -rf -- "$auth_dir"' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
xauth -i nlist "$DISPLAY" > "$auth_dir/records"
[[ -s $auth_dir/records ]] || fail "No display cookie found. Run as your desktop user, without sudo, and check XAUTHORITY."
export WAYBIONIC_XAUTHORITY="$auth_dir/Xauthority"
touch "$WAYBIONIC_XAUTHORITY"
# FamilyWild allows the cookie to match the container's different hostname.
sed 's/^..../ffff/' "$auth_dir/records" | xauth -f "$WAYBIONIC_XAUTHORITY" nmerge -
# The parent directory is private; the mounted file must be readable by ubuntu.
chmod 644 "$WAYBIONIC_XAUTHORITY"
run_options=(--rm --pull never --name "$container_name")
if $build; then run_options+=(--build); fi
if $detach; then run_options+=(--detach --interactive=false); fi
"${compose[@]}" run "${run_options[@]}" linux-gui "$@"
if $detach; then
  printf 'GUI running in background. Stop with: %s/scripts/linux-gui.sh --stop\n' "$repo_root"
fi
