#!/bin/bash
# Double-click to install the WayBionic ground station on this Mac.
#
# The app starts the station from a folder, so this keeps a copy of this one in ~/WayBionic
# and puts "WayBionic Ground Station" in ~/Applications. It does not set up the environment:
# the app offers that the first time it is opened.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
VERSION="$(head -n 1 "$HERE/VERSION")"
DEST="$HOME/WayBionic/ground-station-$VERSION"

echo "WayBionic Ground Station $VERSION"
if [[ "$HERE" == "$DEST" ]]; then
  echo "This folder is already in place."
elif [[ -e "$DEST" ]]; then
  echo "$DEST already exists; using it."
else
  mkdir -p "$HOME/WayBionic"
  echo "Copying to $DEST ..."
  cp -R "$HERE" "$DEST"
  # macOS marks the files of a download, which would stop the app from opening. Clear that on
  # this copy, which the person installing has chosen to run.
  xattr -cr "$DEST" 2>/dev/null || true
fi

cd "$DEST"
./scripts/macos.sh app
echo
echo "Installed. Open \"WayBionic Ground Station\" from ~/Applications, which is opening now."
echo "The first time, it offers to set up the environment: 30 to 60 minutes and a few GB, once."
open "$HOME/Applications" 2>/dev/null || true
read -r -p "Press Return to close this window. " _ || true
