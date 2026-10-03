#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [[ -f /waybionic_ws/install/setup.bash ]]; then
	source /waybionic_ws/install/setup.bash
fi

exec "$@"
