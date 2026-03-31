#!/usr/bin/env bash

set -euo pipefail

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"

set +u
source /opt/ros/humble/setup.bash
set -u

cd /workspaces/project/src/ros2_ws

colcon build --symlink-install --packages-select frontier_explorer

set +u
source /workspaces/project/src/ros2_ws/install/setup.bash
set -u

exec "$@"
