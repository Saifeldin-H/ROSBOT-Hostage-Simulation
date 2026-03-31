#!/usr/bin/env bash

set -euo pipefail

source /opt/ros/humble/setup.bash

cd /workspaces/project/src/ros2_ws

colcon build --symlink-install --packages-select frontier_explorer

source /workspaces/project/src/ros2_ws/install/setup.bash

exec "$@"
