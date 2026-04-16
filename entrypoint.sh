#!/usr/bin/env bash
set -e
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
cd /ros2_ws
exec "$@"
