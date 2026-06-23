#!/usr/bin/env bash
set -e

# Source ROS and the workspace overlay if it has been built.
source /opt/ros/humble/setup.bash
if [ -f "${AUV_WS:-/opt/auv_slam_ws}/install/setup.bash" ]; then
  source "${AUV_WS:-/opt/auv_slam_ws}/install/setup.bash"
fi

# Simulation-first: default to sim time everywhere unless overridden.
export AUV_USE_SIM_TIME="${AUV_USE_SIM_TIME:-true}"

exec "$@"
