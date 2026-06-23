#!/usr/bin/env bash
# Build ros_gz against Gazebo Harmonic for ROS 2 Humble (spec Track B seam).
#
# Humble apt ros_gz links ignition-transport11 (Fortress). Harmonic 8 uses
# gz-transport13, so the stock bridge/create cannot talk to gz sim 8.
# This script vendors ros_gz and builds the Harmonic-compatible overlay.
#
# Usage (from workspace root):
#   src/auv_sim/auv_sim_gazebo/scripts/build_ros_gz_harmonic.sh
#   source install/setup.bash
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
SRC="${WS}/src/third_party/ros_gz"

echo "[ros_gz] workspace: ${WS}"

if [[ ! -d "${SRC}/.git" ]]; then
  mkdir -p "$(dirname "${SRC}")"
  git clone --depth 1 --branch humble https://github.com/gazebosim/ros_gz.git "${SRC}"
fi

cd "${WS}"
source /opt/ros/humble/setup.bash

export GZ_VERSION=harmonic
rosdep install --from-paths "${SRC}" --ignore-src -y -r || true

colcon build \
  --packages-up-to ros_gz ros_gz_sim ros_gz_bridge ros_gz_interfaces ros_gz_image \
  --cmake-args -DBUILD_TESTING=OFF

echo
echo "[ros_gz] Done. Re-source and verify transport linkage:"
echo "  source ${WS}/install/setup.bash"
echo "  ldd ${WS}/install/ros_gz_sim/lib/ros_gz_sim/create | grep transport"
