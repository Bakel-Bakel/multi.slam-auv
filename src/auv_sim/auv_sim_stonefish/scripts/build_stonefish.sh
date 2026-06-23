#!/usr/bin/env bash
# Vendor + build Stonefish and stonefish_ros2 (spec §7.2, §16).
# Pins are tracked in dependencies.repos; set fixed commits before relying on this.
set -euo pipefail

WS_SRC="${1:-$(pwd)}/src/third_party"
mkdir -p "${WS_SRC}"
cd "${WS_SRC}"

echo "[1/3] Cloning Stonefish + stonefish_ros2 ..."
[ -d stonefish ] || git clone https://github.com/patrykcieslak/stonefish.git
[ -d stonefish_ros2 ] || git clone https://github.com/patrykcieslak/stonefish_ros2.git

echo "[2/3] Building the Stonefish C++ library (distro-agnostic) ..."
# Build deps (Ubuntu): sudo apt install libglm-dev libsdl2-dev libfreetype-dev \
#   libglew-dev libopenal-dev
cd stonefish
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j"$(nproc)"
sudo cmake --install .
cd "${WS_SRC}"

echo "[3/3] stonefish_ros2 will be built by colcon as part of the workspace."
echo "Done. Now: cd <ws> && colcon build --packages-select stonefish_ros2 auv_sim_stonefish"
