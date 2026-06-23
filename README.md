# AUV Multi-Modal SLAM Stack

A modular ROS 2 system that runs every practical form of SLAM on an autonomous
underwater vehicle (visual, visual-inertial, imaging-sonar, bathymetric-sonar,
side-scan, plus DVL/IMU/depth/USBL dead-reckoning), fused through a single
GTSAM/iSAM2 factor-graph back-end, validated in two physics simulators
(**Gazebo Harmonic / Project DAVE** and **Stonefish**) on a **BlueROV2**.


> **Build track:** This workspace targets the spec's **Track B** — ROS 2
> **Humble** + Ubuntu **22.04** + **Gazebo Harmonic** (gz-sim 8) via `ros_gz`.
> The architecture is distro-agnostic; moving to Track A (Jazzy/24.04) is a
> Docker base-image change, not a code change.

## Architecture 

| Package | Role |
|---|---|
| `auv_bringup` | Top-level launch, global params, RViz config, `INTERFACE.md` |
| `auv_interfaces` | All custom msgs/srvs/actions (the frozen API) |
| `auv_description` | BlueROV2 URDF/xacro, sensor macros, TF tree |
| `auv_sim_common` | Sim-abstraction layer: ground-truth republisher, depth-from-pressure |
| `auv_sim_gazebo` | Gazebo Harmonic worlds, spawn, `ros_gz` bridge |
| `auv_sim_stonefish` | Stonefish scenario XML + adapter (vendored sim) |
| `auv_control` | Thruster allocation + teleop |
| `auv_state_estimation` | `robot_localization` EKF + GTSAM nav |
| `auv_dead_reckoning` | DVL+IMU dead-reckoning reference |
| `auv_image_enhancement` | Underwater image restoration (CLAHE/WB/UDCP) |
| `auv_visual_slam` | ORB-SLAM3 / RTAB-Map wrappers + constraint adapters |
| `auv_sonar_slam` | FLS/MSIS feature SLAM + MBES submap SLAM |
| `auv_place_recognition` | Visual + acoustic loop-closure detection |
| `auv_slam_core` | GTSAM/iSAM2 factor-graph integrator (the unifier) |
| `auv_mapping` | Dense cloud / occupancy / 2.5D bathymetry / mosaics |
| `auv_evaluation` | `evo` ATE/RPE + rosbag regression harness |
| `auv_drivers` | Sim-to-real contract stubs |

## Quick start (Docker — reproducible)

```bash
# 1. Build the pinned environment (Ubuntu 22.04 + Humble + Gazebo Harmonic + GTSAM)
docker build -t auv-slam -f docker/Dockerfile .

# 2. Pull vendored upstream repos (sonar msgs, Stonefish, BlueROV2 refs)
cd src/third_party && vcs import < ../../dependencies.repos ; cd ../..

# 3. Build the workspace
docker run --rm -it -v "$PWD:/opt/auv_slam_ws" auv-slam \
    bash -lc "colcon build --symlink-install"

# 4. Run the full stack in Gazebo
docker run --rm -it --net=host -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix -v "$PWD:/opt/auv_slam_ws" auv-slam \
    bash -lc "source install/setup.bash && \
      ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=all rviz:=true"
```

## Quick start (native, Humble already installed)

```bash
source /opt/ros/humble/setup.bash
pip3 install "gtsam==4.2" "evo==1.28.0" "numpy<2" scipy
rosdep install --from-paths src --ignore-src -y -r || true
colcon build --symlink-install
source install/setup.bash

# Launch presets (see auv_bringup/launch/)
ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=all rviz:=true
ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=visual
ros2 launch auv_bringup bringup.launch.py sim:=stonefish modalities:=sonar
```

Launch arguments: `sim:={gazebo,stonefish}`, `modalities:={visual,sonar,all}`,
`use_sim_time:=true`, `rviz:=true`, `record:=false`.

## Build status by milestone

The spec is built milestone-by-milestone (M0–M11). See
[`docs/BUILD_STATUS.md`](docs/BUILD_STATUS.md) for what is implemented and runnable
in this environment versus what requires vendored GPU simulators / external libs.

## Conventions you must respect

- ENU world / FLU body **everywhere** in SLAM. Convert NED/FRD at sensor boundaries only.
- Depth sign: pressure increases with depth; in ENU `z = -d`. (The #1 catastrophic bug.)
- Exactly one publisher per TF edge. Never let SLAM consume ground truth.

## License

Code in this repository: Apache-2.0 (see `LICENSE`). Vendored upstream repositories
retain their own licenses (e.g. ORB-SLAM3 is GPLv3 — keep it isolated as a submodule).
