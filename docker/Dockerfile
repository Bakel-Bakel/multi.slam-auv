# AUV Multi-Modal SLAM Stack — reproducible build environment (Track B)
#
# Spec §3 Track A is Jazzy/24.04; this image targets the spec's documented Track B:
# ROS 2 Humble on Ubuntu 22.04 with Gazebo Harmonic via the OSRF vendor packages.
# To move to Track A later, swap the base image to ubuntu:24.04 + ros:jazzy and the
# gz-harmonic apt source pin below; the workspace itself is distro-agnostic.

FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Base tooling ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl gnupg lsb-release \
        python3-pip python3-vcstool python3-colcon-common-extensions \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# --- Gazebo Harmonic (gz-sim 8) via OSRF apt repo -------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --- ROS / perception / sim bridge deps -----------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-robot-localization \
        ros-humble-robot-state-publisher \
        ros-humble-xacro \
        ros-humble-tf2-ros ros-humble-tf2-tools \
        ros-humble-rtabmap-ros \
        ros-humble-rviz2 \
        ros-humble-image-transport ros-humble-cv-bridge \
        ros-humble-pcl-ros ros-humble-pcl-conversions \
        ros-humble-octomap ros-humble-octomap-msgs \
        ros-humble-teleop-twist-keyboard ros-humble-joy ros-humble-teleop-twist-joy \
        ros-humble-launch-testing-ament-cmake \
        libpcl-dev libopencv-dev libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# --- Python estimation / evaluation stack (pinned) ------------------------
# GTSAM 4.2 supplies the iSAM2 factor-graph backend used by auv_slam_core.
RUN pip3 install --no-cache-dir \
        gtsam==4.2 \
        "evo==1.28.0" \
        "numpy<2" scipy

# --- Workspace ------------------------------------------------------------
ENV AUV_WS=/opt/auv_slam_ws
WORKDIR ${AUV_WS}
COPY . ${AUV_WS}/

# Resolve and build (third_party vendored repos pulled via vcs; see dependencies.repos)
RUN source /opt/ros/humble/setup.bash \
    && rosdep update \
    && (rosdep install --from-paths src --ignore-src -y -r || true)

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
