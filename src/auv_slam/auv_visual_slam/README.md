# auv_visual_slam

Visual / visual-inertial SLAM front-end + constraint adapter (spec §10.2, M7).

## Node: `visual_slam_adapter`
Consumes any visual pose source (`/visual/odom`, `nav_msgs/Odometry`) and emits
keyframe-to-keyframe relative constraints (`auv_interfaces/LoopClosure`,
`modality="visual"`, `is_sequential=True`) on `/slam/loop_closures` into `auv_slam_core`.
Loop closures from the backend are forwarded with `is_sequential=False` (robust kernel).

## Backends (`backend:=` launch arg)
- `rtabmap` (default, recommended cross-check): stereo RTAB-Map from the **enhanced**
  stream. Binary package `ros-humble-rtabmap-ros`. Publishes `/visual/odom`.
- `none`: adapter only; expects an external `/visual/odom`.
- `orbslam3`: **vendoring seam** — see below.

## ORB-SLAM3 seam (spec §16, §15.9)
Primary backend per spec is ORB-SLAM3 via `Mechazo11/ros2_orb_slam3` (`humble` branch).
Vendor it under `src/third_party/` (uncomment in `dependencies.repos`), build ORB-SLAM3
as a shared lib with **its pinned** DBoW2/g2o/Sophus (do **not** mix with system g2o),
pin OpenCV ≥4.2 + Pangolin. Remap its keyframe-pose output to `/visual/odom` and run with
`backend:=none`. Supports mono/stereo/RGB-D + visual-inertial + multi-map (Atlas) loop
closure — covering both "visual" and "visual-inertial" alone.

## Run (M7 acceptance)
```bash
ros2 launch auv_visual_slam visual_slam.launch.py backend:=rtabmap
# over a textured-seafloor loop, expect init/track + loop closure; evo beats raw DR.
# (Tracking loss over textureless sand is realistic — rely on fusion.)
```
