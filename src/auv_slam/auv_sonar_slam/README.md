# auv_sonar_slam

Imaging-sonar (FLS/MSIS) feature SLAM + bathymetric (MBES) submap SLAM (spec §10.3/§10.4).

## Pure, tested building blocks
- `icp.py` — point-to-point ICP (numpy + scipy KD-tree). Seam for PCL GICP / probabilistic
  ICP. Tested: recovers known translation + yaw.
- `cfar.py` — CA-CFAR detector + range/bearing fan -> XYZ feature points. Tested.

## Nodes
- `mbes_submap_slam` — compounds `/sonar/mbes/points` swaths into submaps via `/odom`,
  voxel-subsamples, registers overlapping submaps with ICP, and emits `mbes` loop closures
  (`auv_interfaces/LoopClosure`) into `auv_slam_core`. Publishes `/mbes/submap_cloud`.
- `fls_feature_slam` — CFAR features on a mono FLS image -> scan-to-scan ICP -> sequential
  constraints. (Adapt `marine_acoustic_msgs/ProjectedSonarImage` at the input seam.)

## Run (M8 acceptance)
```bash
ros2 launch auv_sonar_slam sonar_slam.launch.py mbes:=true fls:=false
# On terrain with relief, MBES submap registration reduces trajectory error vs DR.
# Validate FLS features/registration on Stonefish's high-fidelity sonar.
```
