# auv_slam_core

The factor-graph integrator (spec §10.6, M9) — **the literal answer to "do all forms of
SLAM": every modality is a factor type into one graph.**

## Backend (`graph_backend.py`, ROS-free, unit-tested)
GTSAM + iSAM2 pose graph: `PriorFactorPose3` (anchor + depth partial prior),
`BetweenFactorPose3` (odometry + loop closures), `GPSFactor` (USBL position prior).
Loop closures use a **robust Huber kernel** so false matches don't break the estimate.
Tested: a drifted square loop closes; a USBL prior pulls the estimate.

## Node (`slam_core`)
Ingests through one uniform path:
- `/odom` (estimation backbone) -> sequential `BetweenFactor`s + keyframes,
- `/depth/odometry` -> per-keyframe depth (z) prior,
- `/usbl` -> absolute position prior,
- `/slam/loop_closures` (`auv_interfaces/LoopClosure` from visual/sonar/place-recognition)
  -> between-keyframe factors (robust for non-sequential).

Publishes `/slam/pose`, `/slam/trajectory`, the `map->odom` correction (the only publisher
of that edge), and `/slam/status`. Modalities toggle via the `modalities` param. Never
consumes ground truth.

## Run (M9 acceptance)
```bash
ros2 launch auv_slam_core slam_core.launch.py
# With all modalities, fused evo ATE/RPE beats every single-modality result;
# toggling any modality works; a false loop closure does not break the estimate.
ros2 topic echo /slam/status --once
```
