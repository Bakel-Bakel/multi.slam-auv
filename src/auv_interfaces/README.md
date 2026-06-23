# auv_interfaces

The project's frozen API (spec M1). **All** custom messages, services, and actions
live here; no other package defines its own interfaces.

## Messages
- `msg/DVL.msg` — per-beam ranges/velocities, aggregated body velocity + covariance,
  `bottom_locked`, figure-of-merit, altitude. (Reported in `header.frame_id`, the DVL
  sensor frame; transform to `base_link` before fusing.)
- `msg/USBLFix.msg` — relative range/bearing/elevation (+ Cartesian) with covariance,
  beacon id, and an optional absolute lat/lon/alt fix when surface GNSS is available.
- `msg/SlamStatus.msg` — active modules, keyframe/loop/factor counts, optimization timing,
  drift estimate. Published by `auv_slam_core`.

## Services
- `srv/SaveMap.srv` — write a map product (`pcd`/`ply`/`octomap`/`bathymetry`/`all`).
- `srv/TriggerLoopSearch.srv` — force a place-recognition loop search by modality.

## Actions
- `action/RunTrajectory.action` — drive the vehicle through waypoints for data collection.

## Standard messages (reused, defined upstream — see INTERFACE.md)
- `sensor_msgs/Imu`, `sensor_msgs/FluidPressure`, `sensor_msgs/MagneticField`,
  `sensor_msgs/Image`, `sensor_msgs/CameraInfo`, `sensor_msgs/PointCloud2`,
  `nav_msgs/Odometry`, `nav_msgs/Path`, `geometry_msgs/PoseWithCovarianceStamped`.
- Imaging sonar uses the community standard `marine_acoustic_msgs`
  (`ProjectedSonarImage`, `RawSonarImage`), vendored under `src/third_party/`.

## Verify (M1 acceptance)
```bash
ros2 interface show auv_interfaces/msg/DVL
ros2 interface show auv_interfaces/msg/USBLFix
ros2 interface show auv_interfaces/msg/SlamStatus
ros2 interface show auv_interfaces/srv/SaveMap
ros2 interface show auv_interfaces/action/RunTrajectory
```
