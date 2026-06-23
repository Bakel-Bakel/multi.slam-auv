# Interface Contract (spec §6) — FROZEN at M1

This is the project's API. Both simulators publish into it; every front-end consumes
only from it. Changing anything here is a breaking change reviewed against all packages.

## Frames (ENU world, FLU body — REP-103/REP-105)

```
earth (optional, geo-ref / USBL/GNSS)
 └── map                 # SLAM-optimized world frame (ENU). Publisher: auv_slam_core
      └── odom           # smooth, drifts slowly (ENU). Publisher: auv_state_estimation
           └── base_link # vehicle body (FLU). Publisher: estimation (odom->base_link)
                ├── base_link_frd       # static FRD mirror for marine-convention sensors
                ├── imu_link
                ├── dvl_link
                ├── pressure_link
                ├── camera_left_link / camera_right_link / camera_optical_link
                ├── fls_link            # forward-looking sonar (FRD for DAVE multibeam)
                ├── sss_port_link / sss_stbd_link
                └── mbes_link
```
Exactly one publisher per edge: static extrinsics from `robot_state_publisher` (URDF);
`odom->base_link` from estimation only; `map->odom` from `auv_slam_core` only.
Ground truth lives on a **separate** `gt/*` frame namespace (eval overlay only).

## Topics

| Topic | Type | Frame | QoS | Producer → Consumer |
|---|---|---|---|---|
| `/clock` | `rosgraph_msgs/Clock` | — | Reliable | sim → all (`use_sim_time`) |
| `/imu/data` | `sensor_msgs/Imu` | `imu_link` | SensorData (BE/Volatile) | sim → estimation, VIO, slam_core |
| `/pressure` | `sensor_msgs/FluidPressure` | `pressure_link` | SensorData | sim → depth node → estimation |
| `/depth` | `geometry_msgs/PointStamped` | `map` | Reliable | `depth_from_pressure` (z = -d) → estimation |
| `/magnetic` | `sensor_msgs/MagneticField` | `imu_link` | SensorData | sim → estimation (heading aid) |
| `/dvl` | `auv_interfaces/DVL` | `dvl_link` | SensorData | sim → dead_reckoning, estimation, slam_core |
| `/usbl` | `auv_interfaces/USBLFix` | `usbl_link` | Reliable | sim → slam_core (absolute prior) |
| `/cam/left/image_raw` | `sensor_msgs/Image` | `camera_left_optical_link` | SensorData | sim → image_enhancement |
| `/cam/right/image_raw` | `sensor_msgs/Image` | `camera_right_optical_link` | SensorData | sim → image_enhancement |
| `/cam/left/camera_info` | `sensor_msgs/CameraInfo` | — | SensorData | sim → visual_slam |
| `/cam/right/camera_info` | `sensor_msgs/CameraInfo` | — | SensorData | sim → visual_slam |
| `/cam/left/image_enhanced` | `sensor_msgs/Image` | `camera_left_optical_link` | SensorData | image_enhancement → visual_slam |
| `/cam/right/image_enhanced` | `sensor_msgs/Image` | `camera_right_optical_link` | SensorData | image_enhancement → visual_slam |
| `/sonar/fls/image` | `marine_acoustic_msgs/ProjectedSonarImage` | `fls_link` | SensorData | sim → sonar_slam |
| `/sonar/sss/image` | `marine_acoustic_msgs/ProjectedSonarImage` | `sss_*_link` | SensorData | sim → mapping, place_recognition |
| `/sonar/msis/image` | `marine_acoustic_msgs/ProjectedSonarImage` | `fls_link` | SensorData | sim → sonar_slam |
| `/sonar/mbes/points` | `sensor_msgs/PointCloud2` | `mbes_link` | SensorData | sim → sonar_slam, mapping |
| `/odom` | `nav_msgs/Odometry` | `odom`→`base_link` | Reliable | estimation → slam_core, control |
| `/slam/pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map` | Reliable | slam_core → mapping, eval |
| `/slam/trajectory` | `nav_msgs/Path` | `map` | Reliable(TL) | slam_core → mapping, eval |
| `/slam/status` | `auv_interfaces/SlamStatus` | — | Reliable | slam_core → bringup/UI |
| `/slam/constraints` | `auv_interfaces/*` (per modality) | `map` | Reliable | front-ends → slam_core |
| `/ground_truth/pose` | `geometry_msgs/PoseStamped` | `gt_map` | Reliable | sim → evaluation ONLY |
| `/cmd_vel` | `geometry_msgs/Twist` | `base_link` | Reliable | teleop/control → thruster alloc |
| `/thruster_cmds` | `std_msgs/Float64MultiArray` | — | Reliable | control → sim |

## QoS profiles (spec §6)
- **SensorData**: Best-Effort, Volatile, depth 5/10. Images, sonar, IMU, DVL.
- **Reliable**: Reliable, Volatile, depth 10. State/odom/pose/cmd.
- **Reliable(TL)**: Reliable, **Transient-Local**. Trajectory, `/tf_static`.
- Mismatched QoS = silent "no messages". Every endpoint documents its profile in code.

## Custom types
Defined in `auv_interfaces`: `DVL`, `USBLFix`, `SlamStatus`, `SaveMap`, `TriggerLoopSearch`,
`RunTrajectory`. Imaging sonar uses vendored `marine_acoustic_msgs`.
