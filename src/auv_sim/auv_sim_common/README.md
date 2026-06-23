# auv_sim_common

The simulation-abstraction keystone (spec §7.4). Downstream code subscribes **only**
to the interface contract; per-simulator adapters normalize native topics onto it, and
the shared nodes here finish the job.

## Nodes
- `ground_truth_republisher` — takes a sim's ground-truth `nav_msgs/Odometry` and
  republishes `/ground_truth/pose` (frame `gt_map`) + a `gt_map -> gt_base_link` TF on a
  **separate** frame namespace. SLAM must never subscribe to it (eval only).
- `depth_from_pressure` — converts `/pressure` (`sensor_msgs/FluidPressure`) into
  `/depth` (`PointStamped`) and `/depth/odometry` for `robot_localization`, using the
  **`z = -d`** convention (spec §4). Params: `water_density`, `gravity`,
  `atmospheric_pressure`, `pressure_units`, `depth_stddev`.

## Library
- `conversions.py` — pure, dependency-free `pressure_to_depth`, `depth_to_enu_z`,
  ENU/NED helpers. Unit-tested in `test/test_conversions.py` (the depth-sign guard).

## Consumed / produced
- Consumes: `/ground_truth/odom`, `/pressure`.
- Produces: `/ground_truth/pose`, `/depth`, `/depth/odometry`, `gt_*` TF.
