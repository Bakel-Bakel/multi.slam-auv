# auv_dead_reckoning

DVL+IMU dead-reckoning reference (spec §10.1, M6). The baseline trajectory that the
fused `auv_slam_core` estimate must beat in `evo` ATE/RPE.

- `integrator.py` — pure, ROS-free `DeadReckoner` (unit-tested in `test/`).
- `dead_reckoning` node — integrates `/dvl` body velocity rotated by `/imu/data`
  orientation; publishes `/dead_reckoning/odom`. **No TF broadcast** (never competes
  with the EKF's `odom -> base_link`). Gates DVL drop-outs.
