# auv_state_estimation

Loosely-coupled state-estimation backbone (spec §10.1, M6) — always on.

- `dvl_to_twist` — converts `/dvl` (`auv_interfaces/DVL`) to `/dvl/twist`
  (`TwistWithCovarianceStamped`, frame `base_link`), **gating** measurements with no
  bottom lock so the EKF never integrates invalid velocity.
- `robot_localization` EKF (`config/ekf.yaml`) — fuses IMU (orientation + angular vel +
  linear accel), DVL twist (vx,vy,vz), and depth (z = −d). Publishes `/odom` and the
  `odom -> base_link` TF. `map -> odom` is owned by `auv_slam_core`, never here.

## Run (M6 acceptance)
```bash
ros2 launch auv_state_estimation estimation.launch.py
# drive a lawnmower mission, then:
ros2 launch auv_evaluation evaluate.launch.py   # evo ATE of /odom vs /ground_truth/pose
# force DVL dropout to confirm graceful degradation (no drift spikes):
ros2 param set /sim_dvl dropout true
```
Expect bounded, slowly-growing ATE (dead-reckoning-like), depth within a few cm, and no
drift spike when the DVL loses lock (gating holds).
