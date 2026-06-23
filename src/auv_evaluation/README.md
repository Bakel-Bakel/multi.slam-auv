# auv_evaluation

evo-based ATE/RPE + rosbag regression (spec §12). Ground truth is used for evaluation
ONLY and is never fed back into SLAM.

- `ate.py` — pure numpy ATE/RPE with Umeyama (SE3/Sim3) alignment. Unit-tested; lets CI
  assert accuracy without the evo binary.
- `trajectory_logger` node — logs estimate (`/slam/pose`/`/odom`) + `/ground_truth/pose`
  to TUM files.
- `run_evo` — wraps `evo_ape`/`evo_rpe` (falls back to `ate.py`), exits nonzero if a
  threshold is exceeded.
- `scripts/regression_test.sh` — deterministic rosbag regression for CI.

```bash
ros2 launch auv_evaluation evaluate.launch.py estimate_topic:=/slam/pose
# after a mission:
ros2 run auv_evaluation run_evo /tmp/auv_eval/estimate.tum /tmp/auv_eval/ground_truth.tum --ate-threshold 0.5
```
