# Health/telemetry published by auv_slam_core.
std_msgs/Header header

string[] active_modules          # e.g. ["state_estimation","visual","mbes"]
uint32   num_keyframes
uint32   num_loop_closures
uint32   num_factors
uint32   num_variables
float64  last_optimization_time  # [s] sim/wall stamp of last iSAM2 update
float64  optimization_duration   # [s] wall time of last update
float64  estimated_drift         # [m] current odom drift estimate (map->odom translation)
