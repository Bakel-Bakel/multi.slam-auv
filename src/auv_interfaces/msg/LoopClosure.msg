# A relative-pose constraint between two keyframes, emitted by any front-end
# (visual / acoustic / bathymetric) into the factor graph (auv_slam_core).
# The graph associates keyframes to these timestamps by nearest match.
std_msgs/Header header        # stamp = "to" keyframe time; frame_id = "map"

builtin_interfaces/Time from_stamp     # earlier keyframe time
builtin_interfaces/Time to_stamp       # later keyframe time

geometry_msgs/Pose relative_pose       # T_from_to (pose of "to" in "from" frame)
float64[36] covariance                 # row-major 6x6 (x,y,z,roll,pitch,yaw)

string  modality              # "visual" | "acoustic" | "mbes" | "msis" | "sss"
float64 confidence            # 0..1; loop closures use a robust kernel regardless
bool    is_sequential         # true = odometry-like adjacent constraint, false = loop
