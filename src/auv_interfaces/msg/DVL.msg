# Doppler Velocity Log measurement.
# No universal ROS DVL standard exists; this is the project's clean definition.
# Velocity is reported in the sensor frame named by header.frame_id (typically dvl_link).
# Consumers must transform to base_link (FLU) via the static extrinsic before fusing.
std_msgs/Header header

# Per-beam data (typically 4 beams in a Janus configuration).
float64[] beam_ranges        # [m]   range to bottom per beam
float64[] beam_velocities    # [m/s] along-beam (radial) velocity per beam
bool[]    beam_valid         #       per-beam validity flag

# Aggregated 3-DOF body velocity in the sensor frame.
geometry_msgs/Vector3 velocity        # [m/s]
float64[9]            velocity_covariance  # row-major 3x3 [m/s]^2

bool    bottom_locked        # true iff >=3 of 4 beams have bottom-track lock
float64 figure_of_merit      # [m/s] velocity uncertainty estimate (lower is better)
float64 altitude             # [m]   range to seafloor along boresight (NaN if unknown)
