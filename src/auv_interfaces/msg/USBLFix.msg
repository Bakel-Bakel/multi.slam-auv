# Ultra-Short BaseLine acoustic positioning fix.
# Relative measurement is expressed in the receiver frame (header.frame_id, usbl_link).
std_msgs/Header header

uint32 beacon_id             # transponder/beacon identifier

# Relative measurement (spherical) in the receiver frame.
float64 range                # [m]
float64 bearing              # [rad] azimuth, ENU convention (CCW from +x)
float64 elevation            # [rad] elevation above the sensor xy-plane

# Cartesian relative position in the receiver frame (if solved).
geometry_msgs/Point relative_position   # [m]
float64[9]          position_covariance # row-major 3x3 [m]^2

# Absolute fix, available when the surface beacon has GNSS.
bool    has_absolute_fix
float64 latitude             # [deg]
float64 longitude            # [deg]
float64 altitude             # [m] above ellipsoid; for AUV typically negative (depth)
