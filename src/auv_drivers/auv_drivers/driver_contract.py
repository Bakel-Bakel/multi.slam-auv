"""Sim-to-real driver contract (spec §9.3) — v1 stubs/interfaces only.

These define the EXACT interface-contract topics a real driver must publish, so swapping
sim->real is "launch the driver instead of the sim adapter" with zero SLAM-code changes.
v1 contains no hardware; each stub warns and (optionally) is a place to drop a real driver.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class DriverSpec:
    """What a real driver must provide to satisfy the contract."""
    name: str
    topic: str
    msg_type: str
    frame_id: str
    notes: str


# The real-hardware contract (mirrors INTERFACE.md / spec §9).
DRIVER_CONTRACT = [
    DriverSpec("imu", "/imu/data", "sensor_msgs/Imu", "imu_link",
               "real AHRS/IMU; fill covariances; ENU/FLU at the boundary"),
    DriverSpec("pressure", "/pressure", "sensor_msgs/FluidPressure", "pressure_link",
               "depth via depth_from_pressure (z = -d)"),
    DriverSpec("dvl", "/dvl", "auv_interfaces/DVL", "dvl_link",
               "Water Linked / Nortek; tag bottom_locked; convert to base_link"),
    DriverSpec("usbl", "/usbl", "auv_interfaces/USBLFix", "usbl_link",
               "absolute-position prior; NED->ENU at the boundary"),
    DriverSpec("camera", "/cam/left/image_raw", "sensor_msgs/Image",
               "camera_left_optical_link", "stereo pair + camera_info"),
    DriverSpec("fls", "/sonar/fls/image", "marine_acoustic_msgs/ProjectedSonarImage",
               "fls_link", "FRD frame for multibeam-style devices"),
    DriverSpec("mbes", "/sonar/mbes/points", "sensor_msgs/PointCloud2", "mbes_link",
               "bathymetric returns"),
]
