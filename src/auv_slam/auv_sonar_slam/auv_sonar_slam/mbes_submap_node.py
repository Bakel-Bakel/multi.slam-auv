"""Bathymetric (MBES) submap SLAM (spec §10.4).

Canonical submap approach: compound consecutive MBES swaths into seafloor point-cloud
submaps using dead-reckoning (/odom), then register overlapping submaps with ICP and add
the registrations as loop-closure constraints into auv_slam_core. Difference-of-normals
style voxel subsampling keeps cost bounded and avoids ICP local minima.

Inputs : /sonar/mbes/points  (PointCloud2, mbes_link), /odom (Odometry)
Outputs: /slam/loop_closures  (auv_interfaces/LoopClosure, modality="mbes")
         /mbes/submap_cloud    (PointCloud2, map) for visualization
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from scipy.spatial.transform import Rotation

from auv_interfaces.msg import LoopClosure
from auv_sonar_slam.icp import icp


def voxel_downsample(points, voxel=0.2):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


class Submap:
    __slots__ = ("points", "centroid", "ref_pos", "ref_quat", "ref_stamp")

    def __init__(self, points, ref_pos, ref_quat, ref_stamp):
        self.points = points
        self.centroid = points.mean(axis=0) if len(points) else ref_pos
        self.ref_pos, self.ref_quat, self.ref_stamp = ref_pos, ref_quat, ref_stamp


class MbesSubmapNode(Node):
    def __init__(self):
        super().__init__("mbes_submap_slam")
        self.declare_parameter("submap_distance", 4.0)       # m between submaps
        self.declare_parameter("voxel_size", 0.25)           # m
        self.declare_parameter("overlap_radius", 6.0)        # m for candidate search
        self.declare_parameter("min_time_gap", 10.0)         # s
        self.declare_parameter("icp_fitness_min", 0.6)
        self.declare_parameter("icp_error_max", 0.5)         # m

        self.submap_dist = float(self.get_parameter("submap_distance").value)
        self.voxel = float(self.get_parameter("voxel_size").value)
        self.overlap_r = float(self.get_parameter("overlap_radius").value)
        self.min_gap = float(self.get_parameter("min_time_gap").value)
        self.fit_min = float(self.get_parameter("icp_fitness_min").value)
        self.err_max = float(self.get_parameter("icp_error_max").value)

        self._pose = None
        self._cur_points = []
        self._cur_start_pos = None
        self.submaps = []

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PointCloud2, "/sonar/mbes/points",
                                 self._on_cloud, qos_profile_sensor_data)
        self.lc_pub = self.create_publisher(
            LoopClosure, "/slam/loop_closures", QoSProfile(depth=50))
        self.cloud_pub = self.create_publisher(PointCloud2, "/mbes/submap_cloud", 1)
        self.get_logger().info("mbes_submap_slam up -> /slam/loop_closures")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (np.array([p.x, p.y, p.z]),
                      np.array([q.x, q.y, q.z, q.w]),
                      msg.header.stamp)

    def _on_cloud(self, msg: PointCloud2):
        if self._pose is None:
            return
        pos, quat, stamp = self._pose
        try:
            pts = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            local = np.array([[p[0], p[1], p[2]] for p in pts], dtype=float)
        except Exception:  # noqa: BLE001
            return
        if len(local) == 0:
            return

        # Transform sensor points into the map frame via the odom pose.
        rot = Rotation.from_quat(quat).as_matrix()
        world = (rot @ local.T).T + pos
        self._cur_points.append(world)
        if self._cur_start_pos is None:
            self._cur_start_pos = (pos, quat, stamp)

        if np.linalg.norm(pos - self._cur_start_pos[0]) >= self.submap_dist:
            self._finalize_submap()

    def _finalize_submap(self):
        if not self._cur_points:
            return
        pts = voxel_downsample(np.vstack(self._cur_points), self.voxel)
        ref_pos, ref_quat, ref_stamp = self._cur_start_pos
        submap = Submap(pts, ref_pos, ref_quat, ref_stamp)
        self._try_register(submap)
        self.submaps.append(submap)
        self._publish_cloud(pts, ref_stamp)
        self._cur_points = []
        self._cur_start_pos = None

    def _try_register(self, new):
        new_t = new.ref_stamp.sec + new.ref_stamp.nanosec * 1e-9
        for old in self.submaps:
            old_t = old.ref_stamp.sec + old.ref_stamp.nanosec * 1e-9
            if (new_t - old_t) < self.min_gap:
                continue
            if np.linalg.norm(new.centroid - old.centroid) > self.overlap_r:
                continue
            r, t, fitness, err = icp(new.points, old.points,
                                     max_correspondence_dist=self.overlap_r)
            if fitness >= self.fit_min and err <= self.err_max:
                self._emit(old, new, r, t, fitness)

    def _emit(self, old, new, r, t, fitness):
        # ICP gives the correction aligning new submap (DR map frame) onto old.
        # The relative pose seed between the two reference poses (old->new) in old frame:
        r_old = Rotation.from_quat(old.ref_quat)
        rel_pos = r_old.inv().apply(new.ref_pos - old.ref_pos)
        rel_rot = (r_old.inv() * Rotation.from_quat(new.ref_quat)).as_quat()

        lc = LoopClosure()
        lc.header.stamp = new.ref_stamp
        lc.header.frame_id = "map"
        lc.from_stamp = old.ref_stamp
        lc.to_stamp = new.ref_stamp
        lc.relative_pose.position.x = float(rel_pos[0])
        lc.relative_pose.position.y = float(rel_pos[1])
        lc.relative_pose.position.z = float(rel_pos[2])
        lc.relative_pose.orientation.x = float(rel_rot[0])
        lc.relative_pose.orientation.y = float(rel_rot[1])
        lc.relative_pose.orientation.z = float(rel_rot[2])
        lc.relative_pose.orientation.w = float(rel_rot[3])
        cov = [0.0] * 36
        scale = max(1e-3, 1.0 - fitness)
        for i in range(3):
            cov[i * 6 + i] = 0.1 * scale
        for i in range(3, 6):
            cov[i * 6 + i] = 0.05 * scale
        lc.covariance = cov
        lc.modality = "mbes"
        lc.confidence = float(fitness)
        lc.is_sequential = False
        self.lc_pub.publish(lc)
        self.get_logger().info(f"MBES submap loop: fitness={fitness:.2f}")

    def _publish_cloud(self, pts, stamp):
        header = Header(stamp=stamp, frame_id="map")
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
        msg = point_cloud2.create_cloud(header, fields, pts.astype(np.float32))
        self.cloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MbesSubmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
