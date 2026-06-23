"""Mapping outputs anchored to the optimized trajectory (spec §10.7, M10).

Accumulates MBES submap clouds (already in the map frame) into a global seafloor cloud,
voxel-downsamples it, derives a 2.5D bathymetry grid (gridded seafloor elevation), and
serves SaveMap to write cloud/bathymetry to disk. RTAB-Map provides the dense visual
reconstruction in parallel; both are re-anchored to auv_slam_core's optimized trajectory.

Inputs : /mbes/submap_cloud (PointCloud2, map)
Outputs: /map/cloud (PointCloud2), /map/bathymetry (OccupancyGrid), SaveMap service
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

from auv_interfaces.srv import SaveMap


def voxel_downsample(points, voxel):
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return points[idx]


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping")
        self.declare_parameter("voxel_size", 0.2)
        self.declare_parameter("grid_resolution", 0.5)      # m/cell for bathymetry
        self.declare_parameter("publish_period", 2.0)
        self.voxel = float(self.get_parameter("voxel_size").value)
        self.grid_res = float(self.get_parameter("grid_resolution").value)

        self._cloud = np.empty((0, 3))
        self.create_subscription(PointCloud2, "/mbes/submap_cloud",
                                 self._on_cloud, qos_profile_sensor_data)
        self.pub_cloud = self.create_publisher(PointCloud2, "/map/cloud", 1)
        self.pub_grid = self.create_publisher(OccupancyGrid, "/map/bathymetry", 1)
        self.srv = self.create_service(SaveMap, "/mapping/save_map", self._save_map)
        self.create_timer(
            float(self.get_parameter("publish_period").value), self._publish)
        self.get_logger().info("mapping up -> /map/cloud, /map/bathymetry")

    def _on_cloud(self, msg: PointCloud2):
        try:
            pts = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.array([[p[0], p[1], p[2]] for p in pts], dtype=float)
        except Exception:  # noqa: BLE001
            return
        if len(arr):
            self._cloud = voxel_downsample(
                np.vstack([self._cloud, arr]), self.voxel)

    def _publish(self):
        if len(self._cloud) == 0:
            return
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id="map")
        fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
        self.pub_cloud.publish(
            point_cloud2.create_cloud(header, fields, self._cloud.astype(np.float32)))
        self.pub_grid.publish(self._bathymetry(stamp))

    def _bathymetry(self, stamp):
        pts = self._cloud
        mins = pts[:, :2].min(axis=0)
        maxs = pts[:, :2].max(axis=0)
        size = np.maximum(((maxs - mins) / self.grid_res).astype(int) + 1, 1)
        w, h = int(size[0]), int(size[1])
        # max-z (shallowest return) per cell -> 2.5D elevation
        elev = np.full((h, w), np.nan)
        ij = ((pts[:, :2] - mins) / self.grid_res).astype(int)
        for (i, j), z in zip(ij, pts[:, 2]):
            if np.isnan(elev[j, i]) or z > elev[j, i]:
                elev[j, i] = z
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = "map"
        grid.info.resolution = self.grid_res
        grid.info.width = w
        grid.info.height = h
        grid.info.origin.position.x = float(mins[0])
        grid.info.origin.position.y = float(mins[1])
        grid.info.origin.orientation.w = 1.0
        valid = ~np.isnan(elev)
        data = np.full((h, w), -1, dtype=np.int8)
        if valid.any():
            lo, hi = np.nanmin(elev), np.nanmax(elev)
            rng = max(hi - lo, 1e-6)
            norm = ((elev - lo) / rng * 100.0)
            data[valid] = norm[valid].astype(np.int8)
        grid.data = data.ravel(order="C").tolist()
        return grid

    def _save_map(self, request, response):
        path = request.path or "/tmp/auv_map"
        fmt = (request.format or "pcd").lower()
        try:
            if fmt in ("pcd", "all"):
                self._write_pcd(path + ".pcd")
            if fmt in ("ply", "all"):
                self._write_ply(path + ".ply")
            response.success = True
            response.message = f"wrote {len(self._cloud)} points to {path}.{fmt}"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _write_pcd(self, path):
        n = len(self._cloud)
        with open(path, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\n")
            f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
            f.write(f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {n}\nDATA ascii\n")
            for p in self._cloud:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    def _write_ply(self, path):
        n = len(self._cloud)
        with open(path, "w") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {n}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("end_header\n")
            for p in self._cloud:
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
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
