"""auv_slam_core — the factor-graph integrator (spec §10.6, M9).

The single global pose graph. It ingests odometry (from the estimation backbone),
depth priors, USBL absolute priors, and loop closures from every front-end
(visual / sonar / place recognition) through one uniform path, runs incremental iSAM2
optimization, and publishes /slam/pose, /slam/trajectory, the map->odom correction, and
SlamStatus. Modalities are toggled by the `modalities` param. Robust kernels on loop
closures survive false matches. This node never consumes ground truth.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster

from auv_interfaces.msg import USBLFix, SlamStatus, LoopClosure
from auv_slam_core.graph_backend import PoseGraphBackend, pose3


def pose_msg_to_pose3(p):
    return pose3([p.position.x, p.position.y, p.position.z],
                 [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def cov_to_sigmas(cov, floor=1e-3):
    """ROS 6x6 [x,y,z,r,p,y] row-major -> gtsam sigmas [r,p,y,x,y,z]."""
    vx, vy, vz = cov[0], cov[7], cov[14]
    vr, vp, vyaw = cov[21], cov[28], cov[35]
    s = [math.sqrt(max(v, floor ** 2)) for v in (vr, vp, vyaw, vx, vy, vz)]
    return s


def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class SlamCore(Node):
    def __init__(self):
        super().__init__("slam_core")
        self.declare_parameter("keyframe_distance", 0.5)
        self.declare_parameter("keyframe_dt", 1.0)
        self.declare_parameter("optimize_period", 0.5)
        self.declare_parameter("modalities", ["visual", "mbes", "fls", "sss", "msis"])
        self.declare_parameter("use_usbl", True)
        self.declare_parameter("use_depth", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("stamp_match_tol", 1.0)  # s

        self.kf_dist = float(self.get_parameter("keyframe_distance").value)
        self.kf_dt = float(self.get_parameter("keyframe_dt").value)
        self.enabled = set(self.get_parameter("modalities").value)
        self.use_usbl = bool(self.get_parameter("use_usbl").value)
        self.use_depth = bool(self.get_parameter("use_depth").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.match_tol = float(self.get_parameter("stamp_match_tol").value)

        self.graph = PoseGraphBackend()
        self.keyframes = []          # list of (stamp_float, index, odom_pose3)
        self._last_odom_pose3 = None
        self._last_kf_odom_pose3 = None
        self._last_kf_t = None
        self._latest_depth_z = None
        self._dirty = False

        rel = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)
        tl = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Odometry, "/odom", self._on_odom, rel)
        self.create_subscription(LoopClosure, "/slam/loop_closures",
                                 self._on_loop, QoSProfile(depth=50))
        if self.use_usbl:
            self.create_subscription(USBLFix, "/usbl", self._on_usbl, rel)
        if self.use_depth:
            self.create_subscription(Odometry, "/depth/odometry",
                                     self._on_depth, rel)

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/slam/pose", rel)
        self.pub_traj = self.create_publisher(Path, "/slam/trajectory", tl)
        self.pub_status = self.create_publisher(SlamStatus, "/slam/status", rel)
        self.bc = TransformBroadcaster(self)

        self.create_timer(
            float(self.get_parameter("optimize_period").value), self._optimize)
        self.get_logger().info(
            f"slam_core up; modalities={sorted(self.enabled)} "
            f"usbl={self.use_usbl} depth={self.use_depth}")

    # ---------- callbacks ----------
    def _on_odom(self, msg: Odometry):
        cur = pose_msg_to_pose3(msg.pose.pose)
        self._last_odom_pose3 = cur
        t = stamp_to_float(msg.header.stamp)

        if not self.keyframes:
            idx = self.graph.add_first_keyframe(cur)
            self._register_kf(t, idx, cur)
            self._maybe_depth_prior(idx)
            return

        rel = self._last_kf_odom_pose3.between(cur)
        moved = np.linalg.norm(rel.translation())
        if moved < self.kf_dist and (t - self._last_kf_t) < self.kf_dt:
            return
        sigmas = cov_to_sigmas(list(msg.pose.covariance)) \
            if any(msg.pose.covariance) else [0.05] * 6
        idx = self.graph.add_odometry_keyframe(
            self.keyframes[-1][1], rel, sigmas)
        self._register_kf(t, idx, cur)
        self._maybe_depth_prior(idx)
        self._dirty = True

    def _register_kf(self, t, idx, odom_pose3):
        self.keyframes.append((t, idx, odom_pose3))
        self._last_kf_odom_pose3 = odom_pose3
        self._last_kf_t = t

    def _maybe_depth_prior(self, idx):
        if self.use_depth and self._latest_depth_z is not None:
            self.graph.add_depth_prior(idx, self._latest_depth_z, 0.05)

    def _on_depth(self, msg: Odometry):
        self._latest_depth_z = msg.pose.pose.position.z

    def _on_usbl(self, msg: USBLFix):
        idx = self._nearest_kf(stamp_to_float(msg.header.stamp))
        if idx is None:
            return
        pos = [msg.relative_position.x, msg.relative_position.y,
               msg.relative_position.z]
        sigma = math.sqrt(max(msg.position_covariance[0], 1e-2))
        self.graph.add_position_prior(idx, pos, sigma)
        self._dirty = True

    def _on_loop(self, msg: LoopClosure):
        if msg.modality not in self.enabled and not msg.is_sequential:
            return
        i_from = self._nearest_kf(stamp_to_float(msg.from_stamp))
        i_to = self._nearest_kf(stamp_to_float(msg.to_stamp))
        if i_from is None or i_to is None or i_from == i_to:
            return
        rel = pose_msg_to_pose3(msg.relative_pose)
        sigmas = cov_to_sigmas(list(msg.covariance)) \
            if any(msg.covariance) else [0.1] * 6
        self.graph.add_loop_closure(
            i_from, i_to, rel, sigmas, robust=not msg.is_sequential)
        self._dirty = True

    def _nearest_kf(self, t):
        if not self.keyframes:
            return None
        best_idx, best_dt = None, self.match_tol
        for (kt, idx, _) in self.keyframes:
            dt = abs(kt - t)
            if dt <= best_dt:
                best_idx, best_dt = idx, dt
        return best_idx

    # ---------- optimize + publish ----------
    def _optimize(self):
        if not self._dirty or not self.keyframes:
            return
        try:
            estimates = self.graph.update()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"iSAM2 update failed: {exc}")
            return
        self._dirty = False
        now = self.get_clock().now().to_msg()
        last_t, last_idx, last_odom = self.keyframes[-1]
        last_pose = estimates[last_idx]

        ps = PoseWithCovarianceStamped()
        ps.header.stamp = now
        ps.header.frame_id = "map"
        self._fill_pose(ps.pose.pose, last_pose)
        self.pub_pose.publish(ps)

        path = Path()
        path.header.stamp = now
        path.header.frame_id = "map"
        for (_, idx, _) in self.keyframes:
            sp = PoseStamped()
            sp.header.frame_id = "map"
            self._fill_pose(sp.pose, estimates[idx])
            path.poses.append(sp)
        self.pub_traj.publish(path)

        if self.publish_tf and self._last_odom_pose3 is not None:
            self._publish_map_odom(last_pose, last_odom, now)

        self._publish_status(now, last_pose, last_odom)

    @staticmethod
    def _fill_pose(pose_msg, p3):
        t = p3.translation()
        q = p3.rotation().toQuaternion()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = \
            float(t[0]), float(t[1]), float(t[2])
        pose_msg.orientation.w = q.w()
        pose_msg.orientation.x = q.x()
        pose_msg.orientation.y = q.y()
        pose_msg.orientation.z = q.z()

    def _publish_map_odom(self, map_base, odom_base, stamp):
        # map->odom = map_base * (odom_base)^-1
        map_odom = map_base.compose(odom_base.inverse())
        t = map_odom.translation()
        q = map_odom.rotation().toQuaternion()
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = float(t[0])
        tf.transform.translation.y = float(t[1])
        tf.transform.translation.z = float(t[2])
        tf.transform.rotation.w = q.w()
        tf.transform.rotation.x = q.x()
        tf.transform.rotation.y = q.y()
        tf.transform.rotation.z = q.z()
        self.bc.sendTransform(tf)

    def _publish_status(self, stamp, map_base, odom_base):
        st = SlamStatus()
        st.header.stamp = stamp
        st.active_modules = sorted(self.enabled)
        st.num_keyframes = len(self.keyframes)
        st.num_loop_closures = self.graph.num_loops
        st.num_factors = self.graph.num_factors
        st.num_variables = self.graph.next_index
        st.last_optimization_time = stamp_to_float(stamp)
        drift = map_base.compose(odom_base.inverse()).translation()
        st.estimated_drift = float(np.linalg.norm(drift))
        self.pub_status.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = SlamCore()
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
