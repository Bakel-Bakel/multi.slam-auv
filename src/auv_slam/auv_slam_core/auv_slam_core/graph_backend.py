"""GTSAM/iSAM2 pose-graph backend (spec §10.6) — ROS-free so it is unit-testable.

Owns the single global pose graph: keyframes are Pose3 variables; constraints are
BetweenFactor (odometry + loop closures), GPS-style position priors (USBL), and partial
priors (depth). Loop closures use a robust (Huber) kernel so false acoustic/visual matches
do not break the estimate. The ROS node (slam_core_node.py) only feeds this class.
"""
import numpy as np
import gtsam
from gtsam import symbol_shorthand

X = symbol_shorthand.X


def pose3(position, quat_xyzw):
    qx, qy, qz, qw = quat_xyzw
    return gtsam.Pose3(gtsam.Rot3.Quaternion(qw, qx, qy, qz),
                       gtsam.Point3(float(position[0]), float(position[1]),
                                    float(position[2])))


def matrix_pose3(T):
    return gtsam.Pose3(T)


class PoseGraphBackend:
    def __init__(self, prior_sigma=0.1, robust_k=1.345, relinearize_threshold=0.1):
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(relinearize_threshold)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)
        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()

        self.prior_sigma = prior_sigma
        self.robust_k = robust_k
        self.next_index = 0
        self.estimates = {}           # index -> gtsam.Pose3 (latest optimized)
        self.num_loops = 0
        self.num_factors = 0

    # ---- keyframe management ----
    def add_first_keyframe(self, pose: gtsam.Pose3):
        idx = self.next_index
        noise = gtsam.noiseModel.Diagonal.Sigmas(
            np.array([self.prior_sigma] * 6, dtype=float))
        self.graph.add(gtsam.PriorFactorPose3(X(idx), pose, noise))
        self.values.insert(X(idx), pose)
        self.estimates[idx] = pose
        self.next_index += 1
        self.num_factors += 1
        return idx

    def add_odometry_keyframe(self, prev_idx, relative_pose: gtsam.Pose3, sigmas):
        """Add a new keyframe linked to prev_idx by a BetweenFactor."""
        idx = self.next_index
        noise = gtsam.noiseModel.Diagonal.Sigmas(np.asarray(sigmas, dtype=float))
        self.graph.add(gtsam.BetweenFactorPose3(
            X(prev_idx), X(idx), relative_pose, noise))
        init = self.estimates[prev_idx].compose(relative_pose)
        self.values.insert(X(idx), init)
        self.estimates[idx] = init
        self.next_index += 1
        self.num_factors += 1
        return idx

    # ---- constraints ----
    def add_loop_closure(self, idx_a, idx_b, relative_pose: gtsam.Pose3, sigmas,
                         robust=True):
        base = gtsam.noiseModel.Diagonal.Sigmas(np.asarray(sigmas, dtype=float))
        noise = base
        if robust:
            noise = gtsam.noiseModel.Robust.Create(
                gtsam.noiseModel.mEstimator.Huber.Create(self.robust_k), base)
        self.graph.add(gtsam.BetweenFactorPose3(
            X(idx_a), X(idx_b), relative_pose, noise))
        self.num_loops += 1
        self.num_factors += 1

    def add_position_prior(self, idx, position, sigma_xyz):
        """USBL-style absolute position prior on a keyframe."""
        noise = gtsam.noiseModel.Isotropic.Sigma(3, float(sigma_xyz))
        self.graph.add(gtsam.GPSFactor(
            X(idx), gtsam.Point3(float(position[0]), float(position[1]),
                                 float(position[2])), noise))
        self.num_factors += 1

    def add_depth_prior(self, idx, z, sigma_z):
        """Partial prior constraining only the z coordinate (depth, z = -d)."""
        # PriorFactorPose3 with huge sigma on everything except z.
        big = 1e6
        sigmas = np.array([big, big, big, big, big, sigma_z], dtype=float)
        # order in gtsam Pose3 tangent: [rx,ry,rz, x,y,z]; z is index 5.
        noise = gtsam.noiseModel.Diagonal.Sigmas(sigmas)
        cur = self.estimates[idx]
        target = gtsam.Pose3(cur.rotation(),
                             gtsam.Point3(cur.x(), cur.y(), float(z)))
        self.graph.add(gtsam.PriorFactorPose3(X(idx), target, noise))
        self.num_factors += 1

    # ---- optimize ----
    def update(self, extra_iterations=0):
        self.isam.update(self.graph, self.values)
        for _ in range(extra_iterations):
            self.isam.update()
        result = self.isam.calculateEstimate()
        for idx in list(self.estimates.keys()):
            self.estimates[idx] = result.atPose3(X(idx))
        # graph/values are consumed by iSAM2; reset the incremental batch.
        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()
        return self.estimates

    def latest_index(self):
        return self.next_index - 1
