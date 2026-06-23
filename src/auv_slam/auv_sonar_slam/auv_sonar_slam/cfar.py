"""CFAR detection for imaging-sonar feature extraction (spec §10.3).

Cell-Averaging CFAR (CA-CFAR) with a guard band: a pixel is a detection if it exceeds the
local background mean (estimated from a training ring) times an adaptive threshold factor.
This is the front of the FLS/MSIS feature-SLAM pipeline (CFAR -> feature points -> ICP).
Pure numpy so it is unit-testable and validatable on Stonefish's high-fidelity sonar.
"""
import numpy as np


def ca_cfar(image, guard=2, train=6, threshold_factor=3.0, min_value=0.0):
    """Return a boolean detection mask the same shape as `image`.

    guard:  half-width of the guard band (excluded from background estimate)
    train:  half-width of the training band (used for background estimate)
    threshold_factor: detection if pixel > factor * local_background_mean
    """
    img = np.asarray(image, dtype=np.float32)
    if img.ndim != 2:
        raise ValueError("ca_cfar expects a 2D image")

    win = guard + train
    # Integral image for O(1) box sums.
    integ = np.pad(img, win, mode="reflect")
    sat = integ.cumsum(0).cumsum(1)
    sat = np.pad(sat, ((1, 0), (1, 0)), mode="constant")

    def box_sum(r0, c0, r1, c1):
        return (sat[r1, c1] - sat[r0, c1] - sat[r1, c0] + sat[r0, c0])

    h, w = img.shape
    rr, cc = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
    # offsets into the padded SAT coordinate frame
    base_r, base_c = rr, cc  # top-left of the (2*win+1) window in padded coords
    outer = box_sum(base_r, base_c, base_r + 2 * win + 1, base_c + 2 * win + 1)
    inner = box_sum(base_r + train, base_c + train,
                    base_r + 2 * win + 1 - train, base_c + 2 * win + 1 - train)
    n_outer = (2 * win + 1) ** 2
    n_inner = (2 * guard + 1) ** 2
    train_sum = outer - inner
    train_count = n_outer - n_inner
    background = train_sum / max(1, train_count)

    detections = (img > threshold_factor * background) & (img > min_value)
    return detections


def detections_to_points(mask, range_per_row=0.05, angle_per_col=None,
                         fov_rad=2.094, range_offset=0.0):
    """Convert a fan-shaped sonar detection mask (range x bearing) to XYZ points.

    Rows index range bins, columns index bearing bins across `fov_rad`.
    Returns an (N,3) array in the sensor frame (z=0; sonar is a 2D fan).
    """
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return np.empty((0, 3))
    n_cols = mask.shape[1]
    if angle_per_col is None:
        angle_per_col = fov_rad / max(1, n_cols - 1)
    rng = range_offset + rows * range_per_row
    bearing = (cols - (n_cols - 1) / 2.0) * angle_per_col
    x = rng * np.cos(bearing)
    y = rng * np.sin(bearing)
    z = np.zeros_like(x)
    return np.column_stack([x, y, z])
