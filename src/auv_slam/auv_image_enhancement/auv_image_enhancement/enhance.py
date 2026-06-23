"""Underwater image restoration primitives (spec §10.2, §15).

Pure OpenCV/numpy functions (no ROS) so they are unit-testable and reusable. Raw
underwater frames have a blue/green color cast, haze, and low contrast that wreck ORB
feature matching, so enhancement is part of the pipeline, not optional.
"""
import cv2
import numpy as np


def gray_world_white_balance(bgr: np.ndarray) -> np.ndarray:
    """Classic gray-world assumption: scale channels to a common mean."""
    result = bgr.astype(np.float32)
    means = result.reshape(-1, 3).mean(axis=0)
    gray = float(means.mean())
    for c in range(3):
        if means[c] > 1e-6:
            result[..., c] *= gray / means[c]
    return np.clip(result, 0, 255).astype(np.uint8)


def clahe_lab(bgr: np.ndarray, clip_limit=2.0, tile=8) -> np.ndarray:
    """CLAHE on the L channel in LAB space (contrast without color shift)."""
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile, tile))
    l = clahe.apply(l)
    return cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)


def udcp_dehaze(bgr: np.ndarray, omega=0.9, t_min=0.1, window=15) -> np.ndarray:
    """Underwater Dark Channel Prior dehazing (simplified).

    Uses the G,B channels (R attenuates fastest underwater) for the dark channel.
    """
    img = bgr.astype(np.float32) / 255.0
    gb = img[..., :2]  # B, G channels in BGR order
    dark = cv2.erode(gb.min(axis=2),
                     cv2.getStructuringElement(cv2.MORPH_RECT, (window, window)))
    flat = dark.ravel()
    n = max(1, int(flat.size * 0.001))
    idx = np.argpartition(flat, -n)[-n:]
    atmospheric = np.array([img[..., c].ravel()[idx].max() for c in range(3)])
    atmospheric = np.maximum(atmospheric, 1e-3)
    trans = 1.0 - omega * (gb / atmospheric[:2]).min(axis=2)
    trans = np.clip(trans, t_min, 1.0)[..., None]
    out = (img - atmospheric) / trans + atmospheric
    return np.clip(out * 255.0, 0, 255).astype(np.uint8)


def enhance(bgr: np.ndarray, method="clahe_wb") -> np.ndarray:
    """Dispatch by method: 'none' | 'wb' | 'clahe' | 'clahe_wb' | 'full'."""
    if bgr is None or bgr.size == 0:
        return bgr
    if method == "none":
        return bgr
    if method == "wb":
        return gray_world_white_balance(bgr)
    if method == "clahe":
        return clahe_lab(bgr)
    if method == "clahe_wb":
        return clahe_lab(gray_world_white_balance(bgr))
    if method == "full":
        return clahe_lab(gray_world_white_balance(udcp_dehaze(bgr)))
    raise ValueError(f"unknown enhancement method: {method}")
