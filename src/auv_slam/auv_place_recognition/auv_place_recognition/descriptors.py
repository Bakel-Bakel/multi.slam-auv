"""Lightweight global image descriptors + matching for place recognition.

Pure numpy/cv2 (unit-testable). A compact "tiny-image" / gist-style global descriptor is
enough for sim-grade revisit detection and is modality-agnostic (optical or sonar
waterfall imagery). Swap for DBoW2 / learned descriptors at the same seam (spec §10.5, §17).
"""
import cv2
import numpy as np


def global_descriptor(image: np.ndarray, size: int = 32) -> np.ndarray:
    """Normalized, contrast-stretched, flattened low-res grayscale descriptor."""
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(image, (size, size), interpolation=cv2.INTER_AREA).astype(np.float32)
    small -= small.mean()
    norm = np.linalg.norm(small)
    if norm > 1e-6:
        small /= norm
    return small.ravel()


def cosine_similarity(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return float(np.dot(a, b) / (na * nb))
