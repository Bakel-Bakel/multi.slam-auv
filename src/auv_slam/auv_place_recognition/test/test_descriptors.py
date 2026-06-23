"""Tests for place-recognition global descriptors."""
import numpy as np

from auv_place_recognition.descriptors import global_descriptor, cosine_similarity


def _img(seed):
    rng = np.random.default_rng(seed)
    return (rng.random((120, 160, 3)) * 255).astype(np.uint8)


def test_descriptor_length_and_norm():
    d = global_descriptor(_img(0), size=32)
    assert d.shape == (32 * 32,)
    assert abs(np.linalg.norm(d) - 1.0) < 1e-5


def test_same_image_high_similarity():
    img = _img(1)
    a = global_descriptor(img)
    b = global_descriptor(img.copy())
    assert cosine_similarity(a, b) > 0.99


def test_different_images_lower_similarity():
    a = global_descriptor(_img(2))
    b = global_descriptor(_img(99))
    assert cosine_similarity(a, b) < 0.9


def test_zero_vectors_similarity_zero():
    z = np.zeros(10)
    assert cosine_similarity(z, z) == 0.0
