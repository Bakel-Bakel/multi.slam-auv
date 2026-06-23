"""Tests for underwater image enhancement primitives."""
import numpy as np

from auv_image_enhancement.enhance import (
    enhance, gray_world_white_balance, clahe_lab, udcp_dehaze)


def _blue_cast_image():
    img = np.zeros((64, 64, 3), dtype=np.uint8)
    img[..., 0] = 200   # strong blue (BGR)
    img[..., 1] = 90
    img[..., 2] = 40    # weak red
    # add a little texture so CLAHE has something to work with
    img[16:48, 16:48] = [180, 110, 70]
    return img


def test_shapes_preserved():
    img = _blue_cast_image()
    for m in ("none", "wb", "clahe", "clahe_wb", "full"):
        out = enhance(img, m)
        assert out.shape == img.shape
        assert out.dtype == np.uint8


def test_white_balance_reduces_blue_dominance():
    img = _blue_cast_image()
    before = img.reshape(-1, 3).mean(axis=0)
    after = gray_world_white_balance(img).reshape(-1, 3).mean(axis=0)
    # channel means should be much closer together after gray-world WB
    assert after.std() < before.std()


def test_dehaze_and_clahe_run():
    img = _blue_cast_image()
    assert udcp_dehaze(img).shape == img.shape
    assert clahe_lab(img).shape == img.shape


def test_unknown_method_raises():
    import pytest
    with pytest.raises(ValueError):
        enhance(_blue_cast_image(), "bogus")
