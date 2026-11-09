"""1280x720 直接标定的校正与视差转深度（无缩放，与板端 PS 服务同源）。

与 640 版 depth_geometry.py 的差别：不做 640->1280 换算，map/q 直接取自
board_hd_20261108 板端直接标定（per_eye 1280x720）。无效像素保持无效。
"""
from pathlib import Path
import cv2
import numpy as np


def direct_geometry(calibration):
    """读板端直接标定：map（(720,1280) float32）、q_matrix、单位。

    返回 maps（convertMaps CV_16SC2）、bounds（PL valid_mask 同语义）、q、米系数。
    """
    c = np.load(calibration, allow_pickle=False)
    w, h = tuple(int(v) for v in c['image_size'])
    if (w, h) != (1280, 720):
        raise ValueError('Only the direct 1280x720 calibration is supported')
    if str(np.asarray(c['length_unit']).item()) != 'mm':
        raise ValueError('Calibration does not explicitly declare millimetre units')
    spacing = np.asarray(c['grid_spacing_mm'], np.float64)
    if spacing.shape != (2,) or not np.isfinite(spacing).all() or np.any(spacing <= 0):
        raise ValueError('Missing measured rectangular-grid spacing')
    metres_per_unit = .001
    q = c['q_matrix']
    maps, bounds = [], []
    for s in ('l', 'r'):
        x, y = c['map_' + s + 'x'], c['map_' + s + 'y']
        maps.append(cv2.convertMaps(x, y, cv2.CV_16SC2))
        mask = ((x >= 4) & (x < w - 4) & (y >= 4) & (y < h - 4)).astype(np.uint8) * 255
        bounds.append(cv2.erode(mask, np.ones((9, 9), np.uint8)))
    return maps, bounds, q, metres_per_unit


class DepthFilter1280:
    """与 640 版 DepthFilter 同语义：Q12.4 视差（uint16，/16）-> 米制深度。"""

    def __init__(self, shape, right_bounds, q, metres_per_unit):
        self.h, self.w = shape
        self.yy, self.xx = np.indices(shape, dtype=np.int32)
        self.right_bounds = right_bounds
        self.q = q
        self.metres_per_unit = metres_per_unit

    def filter(self, left, right):
        if left.shape != (self.h, self.w) or left.dtype != np.uint16:
            raise ValueError('Unexpected PL disparity format')
        xr = np.rint(self.xx - left.astype(np.float32) / 16).astype(np.int32)
        inside = (xr >= 0) & (xr < self.w)
        xr = np.clip(xr, 0, self.w - 1)
        valid = (left > 0) & (left < 63 * 16) & inside & (self.right_bounds[self.yy, xr] > 0)
        if right is not None:
            if right.shape != left.shape:
                raise ValueError('Left/right disparity shape mismatch')
            sampled = right[self.yy, xr]
            valid &= (sampled > 0) & (np.abs(left.astype(np.int32) - sampled.astype(np.int32)) <= 16)
        result = left.copy()
        result[~valid] = 0
        cv2.filterSpeckles(result.view(np.int16), 0, 40, 16)
        return result

    def depth_m(self, disparity):
        d = disparity.astype(np.float32) / 16
        q = self.q
        numerator = q[2, 0] * self.xx + q[2, 1] * self.yy + q[2, 2] * d + q[2, 3]
        denominator = q[3, 0] * self.xx + q[3, 1] * self.yy + q[3, 2] * d + q[3, 3]
        good = (disparity > 0) & np.isfinite(denominator) & (np.abs(denominator) > 1e-12)
        depth = np.full(disparity.shape, np.nan, np.float32)
        np.divide(numerator * self.metres_per_unit, denominator, out=depth, where=good)
        depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
        return depth
