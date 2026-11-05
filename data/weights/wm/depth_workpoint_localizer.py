"""Depth-point workpoint selection for the six fixed cup routes."""
from math import hypot, isfinite

WORKPOINTS_XY_M = ((0.176, -0.096), (0.064, -0.200), (0.232, 0.040))
CUP_Z_RANGE_M = (0.010, 0.110)
TARGET_Z_RANGE_M = (-0.003, 0.012)
SITE_RADIUS_M = 0.065
MIN_POINTS = 8
MIN_WINNER_RATIO = 2.0


def _select(points, z_range):
    counts = [0] * len(WORKPOINTS_XY_M)
    for point in points:
        if len(point) < 3:
            raise ValueError("depth points must contain base-frame x, y, z")
        x, y, z = map(float, point[:3])
        if not all(map(isfinite, (x, y, z))):
            continue
        if not z_range[0] <= z <= z_range[1]:
            continue
        for i, (sx, sy) in enumerate(WORKPOINTS_XY_M):
            if hypot(x - sx, y - sy) <= SITE_RADIUS_M:
                counts[i] += 1
    order = sorted(range(len(counts)), key=counts.__getitem__, reverse=True)
    best, second = counts[order[0]], counts[order[1]]
    if best < MIN_POINTS or best < MIN_WINNER_RATIO * max(1, second):
        return {"accepted": False, "counts": counts, "confidence": 0.0}
    return {"accepted": True, "site_index": order[0], "counts": counts,
            "confidence": best / (best + second)}


def localize_depth_workpoints(cup_cloud_base_m, target_cloud_base_m):
    """Select one calibrated source and target site from RGB-D point clouds.

    target_cloud_base_m must already be segmented to the green target.
    shortcut: snaps to the three calibrated sites; add sites or continuous fitting if workpoints move.
    """
    cup = _select(cup_cloud_base_m, CUP_Z_RANGE_M)
    target = _select(target_cloud_base_m, TARGET_Z_RANGE_M)
    if not cup["accepted"] or not target["accepted"]:
        return {"accepted": False, "reason": "ambiguous_workpoint",
                "source": cup, "target": target}
    sx, sy = WORKPOINTS_XY_M[cup["site_index"]]
    tx, ty = WORKPOINTS_XY_M[target["site_index"]]
    return {"accepted": True,
            "source_xyz_m": [sx, sy, 0.041], "source_confidence": cup["confidence"],
            "target_xyz_m": [tx, ty, 0.002], "target_confidence": target["confidence"],
            "source": cup, "target": target}


def _self_check():
    for i, site in enumerate(WORKPOINTS_XY_M):
        cup = [(site[0] + (0.001 if j % 2 else -0.001), site[1], 0.02 + 0.005 * (j % 3))
               for j in range(16)]
        target = [(site[0] + (0.001 if j % 2 else -0.001), site[1], 0.002)
                  for j in range(16)]
        result = localize_depth_workpoints(cup, target)
        assert result["accepted"]
        assert result["source"]["site_index"] == i
        assert result["target"]["site_index"] == i
    assert not localize_depth_workpoints([], []).get("accepted")


if __name__ == "__main__":
    _self_check()
    print("depth workpoint localizer self-check passed")
