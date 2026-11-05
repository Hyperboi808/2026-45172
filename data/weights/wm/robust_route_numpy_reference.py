"""Route matching tolerant to a shared camera-to-base rigid-transform error.

This only selects among the checkpoint's known source/target pairs.  It does not
alter WM weights or synthesize a route for an unknown object pair.
"""
from __future__ import annotations

import numpy as np


def _minimal_rotation(source_vector: np.ndarray, target_vector: np.ndarray):
    """Return the minimum-angle rotation mapping source_vector onto target_vector."""
    u = source_vector / max(float(np.linalg.norm(source_vector)), 1e-12)
    v = target_vector / max(float(np.linalg.norm(target_vector)), 1e-12)
    cross = np.cross(u, v)
    sine = float(np.linalg.norm(cross))
    cosine = float(np.clip(np.dot(u, v), -1.0, 1.0))
    angle = float(np.arctan2(sine, cosine))
    if sine < 1e-10:
        if cosine >= 0:
            return np.eye(3), angle
        basis = np.eye(3)[int(np.argmin(np.abs(u)))]
        axis = np.cross(u, basis)
        axis /= max(float(np.linalg.norm(axis)), 1e-12)
        return 2.0 * np.outer(axis, axis) - np.eye(3), angle
    k = np.array([[0.0, -cross[2], cross[1]],
                  [cross[2], 0.0, -cross[0]],
                  [-cross[1], cross[0], 0.0]])
    return np.eye(3) + k + (k @ k) * ((1.0 - cosine) / (sine * sine)), angle


def select_route_with_extrinsic_tolerance(
    source_xyz_m,
    target_xyz_m,
    task_pairs_xy,
    *,
    source_height_m: float = 0.041,
    target_height_m: float = 0.0,
    translation_scale_m: float = 0.0205113,
    rotation_scale_rad: float = np.deg2rad(6.2873),
    pair_length_scale_m: float = 0.010,
    max_cost: float = 5.0,
    min_margin: float = 4.0,
):
    """Match a detected 3D cup/target pair to a known route under shared SE(3) error.

    For each known pair, find the minimum-angle rigid correction aligning the
    observed source-target vector to that candidate, then score the required
    correction translation/rotation and the pair-length residual.  The caller
    must still enforce object confidence and workspace/visibility checks.
    """
    source = np.asarray(source_xyz_m, dtype=np.float64)
    target = np.asarray(target_xyz_m, dtype=np.float64)
    pairs = np.asarray(task_pairs_xy, dtype=np.float64)
    if source.shape != (3,) or target.shape != (3,) or pairs.ndim != 2 or pairs.shape[1] != 4:
        raise ValueError("Expected source/target XYZ vectors and task_pairs shape (N,4)")
    if not np.isfinite(source).all() or not np.isfinite(target).all() or not np.isfinite(pairs).all():
        raise ValueError("Route inputs must be finite")
    if len(pairs) < 2:
        raise ValueError("At least two route candidates are required")
    src = np.column_stack((pairs[:, :2], np.full(len(pairs), source_height_m)))
    dst = np.column_stack((pairs[:, 2:], np.full(len(pairs), target_height_m)))
    candidate_vectors = dst - src
    observed_vector = target - source
    observed_length = float(np.linalg.norm(observed_vector))
    if observed_length < 1e-8:
        return {"route_index": None, "accepted": False, "reason": "degenerate_observed_pair"}

    costs = []
    details = []
    for i, (candidate_source, candidate_vector) in enumerate(zip(src, candidate_vectors)):
        rotation, angle = _minimal_rotation(observed_vector, candidate_vector)
        correction_translation = candidate_source - rotation @ source
        translation = float(np.linalg.norm(correction_translation))
        length_residual = abs(observed_length - float(np.linalg.norm(candidate_vector)))
        cost = ((angle / max(rotation_scale_rad, 1e-12)) ** 2
                + (translation / max(translation_scale_m, 1e-12)) ** 2
                + (length_residual / max(pair_length_scale_m, 1e-12)) ** 2)
        costs.append(cost)
        details.append({"route_index": i, "correction_angle_rad": angle,
                        "correction_translation_m": translation,
                        "pair_length_residual_m": length_residual})
    order = np.argsort(np.asarray(costs), kind="stable")
    best, second = int(order[0]), int(order[1])
    best_cost = float(costs[best])
    margin = float(costs[second] - best_cost)
    accepted = bool(best_cost <= max_cost and margin >= min_margin)
    return {"route_index": best if accepted else None,
            "candidate_route_index": best,
            "accepted": accepted,
            "reason": None if accepted else ("route_cost_too_high" if best_cost > max_cost else "route_ambiguous"),
            "best_cost": best_cost,
            "second_cost": float(costs[second]),
            "cost_margin": margin,
            **details[best]}
