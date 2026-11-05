"""Assemble the original feature layout from measured/estimated components.

    No fake FK, camera registration or visual residuals are silently supplied.
    The caller obtains these from its sensor pipeline. No simulator truth is
    required or accepted as an extra inference input.
"""
import math
from .actor import vector, validate_observation
from .servo import ServoCalibration


def make_observation(position_rad, velocity_rad_s, previous_action,
                     visual_residual_m, tip_confidence, jitter_xyz_m,
                     mean_tip_position_base_m, tracking_age_s, elapsed_s,
                     jitter_per_tip_m, current_midi, next_midi,
                     target_position_base_m, target_confidence,
                     sequence_progress, note_elapsed_s, pointcloud, goal_cloud,
                     calibration=None):
    cal = calibration or ServoCalibration()
    position = vector(position_rad, 14, 'position_rad', cast=False)
    velocity = vector(velocity_rad_s, 14, 'velocity_rad_s', cast=False)
    previous = vector(previous_action, 14, 'previous_action', cast=False)
    if not isinstance(visual_residual_m, (list, tuple)) or len(visual_residual_m) != 5:
        raise ValueError('visual_residual_m must be5x3')
    residual = [x for row in visual_residual_m for x in vector(row, 3, 'visual_residual_m', cast=False)]
    quality = vector(tip_confidence, 5, 'tip_confidence', cast=False)
    jitter = vector(jitter_xyz_m, 3, 'jitter_xyz_m', cast=False)
    tips = vector(mean_tip_position_base_m, 3, 'mean_tip_position_base_m', cast=False)
    jitter_tip = vector(jitter_per_tip_m, 5, 'jitter_per_tip_m', cast=False)
    scalars = (tracking_age_s, elapsed_s, target_confidence, sequence_progress, note_elapsed_s)
    if not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in scalars):
        raise ValueError('Feature scalars must be finite numeric values')
    if tracking_age_s < 0 or elapsed_s < 0 or note_elapsed_s < 0:
        raise ValueError('Feature ages and times must be nonnegative')
    if isinstance(current_midi, bool) or not isinstance(current_midi, int) or not 48 <= current_midi <= 72:
        raise ValueError('current_midi must be48..72')
    if next_midi is not None and (isinstance(next_midi, bool) or not isinstance(next_midi, int) or not 48 <= next_midi <= 72):
        raise ValueError('next_midi must be48..72 orNone')
    proprio = [2 * (q - cal.lo[i]) / (cal.hi[i] - cal.lo[i]) - 1 for i, q in enumerate(position)]
    proprio += [v / 10 for v in velocity] + previous + [x * 20 for x in residual] + quality
    proprio += [x * 20 for x in jitter] + tips
    proprio += [min(1, tracking_age_s / .3), sum(quality) / 5, min(1, elapsed_s / 100)]
    proprio += [x * 20 for x in jitter_tip]
    proprio = [min(10, max(-10, x)) for x in proprio]
    goal = [0.0] * 56
    goal[current_midi - 48] = 1.0
    if next_midi is not None:
        goal[25 + next_midi - 48] = 1.0
    goal[50:53] = vector(target_position_base_m, 3, 'target_position_base_m', cast=False)
    goal[53] = target_confidence
    goal[54] = sequence_progress
    timeout = cal.config['training']['startup_timeout_s'] if sequence_progress <= 0 else cal.config['training']['note_timeout_s']
    goal[55] = min(1, note_elapsed_s / timeout)
    return validate_observation({'proprio': proprio, 'pointcloud': pointcloud, 'goal': goal, 'goal_cloud': goal_cloud})
