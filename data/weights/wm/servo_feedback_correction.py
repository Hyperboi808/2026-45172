"""Low-gain arm-pose feedback trim in radians."""
from math import isfinite, pi

GAIN = 0.25
MAX_CORRECTION_RAD = 0.02
ID12_ID14_MAX_STEPS_PER_SECOND = 500
DURATION_MS = 500
ARM_STEP_SCALE = 651.9


def correct_arm_pose_target(target_rad, measured_rad, gain=GAIN,
                            max_correction_rad=MAX_CORRECTION_RAD):
    target = [float(x) for x in target_rad]
    measured = [float(x) for x in measured_rad]
    if len(target) != 14 or len(measured) != 14:
        raise ValueError("expected 14 planned and measured joint angles")
    if not all(map(isfinite, target + measured)):
        raise ValueError("joint angles must be finite")
    if not 0 <= gain <= 1 or not 0 < max_correction_rad <= MAX_CORRECTION_RAD:
        raise ValueError("feedback trim must be <= 0.02 rad")
    correction = [0.0] * 4
    for i in range(4):
        error = target[i] - measured[i]
        if i == 2:
            error = (error + pi) % (2*pi) - pi
        correction[i] = max(-max_correction_rad, min(max_correction_rad, gain * error))
        target[i] += correction[i]
    cap = ID12_ID14_MAX_STEPS_PER_SECOND * DURATION_MS / 1000 / ARM_STEP_SCALE
    for i in (1, 3):
        target[i] = max(measured[i] - cap, min(measured[i] + cap, target[i]))
    target[2] = max(-pi, min(3*pi, target[2]))
    return target, correction


def _self_check():
    target = [0.0, -3.0, 3.0, 0.8] + [0.0] * 10
    for measured in ([0.0, -3.1, 3.1, 0.9] + [0.0] * 10,
                     [0.0, -2.9, 2.9, 0.7] + [0.0] * 10):
        corrected, trim = correct_arm_pose_target(target, measured)
        assert all(abs(x) <= MAX_CORRECTION_RAD for x in trim)
        for i in (1, 3):
            assert abs(corrected[i] - measured[i]) <= 250 / ARM_STEP_SCALE + 1e-9
        assert corrected[4:] == target[4:]
    same, trim = correct_arm_pose_target(target, target)
    assert trim == [0.0] * 4 and same == target
    assert not correct_arm_pose_target([0.0]*14, [0.0]*14)[1][0]


if __name__ == "__main__":
    _self_check()
    print("radian servo-feedback correction self-check passed")
