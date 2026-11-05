"""Calibrated14-servo target conversion. Returns records; never opens hardware."""
import json
import math
from pathlib import Path
from .actor import vector


def clamp(value, lo, hi):
    return min(hi, max(lo, value))


class ServoCalibration:
    def __init__(self, config=None):
        if config is None:
            config = json.loads((Path(__file__).resolve().parent.parent / 'servo_config.json').read_text(encoding='utf-8'))
        self.config = config
        self.servos = sorted(config['servos'], key=lambda item: item['id'])
        if [s['id'] for s in self.servos] != list(range(1, 15)):
            raise ValueError('Calibration needs physical servo IDs1..14')
        self.lo = [s['q_min'] for s in self.servos]
        self.hi = [s['q_max'] for s in self.servos]
        self.home = [s['q_initial'] for s in self.servos]
        self.periodic = []
        for i, servo in enumerate(self.servos):
            if servo.get('angle_branch') == 'around_initial':
                if not servo.get('wrap'):
                    raise ValueError('Periodic branch needs a wrapping encoder')
                self.lo[i], self.hi[i] = self.home[i] - math.pi, self.home[i] + math.pi
                self.periodic.append(i)

    def normalize(self, joint):
        values = vector(joint, 14, 'joint_radians', cast=False)
        for i in self.periodic:
            if values[i] < self.lo[i] or values[i] > self.hi[i]:
                values[i] = self.home[i] + (values[i] - self.home[i] + math.pi) % (2 * math.pi) - math.pi
        return values

    def to_joint(self, ticks, reference=None):
        ticks = vector(ticks, 14, 'feedback_ticks', cast=False)
        ref = self.home if reference is None else vector(reference, 14, 'reference_radians', cast=False)
        result = []
        for i, (servo, tick) in enumerate(zip(self.servos, ticks)):
            if servo.get('steps_per_radian'):
                q = servo['q_initial'] + (tick - servo['initial']) / servo['steps_per_radian']
                if servo.get('wrap'):
                    candidates = [q + k * servo['wrap'] / servo['steps_per_radian'] for k in range(-2, 3)]
                    candidates = [x for x in candidates if self.lo[i] - .002 <= x <= self.hi[i] + .002]
                    if candidates:
                        q = min(candidates, key=lambda x: abs(x - ref[i]))
            else:
                q = servo['q_min'] + (tick - servo['minimum']) / (servo['maximum'] - servo['minimum']) * (servo['q_max'] - servo['q_min'])
            result.append(clamp(q, self.lo[i], self.hi[i]))
        return result

    def to_ticks(self, joint):
        values = self.normalize(joint)
        result = []
        for i, (servo, value) in enumerate(zip(self.servos, values)):
            value = clamp(value, self.lo[i], self.hi[i])
            if servo.get('steps_per_radian'):
                tick = servo['initial'] + (value - servo['q_initial']) * servo['steps_per_radian']
                if servo.get('wrap'):
                    tick %= servo['wrap']
            else:
                tick = servo['minimum'] + (value - servo['q_min']) / (servo['q_max'] - servo['q_min']) * (servo['maximum'] - servo['minimum'])
            result.append(int(clamp(round(tick), servo['minimum'], servo['maximum'])))
        return result

    def limit(self, target, previous, dt):
        if isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        previous = vector(previous, 14, 'previous_radians', cast=False)
        requested = self.normalize(target)
        values = [clamp(q, self.lo[i], self.hi[i]) for i, q in enumerate(requested)]
        for i, servo in enumerate(self.servos):
            if servo.get('max_speed_steps_s'):
                delta = servo['max_speed_steps_s'] * dt / servo['steps_per_radian']
                values[i] = clamp(values[i], previous[i] - delta, previous[i] + delta)
        return values


class ServoCommandLimiter:
    """The original normalized absolute-target decoder and parking schedule.

    tracking_confidence must come from the sensor pipeline. This class does
    not infer visibility, physical contact, collision safety or key success.
    Use actual elapsed dt; a gap over100ms inhibits output until reset.
    """
    def __init__(self, calibration=None, initial_joint=None):
        self.cal = calibration or ServoCalibration()
        self.training = self.cal.config['training']
        if self.training['servo_action_mode'] != 'position_targets':
            raise ValueError('This package uses original normalized absolute targets')
        fingers = self.training['playing_fingers']
        ids = {11, 12, 13, 14}
        for name in fingers:
            i = ('little', 'ring', 'middle', 'index', 'thumb').index(name)
            ids.update((2 * i + 1, 2 * i + 2))
        self.playing_mask = [i in ids for i in range(1, 15)]
        self.correction = self.training.get('folded_finger_correction_rad', 0.0)
        self.hold = self.cal.config['initial_reference']['nonplaying_hold_joint_rad']
        self.reset(initial_joint)

    def reset(self, initial_joint=None):
        if initial_joint is None:
            initial_joint = self.cal.config['initial_reference']['command_joint_rad']
        initial = vector(initial_joint, 14, 'initial_joint', cast=False)
        if any(not self.cal.lo[i] <= q <= self.cal.hi[i] for i, q in enumerate(initial)):
            raise ValueError('Initial joint is outside the calibrated branch')
        self.command = initial[:]
        self.initial_command = initial[:]
        self.previous_action = [0.0] * 14
        self.elapsed = 0.0
        self.needs_reset = False

    def reset_from_ticks(self, feedback_ticks):
        self.reset(self.cal.to_joint(feedback_ticks, reference=self.command))

    def step(self, action, tracking_confidence, dt=.01):
        action = vector(action, 14, 'action', cast=False)
        if not isinstance(dt, (int, float)) or isinstance(dt, bool) or not math.isfinite(dt) or dt <= 0:
            raise ValueError('dt must be finite and positive')
        if not isinstance(tracking_confidence, (int, float)) or isinstance(tracking_confidence, bool) or not math.isfinite(tracking_confidence):
            raise ValueError('tracking_confidence must be finite')
        if dt > .1:
            self.needs_reset = True
        reason = 'control_gap_requires_feedback_reset' if self.needs_reset else None
        if reason is None and tracking_confidence < self.training['tracking_min_confidence']:
            reason = 'visual_registration_lost'
        if reason is not None:
            self.elapsed += dt
            return {'valid': False, 'reason': reason, 'commands': [], 'joint_target_radians': self.command[:], 'previous_action': self.previous_action[:]}
        action = [clamp(x, -1.0, 1.0) for x in action]
        requested = []
        parking_fraction = min(1.0, (self.elapsed + dt) / 3.6)
        folded = self.correction if parking_fraction >= 1.0 else 0.0
        for i in range(14):
            if not self.correction and not self.playing_mask[i]:
                action[i] = 0.0
            width = self.cal.hi[i] - self.cal.lo[i]
            target = self.cal.lo[i] + (action[i] + 1) * .5 * width
            delta = width * self.training['action_delta_fraction']
            q = self.command[i] + clamp(target - self.command[i], -delta, delta)
            if not self.playing_mask[i]:
                parking = self.initial_command[i] + (self.hold[i] - self.initial_command[i]) * parking_fraction
                q = clamp(q, parking - folded, parking + folded)
            requested.append(q)
        self.command = self.cal.limit(requested, self.command, dt)
        self.previous_action = action[:]
        self.elapsed += dt
        ticks = self.cal.to_ticks(self.command)
        records = []
        for i, servo in enumerate(self.cal.servos):
            record = {'id': servo['id'], 'position': ticks[i]}
            if 'max_speed_steps_s' in servo:
                record['max_speed_steps_s'] = servo['max_speed_steps_s']
            records.append(record)
        return {'valid': True, 'reason': None, 'commands': records,
                'joint_target_radians': self.command[:], 'previous_action': self.previous_action[:]}
