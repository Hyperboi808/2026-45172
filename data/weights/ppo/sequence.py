"""v11 random five-key phase controller. Sensors and hardware transport are supplied by the caller."""
import json
import math
import random
from pathlib import Path
from piano_model import ServoCalibration
from piano_model.actor import vector


class V11Sequence:
    def __init__(self, directory=None, seed=None):
        root = Path(directory) if directory else Path(__file__).resolve().parent
        self.config = json.loads((root/'controller_config.json').read_text(encoding='utf-8'))
        self.cal = ServoCalibration(json.loads((root/'servo_config.json').read_text(encoding='utf-8')))
        source = json.loads((root/'teacher_keyframes_v11.json').read_text(encoding='utf-8'))
        self.keys = self.config['task_keys_midi']
        self.rng = random.Random(seed)
        self.targets = {}
        for midi,group in self.config['strategy_by_midi'].items():
            pair = source['pose_pairs'][str(group)]
            self.targets[int(midi)] = {}
            for phase,label in ((0,'prepare_deg'),(1,'play_deg'),(2,'prepare_deg')):
                q = [min(self.cal.hi[i],max(self.cal.lo[i],math.radians(v))) for i,v in enumerate(pair[label])]
                q[12] = 3.14
                self.targets[int(midi)][phase] = q

    def reset(self, now_s=0., sequence=None):
        chosen = self.rng.sample(self.keys,len(self.keys)) if sequence is None else list(sequence)
        if sorted(chosen)!=sorted(self.keys):
            raise ValueError('Sequence must contain each of the five target keys exactly once')
        if not math.isfinite(now_s) or now_s<0:
            raise ValueError('Invalid clock')
        self.sequence = chosen
        self.index = self.phase = 0
        self.start = self.phase_start = self.note_start = self.now = float(now_s)
        self.stable = 0.
        self.pressed = self.done = self.returned_home = False
        self.prepared = self.hits = self.misses = self.wrong_events = 0
        self.previous_down = set()
        return self.context()

    def target(self):
        return self.cal.home if self.index>=5 else self.targets[self.sequence[self.index]][self.phase]

    def close_to_target(self, measured):
        target = self.target()
        tolerance = [.07 if self.phase==3 else .04]*14
        if self.phase!=3 and target[8]>1.5:
            tolerance[8] = .22
        return all(abs(a-b)<t for a,b,t in zip(measured,target,tolerance))

    def advance(self):
        self.index += 1
        self.phase = 3 if self.index>=5 else 0
        self.phase_start = self.note_start = self.now
        self.stable = 0.
        self.pressed = False

    def update(self, joint_radians, key_depth_fraction, now_s, dt=.01, home_tip_errors_m=None):
        if self.done:
            return self.context()
        measured = vector(joint_radians,14,'encoder_joint_radians',cast=False)
        if not math.isfinite(now_s) or now_s<self.now or not math.isfinite(dt) or dt<=0:
            raise ValueError('Invalid control timing')
        self.now = float(now_s)
        depths = {int(k):float(v) for k,v in key_depth_fraction.items()}
        if not set(range(48,73)).issubset(depths) or not all(math.isfinite(v) for v in depths.values()):
            raise ValueError('Supply all 25 key states; MIDI key-down/up can be represented by 1/0')
        down = {k for k,v in depths.items() if v>.55}
        midi = self.sequence[min(self.index,4)]
        allowed = {midi} if self.phase in (1,2) else set()
        self.wrong_events += len((down-self.previous_down)-allowed)
        self.previous_down = down
        elapsed = self.now-self.phase_start
        if self.phase==0:
            self.stable = self.stable+dt if self.close_to_target(measured) else 0.
            if self.stable>=.15:
                self.prepared += 1
                self.phase,self.phase_start,self.stable = 1,self.now,0.
            elif elapsed>8:
                self.misses += 1
                self.advance()
        elif self.phase==1:
            self.pressed = self.pressed or midi in down
            if (self.pressed and elapsed>.15) or elapsed>4:
                self.phase,self.phase_start = 2,self.now
        elif self.phase==2:
            if depths[midi]<.2 or elapsed>2:
                hit = self.pressed and depths[midi]<.2
                self.hits += int(hit)
                self.misses += int(not hit)
                self.advance()
        else:
            if home_tip_errors_m is None:
                raise ValueError('Home phase requires tip errors from calibrated encoder-based FK')
            errors = [float(v) for v in home_tip_errors_m]
            if len(errors)!=2 or not all(math.isfinite(v) and v>=0 for v in errors):
                raise ValueError('Supply middle and index home tip distance errors in meters')
            if self.close_to_target(measured) and max(errors)<=.025:
                self.returned_home = self.done = True
            elif elapsed>8:
                self.done = True
        return self.context()

    def context(self):
        return {'sequence':self.sequence[:],'phase':self.phase,'target_index':self.index,
                'current_midi':self.sequence[min(self.index,4)],
                'next_midi':self.sequence[self.index+1] if self.index<4 else None,
                'sequence_progress':min(1.,self.index/5),
                'note_elapsed_s':max(0.,self.now-self.note_start),
                'elapsed_s':self.now-self.start,'done':self.done,
                'prepared':self.prepared,'hits':self.hits,'misses':self.misses,
                'wrong_events':self.wrong_events,'returned_home':self.returned_home}
