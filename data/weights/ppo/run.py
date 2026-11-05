"""Stateful standard-library v11 actor and servo-target postprocessing. No bus I/O."""
import argparse
import json
import math
from pathlib import Path
from piano_model import PianoActor, ServoCalibration, ServoCommandLimiter
from piano_model.actor import f32, vector, validate_observation

ROOT = Path(__file__).resolve().parent


class V11Runtime:
    def __init__(self, directory=None):
        self.directory = Path(directory) if directory else ROOT
        self.config = json.loads((self.directory/'controller_config.json').read_text(encoding='utf-8'))
        self.cal = ServoCalibration(json.loads((self.directory/'servo_config.json').read_text(encoding='utf-8')))
        self.actor = PianoActor(self.directory)
        self.poses = json.loads((self.directory/'teacher_keyframes_v11.json').read_text(encoding='utf-8'))
        self.fixed13 = 3.14
        self.reset()

    def reset(self, measured_joint=None):
        initial = self.cal.home if measured_joint is None else vector(measured_joint,14,'measured_joint',cast=False)
        self.limiter = ServoCommandLimiter(self.cal,initial)
        self.filtered = [f32(2*(q-self.cal.lo[i])/(self.cal.hi[i]-self.cal.lo[i])-1) for i,q in enumerate(initial)]
        self.previous_action = [0.]*14

    def restore(self, state):
        command = vector(state['command_joint_rad'],14,'command_joint_rad',cast=False)
        initial = vector(state['initial_joint_rad'],14,'initial_joint_rad',cast=False)
        self.limiter.reset(initial)
        if any(not self.cal.lo[i]<=q<=self.cal.hi[i] for i,q in enumerate(command)):
            raise ValueError('State command outside calibrated range')
        elapsed = float(state['elapsed_s'])
        if not math.isfinite(elapsed) or elapsed<0:
            raise ValueError('Invalid state elapsed time')
        self.limiter.command = command
        self.limiter.elapsed = elapsed
        self.limiter.previous_action = vector(state['limiter_previous_action'],14,'limiter_previous_action',cast=False)
        self.limiter.needs_reset = bool(state['needs_reset'])
        self.filtered = vector(state['filtered_normalized_action'],14,'filtered_normalized_action')
        self.previous_action = vector(state['previous_applied_action'],14,'previous_applied_action')

    def state(self):
        return {'command_joint_rad':self.limiter.command[:],
                'initial_joint_rad':self.limiter.initial_command[:],
                'elapsed_s':self.limiter.elapsed,'needs_reset':self.limiter.needs_reset,
                'limiter_previous_action':self.limiter.previous_action[:],
                'filtered_normalized_action':self.filtered[:],
                'previous_applied_action':self.previous_action[:]}

    def apply(self, normalized_action, observation, dt=.01):
        obs = validate_observation(observation)
        flags = obs['proprio'][42:46]
        if abs(sum(flags)-1)>1e-5 or any(min(abs(x),abs(x-1))>1e-5 for x in flags):
            raise ValueError('proprio[42:46] must encode prepare/press/release/home')
        phase = max(range(4),key=lambda i:flags[i])
        if (phase==3) != (obs['goal'][54]>=.999):
            raise ValueError('Home phase and completed sequence progress disagree')
        raw = [f32(max(-1.,min(1.,x))) for x in vector(normalized_action,14,'normalized_action')]
        if phase==1:
            midi = 48+max(range(25),key=lambda i:obs['goal'][i])
            group = self.config['strategy_by_midi'].get(str(midi))
            if group is None:
                raise ValueError('Unexpected target key')
            reference = self.poses['pose_pairs'][str(group)]['play_deg']
            bounds = self.config['press_correction_by_strategy'][str(group)]
            for i in range(14):
                width = self.cal.hi[i]-self.cal.lo[i]
                desired = self.cal.lo[i]+f32(f32(raw[i]+1)*.5)*width
                target = min(self.cal.hi[i],max(self.cal.lo[i],math.radians(reference[i])))
                lower = math.radians(bounds['min_deg'][i])
                upper = math.radians(bounds['max_deg'][i])
                desired = min(target+upper,max(target+lower,desired))
                raw[i] = f32(max(-1.,min(1.,2*(desired-self.cal.lo[i])/width-1)))
        raw[12] = f32(2*(self.fixed13-self.cal.lo[12])/(self.cal.hi[12]-self.cal.lo[12])-1)
        self.filtered = [f32(f32(f32(.1)*x)+f32(f32(.9)*previous))
                         for x,previous in zip(raw,self.filtered)]
        result = self.limiter.step(self.filtered,tracking_confidence=1.,dt=dt)
        self.limiter.command[12] = self.fixed13
        result['joint_target_radians'][12] = self.fixed13
        if result['valid']:
            ticks = self.cal.to_ticks(self.limiter.command)
            for i,item in enumerate(result['commands']):
                item['position'] = ticks[i]
        self.previous_action = raw[:]
        return {'command_valid':result['valid'],'reason':result.get('reason'),
                'phase':phase,'normalized_action':raw,'filtered_normalized_action':self.filtered[:],
                'joint_target_radians':result['joint_target_radians'],
                'servo_commands_steps':result['commands'],'hardware_commands_sent':False}

    def step(self, observation, dt=.01):
        return self.apply(self.actor.predict(observation),observation,dt)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,default=ROOT/'example_observation.json')
    parser.add_argument('--state',type=Path,required=True,help='Persist full filter and limiter state between frames')
    parser.add_argument('--output',type=Path)
    parser.add_argument('--dt',type=float,default=.01)
    args = parser.parse_args()
    runtime = V11Runtime()
    if args.state.exists():
        runtime.restore(json.loads(args.state.read_text(encoding='utf-8')))
    result = runtime.step(json.loads(args.input.read_text(encoding='utf-8')),args.dt)
    args.state.write_text(json.dumps(runtime.state(),indent=2),encoding='utf-8')
    text = json.dumps(result,indent=2)
    if args.output:
        args.output.write_text(text,encoding='utf-8')
    print(text)

if __name__=='__main__':
    main()
