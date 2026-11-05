"""Meaningful v11 action/controller regression, plus fresh HLS test vectors."""
import argparse
import copy
import json
from pathlib import Path
import struct
import sys
import numpy as np
from accelerated_actor import AcceleratedActor, AcceleratedRuntime, ROOT
from piano_model import PianoActor
from run import V11Runtime


def cases():
    rng = np.random.default_rng(20261102)
    example = json.loads((ROOT/'model/example_observation.json').read_text(encoding='utf-8'))
    yield 'archive_example', example
    for index in range(48):
        obs = copy.deepcopy(example)
        phase = index % 4
        obs['proprio'] = np.clip(rng.normal(0,.35,76),-1,1).tolist()
        obs['proprio'][42:46] = [int(x==phase) for x in range(4)]
        obs['goal'] = [0.0]*56
        midi = (69,65,72,62,71)[index%5]
        obs['goal'][midi-48] = 1
        obs['goal'][25+(index*7)%25] = 1
        obs['goal'][50:54] = [.45,.085,.026,1.0]
        obs['goal'][54] = 1.0 if phase==3 else .2*(index%5)
        obs['goal'][55] = float(rng.uniform())
        for key,count in (('pointcloud',256),('goal_cloud',32)):
            points = rng.uniform(-.5,.7,(count,4)).astype(np.float32)
            points[:,3] = rng.uniform(-.3,1.3,count)
            if index % 6 == 0:
                points[:,3] = 0
            elif index % 6 == 1:
                points[:,3] = 1
            elif index % 6 == 2:
                points[:,3] = 0
                points[0,3] = .25
            obs[key] = points.tolist()
        yield 'random_%02d_phase%d'%(index,phase), obs


def generate_dense_vectors(actor):
    rng = np.random.default_rng(117)
    core = actor.core
    items = []
    for name,spec in actor.manifest['tensors'].items():
        if not name.endswith('.weight'):
            continue
        layer = name[:-7]
        outputs,inputs = spec['shape']
        batch = 256 if '.cloud.mlp.' in layer else 32 if '.target_cloud.mlp.' in layer else 1
        x = rng.normal(0,.35,(batch,inputs)).astype('<f4')
        weight = core.weights[spec['offset_bytes']//4:(spec['offset_bytes']+spec['length_bytes'])//4].reshape(outputs,inputs)
        b = actor.manifest['tensors'][layer+'.bias']
        bias = core.weights[b['offset_bytes']//4:(b['offset_bytes']+b['length_bytes'])//4]
        relu = int('features_extractor' in layer)
        expected = core.run(layer,x,relu)
        items.append((batch,inputs,outputs,relu,x,weight,bias,expected))
    dest = ROOT/'verification'
    dest.mkdir(exist_ok=True)
    with (dest/'dense_cases.bin').open('wb') as handle:
        handle.write(struct.pack('<I',len(items)))
        for batch,inputs,outputs,relu,*arrays in items:
            handle.write(struct.pack('<4I',batch,inputs,outputs,relu))
            for array in arrays:
                handle.write(np.ascontiguousarray(array,dtype='<f4').tobytes())


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--backend',choices=('emulated','pl'),default='emulated')
    p.add_argument('--bitfile')
    p.add_argument('--sha256')
    p.add_argument('--output',type=Path,default=ROOT/'verification/model_parity.json')
    args = p.parse_args()
    reference = PianoActor(ROOT/'model')
    accelerated = AcceleratedActor(ROOT/'model',args.backend,args.bitfile,args.sha256)
    original_control = V11Runtime(ROOT/'model')
    accelerated_control = V11Runtime(ROOT/'model')
    rows = []
    try:
        for name,observation in cases():
            expected = reference.predict(observation)
            actual = accelerated.predict(observation)
            error = max(abs(x-y) for x,y in zip(expected,actual))
            if error > 1e-4:
                raise AssertionError((name,error))
            before = original_control.limiter.command[:]
            a = original_control.apply(expected,observation,.01)
            b = accelerated_control.apply(actual,observation,.01)
            command_error = max(abs(x-y) for x,y in zip(a['joint_target_radians'],b['joint_target_radians']))
            assert a['command_valid'] == b['command_valid']
            assert b['joint_target_radians'][12] == 3.14
            speed12 = abs(b['joint_target_radians'][11]-before[11])*652/.01
            assert speed12 <= 500+1e-6
            assert command_error < 1e-4
            state = json.loads(json.dumps(accelerated_control.state()))
            accelerated_control.restore(state)
            rows.append(dict(name=name,action_max_abs_error=error,command_max_abs_error_rad=command_error,ms=accelerated.last_ms))
        if args.backend == 'emulated':
            generate_dense_vectors(accelerated)
        report = dict(backend=args.backend,cases=len(rows),passed=True,
                      max_action_error=max(r['action_max_abs_error'] for r in rows),
                      max_joint_error_rad=max(r['command_max_abs_error_rad'] for r in rows),
                      servo13_fixed_rad=3.14,servo12_speed_cap_checked=True,
                      pooling_confidence_edge_cases_checked=True,state_restore_checked=True,
                      board_tested=args.backend=='pl',motor_commands_sent=False,
                      measured_hardware_speedup=None,rows=rows)
        args.output.parent.mkdir(exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='rows'},indent=2))
    finally:
        accelerated.close()


if __name__ == '__main__':
    main()
