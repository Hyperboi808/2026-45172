"""No-actuator verification of the real WM kernel and both UART output paths."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import time
import numpy as np

ROOT=Path(__file__).resolve().parent


def wm_parity():
    from wm_accelerated import AcceleratedWmRuntime,PynqWmCpuRuntime
    cfg=json.loads((ROOT/'modes.json').read_text())['modes']['320']
    cpu=PynqWmCpuRuntime(ROOT/'wm_model')
    pl=AcceleratedWmRuntime('pl',str(ROOT/cfg['bit']),cfg['bit_sha256'])
    rows=[];worst=0;values=0
    try:
        for task in cpu.model['enabled_task_ids']:
            route=cpu.task_ids.index(task);pair=cpu.pairs[route]
            obs=dict(source_xyz_m=[*pair[:2],cpu.model['source_height_m']],
                target_xyz_m=[*pair[2:],cpu.model['target_height_m']],source_confidence=1,target_confidence=1,valid_point_count=256)
            expected=cpu.plan(obs);actual=pl.plan(obs)
            assert expected['accepted'] and actual['accepted'] and expected['task_id']==actual['task_id']
            assert len(expected['waypoints'])==len(actual['waypoints'])
            for a,b in zip(expected['waypoints'],actual['waypoints']):
                assert a['target_steps']==b['target_steps']
                error=max(abs(x-y) for x,y in zip(a['target_rad'],b['target_rad']))
                worst=max(worst,error);values+=14;assert error<=1e-12
            rows.append(dict(task=task,waypoints=actual['waypoint_count'],hardware_ms=pl.core.last_ms))
        example=json.loads((ROOT/'wm_model/example_input_task30001.json').read_text())
        calls=pl.core.calls
        for bad in (dict(example,source_confidence=0),dict(example,valid_point_count=0),dict(example,target_xyz_m=example['source_xyz_m'])):
            assert not pl.plan(bad)['accepted']
        assert pl.core.calls==calls
        timings={}
        for name,runtime in (('CPU_reference',cpu),('PL_WM',pl)):
            samples=[]
            for i in range(12):
                start=time.perf_counter();result=runtime.plan(example);elapsed=(time.perf_counter()-start)*1000
                assert result['accepted']
                if i>=2:samples.append(elapsed)
            timings[name]=dict(p50_ms=float(np.median(samples)),p95_ms=float(np.percentile(samples,95)),samples_ms=samples)
        return dict(passed=True,backend='pl_wm_trajectory',routes=rows,joint_values=values,
            max_radian_error=worst,steps_exact=True,rejected_inputs_do_not_start_hardware=True,
            timings=timings,speedup=timings['CPU_reference']['p50_ms']/timings['PL_WM']['p50_ms'],
            hardware_commands_sent=False)
    finally:pl.close()


def uart_outputs():
    from accelerated_actor import AcceleratedRuntime
    from wm_accelerated import AcceleratedWmRuntime
    from model_uart import ModelUartOutput
    cfg=json.loads((ROOT/'modes.json').read_text())['modes']['320']
    bit=str(ROOT/cfg['bit']);rows={}
    for model in ('ppo','wm'):
        runtime=AcceleratedRuntime('pl',bit,cfg['bit_sha256']) if model=='ppo' else AcceleratedWmRuntime('pl',bit,cfg['bit_sha256'])
        sink=None
        try:
            source=ROOT/('model/example_observation.json' if model=='ppo' else 'wm_model/example_input_task30001.json')
            observation=json.loads(source.read_text())
            result=runtime.step(observation,.05) if model=='ppo' else runtime.plan(observation)
            sink=ModelUartOutput()
            points=[0] if model=='ppo' else [0,len(result['waypoints'])//2,len(result['waypoints'])-1]
            samples=[sink.send_model_result(model,result,index) for index in points]
            for sample in samples:
                assert sample['pmod_A_ids']==list(range(1,11)) and sample['pmod_B_ids']==list(range(11,15))
                assert sample['frame_count']==14 and sample['loopback_passed'] and sample['external_uart_bytes_sent']==0
            rows[model]=samples
        finally:
            if sink:sink.close()
            runtime.close()
    return dict(passed=True,models=rows,shared_frame_builder=True,external_uart_bytes_sent=0,hardware_commands_sent=False)


def service(model,socket_path):
    source=ROOT/('model/example_observation.json' if model=='ppo' else 'wm_model/example_input_task30001.json')
    obs=json.loads(source.read_text());rows=[]
    for i in range(3):
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as client:
            client.settimeout(20);client.connect(socket_path)
            request=dict(model=model,frame_id=i,observation=obs,dt=.05,waypoint_index=i if model=='wm' else 0)
            client.sendall(json.dumps(request).encode()+b'\n')
            with client.makefile('rb') as stream:result=json.loads(stream.readline())
        assert result['model']==model and result['hardware_commands_sent'] is False
        assert result['uart_output']['loopback_passed'] and result['uart_output']['frame_count']==14
        rows.append(dict(frame_id=i,elapsed_ms=result['elapsed_ms'],backend=result['backend'],uart_output=result['uart_output']))
    # The running model owns a shared overlay lock and has completed READY=1.
    import fcntl,os
    with os.fdopen(os.open('/run/lock/pynq-stereo-overlay.lock',os.O_RDONLY|os.O_NOFOLLOW),'rb') as handle:
        fcntl.flock(handle,fcntl.LOCK_SH|fcntl.LOCK_NB)
        from status_led import attach
        led=int(attach().read(0))&15
    assert led==(9 if model=='ppo' else 10)
    return dict(passed=True,model=model,rows=rows,led_mask=led,external_uart_bytes_sent=0,hardware_commands_sent=False)


def main():
    p=argparse.ArgumentParser()
    p.add_argument('action',choices=('wm_parity','uart_outputs','service'))
    p.add_argument('--model',choices=('ppo','wm'),default='wm')
    p.add_argument('--socket',default='/run/pynq-piano/inference.sock')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError('Preserve verification evidence')
    result=service(args.model,args.socket) if args.action=='service' else globals()[args.action]()
    args.output.write_text(json.dumps(result,indent=2))
    compact={k:v for k,v in result.items() if k not in ('rows','models')}
    print(json.dumps(compact,indent=2),flush=True)


if __name__=='__main__':main()
