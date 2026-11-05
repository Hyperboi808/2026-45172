"""Selected PPO/WM PL runtime with both output paths connected to UART loopback."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model',choices=('ppo','wm'),default=os.environ.get('PYNQ_ACTIVE_MODEL','ppo'))
    p.add_argument('--socket',type=Path,default=Path('/run/pynq-piano/inference.sock'))
    args=p.parse_args()
    active=subprocess.run(['systemctl','is-active','pynq-preprocess-overlay.service'],capture_output=True,text=True)
    if active.returncode or active.stdout.strip()!='active':raise RuntimeError('Wait for boot overlay service')
    from accelerated_actor import AcceleratedRuntime,V11Runtime
    from piano_model import PianoActor
    from wm_accelerated import AcceleratedWmRuntime,PynqWmCpuRuntime
    from status_led import publish
    from model_uart import ModelUartOutput
    cfg=json.loads((ROOT/'modes.json').read_text())['modes']['320']
    bit=str(ROOT/cfg['bit'])
    runtime=None;uart=None;server=None;published=False;owned=False;stopping=False
    def stop(*unused):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    try:
        if args.model=='ppo':
            runtime=AcceleratedRuntime('pl',bit,cfg['bit_sha256'])
            example=json.loads((ROOT/'model/example_observation.json').read_text())
            expected=PianoActor(ROOT/'model').predict(example);actual=runtime.actor.predict(example)
            if max(abs(a-b) for a,b in zip(actual,expected))>1e-4:raise RuntimeError('PPO readiness mismatch')
            probe=V11Runtime(ROOT/'model').apply(actual,example,.05)
        else:
            runtime=AcceleratedWmRuntime('pl',bit,cfg['bit_sha256'])
            example=json.loads((ROOT/'wm_model/example_input_task30001.json').read_text())
            expected=PynqWmCpuRuntime(ROOT/'wm_model').plan(example);probe=runtime.plan(example)
            if not probe['accepted'] or len(probe['waypoints'])!=len(expected['waypoints']):raise RuntimeError('WM readiness route mismatch')
            for a,b in zip(probe['waypoints'],expected['waypoints']):
                if a['target_steps']!=b['target_steps'] or max(abs(x-y) for x,y in zip(a['target_rad'],b['target_rad']))>1e-12:
                    raise RuntimeError('WM readiness target mismatch')
        uart=ModelUartOutput('internal_loopback')
        uart_probe=uart.send_model_result(args.model,probe)
        args.socket.parent.mkdir(mode=0o755,exist_ok=True)
        if args.socket.exists():raise FileExistsError('Preserve existing inference socket')
        server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        server.bind(str(args.socket));owned=True;os.chmod(args.socket,0o660)
        server.listen(4);server.settimeout(.5)
        publish('320',args.model);published=True
        print(json.dumps(dict(ready=True,model=args.model,backend='pl',uart_mode='internal_loopback',
              uart_frame_count=uart_probe['frame_count'],external_uart_bytes_sent=0)),flush=True)
        notify=os.environ.get('NOTIFY_SOCKET')
        if notify:
            with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sender:
                sender.connect('\0'+notify[1:] if notify.startswith('@') else notify);sender.sendall(b'READY=1')
        while not stopping:
            try:connection,_=server.accept()
            except socket.timeout:continue
            with connection:
                connection.settimeout(3)
                try:
                    with connection.makefile('rb') as stream:line=stream.readline(1024*1024+1)
                    if len(line)>1024*1024 or not line.endswith(b'\n'):raise ValueError('One bounded JSON request required')
                    request=json.loads(line)
                    if request.get('model',args.model)!=args.model:raise ValueError('Select model before inference')
                    if request.get('uart_mode','internal_loopback')!='internal_loopback':raise ValueError('This round permits internal UART only')
                    start=time.perf_counter()
                    if args.model=='ppo':
                        if 'reset_joint_radians' in request:runtime.reset(request['reset_joint_radians'])
                        result=runtime.step(request['observation'],request['dt']);valid=result['command_valid']
                    else:
                        result=runtime.plan(request['observation']);valid=result['accepted']
                    if valid:result['uart_output']=uart.send_model_result(args.model,result,request.get('waypoint_index',0))
                    else:result['uart_output']=dict(routed_to='dual_pmod_uart',mode='internal_loopback',
                        frame_count=0,reason='model_rejected_command',external_uart_bytes_sent=0)
                    result.update(model=args.model,backend='pl' if args.model=='ppo' else 'pl_wm_trajectory',
                        frame_id=request.get('frame_id'),elapsed_ms=(time.perf_counter()-start)*1000,
                        hardware_commands_sent=False)
                    connection.sendall(json.dumps(result).encode()+b'\n')
                except (ValueError,KeyError,TypeError,json.JSONDecodeError,socket.timeout) as error:
                    try:connection.sendall(json.dumps(dict(error=str(error),hardware_commands_sent=False)).encode()+b'\n')
                    except OSError:pass
                except OSError:
                    if runtime.core.failed if args.model=='wm' else runtime.actor.core.failed:raise
    finally:
        if published:
            try:publish('320')
            except (RuntimeError,OSError):pass
        if server is not None:server.close()
        if owned:args.socket.unlink(missing_ok=True)
        if uart is not None:uart.close()
        if runtime is not None:
            while True:
                try:runtime.close();break
                except (RuntimeError,OSError):time.sleep(.1)


if __name__=='__main__':main()
