"""Persistent v11 PL inference socket, with LED1 set only after readiness checks."""
import argparse
import json
import os
from pathlib import Path
import signal
import socket
import time
from accelerated_actor import AcceleratedRuntime, ROOT
from piano_model import PianoActor
from status_led import publish


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--socket',type=Path,default=Path('/run/pynq-piano/inference.sock'))
    args = p.parse_args()
    modes = json.loads((ROOT/'modes.json').read_text(encoding='utf-8'))['modes']
    selected = modes['320']
    bit = ROOT/selected['bit']
    runtime = AcceleratedRuntime('pl',str(bit),selected['bit_sha256'])
    stopping = False
    def stop(*unused):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    server = None
    published = False
    socket_owned = False
    try:
        example = json.loads((ROOT/'model/example_observation.json').read_text(encoding='utf-8'))
        expected = PianoActor(ROOT/'model').predict(example)
        actual = runtime.actor.predict(example)
        error = max(abs(x-y) for x,y in zip(expected,actual))
        if error > 1e-4:
            raise RuntimeError('v11 PL readiness parity failed: '+str(error))
        args.socket.parent.mkdir(mode=0o755,exist_ok=True)
        if args.socket.exists():
            raise FileExistsError('Existing socket; preserve possible other server')
        server = socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        server.bind(str(args.socket)); socket_owned = True
        os.chmod(args.socket,0o660)
        server.listen(4); server.settimeout(.5)
        publish('320','ppo'); published = True
        print(json.dumps(dict(ready=True,model='ppo',socket=str(args.socket),readiness_error=error)),flush=True)
        # systemd Type=notify publishes active only after model, PL and socket checks.
        notify = os.environ.get('NOTIFY_SOCKET')
        if notify:
            with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as sender:
                sender.connect('\0'+notify[1:] if notify.startswith('@') else notify)
                sender.sendall(b'READY=1')
        while not stopping:
            try:
                connection,_ = server.accept()
            except socket.timeout:
                continue
            with connection:
                connection.settimeout(3)
                try:
                    with connection.makefile('rb') as stream:
                        line = stream.readline(1024*1024+1)
                    if len(line)>1024*1024 or not line.endswith(b'\n'):
                        raise ValueError('One bounded newline-terminated JSON request required')
                    request = json.loads(line)
                    if 'reset_joint_radians' in request:
                        runtime.reset(request['reset_joint_radians'])
                    began = time.perf_counter()
                    result = runtime.step(request['observation'],request['dt'])
                    result.update(backend='pl',model='ppo',elapsed_ms=(time.perf_counter()-began)*1000,
                                  frame_id=request.get('frame_id'),hardware_commands_sent=False)
                    connection.sendall(json.dumps(result).encode()+b'\n')
                except (ValueError,KeyError,TypeError,json.JSONDecodeError,socket.timeout) as error:
                    if runtime.actor.core.failed:
                        raise
                    try:connection.sendall(json.dumps(dict(error=str(error),hardware_commands_sent=False)).encode()+b'\n')
                    except OSError:pass
                except OSError:
                    if runtime.actor.core.failed:
                        raise
                    # A client may disappear after inference. Keep the model resident.
                    pass
    finally:
        if published:
            try:publish('320')
            except (OSError,RuntimeError):pass
        if server is not None:
            server.close()
        if socket_owned:
            args.socket.unlink(missing_ok=True)
        # Preserve CMA ownership and locks if the hardware is still doing DMA.
        # Ordinary SIGTERM finishes immediately after the current bounded call.
        while True:
            try:
                runtime.close()
                break
            except (RuntimeError,OSError):
                time.sleep(.1)


if __name__ == '__main__':
    main()
