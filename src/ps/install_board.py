"""Install versioned root-owned package and boot units; activation is explicit."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess

SOURCE = Path(__file__).resolve().parent
TARGET = Path('/usr/local/lib/pynq-robot-models-v11-20261102-01')
PYTHON = '/usr/local/share/pynq-venv/bin/python'


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--activate',action='store_true',help='Switch now and install enabled 320/UART boot units')
    p.add_argument('--cold-boot',action='store_true',help='Use explicit boot initialization only when PL metadata is empty')
    args = p.parse_args()
    if args.cold_boot and not args.activate:p.error('--cold-boot requires --activate')
    if os.geteuid()!=0:raise RuntimeError('Run as root')
    manifest = json.loads((SOURCE/'package_manifest.json').read_text(encoding='utf-8'))['sha256']
    for rel,digest in manifest.items():
        path = SOURCE/rel
        if path.is_symlink() or not path.resolve().is_relative_to(SOURCE) or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Package changed: '+rel)
    if TARGET.exists():
        if not args.activate:raise FileExistsError('Preserve existing installation: '+str(TARGET))
        for rel,digest in manifest.items():
            path = TARGET/rel
            if path.is_symlink() or not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
                raise RuntimeError('Existing installation differs; preserve it: '+rel)
    else:
        shutil.copytree(SOURCE,TARGET,ignore=shutil.ignore_patterns('__pycache__'))
    for path in [TARGET,*TARGET.rglob('*')]:
        os.chown(path,0,0);os.chmod(path,0o755 if path.is_dir() else 0o644)
    if not args.activate:
        print(json.dumps(dict(installed=str(TARGET),activated=False,boot_changed=False)))
        return
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    backup = Path('/var/backups')/('pynq-piano-v11-'+stamp)
    backup.mkdir(mode=0o700)
    unit_dir = Path('/etc/systemd/system')
    units = ('pynq-preprocess-overlay.service','pynq-pmod-uart.service','pynq-piano-model.service')
    previous = {}
    servo_driver = Path('/home/xilinx/pmod_servo_control/pmod_b_transport.py')
    previous_servo_driver = servo_driver.read_bytes() if servo_driver.is_file() and not servo_driver.is_symlink() else None
    if servo_driver.exists() and previous_servo_driver is None:
        raise RuntimeError('Preserve unusual servo driver entry point')
    if previous_servo_driver is not None:(backup/'pmod_b_transport.py').write_bytes(previous_servo_driver)
    for name in units:
        path = unit_dir/name
        if path.is_symlink():raise RuntimeError('Preserve symlinked unit: '+name)
        previous[name] = path.read_bytes() if path.exists() else None
        if previous[name] is not None:(backup/name).write_bytes(previous[name])
    try:
        for name in units:shutil.copyfile(TARGET/'systemd'/name,unit_dir/name)
        subprocess.run(['systemctl','daemon-reload'],check=True)
        selection=['--boot'] if args.cold_boot else ['--mode','320']
        subprocess.run([PYTHON,'-B',str(TARGET/'switch_mode.py'),*selection],check=True,timeout=120)
        if previous_servo_driver is not None:
            shutil.copyfile(TARGET/'uart_transport_shim.py',servo_driver)
        subprocess.run(['systemctl','enable','pynq-preprocess-overlay.service','pynq-pmod-uart.service'],check=True)
        print(json.dumps(dict(installed=str(TARGET),activated=True,boot_mode='320',backup=str(backup),model_autostart=False,motor_commands_sent=False)))
    except Exception:
        if previous_servo_driver is not None:servo_driver.write_bytes(previous_servo_driver)
        for name,payload in previous.items():
            if payload is None:(unit_dir/name).unlink(missing_ok=True)
            else:(unit_dir/name).write_bytes(payload)
        subprocess.run(['systemctl','daemon-reload'],check=True)
        raise


if __name__=='__main__':main()
