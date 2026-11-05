"""Guarded full-PL selection: 320+UART, PPO on 320+UART, or 640 depth.

WM is intentionally unavailable until an actual WM model/backend is supplied.
The four LED meanings are independent model/vision status pairs.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time
from status_led import publish, dark

ROOT = Path(__file__).resolve().parent
CURRENT = Path('/usr/local/lib/pynq-vision-current')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def verify_resident_addresses(mode,ip_dict):
    # Check metadata before MMIO: an absent AXI slave can abort the PS process.
    for name,address in mode['engine_addresses'].items():
        if ip_dict.get(name,{}).get('phys_addr')!=address:
            raise RuntimeError('Resident IP catalog mismatch before register access: '+name)


def verify_package():
    for rel,digest in json.loads((ROOT/'package_manifest.json').read_text(encoding='utf-8'))['sha256'].items():
        path = ROOT/rel
        if path.is_symlink() or not path.resolve().is_relative_to(ROOT) or sha(path)!=digest:
            raise RuntimeError('Package integrity mismatch: '+rel)


def wait_idle(mode,mmio):
    for name,address in mode['engine_addresses'].items():
        if not mmio(address,65536).read(0)&4:
            raise RuntimeError('Active accelerator: '+name)
    if mode['uart']:
        io = mmio(0x40030000,4096)
        for offset in (0,256):
            status = int(io.read(offset+4))
            if status&4 or (status>>16)&255:
                raise RuntimeError('UART transmitting; postpone mode switch')
    if mode.get('blank_slot'):
        slot = mmio(0x40050000,4096)
        if slot.read(0)&3!=1 or slot.read(8)&3!=3:
            raise RuntimeError('Old model region not blank and isolated')


def initialize(mode,mmio):
    if mode['uart']:
        io = mmio(0x40030000,4096)
        for offset in (0,256):
            if io.read(offset)!=0x55415232 or io.read(offset+8)!=1000000:
                raise RuntimeError('UART ABI mismatch')
            io.write(offset+16,5)


def download(bit,overlay_type):
    overlay = overlay_type(str(bit),download=False)
    if hasattr(overlay.parser,'_from_cache'):
        delattr(overlay.parser,'_from_cache')
    overlay.download()


def read_status():
    result=dict(active_bitfile=None,led_mask=None,wm_available=False,status='not_ready')
    service=subprocess.run(['systemctl','is-active','pynq-preprocess-overlay.service'],
                           capture_output=True,text=True,timeout=5)
    if service.returncode or service.stdout.strip()!='active':return result
    import fcntl
    try:
        fd=os.open('/run/lock/pynq-stereo-overlay.lock',os.O_RDONLY|os.O_NOFOLLOW)
        handle=os.fdopen(fd,'rb')
        try:fcntl.flock(handle,fcntl.LOCK_SH|fcntl.LOCK_NB)
        except Exception:handle.close();raise
    except (OSError,BlockingIOError):return result
    with handle:
        if Path('/sys/class/fpga_manager/fpga0/state').read_text(encoding='utf-8').strip()!='operating':return result
        from pynq import PL
        active=Path(str(PL.bitfile_name));result['active_bitfile']=str(PL.bitfile_name)
        catalog=json.loads((ROOT/'modes.json').read_text(encoding='utf-8'))['modes']
        if not active.is_file() or sha(active) not in {m['bit_sha256'] for m in catalog.values()}:return result
        from status_led import attach
        value=int(attach().read(0))&15
        result.update(led_mask=value,status='ready',wm_available=True,
                      selected_model={9:'ppo',10:'wm'}.get(value),uart_output_mode='internal_loopback')
        return result


def select_vision(mode,boot=False):
    from pynq import PL, MMIO, Overlay, Clocks
    import fcntl
    verify_package()
    catalog = json.loads((ROOT/'modes.json').read_text(encoding='utf-8'))
    known = dict(catalog['modes'],**catalog['accepted_previous_modes'])
    before = str(PL.bitfile_name)
    active = Path(before)
    active_sha = sha(active) if active.is_file() else None
    old = next((v for v in known.values() if v['bit_sha256']==active_sha),None)
    if old is None and not (boot and before in ('','None')):
        raise RuntimeError('Unknown resident overlay; preserve it')
    target = catalog['modes'][mode]
    bit = ROOT/target['bit']
    if sha(bit)!=target['bit_sha256'] or sha(bit.with_suffix('.hwh'))!=target['hwh_sha256']:
        raise RuntimeError('Bitstream/HWH mismatch')
    for device in ('/dev/video0','/dev/video1'):
        if Path(device).exists():
            check = subprocess.run(['fuser',device],capture_output=True,text=True,timeout=5)
            if check.returncode not in (0,1) or check.stdout.strip():
                raise RuntimeError('Camera in use; close its owner first')
    # The model service releases its buffers and shared overlay lock on stop.
    subprocess.run(['systemctl','stop','pynq-piano-model.service'],check=True,timeout=30)
    for unit in ('robot-control','robot-command','robot-reconfig','robot-h','robot-camera-tx','robot-usb-camera','pynq-model-inference'):
        check = subprocess.run(['systemctl','show',unit+'.service','-p','ActiveState','--value'],capture_output=True,text=True,timeout=5)
        if check.returncode or check.stdout.strip() not in ('inactive','failed'):
            raise RuntimeError('Conflicting service: '+unit)
    with ExitStack() as locks:
        for path in ('/run/lock/pynq-stereo-overlay.lock','/run/lock/pynq-uart-A.lock','/run/lock/pynq-uart-B.lock',
                     '/run/lock/pynq-fp32-dense.lock','/run/lock/pynq-wm-trajectory.lock','/run/lock/pynq-depth640-runtime.lock',
                     '/home/xilinx/stereosgm_rt_tracking_20261017_01/stereo_tracking_deployment_20261017_01/.runtime.lock'):
            p = Path(path)
            if not p.parent.is_dir():
                if p.parent == Path('/run/lock'):raise RuntimeError('Runtime lock directory missing')
                continue
            fd = os.open(path,os.O_RDONLY|os.O_NOFOLLOW) if p.exists() else os.open(path,os.O_CREAT|os.O_EXCL|os.O_RDWR|os.O_NOFOLLOW,0o644)
            handle = locks.enter_context(os.fdopen(fd,'rb'))
            fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if str(PL.bitfile_name)!=before:
            raise RuntimeError('Hardware changed during lock acquisition')
        if old is not None:
            verify_resident_addresses(old,PL.ip_dict)
            wait_idle(old,MMIO)
        if active_sha==target['bit_sha256'] and active.resolve()==bit.resolve():
            publish(mode)
            return dict(mode=mode,changed=False,led_mask=8 if mode=='320' else 4)
        previous_link = CURRENT.readlink() if CURRENT.is_symlink() else None
        if CURRENT.exists() and not CURRENT.is_symlink():
            raise RuntimeError('Current vision path is not a symlink; preserve it')
        changed = False
        link_changed = False
        try:
            if old and old.get('status_led'):dark()
            changed = True
            download(bit,Overlay)
            for name,address in dict(target['engine_addresses'],status_led_gpio=0x40060000).items():
                if PL.ip_dict.get(name,{}).get('phys_addr')!=address:
                    raise RuntimeError('IP address mismatch: '+name)
            if mode=='640' and any(name in PL.ip_dict for name in ('dual_pmod_uart_0','dense_fp32_0')):
                raise RuntimeError('Unexpected UART/model in 640 overlay')
            if not 99.99<float(Clocks.fclk0_mhz)<100.01:
                raise RuntimeError('100 MHz clock required')
            initialize(target,MMIO);wait_idle(target,MMIO)
            if Path('/sys/class/fpga_manager/fpga0/state').read_text(encoding='utf-8').strip()!='operating':
                raise RuntimeError('FPGA not operating')
            link = CURRENT.with_name(CURRENT.name+'.piano-new-'+str(os.getpid()))
            link.symlink_to(bit.parent,target_is_directory=True);link.replace(CURRENT);link_changed = True
            value = publish(mode)
            return dict(mode=mode,changed=True,led_mask=value,full_PL_switch=True,motor_commands_sent=False)
        except Exception:
            if changed and old is not None:
                download(active,Overlay)
                initialize(old,MMIO)
                if old.get('status_led'):publish('320' if old['uart'] else '640')
            if link_changed and previous_link is not None:
                link = CURRENT.with_name(CURRENT.name+'.piano-rollback-'+str(os.getpid()))
                link.symlink_to(previous_link,target_is_directory=True);link.replace(CURRENT)
            elif link_changed and CURRENT.is_symlink() and CURRENT.resolve()==bit.parent.resolve():
                CURRENT.unlink()
            raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode',choices=('320','640','ppo','wm'))
    p.add_argument('--boot',action='store_true')
    p.add_argument('--status',action='store_true')
    args = p.parse_args()
    if sum((args.mode is not None,args.boot,args.status))!=1:
        p.error('Choose exactly one of --mode, --boot, --status')
    os.environ['XILINX_XRT'] = '/usr'
    if args.status:
        print(json.dumps(read_status(),indent=2))
        return
    if os.geteuid()!=0:raise RuntimeError('Root required for FPGA/service selection')
    mode = '320' if args.boot or args.mode in ('ppo','wm') else args.mode
    result = select_vision(mode,args.boot)
    if args.mode in ('ppo','wm'):
        selection=Path('/run/pynq-active-model.env')
        if selection.is_symlink():raise RuntimeError('Preserve unexpected model environment symlink')
        temporary=selection.with_name(selection.name+'.'+str(os.getpid()))
        with temporary.open('x',encoding='ascii') as handle:handle.write('PYNQ_ACTIVE_MODEL='+args.mode+'\n')
        os.chmod(temporary,0o644);temporary.replace(selection)
        subprocess.run(['systemctl','start','pynq-piano-model.service'],check=True,timeout=60)
        result.update(model=args.mode,led_mask=9 if args.mode=='ppo' else 10,uart_output_mode='internal_loopback')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':main()
