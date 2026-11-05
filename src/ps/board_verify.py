"""Board diagnostics: GPIO/UART readbacks; no servo packets or camera capture."""
import argparse
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--uart-only',action='store_true')
    args = p.parse_args()
    import fcntl
    from pynq import PL, MMIO, Clocks
    catalog = json.loads((ROOT/'modes.json').read_text(encoding='utf-8'))['modes']
    with ExitStack() as locks:
        for path,mode in (('/run/lock/pynq-stereo-overlay.lock',fcntl.LOCK_SH),
                          ('/run/lock/pynq-uart-A.lock',fcntl.LOCK_EX),
                          ('/run/lock/pynq-uart-B.lock',fcntl.LOCK_EX)):
            handle = locks.enter_context(os.fdopen(os.open(path,os.O_RDONLY|os.O_NOFOLLOW),'rb'))
            fcntl.flock(handle,mode|fcntl.LOCK_NB)
        active = Path(str(PL.bitfile_name))
        sha = hashlib.sha256(active.read_bytes()).hexdigest()
        key = next((k for k,m in catalog.items() if m['bit_sha256']==sha),None)
        if key is None:raise RuntimeError('Unrecognized bitstream')
        if not 99.99<float(Clocks.fclk0_mhz)<100.01:raise RuntimeError('Clock mismatch')
        led = MMIO(0x40060000,4096)
        value = int(led.read(0))&15
        rows = []
        if args.uart_only and key!='320':raise RuntimeError('UART requires 320 mode')
        if key=='320':
            uart = MMIO(0x40030000,4096)
            for offset in (0,256):
                assert uart.read(offset)==0x55415232
                assert uart.read(offset+8)==1000000
                assert uart.read(offset+16)&3==1
                rows.append(dict(pmod='A' if offset==0 else 'B',status=int(uart.read(offset+4)),errors=int(uart.read(offset+20))))
            assert value in (8,9,10)
        else:assert value==4
        print(json.dumps(dict(passed=True,mode=key,led_mask=value,uart=rows,motor_commands_sent=False),indent=2))


if __name__=='__main__':main()
