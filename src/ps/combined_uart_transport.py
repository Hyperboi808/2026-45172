"""PMOD A/B RTL UART transport; attaches to resident combined PL, never loads it."""
from contextlib import ExitStack
import fcntl
import hashlib
import json
import os
from pathlib import Path
import threading
import time

ROOT=Path(__file__).resolve().parent/'mode320'
BASE=0x40030000
MAGIC=0x55415232

class UartReceiveTimeout(TimeoutError):
    def __init__(self,raw,diagnostics):
        self.raw=raw;self.diagnostics=diagnostics
        super().__init__('UART response timeout: '+repr(diagnostics))

class PmodServoTransport:
    BAUD=1_000_000
    def __init__(self,firmware=None,fclk_hz=None,*,pmod='B',rx_timeout_us=25_000):
        from pynq import PL,MMIO,Clocks
        pmod=pmod.upper()
        if pmod not in ('A','B') or type(rx_timeout_us) is not int or not 1<=rx_timeout_us<=1_000_000:
            raise ValueError('Pmod A/B and bounded receive timeout required')
        if fclk_hz not in (None,100_000_000):raise ValueError('100 MHz fixed UART required')
        manifest=json.loads((ROOT/'module_manifest.json').read_text())
        active=Path(str(PL.bitfile_name))
        if not active.is_file() or hashlib.sha256(active.read_bytes()).hexdigest()!=manifest['sha256']['vision_base.bit']:
            raise RuntimeError('Combined overlay must already be resident; UART never loads base.bit')
        if not 99.99<float(Clocks.fclk0_mhz)<100.01:raise RuntimeError('UART clock must be 100 MHz')
        if PL.ip_dict.get('dual_pmod_uart_0',{}).get('phys_addr')!=BASE:raise RuntimeError('UART address map missing')
        self._locks=ExitStack();self._closed=False;self._lock=threading.Lock()
        self._pmod=pmod;self._rx_timeout_us=rx_timeout_us;self._offset=0 if pmod=='A' else 0x100
        try:
            for path,mode in (('/run/lock/pynq-stereo-overlay.lock',fcntl.LOCK_SH),
                              (f'/run/lock/pynq-uart-{pmod}.lock',fcntl.LOCK_EX)):
                f=self._locks.enter_context(os.fdopen(os.open(path,os.O_RDONLY|os.O_NOFOLLOW),'rb'))
                fcntl.flock(f,mode|fcntl.LOCK_NB)
            if str(PL.bitfile_name)!=str(active):raise RuntimeError('Overlay changed while opening UART')
            self._mmio=MMIO(BASE,4096)
            if self._read(0)!=MAGIC or not self._read(0x10)&1:
                raise RuntimeError('UART boot initialization has not completed')
        except Exception:self._locks.close();raise

    def _read(self,reg):return int(self._mmio.read(self._offset+reg))
    def _write(self,reg,value):self._mmio.write(self._offset+reg,int(value))
    def read_diagnostics(self):
        status=self._read(4)
        return dict(pmod=self._pmod,status=status,received=(status>>8)&255,
            tx_queued=(status>>16)&255,tx_busy=bool(status&4),rx_level=bool(status&16),
            errors=self._read(0x14),control=self._read(0x10),baud=self._read(8))
    def snapshot(self):return self.read_diagnostics()

    def transact(self,gpio_index,packet,response_length=0,timeout_s=3.0):
        raise ValueError('Current split UART requires transact_split(2,3,...)')

    def _exchange(self,packet,response_length,timeout_s):
        if self._closed:raise RuntimeError('UART closed')
        if not isinstance(packet,(bytes,bytearray,memoryview)):raise TypeError('Byte packet required')
        packet=bytes(packet)
        if not 1<=len(packet)<=128 or type(response_length) is not int or not 0<=response_length<=128 or not 0<timeout_s<=30:
            raise ValueError('Bounded UART exchange required')
        began=time.monotonic();deadline=began+timeout_s
        while self._read(4)&4 or (self._read(4)>>16)&255:
            if time.monotonic()>deadline:raise TimeoutError('Previous UART TX still busy')
            time.sleep(.00005)
        mode=self._read(0x10)&3
        if not mode&1:raise RuntimeError('UART disabled')
        self._write(0x10,mode|4)  # Owned channel only: clear stale RX/errors.
        for byte in packet:
            while self._read(4)&2:
                if time.monotonic()>deadline:raise TimeoutError('UART TX FIFO full')
            self._write(8,byte)
        raw=bytearray();receive_deadline=None
        while True:
            status=self._read(4);error=self._read(0x14)
            if error:raise RuntimeError('UART hardware error '+repr(self.read_diagnostics()))
            available=(status>>8)&255
            received_now=bool(response_length and available)
            if received_now:
                if len(raw)+available>512:raise RuntimeError('Unbounded UART noise/echo')
                # Drain the FIFO snapshot before applying receive-wait timeout.
                raw.extend(self._read(0xc)&255 for _ in range(available))
            tx_done=not status&4 and not (status>>16)&255
            if tx_done:
                if response_length==0:return b''
                if mode&2 and len(raw)==response_length:return bytes(raw)
                if not mode&2 and packet[:2]!=b'\xff\xff' and len(raw)==response_length:return bytes(raw)
                if not mode&2 and response_length>=6:
                    # Preserve FEETECH packet semantics even when an adapter echoes TX.
                    for start in range(max(0,len(raw)-response_length+1)):
                        frame=bytes(raw[start:start+response_length])
                        if (len(frame)==response_length and frame[:2]==b'\xff\xff' and
                            len(packet)>=3 and frame[2]==packet[2] and frame[3]+4==response_length and
                            frame!=packet and ((~sum(frame[2:-1]))&255)==frame[-1]):
                            return frame
            now=time.monotonic()
            if tx_done and (receive_deadline is None or received_now):receive_deadline=now+self._rx_timeout_us/1e6
            if now>deadline or (not received_now and receive_deadline is not None and now>receive_deadline):
                raise UartReceiveTimeout(bytes(raw),self.read_diagnostics())
            if not status&1:time.sleep(.00002)

    def transact_split(self,tx_gpio_index,rx_gpio_index,packet,response_length=0,timeout_s=3.0):
        if (tx_gpio_index,rx_gpio_index)!=(2,3):raise ValueError('Fixed D2 TX, D3 RX wiring')
        with self._lock:return self._exchange(packet,response_length,timeout_s)

    def diagnostic_loopback(self,data):
        with self._lock:
            if self._read(4)&4 or (self._read(4)>>16)&255:raise RuntimeError('UART busy')
            self._write(0x10,7)  # Loopback isolates external TX, which stays high.
            try:return self._exchange(data,len(data),1.0)
            finally:
                # Never reconnect external TX while diagnostic bytes are still
                # queued. On a fault retain the hardware loopback isolation.
                status=self._read(4)
                if not status&4 and not (status>>16)&255:self._write(0x10,5)

    def close(self):
        if not self._closed:
            self._locks.close();self._closed=True
    def __enter__(self):return self
    def __exit__(self,*args):self.close()

class PmodATransport(PmodServoTransport):
    def __init__(self,firmware=None,fclk_hz=None,*,rx_timeout_us=25_000):
        super().__init__(firmware,fclk_hz,pmod='A',rx_timeout_us=rx_timeout_us)

class PmodBTransport(PmodServoTransport):
    def __init__(self,firmware=None,fclk_hz=None,*,rx_timeout_us=25_000):
        super().__init__(firmware,fclk_hz,pmod='B',rx_timeout_us=rx_timeout_us)
