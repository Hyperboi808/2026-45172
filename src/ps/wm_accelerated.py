"""Real WM reference-head gather and radian decoding on the resident PL kernel."""
from contextlib import ExitStack
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time
import numpy as np

ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'wm_model'))
from edge_runtime import PynqWmCpuRuntime


class WmPLCore:
    def __init__(self,root,bitfile,expected_sha):
        import fcntl
        from pynq import PL,Overlay,allocate
        self.memory=None;self.failed=False;self.calls=0;self.last_ms=None
        self.locks=ExitStack()
        try:
            for path,mode in (('/run/lock/pynq-stereo-overlay.lock',fcntl.LOCK_SH),
                              ('/run/lock/pynq-wm-trajectory.lock',fcntl.LOCK_EX)):
                h=self.locks.enter_context(os.fdopen(os.open(path,os.O_RDONLY|os.O_NOFOLLOW),'rb'))
                fcntl.flock(h,mode|fcntl.LOCK_NB)
            active=Path(str(PL.bitfile_name))
            if active.resolve()!=Path(bitfile).resolve() or hashlib.sha256(active.read_bytes()).hexdigest()!=expected_sha:
                raise RuntimeError('Select the combined PPO/WM 320 overlay first')
            self.overlay=Overlay(str(active),download=False)
            info=self.overlay.ip_dict.get('wm_trajectory_0',{})
            if info.get('phys_addr')!=0x40040000:raise RuntimeError('WM hardware core absent')
            self.ip=self.overlay.wm_trajectory_0
            if not self.ip.read(0)&4:raise RuntimeError('WM core busy')
            self.regs=info['registers'];self.control=self.ip.mmio.array
            manifest=json.loads((root/'tensor_manifest.json').read_text())
            weights=(root/manifest['weights_file']).read_bytes()
            if hashlib.sha256(weights).hexdigest()!=manifest['weights_file_sha256']:raise ValueError('WM weights changed')
            words=np.frombuffer(weights,dtype='<u4')
            self.request_offset=len(words);self.output_offset=len(words)+128
            self.memory=allocate(shape=(len(words)+128+128*42,),dtype=np.uint32)
            self.data=np.asarray(self.memory);self.data[:len(words)]=words;self.memory.flush()
            pointer=next(k for k in self.regs if k in ('memory_offset_1','memory_offset','memory','memory_1'))
            address=int(self.memory.physical_address);reg=self.regs[pointer]['address_offset']
            self.ip.write(reg,address&0xffffffff);self.ip.write(reg+4,address>>32)
            self.return_reg=self.regs['ap_return']['address_offset']
        except Exception:
            self.close();raise

    def execute(self,route,phase_ticks):
        if self.failed or self.memory is None:raise RuntimeError('WM backend unavailable')
        if type(route) is not int or not 0<=route<20 or len(phase_ticks)!=7:
            raise ValueError('WM route and seven phases required')
        if any(type(n) is not int or not 0<=n<=60 for n in phase_ticks):raise ValueError('Invalid phase ticks')
        requests=np.asarray([(route<<16)|(phase<<8)|tick for phase,n in enumerate(phase_ticks) for tick in range(n)],dtype=np.uint32)
        count=len(requests)
        if not 1<=count<=128:raise ValueError('WM plan exceeds synthesized capacity')
        start=time.perf_counter()
        self.data[self.request_offset:self.request_offset+count]=requests
        self.memory[self.request_offset:self.request_offset+count].flush()
        for name,value in dict(reference_offset=0,request_offset=self.request_offset,output_offset=self.output_offset,count=count).items():
            self.control[self.regs[name]['address_offset']//4]=np.uint32(value)
        self.control[0]=np.uint32(1)
        deadline=time.monotonic()+2
        while not int(self.control[0])&2:
            if time.monotonic()>deadline:
                self.failed=True;raise TimeoutError('WM timeout: preserve CMA and overlay locks')
        error=int(self.control[self.return_reg//4])
        if error:raise RuntimeError('WM hardware rejected request: '+str(error))
        self.memory[self.output_offset:self.output_offset+count*42].invalidate()
        out=self.data[self.output_offset:self.output_offset+count*42].copy().reshape(count,14,3)
        steps=out[:,:,0].tolist()
        angles=out[:,:,1:3].copy().view('<f8').reshape(count,14)
        if not np.isfinite(angles).all():raise RuntimeError('Nonfinite WM hardware output')
        self.calls+=1;self.last_ms=(time.perf_counter()-start)*1000
        return steps,angles.tolist()

    def close(self):
        if self.memory is not None:
            if not self.ip.read(0)&4:raise RuntimeError('WM DMA active: keep buffers and locks')
            self.data=None;self.memory.freebuffer();self.memory=None
        self.locks.close()


class ReferenceBatchCore:
    """Host-only reference for adapter tests; never reports hardware execution."""
    def __init__(self,reference):self.reference=reference;self.calls=0;self.last_ms=None
    def execute(self,route,ticks):
        steps=[];angles=[]
        for phase,count in enumerate(ticks):
            for tick in range(count):
                a,b=PynqWmCpuRuntime._decode_steps_and_radians(self.reference,
                    PynqWmCpuRuntime._features(self.reference,route,phase,tick))
                steps.append(a);angles.append(b)
        self.calls+=1;return steps,angles
    def close(self):pass


class AcceleratedWmRuntime(PynqWmCpuRuntime):
    def __init__(self,backend='pl',bitfile=None,expected_sha=None):
        super().__init__(ROOT/'wm_model')
        self.core=WmPLCore(self.root,bitfile,expected_sha) if backend=='pl' else ReferenceBatchCore(self)
        if backend not in ('pl','emulated'):self.core.close();raise ValueError('Explicit backend required')
        self.backend=backend;self.lock=threading.Lock();self.cache=None

    def plan(self,observation):
        with self.lock:
            self.cache=None
            result=super().plan(observation)
            result.update(model='wm',backend='pl_wm_trajectory' if self.backend=='pl' else 'emulated',
                          action_path='checkpoint_reference_head_and_radian_decode',
                          wm_hardware_ms=self.core.last_ms,hardware_commands_sent=False)
            return result

    def _features(self,route,phase,tick):
        if self.cache is None:
            ticks=self.model['phase_min_ticks'][route]
            steps,angles=self.core.execute(route,ticks)
            self.cache={};index=0
            for p,count in enumerate(ticks):
                for t in range(count):
                    self.cache[(route,p,t)]=(steps[index],angles[index]);index+=1
        return (route,phase,tick)

    def _decode_steps_and_radians(self,key):return self.cache[key]
    def close(self):self.core.close()
