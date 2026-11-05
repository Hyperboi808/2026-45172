"""v11 FP32 actor: all 11 Dense layers on PL; original PS pooling/controller.

The 'emulated' backend reproduces the HLS reduction order for host regression.
It is explicitly not a hardware execution or performance measurement.
"""
from contextlib import ExitStack
from array import array
from itertools import chain
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time
from types import FunctionType
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'model'))
from piano_model.actor import PianoActor, f32, OBSERVATION_SHAPES
from run import V11Runtime


class EmulatedDense:
    def __init__(self, weights, manifest):
        self.weights, self.manifest = weights, manifest

    def run(self, name, x, relu=False):
        w = self.manifest['tensors'][name+'.weight']
        b = self.manifest['tensors'][name+'.bias']
        matrix = self.weights[w['offset_bytes']//4:(w['offset_bytes']+w['length_bytes'])//4].reshape(w['shape'])
        bias = self.weights[b['offset_bytes']//4:(b['offset_bytes']+b['length_bytes'])//4]
        sums = np.zeros((len(x), len(bias), 8), np.float32)
        for k in range(x.shape[1]):
            product = np.asarray(x[:, k, None] * matrix[None, :, k], np.float32)
            sums[:, :, k & 7] = sums[:, :, k & 7] + product
        result = np.broadcast_to(bias, (len(x), len(bias))).copy()
        for k in range(8):
            result = result + sums[:, :, k]
        if relu:
            np.maximum(result, np.float32(0), out=result)
        return result

    def close(self):
        pass


class PLDense:
    """Attach to the exact resident full overlay; never download from inference."""
    def __init__(self, weights, manifest, bitfile, expected_sha):
        import fcntl
        import os
        from pynq import Overlay, PL, allocate, Clocks
        self.locks = ExitStack()
        self.memory = None
        self.failed = False
        try:
            for path, mode in (('/run/lock/pynq-stereo-overlay.lock', fcntl.LOCK_SH),
                               ('/run/lock/pynq-fp32-dense.lock', fcntl.LOCK_EX)):
                handle = self.locks.enter_context(os.fdopen(os.open(path, os.O_RDONLY|os.O_NOFOLLOW), 'rb'))
                fcntl.flock(handle, mode|fcntl.LOCK_NB)
            active = Path(str(PL.bitfile_name))
            if not expected_sha or active.resolve() != Path(bitfile).resolve() or hashlib.sha256(active.read_bytes()).hexdigest() != expected_sha:
                raise RuntimeError('Select this package 320 overlay first')
            if not 99.99 < float(Clocks.fclk0_mhz) < 100.01:
                raise RuntimeError('Dense/UART requires 100 MHz')
            self.overlay = Overlay(str(active), download=False)
            self.ip = self.overlay.dense_fp32_0
            info = self.overlay.ip_dict['dense_fp32_0']
            if info['phys_addr'] != 0x40040000 or not self.ip.read(0) & 4:
                raise RuntimeError('Dense core missing or busy')
            self.regs = info['registers']
            self.manifest = manifest
            self.memory = allocate(shape=(len(weights)+16384,), dtype=np.float32)
            self.memory[:len(weights)] = weights
            self.memory.flush()
            self.memory_data = np.asarray(self.memory)
            self.control = self.ip.mmio.array
            self.argument_names = ('input_offset','weight_offset','bias_offset','output_offset',
                                   'batch','inputs','outputs','relu')
            self.argument_indices = np.asarray([self.regs[k]['address_offset']//4
                                               for k in self.argument_names],dtype=np.intp)
            self.configuration_cache = {}
            self.sync_views = {}
            self.input_offset, self.output_offset = len(weights), len(weights)+8192
            pointer = next(k for k in self.regs if k in ('memory_offset_1','memory_offset','memory','memory_1'))
            address = int(self.memory.physical_address)
            reg = self.regs[pointer]['address_offset']
            self.ip.write(reg, address & 0xffffffff)
            self.ip.write(reg+4, address >> 32)
        except Exception:
            self.close()
            raise

    def run(self, name, x, relu=False):
        try:
            return self._execute(name,x,relu)
        except Exception:
            self.failed = True
            raise

    def _execute(self, name, x, relu=False):
        if self.failed or self.memory is None:
            raise RuntimeError('Dense backend unavailable; explicit recovery required')
        batch, inputs = x.shape
        w, b = (self.manifest['tensors'][name+s] for s in ('.weight','.bias'))
        outputs = w['shape'][0]
        if w['shape'][1] != inputs or not (0 < batch <= 256 and 0 < inputs <= 288 and 0 < outputs <= 256) or max(batch*inputs,batch*outputs)>8192:
            raise ValueError('Dense shape exceeds synthesized limits')
        key=(name,batch,inputs,bool(relu))
        if key not in self.configuration_cache:
            self.configuration_cache[key]=np.asarray((self.input_offset,w['offset_bytes']//4,
                b['offset_bytes']//4,self.output_offset,batch,inputs,outputs,int(relu)),dtype=np.uint32)
            self.sync_views[key]=(self.memory[self.input_offset:self.input_offset+x.size],
                                  self.memory[self.output_offset:self.output_offset+batch*outputs])
        input_view,output_view=self.sync_views[key]
        self.memory_data[self.input_offset:self.input_offset+x.size] = x.ravel()
        input_view.flush()
        # MMIO uses the same mapped uint32 array as DefaultIP.write. Scatter only
        # the named argument registers; start is a separate, subsequent write.
        self.control[self.argument_indices]=self.configuration_cache[key]
        self.control[0]=np.uint32(1)
        deadline = time.monotonic()+2
        while not int(self.control[0]) & 2:
            if time.monotonic() > deadline:
                self.failed = True
                raise TimeoutError('PL timeout: retain DMA buffer and locks until core is idle')
        output_view.invalidate()
        return self.memory_data[self.output_offset:self.output_offset+batch*outputs].copy().reshape(batch,outputs)

    def close(self):
        if self.memory is not None:
            if not self.ip.read(0) & 4:
                raise RuntimeError('Core active: keep this owner alive; do not free DMA buffers')
            if hasattr(self,'sync_views'):self.sync_views.clear()
            self.memory_data=None
            self.memory.freebuffer()
            self.memory = None
        self.locks.close()


class DenseLayer:
    def __init__(self, core, name):
        self.core, self.name = core, name

    def batch_array(self, x):
        result = self.core.run(self.name, np.ascontiguousarray(x, dtype=np.float32), False)
        if not np.isfinite(result).all():
            raise ValueError('Nonfinite PL activation')
        return result

    def batch(self, x):
        return self.batch_array(x).tolist()

    def __call__(self, values):
        return self.batch([values])[0]


class AcceleratedActor(PianoActor):
    def __init__(self, directory=None, backend='pl', bitfile=None, expected_sha=None):
        super().__init__(directory or ROOT/'model')
        weights = np.frombuffer((self.directory/'weights_fp32.bin').read_bytes(), dtype='<f4').copy()
        if backend == 'pl':
            self.core = PLDense(weights, self.manifest, bitfile, expected_sha)
        elif backend == 'emulated':
            self.core = EmulatedDense(weights, self.manifest)
        else:
            raise ValueError('Choose pl or emulated explicitly')
        self.layers = {name:DenseLayer(self.core,name) for name in self.layers}
        self.lock = threading.Lock()
        self.backend = backend

    def _pointnet(self, points, name, features):
        first = self.layers['features_extractor.'+name+'.mlp.0']
        second = self.layers['features_extractor.'+name+'.mlp.2']
        points=np.asarray(points,dtype=np.float32)
        intermediate=first.batch_array(points[:,:3])
        np.maximum(intermediate,np.float32(0),out=intermediate)
        encoded=second.batch_array(intermediate)
        np.maximum(encoded,np.float32(0),out=encoded)
        confidence=np.clip(points[:,3],np.float32(0),np.float32(1))
        weighted=np.asarray(encoded*confidence[:,None],dtype=np.float32)
        # cumsum retains sequential FP32 additions along the point dimension;
        # np.sum may select pairwise reduction and change the original order.
        confidence_sum=float(np.cumsum(confidence,dtype=np.float32)[-1])
        sums=np.cumsum(weighted,axis=0,dtype=np.float32)[-1]
        maximum=np.max(weighted,axis=0)
        denominator=max(1.0,confidence_sum)
        quality = f32(confidence_sum/len(points))
        mean=np.asarray(sums.astype(np.float64)/denominator,dtype=np.float32)
        return np.asarray(np.concatenate((mean,maximum)).astype(np.float64)*quality,dtype=np.float32)

    @staticmethod
    def validate_arrays(observation):
        if not isinstance(observation,dict) or set(observation)!=set(OBSERVATION_SHAPES):
            raise ValueError('Provide all four original observation tensors')
        result={}
        for name,shape in OBSERVATION_SHAPES.items():
            value=observation[name]
            rows=[value] if len(shape)==1 else value
            if not isinstance(value,(list,tuple,array)) or len(value)!=shape[0]:
                raise ValueError('Invalid observation shape: '+name)
            if len(shape)==2 and not isinstance(value,(list,tuple)):
                raise ValueError('Point clouds must be row sequences')
            width=shape[-1]
            for row in rows:
                if not isinstance(row,(list,tuple,array)) or len(row)!=width:
                    raise ValueError('Invalid observation row: '+name)
            # Fast path for ordinary JSON numbers. Keep the original isinstance
            # rules for numeric subclasses, while still rejecting mixed bools.
            if not set(map(type,chain.from_iterable(rows))).issubset((int,float)):
                if any(isinstance(x,bool) or not isinstance(x,(int,float)) for row in rows for x in row):
                    raise ValueError('Finite numeric values required: '+name)
            try:
                with np.errstate(over='ignore',invalid='ignore'):
                    data=np.ascontiguousarray(value,dtype=np.float32)
            except (ValueError,TypeError,OverflowError) as error:
                raise ValueError('Invalid float32 observation: '+name) from error
            if data.shape!=shape or not np.isfinite(data).all():
                raise ValueError('Finite float32 observation required: '+name)
            result[name]=data
        return result

    def _linear(self,name,value,relu=False):
        result=self.layers[name].batch_array(np.asarray(value,dtype=np.float32).reshape(1,-1))[0]
        if relu:np.maximum(result,np.float32(0),out=result)
        return result

    def predict(self, observation, clip=True):
        with self.lock:
            start = time.perf_counter()
            obs=self.validate_arrays(observation)
            feedback=self._linear('features_extractor.feedback.0',obs['proprio'],True)
            feedback=self._linear('features_extractor.feedback.2',feedback,True)
            cloud=self._pointnet(obs['pointcloud'],'cloud',32)
            target=self._pointnet(obs['goal_cloud'],'target_cloud',16)
            goal=self._linear('features_extractor.goal.0',obs['goal'],True)
            fused=self._linear('features_extractor.fusion.0',np.concatenate((feedback,cloud,target,goal)),True)
            hidden=np.tanh(self._linear('mlp_extractor.policy_net.0',fused).astype(np.float64)).astype(np.float32)
            hidden=np.tanh(self._linear('mlp_extractor.policy_net.2',hidden).astype(np.float64)).astype(np.float32)
            result=self._linear('action_net',hidden)
            if clip:result=np.clip(result,np.float32(-1),np.float32(1))
            result=result.tolist()
            self.last_ms = (time.perf_counter()-start)*1000
            return result

    def close(self):
        self.core.close()


def _fast_observation_lists(observation):
    return {name:value.tolist() for name,value in AcceleratedActor.validate_arrays(observation).items()}


# Reuse original controller bytecode, replacing only its observation validator.
# The phase logic, filter, limits and dt checks are the unchanged original code;
# the entire observation is still validated on every controller call.
_CONTROL_APPLY=FunctionType(V11Runtime.apply.__code__,
    dict(V11Runtime.apply.__globals__,validate_observation=_fast_observation_lists),
    name='apply',argdefs=V11Runtime.apply.__defaults__)


class AcceleratedRuntime(V11Runtime):
    apply=_CONTROL_APPLY

    def __init__(self, backend='pl', bitfile=None, expected_sha=None):
        super().__init__(ROOT/'model')
        self.actor = AcceleratedActor(self.directory,backend,bitfile,expected_sha)

    def close(self):
        self.actor.close()
