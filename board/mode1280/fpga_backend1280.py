"""DFX 1280 档 PL 驱动（mmap 版，不下载位流、不依赖 pynq Overlay）。

位流由 switch_vision.py --pixel 1280 加载（fpga_manager partial + 复位脉冲），
本模块只做 CMA 缓冲分配（pynq xlnk）与 /dev/mem MMIO 访问。

寄存器偏移来自 Vitis HLS 2025.2 生成的 x*_hw.h（本机实测提取）：
  stereo_rt_accel_0   base 0x40010000（dfx_vision.tcl assign_bd_address）
  preprocess_accel_0  base 0x40020000
"""
import os
import struct
import time

import numpy as np

STEREO_BASE = 0x40010000
PREPROCESS_BASE = 0x40020000

# s_axilite 偏移（Vitis HLS 标准布局 + 64bit 指针的 reserved 间隙）
ST_AP_CTRL = 0x00
ST_IMG_IN_L = 0x10
ST_IMG_IN_R = 0x1C
ST_VALID_MASK = 0x28
ST_IMG_OUT = 0x34
ST_ROWS = 0x40
ST_COLS = 0x48
ST_UNIQUENESS = 0x50
ST_TEXTURE = 0x58
ST_TOLERANCE = 0x60
ST_MIN_SUPPORT = 0x68

PR_AP_CTRL = 0x00
PR_SOURCE = 0x10
PR_MAPS = 0x1C
PR_OUTPUT = 0x28
PR_EYE = 0x34

AP_START = 1
AP_DONE = 2
AP_IDLE = 4


class MMapReg:
    """/dev/mem 32bit MMIO（与 switch_vision.py 的 LED 访问同风格）。"""

    def __init__(self, base, size=0x1000):
        import mmap as _mmap
        self._fd = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
        self._mm = _mmap.mmap(self._fd, size, offset=base)
        self._base = base

    def read(self, offset):
        return struct.unpack('<I', self._mm[offset:offset + 4])[0]

    def write(self, offset, value):
        self._mm[offset:offset + 4] = struct.pack('<I', int(value) & 0xFFFFFFFF)

    def write64(self, offset, value):
        self.write(offset, int(value) & 0xFFFFFFFF)
        self.write(offset + 4, (int(value) >> 32) & 0xFFFFFFFF)

    def close(self):
        self._mm.close()
        os.close(self._fd)


def _check_idle(regs, base, what):
    if not regs.read(base + 0x00) & AP_IDLE:
        raise RuntimeError(what + ' not idle; run switch_vision.py first or reset the board')


def _run_kernel(regs, base, what, timeout_s=5):
    regs.write(base + 0x00, AP_START)
    start = time.monotonic()
    while not regs.read(base + 0x00) & AP_DONE:
        if time.monotonic() - start > timeout_s:
            raise TimeoutError(what + ' did not finish; stop testing and reset the board')
        time.sleep(0.00005)


class StereoPL1280:
    """1280x720 StereoBM RM（STEREO_D=64, WSIZE=5, 16 路并行）。

    输入：左/右校正图 (720,1280) uint8 + valid_mask (720,1280) uint8
    输出：左视差 (720,1280) uint16（Q12.4，/16 为像素视差）
    """

    def __init__(self, w=1280, h=720):
        if (w, h) != (1280, 720):
            raise ValueError('This RM is fixed at 1280x720')
        self.w, self.h = w, h
        from pynq import allocate
        self.regs = MMapReg(STEREO_BASE)
        _check_idle(self.regs, 0, 'stereo_rt_accel_0')
        self.left = allocate((h, w), dtype=np.uint8)
        self.right = allocate((h, w), dtype=np.uint8)
        self.mask = allocate((h, w), dtype=np.uint8)
        self.out = allocate((h, w), dtype=np.uint16)
        self.buffers = [self.left, self.right, self.mask, self.out]
        for off, buf in ((ST_IMG_IN_L, self.left), (ST_IMG_IN_R, self.right),
                         (ST_VALID_MASK, self.mask), (ST_IMG_OUT, self.out)):
            self.regs.write64(off, buf.device_address)
        for off, value in ((ST_ROWS, h), (ST_COLS, w)):
            self.regs.write(off, value)

    def write(self, name, value):
        off = {'uniqueness': ST_UNIQUENESS, 'texture': ST_TEXTURE,
               'tolerance': ST_TOLERANCE, 'min_support': ST_MIN_SUPPORT}[name]
        self.regs.write(off, int(value) & 0xFFFFFFFF)

    def run(self, left, right, mask):
        for image in (left, right, mask):
            if image.shape != (self.h, self.w) or image.dtype != np.uint8:
                raise ValueError('PL expects shape-matched grayscale uint8 inputs')
        t = time.perf_counter()
        np.copyto(self.left, left)
        np.copyto(self.right, right)
        np.copyto(self.mask, mask)
        self.mask.sync_to_device()
        self.left.sync_to_device()
        self.right.sync_to_device()
        start = time.perf_counter()
        _run_kernel(self.regs, 0, 'stereo_rt_accel_0')
        done = time.perf_counter()
        self.out.sync_from_device()
        out = np.array(self.out, copy=True)
        return out, {'pl_host_bound_ms': (done - start) * 1000,
                     'pl_with_ps_copy_ms': (time.perf_counter() - t) * 1000}

    def close(self):
        _check_idle(self.regs, 0, 'stereo_rt_accel_0')
        for buf in self.buffers:
            buf.freebuffer()
        self.regs.close()


class Preprocess1280:
    """1280x720 双目标定校正 RM：packed 2560x720 -> 左右校正图。

    map 条目：921600 像素 + 90 tile header（uint64，TILE=8/MAX_ROWS=72）。
    """

    def __init__(self, root, w=1280, h=720):
        from pynq import allocate
        self.w, self.h = w, h
        self.regs = MMapReg(PREPROCESS_BASE)
        _check_idle(self.regs, 0, 'preprocess_accel_0')
        self.buffers = []
        try:
            self.source = allocate((h, 2 * w), np.uint8)
            self.buffers.append(self.source)
            self.maps = []
            self.outputs = []
            for eye in range(2):
                m = np.fromfile(os.path.join(root, 'map_%d.bin' % eye), np.uint64)
                if m.shape != (921690,) or np.any(m[:921600] >> 48):
                    raise ValueError('Unexpected native map format')
                xy = m[:921600].reshape(h, w)
                sx = (xy & 65535).astype(np.uint16).view(np.int16)
                sy = ((xy >> 16) & 65535).astype(np.uint16).view(np.int16)
                if np.any(((xy >> 32) & 65535) >= 1024):
                    raise ValueError('Invalid fixed bilinear coefficient')
                for tile, header in enumerate(m[921600:]):
                    first = int(header) & 65535
                    count = (int(header) >> 16) & 65535
                    if not 1 <= count <= 72 or first + count > h:
                        raise ValueError('Invalid source stripe')
                    a = sx[tile * 8:tile * 8 + 8]
                    b = sy[tile * 8:tile * 8 + 8]
                    relevant = (a >= -1) & (a < w) & (b >= -1) & (b < h)
                    for offset in (0, 1):
                        ys = b.astype(np.int32) + offset
                        valid = relevant & (ys >= 0) & (ys < h)
                        if np.any(valid & ((ys < first) | (ys >= first + count))):
                            raise ValueError('Map exceeds source cache stripe')
                buf = allocate(m.shape, np.uint64)
                self.buffers.append(buf)
                buf[:] = m
                buf.sync_to_device()
                self.maps.append(buf)
                output = allocate((h, w), np.uint8)
                self.buffers.append(output)
                self.outputs.append(output)
            self.regs.write64(PR_SOURCE, self.source.device_address)
        except Exception:
            for b in self.buffers:
                b.freebuffer()
            self.regs.close()
            raise

    def run(self, image):
        if image.shape != (self.h, 2 * self.w) or image.dtype != np.uint8:
            raise ValueError('Native packed grayscale 2560x720 required')
        _check_idle(self.regs, 0, 'preprocess_accel_0')
        began = time.perf_counter()
        self.source[:] = image
        self.source.sync_to_device()
        result = []
        for eye in range(2):
            self.regs.write(PR_EYE, eye)
            self.regs.write64(PR_MAPS, self.maps[eye].device_address)
            self.regs.write64(PR_OUTPUT, self.outputs[eye].device_address)
            start = time.monotonic()
            _run_kernel(self.regs, 0, 'preprocess_accel_0', timeout_s=8)
            self.outputs[eye].sync_from_device()
            result.append(np.array(self.outputs[eye], copy=True))
        return tuple(result), dict(processing_ms=(time.perf_counter() - began) * 1000)

    def close(self):
        _check_idle(self.regs, 0, 'preprocess_accel_0')
        for b in self.buffers:
            b.freebuffer()
        self.regs.close()
