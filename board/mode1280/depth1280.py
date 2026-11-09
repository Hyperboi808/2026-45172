"""1280x720 深度服务核心（PS/PL 双路径，与 640 版 depth640.py 同构）。

PL 路径：DFX 1280 档 RM（switch_vision.py --pixel 1280 已加载）——
  preprocess_accel_0（0x40020000）校正 + stereo_rt_accel_0（0x40010000）立体匹配。
  不下载任何位流；初始化校验 /run/vision_overlay_state.json 与位流 sha256。
PS 路径：cv2.remap + cv2.StereoBM（blockSize=5，与 1280 RM 编译参数一致），
  用于对比调试；与板端 PS 深度服务同源标定（board_hd_20261108 直接标定）。

标定：camera_calibration.npz = board_hd_20261108/calibration.npz 原样
（map (720,1280) float32、q_matrix、length_unit='mm'、grid_spacing_mm）。
"""
import hashlib
import json
import os
from contextlib import ExitStack
from pathlib import Path
import threading
import time

import cv2
import numpy as np

from depth_geometry1280 import DepthFilter1280, direct_geometry

ROOT = Path(__file__).resolve().parent
# 板端部署布局：<VISION_ROOT>/mode1280 + <VISION_ROOT>/vision_dfx（与 640 版同约定）
DFX_ROOT = Path(os.environ.get('VISION_ROOT', str(ROOT.parent))) / 'vision_dfx'
STATE_FILE = Path('/run/vision_overlay_state.json')
FPGA_STATE = Path('/sys/class/fpga_manager/fpga0/state')
OVERLAY_LOCK = '/run/lock/pynq-stereo-overlay.lock'
# 与 640 深度服务共用同一把运行时锁：PL 像素档互斥，640/1280 服务不能并存
RUNTIME_LOCK = '/run/lock/pynq-depth640-runtime.lock'


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def _check_dfx_1280(config):
    """1280 RM 在位校验（不下载位流）。"""
    if not STATE_FILE.exists():
        raise RuntimeError('Vision overlay state unknown; run switch_vision.py --full first')
    state = json.loads(STATE_FILE.read_text(encoding='utf-8'))
    if state.get('pixel') != '1280':
        raise RuntimeError('Pixel overlay is %r, not 1280; run switch_vision.py --pixel 1280'
                           % state.get('pixel'))
    if FPGA_STATE.read_text().strip() != 'operating':
        raise RuntimeError('fpga_manager not operating')
    manifest = DFX_ROOT / 'partial_manifest.json'
    if manifest.exists():
        m = json.loads(manifest.read_text(encoding='utf-8'))
        for key, name in (('stereo1280', 'partial_stereo_1280.bit'),
                          ('pre1280', 'partial_pre_1280.bit')):
            bit = DFX_ROOT / name
            if not bit.is_file():
                raise RuntimeError('Partial bitstream missing: ' + name)
            if _sha256(bit) != m.get(key):
                raise RuntimeError('sha256 mismatch for ' + name + ' (manifest)')
    else:
        raise RuntimeError('partial_manifest.json missing under vision_dfx')


class Depth1280:
    def __init__(self, backend='pl', *, assets=None, lr_check=True):
        if backend not in ('ps', 'pl'):
            raise ValueError('backend must be ps or pl')
        self.backend = backend
        self.root = Path(assets or ROOT)
        self.locks = ExitStack()
        self.lr_check = lr_check  # False 时跳过右目匹配（省一次 PL 匹配，帧率近翻倍）
        self.pre = None
        self.bm = None
        self.last_frame_id = None
        self.frame_lock = threading.Lock()
        self.config = json.loads((self.root / 'config1280.json').read_text(encoding='utf-8'))
        self.params = self.config['matching']
        fixed = dict(num_disparities=64, block_size=5, min_disparity=0, pre_filter_cap=31,
                     pre_filter_type='XSOBEL', lr_tolerance_q4=16, speckle_window_size=40,
                     speckle_range_q4=16)
        if any(self.params.get(k) != v for k, v in fixed.items()):
            raise ValueError('Configuration does not match the compiled PS/PL parameter contract')
        calib = self.root / 'camera_calibration.npz'
        if _sha256(calib) != self.config['calibration_sha256']:
            raise RuntimeError('Calibration changed')
        self.maps, self.bounds, self.q, self.scale = direct_geometry(calib)
        self.filter = DepthFilter1280((720, 1280), self.bounds[1], self.q, self.scale)
        if backend == 'ps':
            p = self.params
            self.cpu = cv2.StereoBM_create(numDisparities=p['num_disparities'],
                                           blockSize=p['block_size'])
            self.cpu.setMinDisparity(0)
            self.cpu.setPreFilterType(cv2.STEREO_BM_PREFILTER_XSOBEL)
            self.cpu.setPreFilterCap(p['pre_filter_cap'])
            self.cpu.setTextureThreshold(p['texture_threshold'])
            self.cpu.setUniquenessRatio(p['uniqueness_ratio'])
            self.cpu.setSpeckleWindowSize(0)
            self.cpu.setDisp12MaxDiff(-1)
        else:
            import fcntl
            try:
                _check_dfx_1280(self.config)
                for path, mode in ((OVERLAY_LOCK, fcntl.LOCK_SH),
                                   (RUNTIME_LOCK, fcntl.LOCK_EX)):
                    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
                    handle = self.locks.enter_context(os.fdopen(fd, 'rb'))
                    fcntl.flock(handle, mode | fcntl.LOCK_NB)
                from fpga_backend1280 import Preprocess1280, StereoPL1280
                self.bm = StereoPL1280(1280, 720)
                self.pre = Preprocess1280(self.root)
                for name, value in [('uniqueness', self.params['uniqueness_ratio']),
                                    ('texture', self.params['texture_threshold']),
                                    ('tolerance', self.params['support_tolerance_q4']),
                                    ('min_support', self.params['min_support'])]:
                    self.bm.write(name, value)
            except Exception:
                self.close()
                raise

    def _match_cpu(self, left, right):
        raw = self.cpu.compute(left, right)
        return np.where(raw > 0, raw, 0).astype(np.uint16)

    def process(self, packed_gray, *, frame_id, receipt_s=None):
        if not self.frame_lock.acquire(blocking=False):
            raise RuntimeError('One frame at a time')
        try:
            return self._process(packed_gray, frame_id=frame_id, receipt_s=receipt_s)
        finally:
            self.frame_lock.release()

    def _process(self, packed_gray, *, frame_id, receipt_s=None):
        if packed_gray.shape != (720, 2560) or packed_gray.dtype != np.uint8:
            raise ValueError('Packed grayscale 2560x720 (each eye 1280x720) required')
        if type(frame_id) is not int or not 0 <= frame_id <= 0xffffffff:
            raise ValueError('uint32 frame ID required')
        if self.last_frame_id is not None and frame_id <= self.last_frame_id:
            raise ValueError('Increasing frame IDs required')
        self.last_frame_id = frame_id
        begin = time.perf_counter()
        if self.backend == 'pl':
            (left, right), _ = self.pre.run(packed_gray)
            dl, _ = self.bm.run(left, right, self.bounds[0])
            dr = None
            if self.lr_check:
                dr, _ = self.bm.run(np.ascontiguousarray(right[:, ::-1]),
                                    np.ascontiguousarray(left[:, ::-1]),
                                    np.ascontiguousarray(self.bounds[1][:, ::-1]))
        else:
            left, right = [cv2.remap(np.ascontiguousarray(packed_gray[:, i * 1280:(i + 1) * 1280]),
                                     *self.maps[i], cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_CONSTANT) for i in range(2)]
            dl = self._match_cpu(left, right)
            dr = None
            if self.lr_check:
                dr = self._match_cpu(np.ascontiguousarray(right[:, ::-1]),
                                     np.ascontiguousarray(left[:, ::-1]))
        if dr is not None:
            disparity = self.filter.filter(dl, dr[:, ::-1])
        else:
            disparity = self.filter.filter(dl, None)  # 无右目：跳过 LR 一致性
        depth = self.filter.depth_m(disparity)
        age = None if receipt_s is None else (time.monotonic() - receipt_s) * 1000
        return dict(frame_id=frame_id, left=left, right=right, disparity=disparity,
                    depth_m=depth,
                    metadata=dict(backend=self.backend, per_eye_resolution=[1280, 720],
                                  resize=False,
                                  processing_ms=(time.perf_counter() - begin) * 1000,
                                  receipt_age_ms=age,
                                  valid_pixels=int(np.isfinite(depth).sum()),
                                  uart=False, pointcloud=False,
                                  depth_unit='metre', depth_axis='rectified_left_camera_Z',
                                  robot_control_valid=False, model_connected=False))

    def close(self):
        if self.frame_lock.locked():
            raise RuntimeError('Frame processing active; preserve buffers')
        drivers = [d for d in (self.pre, self.bm) if d is not None]
        if any(not d.regs.read(0) & 4 for d in drivers):
            raise RuntimeError('Keep buffers and locks while hardware is active')
        for d in drivers:
            d.close()
        self.pre = None
        self.bm = None
        self.locks.close()
