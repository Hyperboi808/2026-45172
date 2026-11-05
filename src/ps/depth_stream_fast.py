#!/usr/bin/env python3
"""640 深度图裸 TCP 流服务（最快传输路径，无 HTTP/npy 开销）。

目标：640×480 深度图以最快速度传到 PC。
  - 载荷：uint16 LE 毫米（round(depth_m*1000)，无效/NaN=0），614400 B/帧
  - 传输：裸 TCP 长度前缀帧，无 HTTP 头、无 JPEG、无彩色 remap
  - 流水线：采集+PL 匹配（~700ms，瓶颈）与编码+发送（~50ms）完全并行，
    传输时间被下一帧 PL 匹配隐藏，帧率 ≈ 1/PL匹配时间

帧协议（小端）：
  magic 4B "PYND" | seq u32 | width u32 | height u32 | payload_len u32 | payload
  文本命令（行，\n 结尾）："status" → 回一行 JSON 后继续流；"quit" → 断开

只读视觉前端（需 640 模式 depth640.bit 已在位，同 depth_server640.py）：
  不下载 overlay、不碰舵机 UART。标定来自 VISION_ROOT（默认 robot-models v11）。

深度语义与 depth_server640.py 的 /depth_m.npy 完全一致：
  左目对齐米制深度（校正左相机 Z 轴），float32 米 → uint16 毫米，1mm 精度。

板上运行（root）：
  sudo VISION_ROOT=/usr/local/lib/pynq-robot-models-v11-20261102-01 \
      /usr/local/share/pynq-venv/bin/python3 /home/xilinx/depth_stream_fast.py \
      --port 8099

PC 端客户端：控制平台/深度图查看/depth_client.py
"""
import argparse
import json
import os
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

# 必须在 import pynq / cv2 之前设置（XRT 设备查找）。
os.environ["XILINX_XRT"] = "/usr"
os.environ["OPENBLAS_NUM_THREADS"] = "1"

ROOT = os.environ.get("VISION_ROOT", "/usr/local/lib/pynq-robot-models-v11-20261102-01")
sys.path.insert(0, ROOT + "/mode640")

import cv2
import numpy as np

from depth640 import Depth640

MAGIC = b"PYND"
HDR = struct.Struct("<4sIIII")
MAX_CLIENTS = 8

STOP = threading.Event()
CV = threading.Condition()
LATEST = {"seq": 0, "payload": b"", "width": 640, "height": 480}
STATE = {
    "error": "starting",
    "frames": 0,
    "processing_ms": 0.0,   # 采集解码+PL 匹配+过滤（最近帧）
    "encode_ms": 0.0,
    "fps": 0.0,
    "width": 640,
    "height": 480,
    "valid_depth_pixels": 0,
    "depth_unit": "millimetre",
    "depth_axis": "rectified_left_camera_Z",
    "protocol": "PYND uint16 LE mm",
}
_CLIENTS = set()


def open_camera():
    """同 depth_server640.capture()：身份校验 + MJPG 1280x480 + 预热。"""
    info = subprocess.check_output(
        ["udevadm", "info", "--query=property", "--name=/dev/video0"], text=True
    )
    if "ID_VENDOR_ID=1bcf" not in info or "ID_MODEL_ID=0b15" not in info:
        raise RuntimeError("Camera identity changed")
    if subprocess.run(["fuser", "/dev/video0"], capture_output=True, text=True).stdout.strip():
        raise RuntimeError("Camera already in use")
    cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
    for prop, value in [
        (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG")),
        (cv2.CAP_PROP_FRAME_WIDTH, 1280),
        (cv2.CAP_PROP_FRAME_HEIGHT, 480),
        (cv2.CAP_PROP_FPS, 30),
        (cv2.CAP_PROP_BUFFERSIZE, 2),
        (cv2.CAP_PROP_CONVERT_RGB, 0),
    ]:
        if not cap.set(prop, value):
            raise RuntimeError("Camera mode setting failed")
    if not cap.isOpened():
        raise RuntimeError("Camera open failed")
    for _ in range(30):
        if not cap.read()[0]:
            raise RuntimeError("Camera warmup failed")
    return cap


def capture_and_process():
    """线程 1：读帧（JPEG 软解）→ PL 校正 → PL 立体匹配 → 过滤 → uint16 编码。

    与发送线程并行；Depth640 内部 frame_lock 保证同刻只有一帧在 PL。
    """
    t0 = time.monotonic()
    try:
        cap = open_camera()
        vision = Depth640(backend="pl", lr_check=False)
    except Exception as exc:
        with CV:
            STATE["error"] = str(exc)
        STOP.set()
        return
    try:
        while not STOP.is_set():
            ok, raw = cap.read()
            receipt = time.monotonic()
            if not ok:
                raise RuntimeError("Camera frame read failed")
            jpeg = raw.reshape(-1).tobytes()
            if not jpeg.startswith(b"\xff\xd8"):
                raise RuntimeError("Native JPEG required")
            seq = STATE["frames"] + 1
            # 灰度解码直接得到 PL 需要的 packed_gray（省 COLOR 解码 + cvtColor）
            packed_gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_GRAYSCALE)
            if packed_gray is None or packed_gray.shape != (480, 1280):
                raise RuntimeError("Wrong decoded shape")
            res = vision.process(packed_gray, frame_id=seq, receipt_s=receipt)
            depth_m = res["depth_m"]
            # float32 米 → uint16 毫米（1mm 精度，范围 0-65m；NaN/无效=0）
            mm = np.rint(depth_m * 1000.0)
            valid = np.isfinite(depth_m) & (depth_m > 0)
            mm_u16 = np.zeros(depth_m.shape, np.uint16)
            np.copyto(mm_u16, mm.astype(np.uint16), where=valid)
            t_enc = time.monotonic()
            payload = mm_u16.astype("<u2", copy=False).tobytes()
            encode_ms = (time.monotonic() - t_enc) * 1000
            with CV:
                LATEST.update(seq=seq, payload=payload, width=640, height=480)
                STATE.update(
                    frames=seq,
                    processing_ms=res["metadata"]["processing_ms"],
                    encode_ms=encode_ms,
                    valid_depth_pixels=res["metadata"]["valid_pixels"],
                    fps=seq / (time.monotonic() - t0),
                )
                CV.notify_all()
    except Exception as exc:
        with CV:
            STATE["error"] = str(exc)
        STOP.set()
    finally:
        cap.release()
        vision.close()


def _send_all(sock, data):
    view = memoryview(data)
    while view:
        n = sock.send(view)
        if n <= 0:
            raise ConnectionError
        view = view[n:]


def serve_client(conn, addr):
    """线程 2（每客户端一条）：先收命令，默认流模式持续推最新帧。"""
    conn.settimeout(None)
    _CLIENTS.add(conn)
    buf = b""
    streaming = True
    last_seq = 0
    try:
        while not STOP.is_set():
            if streaming:
                with CV:
                    CV.wait_for(lambda: LATEST["seq"] != last_seq or STOP.is_set(), timeout=0.5)
                    seq, payload, w, h = LATEST["seq"], LATEST["payload"], LATEST["width"], LATEST["height"]
                if seq != last_seq:
                    hdr = HDR.pack(MAGIC, seq, w, h, len(payload))
                    _send_all(conn, hdr)
                    _send_all(conn, payload)
                    last_seq = seq
                    # 顺便收命令（非阻塞，一次最多 1KB）
                    conn.settimeout(0.0)
                    try:
                        buf += conn.recv(4096)
                    except (BlockingIOError, socket.timeout):
                        pass
                    finally:
                        conn.settimeout(None)
                    if b"\n" in buf:
                        line, _, buf = buf.partition(b"\n")
                        cmd = line.strip().lower()
                        if cmd.startswith(b"status"):
                            with CV:
                                st = json.dumps(STATE).encode("utf-8") + b"\n"
                            _send_all(conn, st)
                        elif cmd.startswith(b"quit"):
                            break
            else:
                conn.settimeout(1.0)
                try:
                    buf += conn.recv(4096)
                except socket.timeout:
                    continue
                if not buf:
                    break
                if b"\n" in buf:
                    line, _, buf = buf.partition(b"\n")
                    cmd = line.strip().lower()
                    if cmd.startswith(b"stream"):
                        streaming = True
                    elif cmd.startswith(b"quit"):
                        break
    except (ConnectionError, BrokenPipeError, ConnectionResetError, OSError):
        pass
    finally:
        _CLIENTS.discard(conn)
        try:
            conn.close()
        except OSError:
            pass


def tcp_server(port):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(MAX_CLIENTS)
    srv.settimeout(0.5)
    while not STOP.is_set():
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        threading.Thread(target=serve_client, args=(conn, addr), daemon=True).start()
    srv.close()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", type=int, default=8099)
    p.add_argument("--lr-check", action="store_true", help="开启左右一致性检查（慢 ~350ms，默认关）")
    args = p.parse_args()

    if os.geteuid() != 0:
        raise SystemExit("Root required (camera + PL MMIO)")

    signal.signal(signal.SIGTERM, lambda *_: STOP.set())
    signal.signal(signal.SIGINT, lambda *_: STOP.set())

    t_cap = threading.Thread(target=capture_and_process, daemon=True)
    t_srv = threading.Thread(target=tcp_server, args=(args.port,), daemon=True)
    t_cap.start()
    t_srv.start()
    print("depth_stream_fast listening on :%d (PYND uint16 mm)" % args.port, flush=True)
    while not STOP.is_set():
        STOP.wait(2.0)
        with CV:
            st = dict(STATE)
        print("frames=%d fps=%.2f proc_ms=%.0f enc_ms=%.0f valid=%d err=%s" % (
            st["frames"], st["fps"], st["processing_ms"], st["encode_ms"],
            st["valid_depth_pixels"], st["error"]), flush=True)
    print("stopping", flush=True)


if __name__ == "__main__":
    main()
