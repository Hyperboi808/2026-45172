#!/usr/bin/env python3
"""无 pynq 依赖的四-overlay DFX 快加载器（3 像素档 + UART）。

四个可独立快速加载/卸载的 overlay（对应 4 个物理 LED）:
  - 像素 320 / 640 / 1280（互斥；stereo + preprocess 两个独立 RP，均含深度 + 点云）
  - UART（独立 RP，舵机协议；可与任一像素档并存）

整片位流 vision_base.bit = 静态 + 640 像素 + UART。部分位流从同一
config_full 静态导出（static region 一致），fpga_manager flags=1 编程。

LED 映射（status_led[3:0]，与硬件 status_led.xdc 一致）:
  LED[0]=320 已加载  LED[1]=640 已加载  LED[2]=1280 已加载  LED[3]=UART 已加载
  整片默认 = 640 + UART = 0b1010 = 0xA。

机制（复刻 PYNQ 3.1.1 embedded_device.py + 旧 switch_fast.py 验证版）:
  1. .bit 头解析 + data 按 int32 字节交换 -> .bin -> /lib/firmware
  2. echo 0|1 > fpga0/flags（0=整片, 1=部分）; echo <stem>.bin > fpga0/firmware
  3. 轮询 state == operating（10s 超时）
  4. 部分位流加载后脉冲复位（写 0 -> 2ms -> 写 1）model_rst_gpio(0x40050000)，
     清 DFX 期间 m_axi 翻转导致的 smartconnect 死锁（UG909）。
  5. sha256 校验（partial_manifest.json；缺失时跳过并警告）。

安全规则（沿用板端验收结论）:
  - blank 卸载后绝不读该 RP 的地址（stub 握手虽拉高返回 0，仍保守不读）。
  - 整片/部分编程期间持 overlay 锁，失败立即拒绝。
  - 状态记录在 /run/vision_overlay_state.json（tmpfs，重启随 PL 重置）。

用法（板上 root）:
  sudo python3 switch_vision.py --full [--stop-services]
  sudo python3 switch_vision.py --pixel 320|640|1280|off
  sudo python3 switch_vision.py --uart on|off
  sudo python3 switch_vision.py --status
"""
import argparse
import fcntl
import hashlib
import json
import mmap
import os
import struct
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(os.environ.get("VISION_ROOT", "/usr/local/lib/pynq-robot-models-v11-20261102-01"))
FPGA_MAN = Path("/sys/class/fpga_manager/fpga0")
OVERLAY_LOCK = "/run/lock/pynq-stereo-overlay.lock"
LED_ADDR = 0x40060000
MODEL_RST_ADDR = 0x40050000  # 复位 GPIO：写 0 复位 stereo/pre/uart + bm_smc/aux_smc
FIRMWARE_DIR = Path("/lib/firmware")
STATE_FILE = Path("/run/vision_overlay_state.json")

# DFX 产出目录（Windows dfx_vision.tcl 构建后部署到 ROOT 下）
DFX_DIR = "vision_dfx"
FULL_BIT = DFX_DIR + "/vision_base.bit"
# 像素档由 stereo + preprocess 两个独立 RP partition 组成（两个独立 pblock），
# 切换时需依次加载两个 partial；UART 是单 partition。每个元素 (manifest_key, 相对路径)。
PARTIAL_BITS = {
    "pixel320":   [("stereo320", DFX_DIR + "/partial_stereo_320.bit"),
                   ("pre320", DFX_DIR + "/partial_pre_320.bit")],
    "pixel640":   [("stereo640", DFX_DIR + "/partial_stereo_640.bit"),
                   ("pre640", DFX_DIR + "/partial_pre_640.bit")],
    "pixel1280":  [("stereo1280", DFX_DIR + "/partial_stereo_1280.bit"),
                   ("pre1280", DFX_DIR + "/partial_pre_1280.bit")],
    "pixelblank": [("stereoblank", DFX_DIR + "/partial_stereo_blank.bit"),
                   ("preblank", DFX_DIR + "/partial_pre_blank.bit")],
    "uart":       [("uart", DFX_DIR + "/partial_uart.bit")],
    "uartblank":  [("uartblank", DFX_DIR + "/partial_uart_blank.bit")],
}
PARTIAL_MANIFEST = DFX_DIR + "/partial_manifest.json"

# LED 位
LED_PIXEL = {"320": 1, "640": 2, "1280": 4}
LED_UART = 8


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_bit_header(bit_data):
    finished = False
    offset = 0
    contents = bit_data
    bit_dict = {}
    length = struct.unpack(">h", contents[offset:offset + 2])[0]
    offset += 2 + length
    offset += 2
    while not finished:
        desc = contents[offset]
        offset += 1
        if desc != 0x65:
            length = struct.unpack(">h", contents[offset:offset + 2])[0]
            offset += 2
            data = struct.unpack(">{}s".format(length), contents[offset:offset + length])[0]
            data = data.decode("ascii")[:-1]
            offset += length
        if desc == 0x61:
            bit_dict["design"] = data.split(";")[0]
        elif desc == 0x62:
            bit_dict["part"] = data
        elif desc == 0x65:
            finished = True
            length = struct.unpack(">i", contents[offset:offset + 4])[0]
            offset += 4
            if length + offset != len(contents):
                raise RuntimeError("Invalid bit length found")
            bit_dict["length"] = str(length)
            bit_dict["data"] = contents[offset:offset + length]
    return bit_dict


def bit2bin(bit_data):
    import array
    bit_dict = parse_bit_header(bit_data)
    a = array.array("i")
    a.frombytes(bit_dict["data"])
    if sys.byteorder != "little":
        return bit_dict["data"]
    a.byteswap()
    return a.tobytes()


def _led_write(value):
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    try:
        m = mmap.mmap(fd, 4096, offset=LED_ADDR)
        m[0:4] = struct.pack("<I", value & 15)
        m.close()
    finally:
        os.close(fd)


def _model_rst_write(value):
    fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    try:
        m = mmap.mmap(fd, 4096, offset=MODEL_RST_ADDR)
        m[0:4] = struct.pack("<I", value & 1)
        m.close()
    finally:
        os.close(fd)


def fpga_state():
    try:
        return (FPGA_MAN / "state").read_text().strip()
    except OSError:
        return "unknown"


def read_state():
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text(encoding="utf-8"))
        except (ValueError, OSError):
            pass
    return {"pixel": None, "uart": False}


def write_state(state):
    STATE_FILE.write_text(json.dumps(state))


def compute_led(pixel, uart):
    led = LED_PIXEL.get(pixel, 0)
    if uart:
        led |= LED_UART
    return led


def acquire_overlay_lock():
    fd = os.open(OVERLAY_LOCK, os.O_RDONLY | os.O_NOFOLLOW)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return fd


def release_overlay_lock(fd):
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


def stop_services():
    for unit in ("pynq-piano-model", "pynq-pmod-uart"):
        subprocess.run(["systemctl", "stop", unit], capture_output=True, timeout=60)
    for name in ("depth_server640.py", "depth_stream_fast.py", "depth_server.py"):
        subprocess.run(
            ["pkill", "-f", "^/usr/local/share/pynq-venv/bin/python3.*%s" % name],
            capture_output=True, timeout=20)
    time.sleep(0.3)


def verify_partial_bit(name, bit_path):
    manifest = ROOT / PARTIAL_MANIFEST
    actual = sha256(bit_path)
    if not manifest.exists():
        print("WARNING: {} missing; skipping partial sha256 check".format(manifest),
              file=sys.stderr)
        return actual
    cfg = json.loads(manifest.read_text(encoding="utf-8"))
    expected = cfg.get(name)
    if expected and actual != expected:
        raise RuntimeError("sha256 mismatch for {}: {} != {}".format(bit_path, actual, expected))
    return actual


def load_bitstream(bit_path, partial):
    t0 = time.monotonic()
    bit_data = Path(bit_path).read_bytes()
    bin_data = bit2bin(bit_data)
    stem = Path(bit_path).stem
    bin_path = FIRMWARE_DIR / (stem + ".bin")
    tmp = bin_path.with_suffix(".bin.tmp")
    tmp.write_bytes(bin_data)
    os.chmod(tmp, 0o644)
    os.replace(tmp, bin_path)

    flag = "1" if partial else "0"
    with open(FPGA_MAN / "flags", "w") as f:
        f.write(flag)
    with open(FPGA_MAN / "firmware", "w") as f:
        f.write(bin_path.name)

    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        state = fpga_state()
        if state == "operating":
            break
        if state not in ("operating", "programming", "unknown"):
            raise RuntimeError("fpga_manager state after load: {}".format(state))
        time.sleep(0.05)
    if fpga_state() != "operating":
        raise RuntimeError("fpga_manager did not reach operating in 10s")
    return (time.monotonic() - t0) * 1000


def _pulse_reset():
    """partial 重配置后脉冲复位，清 smartconnect 死锁。写 0 复位 -> 写 1 释放。"""
    _model_rst_write(0)
    time.sleep(0.002)
    _model_rst_write(1)


def load_full(stop_services_first):
    if fpga_state() != "operating":
        raise RuntimeError("fpga_manager not operating; board PL state unknown")
    if stop_services_first:
        stop_services()
    bit_path = ROOT / FULL_BIT
    if not bit_path.is_file():
        raise RuntimeError("Bitstream missing: {} (build with dfx_vision.tcl on Windows "
                           "first)".format(bit_path))
    actual = sha256(bit_path)
    fd = acquire_overlay_lock()
    try:
        ms = load_bitstream(bit_path, partial=False)
        state = {"pixel": "640", "uart": True}
        write_state(state)
        _led_write(compute_led("640", True))
    finally:
        release_overlay_lock(fd)
    return {"action": "full", "pixel": "640", "uart": True, "ms": round(ms, 1),
            "sha256": actual}


def load_pixel(pixel):
    state = read_state()
    if state["pixel"] is None and not state["uart"] and state == {"pixel": None, "uart": False}:
        raise RuntimeError("No vision full design loaded; run --full first")
    name = "pixelblank" if pixel == "off" else "pixel" + pixel
    entries = PARTIAL_BITS[name]
    # 锁外先确认所有位流存在，避免加载一半才报错
    for _, rel in entries:
        if not (ROOT / rel).is_file():
            raise RuntimeError("Partial bitstream missing: {}".format(ROOT / rel))
    fd = acquire_overlay_lock()
    try:
        total_ms = 0.0
        actual = None
        for key, rel in entries:
            bit_path = ROOT / rel
            actual = verify_partial_bit(key, bit_path)
            total_ms += load_bitstream(bit_path, partial=True)
            _pulse_reset()
        new_pixel = None if pixel == "off" else pixel
        state["pixel"] = new_pixel
        write_state(state)
        _led_write(compute_led(new_pixel, state["uart"]))
    finally:
        release_overlay_lock(fd)
    return {"action": "pixel", "pixel": pixel, "uart": state["uart"], "ms": round(total_ms, 1),
            "sha256": actual}


def load_uart(on):
    state = read_state()
    if state["pixel"] is None and not state["uart"]:
        raise RuntimeError("No vision full design loaded; run --full first")
    name = "uart" if on else "uartblank"
    entries = PARTIAL_BITS[name]
    for _, rel in entries:
        if not (ROOT / rel).is_file():
            raise RuntimeError("Partial bitstream missing: {}".format(ROOT / rel))
    fd = acquire_overlay_lock()
    try:
        total_ms = 0.0
        actual = None
        for key, rel in entries:
            bit_path = ROOT / rel
            actual = verify_partial_bit(key, bit_path)
            total_ms += load_bitstream(bit_path, partial=True)
            _pulse_reset()
        state["uart"] = on
        write_state(state)
        _led_write(compute_led(state["pixel"], on))
    finally:
        release_overlay_lock(fd)
    return {"action": "uart", "uart": on, "pixel": state["pixel"], "ms": round(total_ms, 1),
            "sha256": actual}


def status():
    return {
        "fpga_manager_state": fpga_state(),
        "overlay_state": read_state(),
        "full_bit_present": (ROOT / FULL_BIT).is_file(),
        "partial_bits_present": {n: all((ROOT / rel).is_file() for _, rel in entries)
                                for n, entries in PARTIAL_BITS.items()},
        "partial_manifest_present": (ROOT / PARTIAL_MANIFEST).is_file(),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--full", action="store_true", help="加载整片 vision_base.bit（640 + UART）")
    g.add_argument("--pixel", choices=("320", "640", "1280", "off"))
    g.add_argument("--uart", choices=("on", "off"))
    g.add_argument("--status", action="store_true")
    p.add_argument("--stop-services", action="store_true",
                   help="加载整片位流前停模型/舵机/深度服务")
    p.add_argument("--root", default=None, help="权威包根目录（默认环境变量 VISION_ROOT）")
    args = p.parse_args()

    global ROOT
    if args.root:
        ROOT = Path(args.root)

    if os.geteuid() != 0:
        raise SystemExit("Root required (fpga_manager sysfs + /dev/mem)")
    if args.status:
        print(json.dumps(status(), indent=2))
        return
    if args.full:
        print(json.dumps(load_full(args.stop_services), indent=2))
    elif args.pixel:
        print(json.dumps(load_pixel(args.pixel), indent=2))
    else:
        print(json.dumps(load_uart(args.uart == "on"), indent=2))


if __name__ == "__main__":
    main()
