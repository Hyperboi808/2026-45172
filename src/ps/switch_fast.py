#!/usr/bin/env python3
"""无 pynq 依赖的 PL 位流快加载器（整片 + DFX 部分位流）。

目标：四个 overlay（320 视觉 / 640 深度 / PPO dense_fp32 / WM wm_trajectory）
以最快速度加载：
  - 整片（320/640）：跳过 import pynq（~10.5s）与 verify_package（~2s），
    直接走 fpga_manager sysfs，预计从 ~25s 降到 2-4s。
  - 部分位流（dense/wm/blank）：DFX 重配置区，flags=1 编程，位流仅
    RP 区域（数百 KB 级，预计 <0.5s），且不打断 320 静态区
    （相机/舵机/点云全程可用）——即两个模型硬件加速 overlay 与
    320 像素并存、逐个加载。

机制（复刻 PYNQ 3.1.1 pl_server/embedded_device.py，验证自板端 pynq 源码树）：
  1. .bit 头解析（首字段 2B len + data；跳 2B；desc 循环；0x65=data 起点）
  2. data 按 int32 字节交换 -> .bin，写 /lib/firmware/<stem>.bin
  3. echo 0|1 > /sys/class/fpga_manager/fpga0/flags   （0=整片, 1=部分）
  4. echo <stem>.bin > /sys/class/fpga_manager/fpga0/firmware（内核同步编程）
  5. 轮询 state == operating（10s 超时）
  - 不 apply dtbo：pynq 3.1.1 嵌入式 MMIO 走 /dev/mem（embedded_device.mmap），
    本设计所有 IP 均无中断/字符设备绑定需求，uio 不是必需的。
  - 不做 set_axi_port_width：仅 Zynq UltraScale+ 需要，Zynq-7020 无此问题。
  - 不做 shutdown()：仅用于 DFX AXI shutdown manager IP，本设计无此 IP；
    整片重配置由 fpga_manager 驱动复位 PL。

锁与安全协议（与现有服务一致）：
  - 整片加载前抢 /run/lock/pynq-stereo-overlay.lock LOCK_EX|LOCK_NB，
    失败立即拒绝（不等待）——调用方必须先停深度/舵机/模型服务
    （--stop-services 可代劳）。
  - 部分位流加载只要求当前整片位流是 320（静态区匹配），且同样先抢
    overlay 锁（部分编程期间禁止并发访问静态区外设的驱动竞争）。
  - .bit 加载前校验 sha256：整片查 modes.json，部分位流查
    mode320/partial_manifest.json（DFX 构建产出；缺失时跳过校验并警告）。

LED 约定（与 status_led.py 一致）：320=0x8(UART320)，640=0x4(DEPTH640)。

用法（板上 root）：
  sudo python3 switch_fast.py --mode 320|640 [--stop-services]
  sudo python3 switch_fast.py --partial dense|wm|blank
  sudo python3 switch_fast.py --status
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
CURRENT = Path("/usr/local/lib/pynq-vision-current")
FPGA_MAN = Path("/sys/class/fpga_manager/fpga0")
OVERLAY_LOCK = "/run/lock/pynq-stereo-overlay.lock"
LED_ADDR = 0x40060000
FIRMWARE_DIR = Path("/lib/firmware")

# LED 位（必须与 status_led.py 一致）
UART320, DEPTH640 = 8, 4
LED_MODE = {"320": UART320, "640": DEPTH640}

FULL_BITS = {
    "320": "mode320/vision_base.bit",
    "640": "mode640/depth640.bit",
}
# DFX 部分位流（由 Windows Vivado dfx320.tcl 构建产出后放入 mode320/）
PARTIAL_BITS = {
    "dense": "mode320/partial_dense_fp32.bit",
    "wm": "mode320/partial_wm_trajectory.bit",
    "blank": "mode320/partial_models_blank.bit",
}
PARTIAL_MANIFEST = "mode320/partial_manifest.json"


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def parse_bit_header(bit_data):
    """照抄 pynq embedded_device.parse_bit_header，只保留必要字段。"""
    finished = False
    offset = 0
    contents = bit_data
    bit_dict = {}
    length = struct.unpack(">h", contents[offset:offset + 2])[0]
    offset += 2 + length
    offset += 2  # 两字节未知字段（通常 1）
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
    """照抄 pynq embedded_device.bit2bin：data 按 int32 字节交换。

    用 array（C 实现）而非 numpy——本脚本的卖点是无重依赖、启动即快。
    """
    import array
    bit_dict = parse_bit_header(bit_data)
    a = array.array("i")
    a.frombytes(bit_dict["data"])
    if sys.byteorder != "little":  # 主机大端时无需交换（防御性）
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


def fpga_state():
    try:
        return (FPGA_MAN / "state").read_text().strip()
    except OSError:
        return "unknown"


def current_mode():
    """读 symlink 判当前整片位流模式（与 switch_mode.py 协议一致）。"""
    if CURRENT.is_symlink():
        target = str(CURRENT.resolve())
        if target.endswith("/mode640"):
            return "640"
        if target.endswith("/mode320"):
            return "320"
    return None


def acquire_overlay_lock():
    """整片/部分编程期间持 overlay 锁。LOCK_EX|LOCK_NB，失败即拒绝。"""
    fd = os.open(OVERLAY_LOCK, os.O_RDONLY | os.O_NOFOLLOW)
    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    return fd


def release_overlay_lock(fd):
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


def stop_services():
    """停用占用相机/PL/锁的服务（与 switch_overlay.py 编排一致）。"""
    for unit in ("pynq-piano-model", "pynq-pmod-uart"):
        subprocess.run(["systemctl", "stop", unit], capture_output=True, timeout=60)
    # 深度 HTTP/TCP 服务进程（精确匹配 venv python 解释器 + 脚本名）
    for name in ("depth_server640.py", "depth_stream_fast.py", "depth_server.py"):
        subprocess.run(
            ["pkill", "-f",
             "^/usr/local/share/pynq-venv/bin/python3.*%s" % name],
            capture_output=True, timeout=20)
    time.sleep(0.3)


def verify_full_bit(mode, bit_path):
    modes_json = ROOT / "modes.json"
    if modes_json.exists():
        cfg = json.loads(modes_json.read_text(encoding="utf-8"))
        expected = cfg.get("modes", {}).get(mode, {}).get("bit_sha256")
        actual = sha256(bit_path)
        if expected and actual != expected:
            raise RuntimeError("sha256 mismatch for {}: {} != {}".format(bit_path, actual, expected))
        return actual
    return sha256(bit_path)


def verify_partial_bit(name, bit_path):
    manifest = ROOT / PARTIAL_MANIFEST
    actual = sha256(bit_path)
    if not manifest.exists():
        print("WARNING: {} missing; skipping partial sha256 check".format(manifest), file=sys.stderr)
        return actual
    cfg = json.loads(manifest.read_text(encoding="utf-8"))
    expected = cfg.get(name)
    if expected and actual != expected:
        raise RuntimeError("sha256 mismatch for {}: {} != {}".format(bit_path, actual, expected))
    return actual


def load_bitstream(bit_path, partial):
    """bit -> bin -> /lib/firmware -> fpga_manager。返回耗时 ms。"""
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


def load_full(mode, stop_services_first):
    if fpga_state() != "operating":
        raise RuntimeError("fpga_manager not operating; board PL state unknown")
    if stop_services_first:
        stop_services()
    bit_rel = FULL_BITS[mode]
    bit_path = ROOT / bit_rel
    if not bit_path.is_file():
        raise RuntimeError("Bitstream missing: {}".format(bit_path))
    actual = verify_full_bit(mode, bit_path)
    fd = acquire_overlay_lock()
    try:
        ms = load_bitstream(bit_path, partial=False)
        # 更新 current symlink（与 switch_mode.py 协议一致）
        target = ROOT / Path(bit_rel).parent  # mode320 / mode640 目录
        tmp_link = CURRENT.with_suffix(".tmp")
        if tmp_link.exists() or tmp_link.is_symlink():
            tmp_link.unlink()
        tmp_link.symlink_to(target)
        os.replace(tmp_link, CURRENT)
        _led_write(LED_MODE[mode])
    finally:
        release_overlay_lock(fd)
    return {"action": "full", "mode": mode, "ms": round(ms, 1), "sha256": actual}


def load_partial(name):
    if current_mode() != "320":
        raise RuntimeError("Partial bitstreams require the 320 static design; "
                           "current mode: {}".format(current_mode()))
    bit_path = ROOT / PARTIAL_BITS[name]
    if not bit_path.is_file():
        raise RuntimeError("Partial bitstream missing: {} (build it on Windows with "
                           "dfx320.tcl first)".format(bit_path))
    actual = verify_partial_bit(name, bit_path)
    fd = acquire_overlay_lock()
    try:
        ms = load_bitstream(bit_path, partial=True)
    finally:
        release_overlay_lock(fd)
    return {"action": "partial", "cell": name, "ms": round(ms, 1), "sha256": actual}


def status():
    return {
        "fpga_manager_state": fpga_state(),
        "current_full_mode": current_mode(),
        "full_bits_present": {m: (ROOT / rel).is_file() for m, rel in FULL_BITS.items()},
        "partial_bits_present": {n: (ROOT / rel).is_file() for n, rel in PARTIAL_BITS.items()},
        "partial_manifest_present": (ROOT / PARTIAL_MANIFEST).is_file(),
    }


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--mode", choices=("320", "640"))
    g.add_argument("--partial", choices=("dense", "wm", "blank"))
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
    if args.mode:
        print(json.dumps(load_full(args.mode, args.stop_services), indent=2))
    else:
        print(json.dumps(load_partial(args.partial), indent=2))


if __name__ == "__main__":
    main()
