#!/bin/bash
# 注入 DFX 320 partial 位流 + 地址适配后的软件文件到 v2 镜像 p2（rw 挂载）
# 目标镜像：build/robot_custom_v2_20261104.img（"新卡"副本，已授权清空；原始镜像不动）
# 依赖：losetup / mount / install（需 root，本机有免密 sudo）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
IMG="$ROOT/build/robot_custom_v2_20261104.img"
SRC="$(dirname "$0")"
PKG_REL="usr/local/lib/pynq-robot-models-v11-20261102-01"
P2_OFFSET=$((268288 * 512))          # v2 镜像 p2 起始字节
P2_SIZE=$((30267392 * 512))          # v2 镜像 p2 大小（14.4G）

[[ -f "$IMG" ]] || { echo "镜像不存在: $IMG" >&2; exit 1; }

echo "== 1. loop 挂载 p2 (rw) =="
LOOP=$(sudo losetup --find --show --offset "$P2_OFFSET" --sizelimit "$P2_SIZE" "$IMG")
echo "   loop=$LOOP"
MNT=$(mktemp -d)
sudo mount "$LOOP" "$MNT"
PKG="$MNT/$PKG_REL"
echo "   mnt=$MNT"

echo "== 2. 注入替换文件（mode320 位流/hwh + 软件 + manifest）=="
sudo install -o root -g root -m 0644 "$SRC/vision_base.bit"            "$PKG/mode320/vision_base.bit"
sudo install -o root -g root -m 0644 "$SRC/vision_base.hwh"            "$PKG/mode320/vision_base.hwh"
sudo install -o root -g root -m 0644 "$SRC/wm_accelerated.py"          "$PKG/wm_accelerated.py"
sudo install -o root -g root -m 0644 "$SRC/modes.json"                 "$PKG/modes.json"
sudo install -o root -g root -m 0644 "$SRC/package_manifest.json"      "$PKG/package_manifest.json"

echo "== 3. 注入新增 DFX partial 位流 + manifest =="
sudo install -o root -g root -m 0644 "$SRC/partial_dense_fp32.bit"     "$PKG/mode320/partial_dense_fp32.bit"
sudo install -o root -g root -m 0644 "$SRC/partial_wm_trajectory.bit"  "$PKG/mode320/partial_wm_trajectory.bit"
sudo install -o root -g root -m 0644 "$SRC/partial_models_blank.bit"   "$PKG/mode320/partial_models_blank.bit"
sudo install -o root -g root -m 0644 "$SRC/partial_manifest.json"      "$PKG/mode320/partial_manifest.json"

echo "== 4. 注入 switch_fast.py 到 /home/xilinx（DFX partial 加载器）=="
sudo install -o root -g root -m 0755 "$SRC/switch_fast.py" "$MNT/home/xilinx/switch_fast.py"

echo "== 5. sync + 卸载 =="
sync
sudo umount "$MNT"
sudo losetup -d "$LOOP"
rmdir "$MNT"
echo "INJECT_DFX_PARTIAL_OK"
