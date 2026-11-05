#!/bin/bash
# 镜像注入 v2：在 rootfs 克隆完成后执行（root 权限运行）
# 用途：重建 /var/swap + 注入本次新增的板端文件 + 关键文件抽查
# 镜像挂载点：/mnt/img_root（p2 ext4）、/mnt/img_boot（p1 FAT）
set -euo pipefail

IMG_ROOT=/mnt/img_root
PKG="$IMG_ROOT/usr/local/lib/pynq-robot-models-v11-20261102-01"
SRC="$(dirname "$0")"

echo "== 1. 重建 /var/swap (512M，与板端一致 0644 root:root) =="
rm -f "$IMG_ROOT/var/swap"
dd if=/dev/zero of="$IMG_ROOT/var/swap" bs=1M count=512 status=none
mkswap "$IMG_ROOT/var/swap" >/dev/null
chmod 0644 "$IMG_ROOT/var/swap"
chown root:root "$IMG_ROOT/var/swap"

echo "== 2. 注入板端新脚本 → /home/xilinx/ =="
install -o root -g root -m 0755 "$SRC/switch_fast.py"      "$IMG_ROOT/home/xilinx/switch_fast.py"
install -o root -g root -m 0755 "$SRC/depth_stream_fast.py" "$IMG_ROOT/home/xilinx/depth_stream_fast.py"

echo "== 3. 注入 DFX 构建文件 → 权威包（verify_package 只查 manifest 内文件，新增安全）=="
install -o root -g root -m 0644 "$SRC/dfx320.tcl"    "$PKG/dfx320.tcl"
install -o root -g root -m 0644 "$SRC/README_DFX.md" "$PKG/README_DFX.md"

echo "== 4. 注入 DFX 320 partial 位流 + manifest → mode320/（替换 314B 空壳）=="
# 三枚 partial 共享同一 RP 槽位，大小应完全一致；manifest 供 switch_fast --partial 校验
install -o root -g root -m 0644 "$SRC/partial_dense_fp32.bit"    "$PKG/mode320/partial_dense_fp32.bit"
install -o root -g root -m 0644 "$SRC/partial_wm_trajectory.bit" "$PKG/mode320/partial_wm_trajectory.bit"
install -o root -g root -m 0644 "$SRC/partial_models_blank.bit"  "$PKG/mode320/partial_models_blank.bit"
install -o root -g root -m 0644 "$SRC/partial_manifest.json"     "$PKG/mode320/partial_manifest.json"

echo "== 5. 关键文件抽查 =="
check() { # path expected_size
    if [ ! -e "$1" ]; then echo "缺失: $1"; exit 1; fi
    sz=$(stat -c%s "$1")
    echo "  $1  $sz 字节  (期望 $2)"
}
check "$PKG/mode320/vision_base.bit"          4045681
check "$PKG/mode320/partial_dense_fp32.bit"   2059006
check "$PKG/mode320/partial_wm_trajectory.bit" 2059006
check "$PKG/mode320/partial_models_blank.bit" 2059006
check "$PKG/mode320/partial_manifest.json"       299
check "$PKG/mode640/depth640.bit"            4045681
check "$PKG/package_manifest.json"           238606
check "$IMG_ROOT/etc/network/interfaces.d/eth0" ""
check "$IMG_ROOT/home/xilinx/.ssh/authorized_keys" ""
check "$IMG_ROOT/var/swap" 536870912
echo "== 完成 =="
