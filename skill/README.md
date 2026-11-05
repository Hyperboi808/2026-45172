# 技能包（Skill Pack）

本包沉淀本项目可复用的工程技能，每项含「适用场景、前置条件、步骤、验证方法、踩坑」。

---

## 技能 1：DFX 位流构建（7 系列 / Zynq-7020）

**适用场景**：在 Xilinx 7 系列（含 Zynq-7000）上构建**单 RP 多 RM** 部分重配置位流，把多个功能互斥的加速器塞进同一块物理区域、以毫秒级热切换。

**前置条件**：
- **Vivado 2023.2 或更新**（2022.1/2022.2 对 7 系列 DFX 的 `write_bitstream -cell` 有 0 帧空 partial 工具 bug，无 patch，只能升级）。
- 各 RM 顶层 AXI 端口**完全一致**（否则无法共用槽位）。
- HLS 导出的 IP 若有加密浮点核，需先做跨版本适配（见下）。

**步骤**：
1. 建单 RP 的 Pblock：以 tile 矩形给出（整列全类型站点自动纳入，规避 18-993 列劈裂）。
2. `create_partition_def` → `HD.RECONFIGURABLE` → 静态区实现（RM 以黑盒占位）。
3. RM1（dense）OOC 综合 → `read_checkpoint -cell <cell> <dcp>` 换入 → 实现 → `write_bitstream`（整片）+ `write_bitstream -cell <cell>`（partial）。
4. RM2（wm）改名网表换入：OOC 综合时顶层模块名 = 目标 RP cell 的 REF_NAME，再 `read_checkpoint -cell` 换入，锁定静态区后 route，导出 partial。
5. RM3（blank）：stub 生成器从综合网表导出端口、输出拉安全电平，导出 partial。
6. 每对 checkpoint 跑 `pr_verify` 验证静态区一致。

**踩坑（非显而易见）**：
1. **0 帧 partial**：构建成功、`pr_verify` 通过 ≠ 位流非空。**必须核对 partial 位流大小**（本例应 2 MB；314 B 即 0 配置帧，是工具 bug）。
2. **浮点核加密密钥滚动**：HLS 2022.2 产物用 `key_keyname="xilinxt_2021_07"` 加密，2023.2 已移除旧密钥。报 `Synth 8-5809 encrypted envelope` → 用 2023.2 自带 IP 文件替换 `floating_point_lib`（9 个 `*_rfs.vhd`）。
3. **模块名 rev 升级**：换库后报 `module 'floating_point_v7_1_14' not found` → 2023.2 是 rev16，改 wrapper 里 `floating_point_v7_1_14`→`floating_point_v7_1_16`、`x_ipCoreRevision=14`→`16`。两条（密钥 + rev）常同时出现。
4. **cell 句柄失效**：`close_design`/`create_project`/`read_checkpoint` 都会使 `get_cells` 句柄失效，每次操作后须重新 `set cell [get_cells -hierarchical -filter {...}]`。
5. **判别密钥/rev**：`grep key_keyname` 看密钥；wrapper 里 `floating_point_v7_1_<rev>` + `x_ipCoreRevision=<rev>` 看版本。

**验证**：`pr_verify` 输出 `are compatible`；时序 Setup/Hold 0 失败端点；**三枚 partial 位流大小一致且非空**（最易漏的最终校验）。

---

## 技能 2：位流快加载（无 pynq 依赖）

**适用场景**：PYNQ 板上快速加载整片/部分位流，绕开 `import pynq`（约 10.5 s）与 `verify_package`（约 2 s），把切换从 ~25 s 降到 2–4 s（整片）/ <0.5 s（部分）。

**原理**（复刻 PYNQ 3.1.1 `pl_server/embedded_device.py`，直接操作内核 sysfs）：
1. `.bit` 头解析（首字段 2B len + data；跳 2B；desc 循环；0x65 = data 起点）。
2. data 按 int32 字节交换 → `.bin`，写 `/lib/firmware/<stem>.bin`。
3. `echo 0|1 > /sys/class/fpga_manager/fpga0/flags`（0=整片，1=部分）。
4. `echo <stem>.bin > /sys/class/fpga_manager/fpga0/firmware`（内核同步编程）。
5. 轮询 `state == operating`（10 s 超时）。

不 apply dtbo（本设计 IP 均无中断/字符设备，uio 非必需）；不 `shutdown()`（无 DFX AXI shutdown manager IP）；不 set_axi_port_width（仅 UltraScale+ 需要）。

**锁与安全协议**：
- 整片加载前抢 `/run/lock/pynq-stereo-overlay.lock` `LOCK_EX|LOCK_NB`，失败立即拒绝——调用方须先停深度/舵机/模型服务（`--stop-services`）。
- 部分位流只要求当前整片是 320（静态区匹配），同样先抢锁。
- `.bit` 加载前校验 sha256：整片查 `modes.json`，部分查 `mode320/partial_manifest.json`。

**用法**：
```bash
sudo python3 switch_fast.py --mode 320|640 [--stop-services]
sudo python3 switch_fast.py --partial dense|wm|blank
sudo python3 switch_fast.py --status
```

**踩坑**：整片/部分编程期间静态区外设驱动竞争——必须先停占用相机/舵机/模型的 systemd 服务与深度 HTTP/TCP 进程，否则持锁失败或外设状态错乱；`current_mode()` 靠 symlink 判模式；加载后 `fpga_manager` 必须是 `operating`。

---

## 技能 3：板端镜像注入（改板上文件 + manifest 同步）

**适用场景**：在不动原始镜像的前提下，把新位流/软件/配置写进镜像 rootfs（p2 ext4），并保证上板 `verify_package` 不挂。

**步骤**：
1. 挂镜像 p2（只读或 rw）：`losetup --find --show --offset <p2_字节偏移> --sizelimit <p2_字节大小> <镜像>`（偏移 = p2 起始扇区 × 512，`fdisk -l` 查）。
2. rw 挂载后写文件：`sudo install -o root -g root -m 0644 <源> <目标>`（**必须 sudo**：镜像内 root 文件在 0755 目录，普通用户删不动）。
3. 改任何「已在 package_manifest.json 内」的文件后，**必须同步其 sha256**。
4. 全量校验：挂只读后跑
   ```python
   import json, hashlib, os
   man = json.load(open('package_manifest.json'))
   for rel, exp in man['sha256'].items():
       assert hashlib.sha256(open(rel,'rb').read()).hexdigest() == exp, rel
   ```

**踩坑**：
- **`verify_package` 只查 manifest 内文件**：新增文件（不在 manifest）安全，但**替换/修改** manifest 内文件必须同步 sha256，否则上板 overlay 全挂。
- **字节数相同 ≠ 内容相同**：位流替换不能只看大小（本例旧 320 位流与 DFX 版都恰好 4,045,681 B），必须看 sha256。
- **hwh 里的地址有两处**：`MEMRANGE` 的 `BASEVALUE`+`HIGHVALUE` 与 `PARAMETER` 的 `C_S_AXI_CONTROL_BASEADDR` `VALUE`，改地址要一起改（HIGHVALUE 也要 `0x…FFFF` 变）。
- 写文件后 `sync`，卸载后重挂只读复核 sha256 再交付。
