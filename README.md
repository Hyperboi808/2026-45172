# PYNQ-Z2 机械臂具身智能 —— DFX 320 部分重配置加速

基于 Xilinx（AMD）PYNQ-Z2（Zynq-7020 / xc7z020）的机械臂「具身智能」软硬件系统。通过 **Dynamic Function eXchange（DFX，动态部分重配置）**，在同一块 320 视觉静态区上，以毫秒级切换两个神经网络推理加速器（PPO 策略网络 dense_fp32 与 WM 逆运动学轨迹头 wm_trajectory），实现「相机 + 舵机 + 点云」外设全程在线、模型热切换。

## 1. 系统架构

```
Zynq-7020 PL
├── 静态区（常驻，DFX 期间不中断）
│   ├── stereo_rt_accel   双目立体匹配（SGM）      @ 0x40010000
│   ├── preprocess_accel  预处理（640→320 等）     @ 0x40020000
│   ├── pointcloud_accel  点云生成                @ 0x40000000
│   ├── dual_pmod_uart    舵机总线（A/B 两路）     @ 0x40030000
│   └── status_led        状态灯                    @ 0x40060000
└── 重配置区 RP（单槽位，双 RM 互斥）
    ├── RM1  dense_fp32     PPO 策略网络（11 层全连接 FP32）  @ 0x40040000
    ├── RM2  wm_trajectory  WM 轨迹头（FP64 弧度解码）        @ 0x40040000（同槽位）
    └── RM3  blank          卸载模型加速器（stub）
```

- **320 视觉**（整片位流 vision_base_320_dfx.bit）：静态区 + dense RM，功能等价原 320 视觉模式。
- **部分位流** partial_dense_fp32 / partial_wm_trajectory / partial_models_blank：仅重编程 RP 区（2 MB，毫秒级），不打断静态区外设。
- **640 深度**：独立的整片 depth640 位流（非 DFX，本包 build/ 含其构建脚本），用于外参标定与深度定位。

## 2. 目录结构（AMD 选题指南模板）

| 目录 | 内容 |
|---|---|
| `README.md` | 本文件：项目简介 + 复现步骤 |
| `src/hls/` | HLS C 源码（dense_fp32.cpp、wm_trajectory.cpp 等）|
| `src/ps/` | PS 软件（switch_fast.py、wm_accelerated.py、model_service 等）|
| `src/rtl/` | 各 HLS IP 打包产物（component.xml + 生成 Verilog）|
| `sim/` | 验证结果（pr_verify、时序）与汇总 `verification_summary.md` |
| `build/` | 可复现构建脚本（dfx320.tcl、replace_fp2023.ps1）+ 综合/实现报告 |
| `board/` | 上板脚本（switch_fast.py、镜像注入脚本）|
| `data/` | 权重（PPO/WM）、场景 STL、标定/配置 |
| `skill/` | 技能包（构建/部署/标定/调参的可复用技能，含 README.md）|
| `report/` | 设计报告 + 大模型协作记录 |

## 3. 复现步骤

### 3.1 位流构建（Windows + Vivado 2023.2，约 60–120 分钟）

**必须 Vivado 2023.2+**：2022.1/2022.2 对 7 系列 DFX 的 `write_bitstream -cell` 存在空 partial 工具 bug（产出 314 B / 0 配置帧），2023.2 已修复。

1. 准备构建包：`build/dfx320.tcl` + `hw320/`（含 ip_repo 全部 HLS IP、board_files、xdc）。
2. **浮点核跨版本适配**（HLS 2022.2 导出的 dense/wm 含加密浮点 IP，2023.2 无法直接综合）：
   - 密钥滚动：旧 `xilinxt_2021_07` → 2023.2 的 `xilinxt_2022_10`，须用 2023.2 自带 IP 文件替换 dense/wm 的 `floating_point_lib`（9 个 `*_rfs.vhd`）。
   - 模块名 rev 升级：`floating_point_v7_1_14` → `floating_point_v7_1_16`（coreRevision 14→16），改 10 个 `*_ip.v` wrapper。
   - 一键脚本：`build/replace_fp2023.ps1`。
3. 运行：`vivado -mode batch -source dfx320.tcl`（Windows PowerShell 后台启动可脱离 SSH 会话）。
4. 产物 `out320dfx/`：整片 + 三枚 partial 位流 + partial_manifest.json + 时序/DRC/利用率报告。

### 3.2 镜像重构（板端文件注入）

1. 挂载镜像 p2（ext4）为 rw，把以下文件写入权威包 `/usr/local/lib/pynq-robot-models-v11-20261102-01/`：
   - `mode320/vision_base.bit` ← 用 `vision_base_320_dfx.bit` 覆盖（DFX 整片）
   - `mode320/vision_base.hwh` ← wm 的 `C_S_AXI_CONTROL_BASEADDR` 0x40070000 → 0x40040000
   - `mode320/partial_*.bit` ×3 + `partial_manifest.json`（新增）
   - `wm_accelerated.py`、`modes.json` ← wm 地址 0x40070000 → 0x40040000
2. **同步 `package_manifest.json`**（否则上板 verify_package 全挂）：更新上述被改文件的 sha256。
3. 一键脚本：`board/inject_dfx_partial.sh`。

### 3.3 上板部署

1. 写卡：`sudo dd if=robot_custom_v2_20261104.img of=/dev/sdX bs=4M conv=fsync`（镜像已含 DFX 位流 + 软件适配）。
2. 加载整片 320：`sudo python3 /home/xilinx/switch_fast.py --mode 320 --stop-services`
3. 切换模型加速器（部分重配置，毫秒级、不打断静态区）：
   ```bash
   sudo python3 /home/xilinx/switch_fast.py --partial dense   # PPO
   sudo python3 /home/xilinx/switch_fast.py --partial wm      # WM
   sudo python3 /home/xilinx/switch_fast.py --partial blank   # 卸载
   ```

## 4. 关键设计点

- **单 RP 双 RM**：dense 与 wm 顶层 AXI 端口完全一致，共享同一 RP 槽位 0x40040000；wm 由「改名网表换入」（OOC 综合时顶层名 = dense 的 REF_NAME）。
- **地址变化**：wm 控制寄存器地址由独立 0x40070000 变为与 dense 同槽位 0x40040000，软件须同步（hwh / wm_accelerated.py / modes.json 三处）。
- **加载路径**：`switch_fast.py` 不依赖 pynq 导入（省 ~10.5 s），直接走 fpga_manager sysfs；整片 `flags=0`、部分 `flags=1`；加载前抢 overlay 锁 + 校验 sha256（整片查 modes.json、部分查 partial_manifest.json）。
- **完整性**：`switch_mode` 每次切换/开机跑 `verify_package` 全量 sha256，任何板上文件改动必须同步 `package_manifest.json`。

## 5. 验证状态（如实）

| 项 | 状态 |
|---|---|
| DFX 位流构建（Vivado 2023.2）| ✅ 完成，partial 三枚均 2,059,006 B（非空）|
| pr_verify（full-vs-wm / full-vs-blank）| ✅ compatible ×2 |
| 时序 | ✅ Setup 0 失败端点，Worst Slack +0.175 ns |
| 板端软件适配 + 镜像注入 | ✅ 完成，package_manifest 全量 1585 条目 0 不匹配 |

详见 `sim/verification_summary.md`、`report/`。
