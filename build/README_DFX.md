# DFX 320 构建与部署手册（dfx320.tcl）

目标：四个 overlay（320 视觉 / 640 深度 / PPO dense_fp32 / WM wm_trajectory）最快加载。
两个模型硬件加速 overlay 与 320 并存、可逐个加载（部分位流），640 保持整片。

```
静态区（常驻，加载模型部分位流期间相机/舵机/点云不断）:
  PS + DDR + stereo_rt_accel + preprocess_accel + pointcloud_accel
  + dual_pmod_uart + status_led + 各 AXI interconnect
重配置区 RP（单 RP 双 RM，共用同一物理槽位）:
  RP: dense_fp32_0   —— RM: dense_fp32（实现）/ wm_trajectory（改名网表换入）/ blank（stub）
```

## 0. 架构决策（2026-11-04 定案）

- **单 RP 双 RM**：dense 与 wm 顶层端口完全相同（m_axi_memory_* + s_axi_control_*
  + ap_clk/ap_rst_n），共享一个重配置分区。dense 在项目内实现（RM1）；wm 用
  `create_ip -module_name <dense的REF_NAME>` 改名 + `synth_ip` OOC 综合成 DCP 后
  `read_checkpoint -cell` 换入（RM2）；blank stub 卸载模型（RM3）。
- 双 RP 方案已实测否决：wm RP 与 dense RP 同列冲突（HDPR-25）+ FF 绑定量放不进
  小 RP（UTLZ-1）+ 资源不足（HDPR-18）。
- **wm 控制地址变化：0x40070000 → 0x40040000**（与 dense 共用同一 AXI 槽位/地址，
  hwh 只含 dense_fp32_0）。板端 WM 模型服务必须改址适配。
- RP 区域（get_rp_rects 20 38 0 99，tile X20-38 × tileY0-99）：
  ~3000 slices / 40 RAMB36 / 80 RAMB18 / 40 DSP48；wm 为最大 RM（7551 LUT）
  时利用率 ~65%，静态区剩余 ~10000 slices（~78%）。

## 1. 前置条件

- Windows 机器，**Vivado 2022.1**，标准版即可（Zynq-7020 的 DFX 免费）。
  DFX 整片位流与所有部分位流必须同一 Vivado 版本、同一次构建产出，不可混用。
- 把板端权威包 `pynq-robot-models-v11-20261102-01` 整个拷到 Windows（内含 hw320/、
  build320.tcl 等），**在全新目录**放入本目录的 `dfx320.tcl`（与 hw320/ 同级）。
- 磁盘 >= 10GB，构建时长预估 60-120 分钟。

## 2. 构建

```
vivado -mode batch -source dfx320.tcl -journal dfx320.jou -log dfx320.log
```

重跑前必须先删除 `b320dfx/` 与 `out320dfx/`（脚本拒绝覆盖已存在的构建目录）。

已修坑（2026-11-04 历次迭代）：
- Synth 8-439 auto_pc_1 / floating_point_v7_1_15 not found → 不能手动 synth_design
  综合 BD 设计；`launch_runs synth_1`（默认 Hierarchical）自动为每个 BD IP OOC 综合
  出 DCP 后读入；open_run 后 RP 标记/pblock，实现阶段仍手动（内存属性交给 impl_1
  run 会因重读 synth DCP 而丢失）。
- floating_point 加密核心 rev15 仅随 2022.2 发布 → 改用 **2022.1 自带 rev14**（官方
  加密文件，2022.1/2022.2 皆可解密），10 个封装 `_ip.v` 实例名 `_15`→`_14`。
  **风险**：rev14/rev15 实现差异未逐条核对，上板后 PPO/WM 模型推理必须全量回归。
- 声明要放在 xilinx_verilogsynthesis_view_fileset（OOC 走 Verilog 视图）。
  注：板端 hw320 副本仍需同样补丁（换卡后同步）。
- **部分位流 0 帧（314 字节 / 1472 bits）根因（2026-11-04 诊断定案）**：
  `HD.RECONFIGURABLE` 原来放在 open_run synth_1 之后（Phase 3）设置，但 DFX 的
  partition（`HD.PARTITION`）由 `synth_design` 在综合前见到 `HD.RECONFIGURABLE=1`
  时才建立。综合后才设 → `HD.PARTITION` 恒为空 → `write_bitstream -cell` 推导出
  0 配置帧（仅 header+命令开销）。**修复**：用 `STEPS.SYNTH_DESIGN.TCL.PRE` 在
  synth_1 综合前对 dense 设 `HD.RECONFIGURABLE 1`；synth 后 dense 为黑盒 RP，
  再 `read_checkpoint -cell` 换入 dense RM 的 OOC DCP（pa320.gen 下
  `stereo_design_dense_fp32_0_0.dcp`）后实现。同时 pblock Y 从 0-129 改 0-149
  （完整 3 个时钟区域，Y129 跨时钟区域边界会帧不对齐），加 SNAPPING_MODE ON +
  RESET_AFTER_RECONFIG TRUE。partial 位流大小 < 10KB 直接 error（自检）。
  **注意**：此修复为流程时序错误，须全量重跑（删 b320dfx/、out320dfx/）验证。

**pblock 四条铁律**（mini 探针 probe_dfx.tcl 实测，违反任一必失败）：
1. 整列覆盖：pblock 内任一 tile 列，其 Y 范围内所有站点类型都要纳入，否则
   18-993「splitting interconnect tile columns」+ 一半 slice 被排除（18-992）。
2. BRAM/DSP tile 只在 Y=5 倍数存在（1 个 RAMB36/DSP48 site 占 5 tile 行），
   **不能直接拼 tile 矩形**；要 `get_tiles` → `get_sites -of_objects` 反查站点，
   按站点类型分列拼 site 矩形（tileY0-99 ↔ RAMB36 site Y0-19 / RAMB18 site Y0-39
   / DSP48 site Y0-19），FIFO18E1/RAMBFIFO36E1 变体自动包含。
3. `resize_pblock -add` 只收矩形字符串，不收 tile 对象（invalid range）；
   站点只保留 SLICE/RAMB36/RAMB18/FIFO18E1/RAMBFIFO36E1/DSP48 六种，TIEOFF 等
   稀疏类型会产出非法矩形。
4. CMT 列（tile33，BUFG）不得进 RP；时钟从静态 BUFG 进 RP 合法。

产物在 `out320dfx/`：

| 文件 | 说明 |
|---|---|
| vision_base_320_dfx.bit | 完整 320 位流（静态+RP=dense 实现） |
| partial_dense_fp32.bit | RP 部分位流（RM1 = PPO dense） |
| partial_wm_trajectory.bit | RP 部分位流（RM2 = WM wm_trajectory） |
| partial_models_blank.bit | RP 部分位流（RM3 = blank，卸载模型加速器） |
| partial_manifest.json | 四个位流的 sha256（板端 switch_fast.py 校验） |
| config_full_routed.dcp / config_wm_routed.dcp / config_blank_routed.dcp | 锁定静态实现 |
| utilization_hierarchical.rpt | 各层次资源占用（调 pblock 依据） |
| timing_summary_*.rpt / drc_full.rpt | 验收报告 |

成功的标志：日志出现两处 `DFX: pr_verify PASSED (static region identical)` 与
`DFX_BUILD_OK`，且三份 timing_summary 的 setup/hold 均 >= 0。

## 3. 部署到板端

```bash
# 从 PC（板端只读原则：位流验证通过前不覆盖权威包）
scp out320dfx/partial_*.bit out320dfx/partial_manifest.json \
    root@192.168.5.99:/home/xilinx/dfx_parts/
ssh root@192.168.5.99 '
  cp /home/xilinx/dfx_parts/partial_*.bit /home/xilinx/dfx_parts/partial_manifest.json \
     /usr/local/lib/pynq-robot-models-v11-20261102-01/mode320/'
```

DFX 整片位流单独验证（先不覆盖 vision_base.bit）：拷到新包目录
`/usr/local/lib/pynq-robot-models-dfx-320/mode320/vision_base.bit`，
hwh 用现包 mode320 的（IP/地址未变；**注意 hwh 只含 dense_fp32_0**）。

## 4. 上板验证（按顺序，全部通过才把 DFX 版转正）

1. 部分位流快速加载（当前 320 在位即可）：
   ```
   sudo python3 /home/xilinx/switch_fast.py --status        # 看 partial_bits_present
   sudo python3 /home/xilinx/switch_fast.py --partial dense # 预期 <0.5s
   sudo python3 /home/xilinx/switch_fast.py --partial wm
   sudo python3 /home/xilinx/switch_fast.py --partial blank
   ```
   加载后 LED 状态灯不变、相机/舵机服务无报错（静态区未被打断）。
2. 模型功能回归：dense partial + PPO 模型服务推理，结果与整片 320 一致；
   wm partial + WM 模型服务（**改址 0x40040000 后**）同样回归；blank 后模型
   服务应拒绝启动或按约定降级。
3. DFX 整片位流：`switch_fast.py --root /usr/local/lib/pynq-robot-models-dfx-320 --mode 320`
   → board_verify.py + 深度/舵机/模型全链路；再 --mode 640 切回验证互切。
4. 位流转正：全通过后把 DFX 整片位流替换权威包的 vision_base.bit 并
   **同步 package_manifest.json**（板端有完整性校验，见项目工作原则）。

## 5. 已知风险与调参

- 若 place 报错（RM 放不进 RP）或时序差：按 utilization_hierarchical.rpt 里
  dense/wm 的 SLICE/DSP/BRAM 实际用量调整 get_rp_rects 的窗口（20 38 0 99），
  重跑（重跑前删除 b320dfx/ 与 out320dfx/）。
- 静态区资源过满：若静态区（RP 外）SLICE > 78%，DFX 无法收敛。必要时裁静态区
  （如 pointcloud 改为可选）——那是新的设计取舍，先不要动。
- 层次 cell 名：TCL 用通配 `*dense_fp32_0` 定位并断言唯一；BD 层次不同会报错并
  打印实际名字。
- **部分位流与静态区绑定**：任何静态区改动（BD/约束/版本）后，旧部分位流全部
  失效，必须全量重建。
- **切换纪律**：加载 dense/wm/blank 前，模型服务必须已停止（它持 uart-A/B 锁
  且可能在途 AXI 事务）；舵机服务与静态区并存不受影响。整片 320/640 切换前
  仍须停全部服务（switch_fast.py --stop-services 代劳）。

## 6. 与软件链路的配合

- 板端 `switch_fast.py --partial dense|wm|blank`：flags=1 部分编程，无 pynq、
  无 dtbo、无整片复位（静态区外设保持运行）。
- 板端 `switch_fast.py --mode 320|640`：整片快路径（预计 25s → 2-4s）。
- **WM 模型服务改址 0x40040000**（dense 槽位地址；旧 0x40070000 已不存在）。
- PC 端 `控制平台/overlay加载/switch_overlay.py` 把「模型 无/PPO/WM」三选一映射为「部分位流 + 权重」两级切换。
