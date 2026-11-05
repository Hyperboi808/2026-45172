# 设计报告：DFX 320 部分重配置加速的机械臂具身智能

## 1. 项目背景与目标

机械臂具身智能需要「感知 → 决策 → 控制」闭环。感知端（双目立体匹配 → 点云）与控制端（14 路舵机总线）要求**低延迟、持续在线**；决策端涉及两类模型——PPO 策略网络（11 层 FP32 全连接）与 WM 逆运动学轨迹头（FP64 弧度解码）——两者**不会同时使用**，但都很大（dense 44 DSP / wm 7551 LUT）。

在 Zynq-7020（85K 逻辑单元）上同时驻留两个模型加速器会挤占静态区资源、威胁时序收敛。本设计采用 **DFX 单 RP 双 RM**：让两个模型加速器共享同一块物理区域，按需热切换，从而在有限资源内同时满足「外设全程在线 + 双模型硬件加速」。

## 2. 架构设计

- **静态区**（DFX 期间不中断）：stereo_rt_accel（SGM）、preprocess_accel、pointcloud_accel、dual_pmod_uart（舵机总线 A/B）、status_led、PS + 各 interconnect。
- **重配置区 RP**（单槽位，三 RM）：
  - RM1 `dense_fp32`：PPO 策略网络。
  - RM2 `wm_trajectory`：WM 轨迹头。
  - RM3 `blank`：卸载模型加速器的 stub（输出拉安全电平）。

### 地址规划

| IP | 地址 |
|---|---|
| pointcloud_accel_0 | 0x40000000 |
| stereo_rt_accel_0 | 0x40010000 |
| preprocess_accel_0 | 0x40020000 |
| dual_pmod_uart_0 | 0x40030000 |
| **dense_fp32_0 / wm_trajectory_0（RP 同槽位）** | **0x40040000** |
| status_led | 0x40060000 |

wm 从旧设计的独立地址 0x40070000 迁至与 dense 同槽位 0x40040000，是本次 DFX 化的关键改动之一。

### 加载路径

`switch_fast.py` 绕开 `import pynq`，直接走 `fpga_manager` sysfs（整片 `flags=0`、部分 `flags=1`），整片 2–4 s、部分 <0.5 s；加载前抢 overlay 锁并校验 sha256。

## 3. 关键设计决策

1. **单 RP 而非双 RP**：dense 与 wm 顶层 AXI 端口完全一致，可共用槽位；单 RP 简化 Pblock 布局与时序收敛，代价是二者互斥（符合「不会同时用」的语义）。
2. **blank RM**：提供「卸载模型加速器」状态，避免模型不在位时 RP 区输出悬空。
3. **Vivado 2023.2**：2022.1/2022.2 对 7 系列 DFX 存在 0 帧空 partial 工具 bug（`write_bitstream -cell` 产出 314 B），只能升级工具。
4. **浮点核跨版本适配**：HLS 2022.2 导出的 dense/wm 加密浮点 IP 在 2023.2 下须换库 + 改模块名 rev（详见 skill/dfx-build.md）。

## 4. 实现与验证

- 位流构建全流程（综合/实现/route/pr_verify/时序）在 Vivado 2023.2 跑通，`DFX_BUILD_OK`。
- pr_verify 两对（full-vs-wm、full-vs-blank）均 `compatible`。
- 时序 Setup 0 失败端点、Worst Slack +0.175 ns。
- 板端软件适配（位流替换、hwh/wm_accelerated.py/modes.json 地址迁移、package_manifest 同步）完成，全量 1585 条目 sha256 校验 0 不匹配。

## 5. 现状

- DFX 版位流已完成工具链级验证（综合/实现/pr_verify/时序）。
- 控制界面为「模型三选一」整片切换语义。
- 外参标定、深度定位依赖 640 深度模式（非 DFX），与 320 DFX 是两条独立路径。
