# 验证结果汇总（DFX 320）

> 来源：Vivado 2023.2 batch 构建日志（dfx320_2023.log，Windows 构建机）与本地 out320dfx/ 报告。以下均为工具链实测输出，非估算。

## 1. PR Verify（静态区一致性）

DFX 的关键要求：所有部分位流与整片位流共享**完全一致**的静态区。Vivado `pr_verify` 对两对 checkpoint 的判定：

| 对比 | 结果 | 结论 |
|---|---|---|
| config_full vs config_wm | `PR_VERIFY: ... are compatible` | PASSED（静态区一致）|
| config_full vs config_blank | `PR_VERIFY: ... are compatible` | PASSED（静态区一致）|

## 2. 时序收敛（Route 后）

`DFX_BUILD_OK setup_full=0.051 setup_wm=0.088 setup_blank=0.458`

| 配置 | Setup Slack | Hold Slack |
|---|---|---|
| config_full（dense RM）| +0.051 ns | +0.020 ns |
| config_wm（wm RM）| +0.088 ns | +0.020 ns |
| config_blank（blank RM）| +0.458 ns | +0.020 ns |

最终时序报告（timing_summary_*.rpt）汇总：**Setup 0 失败端点、Worst Slack +0.175 ns；Hold 0 失败端点**。

## 3. 位流产物

| 文件 | 大小 | sha256（前 16）|
|---|---|---|
| vision_base_320_dfx.bit（整片，静态+dense RM）| 4,045,681 B | c11cdd9e… |
| partial_dense_fp32.bit | 2,059,006 B | ea8193fd… |
| partial_wm_trajectory.bit | 2,059,006 B | 7aafb41f… |
| partial_models_blank.bit | 2,059,006 B | c1113e4b… |

三枚 partial 位流大小完全一致（共享同一 RP 槽位），均为非空（2 MB）。对照：2022.x 工具的空 partial bug 会产出 314 B（0 配置帧），本构建已规避。

## 4. 资源利用率（dense RM，瓶颈项）

RP 槽位容量设计依据：dense_fp32 为 DSP 瓶颈（44 DSP48E1 / 6737 LUT），wm（7551 LUT / 8 BRAM / 3 DSP）与 blank 远小于 dense。详见 build/utilization_hierarchical.rpt。

## 5. 板端软件适配

- 板端软件适配（地址 0x40070000→0x40040000、位流替换、package_manifest 同步）已完成并全量校验通过（1585 条目 0 不匹配）。
