# board/ —— 上板工程与运行脚本

## 运行脚本

| 文件 | 用途 |
|---|---|
| `switch_fast.py` | 无 pynq 依赖的位流快加载器（整片 320/640 + 部分 dense/wm/blank）|
| `depth_stream_fast.py` | 640 深度裸 TCP 流前端 |
| `inject_dfx_partial.sh` | 镜像注入：DFX partial 位流 + 地址适配软件 + manifest 同步 |
| `inject_v2.sh` | 镜像注入：swap 重建 + 板端脚本 + DFX 构建文件 |

## 验证

> 工具链级验证（pr_verify compatible×2、时序 Worst Slack +0.175 ns、manifest 全量 1585 条目 0 不匹配）见 `sim/verification_summary.md`。
