# 四-overlay 视觉 DFX 新镜像（2026-11-09）

与 DFX 320（模型热切换）不同的新一代 DFX 设计：**像素档 × UART 的整片 DFX**。
静态区 + 两个独立 pblock（stereo RP / preprocess RP）+ UART RP，
四档热插拔：**320 像素 / 640 像素 / 1280 像素 / UART 舵机协议**，
任意像素档可与 UART 并存，切换经 fpga_manager partial + 复位脉冲（0x40050000）。

## 文件清单

| 文件 | 说明 | sha256（前 16 位） |
|---|---|---|
| `vision_base.bit` | 整片位流（静态区 + 默认 RM） | `f91d401db7e32b88` |
| `partial_stereo_320.bit` | 320 档 stereo RM | `1917d99871cc0e76` |
| `partial_pre_320.bit` | 320 档 preprocess RM | `bd9de0d0b48fd9d9` |
| `partial_stereo_640.bit` | 640 档 stereo RM | `62224c9cb68b39db` |
| `partial_pre_640.bit` | 640 档 preprocess RM | `6a98f0d82f03c9b6` |
| `partial_stereo_1280.bit` | 1280 档 stereo RM（D=64, WSIZE=5, 16 路并行） | `498c9d053ae1f916` |
| `partial_pre_1280.bit` | 1280 档 preprocess RM（TILE=8, MAX_ROWS=72） | `64ecdf9eb2b51ee4` |
| `partial_stereo_blank.bit` / `partial_pre_blank.bit` | blank RM（卸载加速器，防 AXI 死锁） | `ea31468ebc6a8de3` / `a407667f1404844e` |
| `partial_uart.bit` / `partial_uart_blank.bit` | UART 舵机协议 RM / 卸载 | `38a213a000f70dc2` / `c6b596cb87cc8c36` |
| `partial_manifest.json` | 全部位流 sha256（switch_vision.py 校验用） | — |
| `switch_vision.py` | 档位切换器（fpga_manager partial + 复位脉冲 + LED） | — |

## 地址映射（静态区）

```
pointcloud      0x40000000    stereo_rt_accel_0   0x40010000
preprocess      0x40020000    dual_pmod_uart_0    0x40030000
model_rst_gpio  0x40050000    status_led_gpio     0x40060000
```

## 板端使用

```bash
# 部署：vision_dfx/ 与 mode1280/ 放 VISION_ROOT（/usr/local/lib/pynq-robot-models-v11-20261102-01）下
sudo python3 vision_dfx/switch_vision.py --full          # 整片加载 + UART（默认 640）
sudo python3 vision_dfx/switch_vision.py --pixel 1280    # 热切 1280 档
sudo python3 vision_dfx/switch_vision.py --pixel off     # 卸载像素档
sudo python3 vision_dfx/switch_vision.py --uart on|off   # UART 档（可与像素档并存）
```

1280 深度服务（PL 版，不下载位流、/dev/mem 直驱）见 `../mode1280/`，
与 640 服务共用 `/run/lock/pynq-depth640-runtime.lock` 互斥。

## 构建

`build/dfx_vision.tcl`（Vivado 2023.2+，7 系列 DFX 空 partial 需要 2023.2 以上修复）
`build/rebuild_1280.tcl`（仅重建 1280 两枚 RM：synth_rm_ooc + swap_rm_pixel + pr_verify）

验收：1280 档 pr_verify PASSED、setup 0.451 ns / hold 0.028 ns；RAMB36=36 ≤ pblock_pre 40。
