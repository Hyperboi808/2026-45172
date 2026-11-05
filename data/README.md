# 数据说明

## weights/

| 目录 | 内容 | 用途 |
|---|---|---|
| `ppo/` | PPO 策略网络权重 `weights_fp32.bin`、`weights_full_fp32.bin` + 模型配置（servo_config.json、model_manifest.json、teacher_keyframes_v11.json 等）| dense_fp32 RM 的推理权重 |
| `wm/` | WM 轨迹头权重 `model_weights.bin` + `tensor_manifest.json`（含权重 sha256）+ 轨迹/标定配置 | wm_trajectory RM 的推理权重 |

## scenes/

| 目录 | 内容 | 用途 |
|---|---|---|
| `cup_simulation_motion_scene/` | 杯子操作仿真场景（76 个 STL + scene.xml）| WM 数字孪生 / 仿真回放 |
| `piano_mujoco_initial_static/` | 钢琴静态场景（72 个 STL + scene.xml）| PPO 任务场景 |

> 场景目录名保留了代码引用的原始相对路径（`scene.xml` 的 meshdir 为相对路径 `../../../场景/...`），供控制平台/仿真直接加载。数据来源于项目仓库 `/home/aup/Real_Sim_Real/`，与板端部署无耦合。
