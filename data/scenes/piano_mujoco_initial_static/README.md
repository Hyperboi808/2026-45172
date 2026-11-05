# MuJoCo初始静态场景

包含机械臂/机械手、25键键盘、双目相机及外壳、定位标记和全部被引用的STL/PNG资源。此包用于查看原完整assembled_open姿态，加载后不进行物理步进、不播放轨迹、不运行PPO或舵机。

## 打开

在装有MuJoCo的Python环境中，进入本目录：

    python view_static.py

Windows也可双击open_static.bat。只有场景查看需要MuJoCo，上一份纯Python标准库Actor推理包仍独立使用。这个查看器只导入MuJoCo与标准库；没有PyTorch、OpenCV或训练代码依赖。本包用MuJoCo3.3.7完成加载和渲染核验。

如需安装匹配的查看环境：

    python -m pip install -r requirements.txt

无图形窗口的加载检查：

    python view_static.py --check

## 三份XML

- **scene.xml**：当前训练摆位，键盘底座位置(0.34,-0.04,0.04)米；保留已设定的旋转朝向、双目与外壳。
- **current_center.xml**：中轴对齐摆位，键盘底座位置(0.34,0,0.04)米；其余布局及机械臂初态相同。
- **original_scene.xml**：最初提供的scene.xml逐字节副本，供对照；该文件保留最初的键盘/相机布局。

打开中轴对齐版本：

    python view_static.py --scene current_center.xml

打开原始版本：

    python view_static.py --scene original_scene.xml

三种版本均由脚本自动选中原始assembled_open关键帧，保留全部主动和被动关节位置、完整控制目标。view_static.py不调用mj_step，持续恢复并显示初态，可以转动观察视角。

如果在其它MuJoCo查看器中直接打开XML，请先选择**assembled_open关键帧**；直接加载XML的qpos0不等于这组完整装配初态。

## 参数与校验

- 机械臂初始角：shoulder_pan=0，shoulder_lift=-3.17，wrist_roll=3.14，nano_mount_pitch=0.802弧度。
- 主场景和居中场景将13号角度分支写入XML为围绕初始位置的原范围，保持已有坐标约定。所有初始qpos/ctrl值与原文件一致。
- 双目基线24.882763毫米；相机参数见camera_config.json，14路舵机标定见servo_calibration.json。
- initial_state.json保存原完整关键帧；verification.json记录关节/控制目标和机械臂部件世界位置的逐项比较。
- preview_initial.png是主场景的实际MuJoCo渲染。assets_manifest.json记录资源SHA256，解压后加载无需访问原工程目录。

静态姿态与资源完整性验证，不代表PPO策略、动态碰撞/按键或实机验收通过。模型包和原实验文件均保留，未修改最初提供的场景/网格。
