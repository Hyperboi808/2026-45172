"""Open the original complete assembled_open posture without physics stepping."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', default='scene.xml', help='scene.xml,current_center.xml,or original_scene.xml')
    parser.add_argument('--check', action='store_true', help='Load and verify without opening a window')
    args = parser.parse_args()
    try:
        import mujoco
    except ImportError:
        raise SystemExit('MuJoCo is required for this scene. Run: python -m pip install -r requirements.txt')
    root = Path(__file__).resolve().parent
    path = Path(args.scene)
    if not path.is_absolute():
        path = root / path
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    data = mujoco.MjData(model)
    key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'assembled_open')
    if key < 0:
        raise SystemExit('The complete original assembled_open keyframe is missing')

    def restore():
        mujoco.mj_resetDataKeyframe(model, data, key)
        mujoco.mj_forward(model, data)

    restore()
    expected = json.loads((root / 'initial_state.json').read_text(encoding='utf-8'))
    pose_matches = list(data.qpos) == expected['qpos'] and list(data.ctrl) == expected['ctrl_xml_actuator_order']
    if not pose_matches:
        raise SystemExit('Loaded posture/control vector differs from the original full keyframe')
    info = {'scene': path.name, 'keyframe': 'assembled_open', 'full_posture_verified': pose_matches,
            'qpos_count': model.nq, 'actuator_count': model.nu, 'time_s': float(data.time),
            'piano_position_m': model.body('k25_frame').pos.tolist(), 'physics_stepping': False}
    print(json.dumps(info, ensure_ascii=False), flush=True)
    if args.check:
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        viewer.cam.lookat[:] = [.17, .12, .30]
        viewer.cam.distance = 1.25
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        while viewer.is_running():
            with viewer.lock():
                restore()
            viewer.sync()
            time.sleep(.02)


if __name__ == '__main__':
    main()
