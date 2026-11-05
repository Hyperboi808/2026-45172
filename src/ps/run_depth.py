"""User-invoked bounded capture or offline processing; no model/motor interface."""
import argparse
import json
from pathlib import Path
import sys
import time

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--backend',choices=('ps','pl'),default='pl')
    p.add_argument('--image',help='Existing packed 1280x480 grayscale/color image')
    p.add_argument('--frames',type=int,default=1)
    p.add_argument('--device',type=int,default=0)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    if not 1<=args.frames<=300:raise ValueError('Bounded 1..300 frame capture required')
    root=Path(__file__).resolve().parent;out=Path(args.output).resolve()
    out.mkdir(exist_ok=False,parents=True)
    sys.path.insert(0,str(root/'mode640'))
    import cv2
    import numpy as np
    from depth640 import Depth640
    from depth_geometry import save_preview
    runtime=Depth640(args.backend);camera=None;rows=[]
    try:
        if args.image:
            image=cv2.imdecode(np.fromfile(args.image,np.uint8),cv2.IMREAD_GRAYSCALE)
            if image is None:raise ValueError('Unreadable image')
            limit=1
        else:
            camera=cv2.VideoCapture(args.device,cv2.CAP_V4L2)
            camera.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc(*'MJPG'))
            camera.set(cv2.CAP_PROP_FRAME_WIDTH,1280);camera.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
            camera.set(cv2.CAP_PROP_FPS,30);camera.set(cv2.CAP_PROP_BUFFERSIZE,1)
            if not camera.isOpened():raise RuntimeError('USB camera unavailable')
            limit=args.frames
        for frame in range(1,limit+1):
            if camera is not None:
                ok,bgr=camera.read();receipt=time.monotonic()
                if not ok:raise RuntimeError('Capture failed')
                image=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY) if bgr.ndim==3 else bgr
            else:receipt=None
            result=runtime.process(image,frame_id=frame,receipt_s=receipt)
            folder=out/f'frame_{frame:06d}';folder.mkdir()
            save_preview(folder,result['left'],result['right'],result['disparity'],result['depth_m'])
            metadata=dict(result['metadata'],frame_id=frame,input_is_live=camera is not None)
            (folder/'metadata.json').write_text(json.dumps(metadata,indent=2))
            rows.append(metadata)
    finally:
        if camera is not None:camera.release()
        runtime.close()
    (out/'capture_manifest.json').write_text(json.dumps(rows,indent=2))
    print(json.dumps(rows,indent=2))
if __name__=='__main__':main()
