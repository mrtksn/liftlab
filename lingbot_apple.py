"""Run LingBot-Map with native PyTorch attention on Apple Silicon."""
import argparse
import base64
import gc
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from lingbot_map.models.gct_stream import GCTStream
from lingbot_map.utils.pose_enc import pose_encoding_to_extri_intri


def reconstruct(folder, weights, dest, stride, limit):
    if not torch.backends.mps.is_available():
        raise RuntimeError("Metal (PyTorch MPS) is required")
    files = sorted(folder.glob("frame*.jpg"))[::stride][:limit]
    if len(files) < 2:
        raise ValueError("At least two rectified images are required")
    if limit is None and files[-1] != sorted(folder.glob("frame*.jpg"))[-1]:
        files.append(sorted(folder.glob("frame*.jpg"))[-1])
    rgb = np.stack([cv2.cvtColor(cv2.resize(cv2.imread(str(f)), (224,168),
                    interpolation=cv2.INTER_AREA),cv2.COLOR_BGR2RGB) for f in files])
    images = torch.from_numpy(rgb).permute(0,3,1,2).float()/255
    print(f"Loading {weights.name} for {len(files)} views",flush=True)
    checkpoint = torch.load(weights,map_location="cpu",weights_only=True,mmap=True)
    model = GCTStream(enable_3d_rope=True,use_sdpa=True,kv_cache_sliding_window=8,
                      kv_cache_scale_frames=2,camera_num_iterations=2)
    model.load_state_dict(checkpoint.get("model",checkpoint),strict=True,assign=True)
    # Assemble the complex rotation table on CPU; M1 cannot concatenate the
    # expanded complex views. Attention, image encoding and heads run on Metal.
    rope=model.aggregator.rope3d
    original=rope.forward
    def positions(ppf,pph,ppw,patch_start_idx,device,**kw):
        return original(ppf,pph,ppw,patch_start_idx,"cpu",**kw).to(torch.complex64).to(device)
    rope.forward=positions
    model.eval().to("mps")
    del checkpoint
    gc.collect()
    start=time.monotonic()
    with torch.inference_mode():
        pred=model.inference_streaming(images,num_scale_frames=2,keyframe_interval=2,output_device=torch.device("cpu"))
        extr,intr=pose_encoding_to_extri_intri(pred['pose_enc'].float(),images.shape[-2:])
    poses=extr[0].numpy()
    depth=pred['depth'][0].reshape(len(files),168,224).float().numpy()
    conf=pred['depth_conf'][0].reshape(depth.shape).float().numpy()
    intr=intr[0].numpy()
    if not all(np.isfinite(a).all() for a in (poses,intr,depth,conf)):
        raise ValueError("Model produced non-finite predictions")
    clouds=[]
    yy,xx=np.mgrid[:168:4,:224:4]
    for pose,k,d,c,col in zip(poses,intr,depth,conf,rgb):
        z=d[yy,xx]
        keep=(z>0)&(c[yy,xx]>1.5)
        xyz=np.stack([(xx-k[0,2])*z/k[0,0],(yy-k[1,2])*z/k[1,1],z],axis=-1)
        world=xyz[keep]@pose[:,:3].T+pose[:,3]
        clouds.append(np.column_stack([world,col[yy,xx][keep]]))
    points=np.concatenate(clouds)
    if not len(points): raise ValueError("No confident depth samples")
    dest.mkdir(parents=True,exist_ok=False)
    packed=np.empty(len(points),dtype=[('xyz','<f4',3),('rgb','u1',3)])
    packed['xyz']=points[:,:3];packed['rgb']=points[:,3:]
    with (dest/'cloud.ply').open('wb') as f:
        f.write(('ply\nformat binary_little_endian 1.0\nelement vertex '+str(len(points))+'\n'+
                 ''.join('property float '+n+'\n' for n in 'xyz')+
                 ''.join('property uchar '+n+'\n' for n in ['red','green','blue'])+'end_header\n').encode())
        f.write(packed.tobytes())
    np.savez_compressed(dest/'predictions.npz',depth=depth,confidence=conf,intrinsics=intr,camera_to_world=poses,frames=[f.name for f in files])
    report=dict(backend='PyTorch MPS',seconds=round(time.monotonic()-start,1),stride=stride,views=len(files),points=len(points),units='relative',frames=[f.name for f in files],centers=poses[:,:,3].tolist())
    (dest/'trajectory.json').write_text(json.dumps(report))
    sampled=[cloud[::3] for cloud in clouds]
    display=np.concatenate(sampled)
    low,high=np.percentile(display[:,:3],[4,96],axis=0)
    def encode(a):
        return base64.b64encode(a.tobytes()).decode('ascii')
    view=dict(xyz=encode(display[:,:3].astype('<f4')),rgb=encode(display[:,3:].astype('u1')),
              center=((high+low)/2).tolist(),extent=max(float(np.max(high-low))*.65,.001),
              ends=np.cumsum([len(c) for c in sampled]).tolist(),centers=report['centers'],frames=report['frames'])
    page=Path(__file__).with_name('lingbot_view.html').read_text().replace('SCENE_DATA',json.dumps(view))
    (dest/'view.html').write_text(page)
    print({k:v for k,v in report.items() if k not in ('frames','centers')},flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder',type=Path);p.add_argument('weights',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--stride',type=int,default=4);p.add_argument('--limit',type=int)
    a=p.parse_args();reconstruct(a.folder,a.weights,a.output,a.stride,a.limit)
