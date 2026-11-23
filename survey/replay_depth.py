"""Generate full-rate depth archives. Run with the Room Survey Python environment."""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np

CACHE = Path.home()/'Library/Caches/RoomSurvey'
LABELS = {'anything':'Depth Anything V2', 'apple':'Apple Depth Pro 2',
          'lingbot':'LingBot-Map', 'gaussian':'Gaussian'}
METHODS = tuple(LABELS)
# Units of the saved depth values; 'map' means the camera map's own units.
UNITS = {'anything': 'relative inverse depth', 'apple': 'm', 'lingbot': 'relative depth', 'gaussian': 'map'}


def output(run, index, method):
    from ai_surfaces import index_path
    if method not in METHODS:
        raise ValueError('Unknown depth method')
    return run/('frame-depth-v1-'+index_path(run,index).stem)/method


def inventory(run, index):
    from gaussian_map import artifact_dir
    results = {}
    for method in METHODS:
        folder = output(run, index, method)
        running = False
        try:
            with (folder/'.build.lock').open('rb') as lock:
                fcntl.flock(lock, fcntl.LOCK_SH | fcntl.LOCK_NB)
        except BlockingIOError:
            running = True
        except FileNotFoundError:
            pass
        available = method != 'gaussian' or (artifact_dir(run,index)/'gaussians.ply').is_file()
        results[method] = dict(available=available, complete=(folder/'complete.json').is_file(), running=running)
    return results


def save(dest, frame, depth):
    # Keep the numeric output independent of the preview colour scale.
    path = dest/(Path(frame).stem+'.npy')
    with tempfile.NamedTemporaryFile(dir=dest, delete=False) as f:
        np.save(f,np.asarray(depth, dtype=np.float32))
    os.replace(f.name,path)


def predict(folder, dest, method):
    started = time.perf_counter()
    def save_timing(ready):
        (folder/'worker-timing.json').write_text(json.dumps({
            'Initialise runtime and model': ready-started,
            'Infer frames and write arrays': time.perf_counter()-ready,
        }))
    files=sorted(folder.glob('*.png'))
    sys.path[:0]=[str(CACHE/'depth-pro-2/packages'),str(CACHE/'depth-pro-2/source/src'),
                  str(CACHE/'lingbot/source')]
    def rgb(path):
        return cv2.cvtColor(cv2.imread(str(path)),cv2.COLOR_BGR2RGB)
    if method=='anything':
        import onnxruntime as ort
        options=ort.SessionOptions()
        options.intra_op_num_threads=4
        session=ort.InferenceSession(str(CACHE/'depth-v1/depth.onnx'),options,providers=['CPUExecutionProvider'])
        mean=np.float32([.485,.456,.406]); std=np.float32([.229,.224,.225])
        ready = time.perf_counter()
        for path in files:
            if (dest/path.with_suffix('.npy').name).exists(): continue
            im=cv2.resize(rgb(path),(392,294)).astype('f')/255
            tensor=((im-mean)/std).transpose(2,0,1)[None]
            save(dest,path,session.run(None,{session.get_inputs()[0].name:tensor})[0].squeeze())
        save_timing(ready)
        return
    import torch
    if method=='apple':
        from dataclasses import replace
        from depth_pro_2 import PRESETS, create_model_and_transforms
        config=replace(PRESETS['depthpro2-s'],checkpoint_uri=str(CACHE/'depth-pro-2/depthpro2-s-bgcc6f8bg3.pt'),fov_checkpoint_uri=None)
        model,transform=create_model_and_transforms(config,precision=torch.float32)
        ready = time.perf_counter()
        frames=(transform(rgb(p)) for p in files)
        for path,pred in zip(files,model.infer_video(frames,window=8,overlap=2,f_px=256.),strict=True):
            save(dest,path,pred['depth'].cpu().numpy().squeeze())
        save_timing(ready)
        return
    from lingbot_apple import load_model
    model=load_model(CACHE/'lingbot/lingbot-map.pt')
    ready = time.perf_counter()
    with torch.inference_mode():
        for start in range(0,len(files),48):
            group=files[start:start+48]
            if all((dest/p.with_suffix('.npy').name).is_file() for p in group): continue
            context=files[max(0,start-4):start]+group
            rgb_stack=np.stack([cv2.resize(rgb(p),(224,168),interpolation=cv2.INTER_AREA) for p in context])
            images=torch.from_numpy(rgb_stack).permute(0,3,1,2).float()/255
            pred=model.inference_streaming(images,num_scale_frames=min(2,len(context)),keyframe_interval=2,output_device=torch.device('cpu'))
            maps=pred['depth'][0].reshape(len(context),168,224).float().numpy()
            for path,z in zip(group,maps[-len(group):]): save(dest,path,z)
            del pred,images,maps
    save_timing(ready)


def gaussian_poses(run, frames, model):
    from scipy.spatial.transform import Rotation, Slerp
    known = {p['frame']: p for p in model['poses'] if 'rotation' in p}
    anchors = [i for i, frame in enumerate(frames) if frame in known]
    if not anchors:
        raise ValueError('This walk has no oriented camera poses.')
    times = np.arange(len(frames), dtype=float)
    try:
        with (run/'timestamps.csv').open() as stream:
            clock = {row['frame']: float(row['pi_camera_relative_us']) for row in csv.DictReader(stream)}
        measured = np.array([clock[f] for f in frames])
        if np.isfinite(measured).all() and np.all(np.diff(measured) > 0):
            times = measured
    except (OSError, KeyError, ValueError):
        pass
    t = times[anchors]
    xyz = np.array([known[frames[i]]['xyz'] for i in anchors])
    rotations = Rotation.from_matrix([known[frames[i]]['rotation'] for i in anchors])
    orientation = Slerp(t, rotations)(times.clip(t[0], t[-1])).as_matrix() if len(t)>1 else np.repeat(rotations.as_matrix(), len(frames), axis=0)
    positions = np.stack([np.interp(times,t,axis) for axis in xyz.T], axis=1)
    for i, name in enumerate(frames):
        source = ('Saved pose' if name in known else 'Nearest saved pose' if times[i]<t[0] or times[i]>t[-1] else 'Interpolated pose')
        yield name, positions[i], orientation[i], source


def build(run, method, index=None):
    m=json.loads((run/'survey.json').read_bytes())
    if index and index!=m['feature_index']: raise ValueError('Camera map changed')
    dest = output(run, m['feature_index'], method)
    dest.mkdir(exist_ok=True, parents=True)
    with (dest/'.build.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('This depth method is already processing this walk.') from None
        return build_frames(run, m, method, dest)


def build_frames(run, m, method, dest):
    from gaussian_map import rectified_frames, artifact_dir
    frames=sorted(p.name for p in (run/'frames').glob('[0-9]'*7+'.jpg'))
    if not frames: raise ValueError('Walk has no saved images')
    sources = {}
    start = time.perf_counter()
    stages = {}
    cached = sum(1 for f in frames if (dest/(Path(f).stem+'.npy')).is_file()) if method in ('anything','lingbot') else 0
    if method=='gaussian':
        from gaussian_mesh import render_depth,read_splats
        splats=read_splats(artifact_dir(run,m['feature_index'])/'gaussians.ply')
        far=4*np.linalg.norm(np.ptp(np.percentile(splats[0],[10,90],axis=0),axis=0))
        stages['Load trained splats'] = time.perf_counter()-start
        tick = time.perf_counter()
        for name, position, rotation, source in gaussian_poses(run, frames, m):
            save(dest,name,render_depth(splats,np.float32(position),np.float32(rotation),far))
            sources[name] = source
        stages['Render and save'] = time.perf_counter()-tick
    else:
        with tempfile.TemporaryDirectory() as tmp:
            for name,img in rectified_frames(run,m,frames):
                if not cv2.imwrite(str(Path(tmp)/Path(name).with_suffix('.png')),img):raise IOError('Cannot save rectified image')
            stages['Rectify and stage images'] = time.perf_counter()-start
            tick = time.perf_counter()
            subprocess.run([str(CACHE/'lingbot/env/bin/python'),'-B',__file__,'--worker',tmp,str(dest),method],check=True)
            elapsed = time.perf_counter()-tick
            worker = json.loads((Path(tmp)/'worker-timing.json').read_text())
            stages['Start and stop worker'] = max(0.,elapsed-sum(worker.values()))
            stages.update(worker)
    count=len(list(dest.glob('*.npy')))
    if count!=len(frames): raise ValueError('Incomplete depth sequence')
    manifest = dest/'complete.tmp'
    manifest.write_text(json.dumps(dict(saved=count,total=len(frames),poses=sources,timing=dict(seconds=time.perf_counter()-start,frames=len(frames)-cached,stages=stages))))
    manifest.replace(dest/'complete.json')
    return count


def preview(path,method):
    z=np.load(path).astype('f')
    good = np.isfinite(z)
    if method != 'anything':
        good &= z > 0
    values=np.zeros_like(z)
    if good.any():
        if method=='apple':values[good]=1-np.clip(z[good]/8,0,1)
        else:
            src=z[good] if method=='anything' else 1/z[good]
            lo,hi=np.percentile(src,[2,98]);values[good]=np.clip((src-lo)/max(hi-lo,1e-8),0,1)
    img=cv2.applyColorMap(np.uint8(values*255),cv2.COLORMAP_VIRIDIS);img[~good]=0
    return cv2.imencode('.png',img)[1].tobytes()


if __name__=='__main__':
    if sys.argv[1]=='--worker':predict(Path(sys.argv[2]),Path(sys.argv[3]),sys.argv[4])
    else:
        parser=argparse.ArgumentParser();parser.add_argument('method',choices=METHODS);parser.add_argument('runs',type=Path,nargs='+')
        args=parser.parse_args()
        for run in args.runs:print(run.name,build(run,args.method),flush=True)
