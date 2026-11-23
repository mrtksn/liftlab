"""Offline point-cloud and Gaussian mapping for Room Survey walks."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

import cv2
import numpy as np

from ai_surfaces import F, H, W, index_path, rectification

# Use a pinhole crop of the stored fisheye calibration for both backends.
WIDTH, HEIGHT, FOCAL = 4 * W, 4 * H, 4 * F
K = np.array([[FOCAL, 0, (WIDTH-1)/2], [0, FOCAL, (HEIGHT-1)/2], [0, 0, 1.]])
BRUSH_CACHE = Path.home() / 'Library/Caches/RoomSurvey/brush-v0.3.0/brush_app'


def artifact_dir(run, index, variant=''):
    """Splats trained with non-default settings live beside the default ones."""
    return run / ('gaussian-v1-' + index_path(run,index).stem + (f'-{variant}' if variant else ''))


def rectified_frames(run, model, frames):
    remap = cv2.resize(rectification(index_path(run, model['feature_index'])),
                       (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
    for name in frames:
        raw = cv2.imread(str(run/'frames'/name))
        if raw is None:
            raise ValueError(f'Missing frame {name}')
        yield name, cv2.remap(raw, remap, None, cv2.INTER_LINEAR)


def export_droid(run, model, dest):
    if dest.exists():
        raise FileExistsError(f'Choose a new output directory: {dest}')
    dest.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(dir=dest.parent) as tmp:
        stage = Path(tmp)/'export'
        stage.mkdir()
        names = sorted(p.name for p in (run/'frames').glob('[0-9]'*7+'.jpg'))
        if not names:
            raise ValueError('Walk has no images')
        for name, image in rectified_frames(run, model, names):
            if not cv2.imwrite(str(stage/('frame'+name)), image):
                raise OSError('Could not save rectified frame')
        shutil.copy2(run/'timestamps.csv', stage/'timestamps.csv')
        cfg = dict(inherit_from='./configs/Dynamic/YouTube/youtube.yaml', scene=run.name,
                   stride=1, save_gt_poses=False, mapping=dict(enable=True),
                   cam=dict(H=HEIGHT, W=WIDTH, H_out=HEIGHT, W_out=WIDTH, fx=FOCAL,
                            fy=FOCAL, cx=K[0,2], cy=K[1,2], H_edge=0, W_edge=0),
                   data=dict(input_folder=str(dest.resolve()), output='./Outputs/RoomSurvey'))
        (stage/'walk.yaml').write_text(json.dumps(cfg, indent=2))
        stage.rename(dest)


def write_cloud(path, points):
    packed = np.empty(len(points), dtype=[('xyz','<f4',3), ('rgb','u1',3)])
    packed['xyz'], packed['rgb'] = points[:,:3], np.clip(points[:,3:], 0, 255)
    header = ['ply','format binary_little_endian 1.0',f'element vertex {len(points)}']
    header += ['property float '+s for s in ('x','y','z')]
    header += ['property uchar '+s for s in ('red','green','blue')]
    with path.open('wb') as out:
        out.write(('\n'.join(header+['end_header',''])).encode())
        out.write(packed.tobytes())


def triangulate(anchor, other, uv, gray):
    image, rotation, eye = anchor
    target, other_rotation, other_eye = other
    new, ok, _ = cv2.calcOpticalFlowPyrLK(gray[image], gray[target], uv, None)
    back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray[target], gray[image], new, None)
    # Coordinates are expressed in the anchor camera for conditioning.
    rot = other_rotation.T @ rotation
    shift = other_rotation.T @ (eye-other_eye)
    p1 = K @ np.eye(3,4)
    p2 = K @ np.column_stack([rot, shift])
    homogeneous = cv2.triangulatePoints(p1, p2, uv.T, new.T)
    with np.errstate(divide='ignore', invalid='ignore'):
        xyz = (homogeneous[:3]/homogeneous[3]).T
        other_xyz = xyz @ rot.T+shift
        reproj = other_xyz @ K.T
        reproj = reproj[:,:2]/reproj[:,2,None]
        angle = np.linalg.norm(np.cross(xyz, xyz-(other_eye-eye)@rotation),axis=1)
        angle /= np.linalg.norm(xyz,axis=1)*np.linalg.norm(other_xyz,axis=1)
    valid = (ok.ravel()>0) & (back_ok.ravel()>0) & (np.linalg.norm(uv-back,axis=1)<.7)
    valid &= (xyz[:,2]>0) & (other_xyz[:,2]>0) & (angle>.018)
    valid &= (np.linalg.norm(reproj-new,axis=1)<.9) & np.isfinite(xyz).all(axis=1)
    return xyz, valid


def write_brush_image(path, image):
    ok, encoded = cv2.imencode('.jpg', image)
    if not ok:
        raise OSError('Could not encode rectified frame')
    data = encoded.tobytes()
    # Brush 0.3.0 reads exactly 16,387 bytes for its initial format probe.
    # Add a legal JPEG comment before EOI without changing decoded pixels.
    missing = 16387 - len(data)
    if missing > 0:
        padding = bytes(max(0, missing - 4))
        comment = b'\xff\xfe' + (len(padding) + 2).to_bytes(2, 'big') + padding
        data = data[:-2] + comment + data[-2:]
    path.write_bytes(data)


def prepare_brush(run, model, dest, dense_seed=True):
    if len(model.get('points',[]))<3:
        raise ValueError('Rebuild the camera landmarks before Gaussian mapping')
    dest.mkdir()
    images = dest/'images'
    images.mkdir()
    poses = sorted((p for p in model['poses'] if 'rotation' in p and not p.get('method')),
                   key=lambda p:p['frame'])
    if len(poses)<6:
        raise ValueError('At least six registered, oriented camera views are required')
    test_indices = set(range(4,len(poses),10))
    transforms = [[], []]
    train_count = 0
    window = deque(maxlen=3)
    points = np.asarray(model['points'],float)[:,:6]
    scale = max(1e-6,np.linalg.norm(np.ptp(np.percentile(points[:,:3],[10,90],axis=0),axis=0)))
    # Validate depth independently on both sides of each anchor. Skip long time gaps.
    y,x = np.mgrid[6:HEIGHT-6:4,6:WIDTH-6:4]
    grid = np.column_stack((x.ravel(),y.ravel())).astype('f')
    dense = []

    def grow_cloud():
        left, anchor, right = [item[0] for item in window]
        if int(right[0][:7])-int(left[0][:7])>16:
            return
        gray = {item[0][0]: item[1] for item in window}
        _, rot, eye = anchor
        a, ok_a = triangulate(anchor,left,grid,gray)
        b, ok_b = triangulate(anchor,right,grid,gray)
        mask = ok_a & ok_b & (np.abs(a[:,2]-b[:,2])<.06*np.minimum(a[:,2],b[:,2]))
        mask &= (a[:,2]<3*scale)
        xyz = ((a[mask]+b[mask])*.5) @ rot.T+eye
        u,v = grid[mask].astype(int).T
        dense.append(np.column_stack([xyz, window[1][2][v,u,::-1]]))

    for i, (p,(name,bgr)) in enumerate(zip(poses,rectified_frames(run,model,[p['frame'] for p in poses]))):
        write_brush_image(images/name, bgr)
        rotation, eye = np.asarray(p['rotation']), np.asarray(p['xyz'])
        c2w = np.eye(4)
        c2w[:3,:3], c2w[:3,3] = rotation @ np.diag([1,-1,-1]), eye
        transforms[i in test_indices].append(dict(file_path='images/'+name,transform_matrix=c2w.tolist()))
        if i not in test_indices:
            train_count += 1
            if dense_seed:
                window.append(((name,rotation,eye),cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY),bgr))
                if len(window)==3:
                    grow_cloud()
    for split, frames in zip(('train','val'),transforms):
        data = dict(w=WIDTH,h=HEIGHT,fl_x=FOCAL,fl_y=FOCAL,cx=WIDTH/2,cy=HEIGHT/2,
                    frames=frames,ply_file_path='cloud.ply')
        (dest/f'transforms_{split}.json').write_text(json.dumps(data))
    samples = np.concatenate([points,*dense])
    cells = np.floor(samples[:,:3]/(scale/420)).astype(np.int64)
    _, keep = np.unique(cells,axis=0,return_index=True)
    if not dense_seed:
        keep = np.arange(len(points))
    write_cloud(dest/'cloud.ply',samples[keep])
    return dict(train_views=train_count,test_views=len(test_indices),registered_oriented=len(poses),
                original_points=len(points),three_view_samples=len(samples)-len(points),cloud_points=len(keep),
                units=model['units'],backend='Brush 0.3.0',feature_index=model['feature_index'])


def build(run, model, dest, brush=None, steps=1500, dense_seed=True):
    if dest.exists() or steps < 1:
        raise ValueError('Choose a new output directory and a positive training step count')
    brush = brush or shutil.which('brush_app') or str(BRUSH_CACHE)
    if not Path(brush).is_file():
        raise ValueError('Install Brush 0.3.0 or set ROOM_SURVEY_BRUSH')
    # Publish only complete training runs, including on cancellation or a failed GPU job.
    with tempfile.TemporaryDirectory(dir=dest.parent,prefix='.gaussian-') as tmp:
        stage = Path(tmp)
        stats = prepare_brush(run,model,stage/'dataset',dense_seed)
        cmd = [brush,str(stage/'dataset'),'--total-steps',str(steps),'--eval-every',str(steps),
               '--export-every',str(steps),'--eval-save-to-disk','--export-path',str(stage),
               '--export-name','gaussians.ply','--max-splats','120000','--sh-degree','0',
               '--refine-every','100','--growth-select-fraction','0.3']
        process = subprocess.run(cmd, stderr=subprocess.PIPE, text=True, errors='replace')
        if process.returncode:
            detail = ' '.join(process.stderr.split())[-450:]
            raise ValueError(detail or f'Brush exited with status {process.returncode}')
        eval_dir = stage/f'eval_{steps}'
        heldout = json.loads((stage/'dataset/transforms_val.json').read_text())['frames']
        scores = []
        for item in heldout:
            name = Path(item['file_path'])
            photo = cv2.imread(str(stage/'dataset'/name))
            render = cv2.imread(str(eval_dir/name.with_suffix('.png').name))
            if render is None or photo.shape!=render.shape:
                raise ValueError(f'Missing evaluation for {name}')
            scores.append(dict(frame=name.name,psnr_db=round(cv2.PSNR(photo,render),2)))
        stats.update(steps=steps,evaluation=scores,mean_psnr_db=round(np.mean([s['psnr_db'] for s in scores]),2))
        ready = stage/'ready'
        ready.mkdir()
        (stage/'gaussians.ply').rename(ready/'gaussians.ply')
        (stage/'dataset/cloud.ply').rename(ready/'cloud.ply')
        eval_dir.rename(ready/'renders')
        (ready/'reference').mkdir()
        for item in heldout:
            name = Path(item['file_path'])
            shutil.copyfile(stage/'dataset'/name,ready/'reference'/name.name)
        (ready/'report.json').write_text(json.dumps(stats,indent=2))
        ready.rename(dest)
    return stats


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('prepare-droid','brush'))
    parser.add_argument('walk',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--steps',type=int,default=1500)
    parser.add_argument('--sparse-seed',action='store_true',help='Use only saved landmarks as the Gaussian initializer')
    args = parser.parse_args()
    model = json.loads((args.walk/'survey.json').read_bytes())
    if args.action=='prepare-droid':
        export_droid(args.walk,model,args.output)
    else:
        print(json.dumps(build(args.walk,model,args.output,os.getenv('ROOM_SURVEY_BRUSH'),args.steps,not args.sparse_seed)))
