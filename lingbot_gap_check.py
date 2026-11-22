"""Check saved LingBot predictions against registered camera poses and independent range evidence."""
import argparse
import csv
import json
from itertools import combinations
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

# Exploratory tolerances for this offline comparison, in metres and degrees.
MIN_GAP_S = 1.
MAX_ANCHORS = 8
MIN_ANCHORS = 4
MAX_POSITION_M = .15
MAX_ANGLE_DEG = 10.


def load_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def median(values):
    return float(np.median(values)) if len(values) else None


def pose_fit(keys, predicted, reference):
    """Similarity fit for nearly straight paths. Paired attitudes resolve rotation."""
    if len(keys) < MIN_ANCHORS:
        return None
    src = np.stack([predicted[i] for i in keys])
    dst = np.stack([reference[i] for i in keys])
    # A medoid is insensitive to isolated attitude flips.
    rel = Rotation.from_matrix(dst[:, :, :3] @ src[:, :, :3].transpose(0, 2, 1))
    scores = [np.median((r.inv()*rel).magnitude()) for r in rel]
    best = rel[np.argmin(scores)]
    angles = (best.inv()*rel).magnitude()
    rot = rel[angles <= max(.035, 2.5*np.median(angles))].mean().as_matrix()
    x = src[:, :, 3] @ rot.T
    y = dst[:, :, 3]
    ratios = []
    for a, b in combinations(range(len(keys)), 2):
        delta = x[b]-x[a]
        if np.linalg.norm(y[b]-y[a]) < .08 or delta@delta < 1e-10:
            continue
        ratios.append((y[b]-y[a])@delta/(delta@delta))
    scale = median(ratios)
    if scale is None or scale <= 0 or len(ratios) < 3:
        return None
    return rot, scale, np.median(y-scale*x, axis=0)


def position_only_fit(keys, predicted, reference):
    # Independent scale/rotation estimate from camera centers (Umeyama).
    # Collinear centers would leave one rotational degree of freedom ambiguous.
    if len(keys) < MIN_ANCHORS:
        return None
    x, y = [np.stack([d[k][:,3] for k in keys]) for d in (predicted,reference)]
    xc, yc = x-x.mean(0), y-y.mean(0)
    u, singular, vt = np.linalg.svd(yc.T @ xc)
    if singular[0] < 1e-9 or singular[1] < .01*singular[0]:
        return None
    signs = np.array([1.,1.,np.linalg.det(u @ vt)])
    rot = (u*signs) @ vt
    scale = (singular @ signs) / np.sum(xc*xc)
    return rot, scale, y.mean(0)-scale*rot@x.mean(0)


def pose_errors(fit, keys, predicted, reference):
    if fit is None or not keys:
        return None
    rot, scale, offset = fit
    errors, angles = [], []
    for key in keys:
        src, dst = predicted[key], reference[key]
        errors.append(float(np.linalg.norm(scale*rot@src[:, 3]+offset-dst[:, 3])))
        angles.append(float(np.degrees(Rotation.from_matrix(dst[:, :3].T @ rot @ src[:, :3]).magnitude())))
    return dict(frames=keys, position_m=errors, attitude_deg=angles, median_m=median(errors),
                median_deg=median(angles), p90_m=float(np.percentile(errors, 90)),
                p90_deg=float(np.percentile(angles, 90)))


def read_ranges(run, frames):
    independent = (run/'range.csv').is_file()
    readings = []
    if independent:
        for row in load_csv(run/'range.csv'):
            a, b = int(row['pi_read_start_ns']), int(row['pi_read_end_ns'])
            if 0 <= b-a <= 80_000_000:
                readings.append(((a+b)//2, int(row['tof_raw'] or 0)))
        readings.sort()
    clocks = np.array([r[0] for r in readings])
    by_frame = {}
    for i,row in frames.items():
        camera = int(row['pi_encoder_callback_ns'])
        if independent and len(clocks):
            j = int(np.searchsorted(clocks, camera))
            j = min({max(0,j-1), min(j,len(clocks)-1)}, key=lambda k:abs(int(clocks[k])-camera))
            stamp, raw = readings[j]
        elif not independent and row.get('pi_tof_read_end_ns') and row.get('tof_raw'):
            stamp, raw = int(row['pi_tof_read_end_ns']), int(row['tof_raw'])
        else:
            continue
        if abs(stamp-camera) <= 80_000_000 and 0 < raw < 250:
            by_frame[i] = raw/104.
    return by_frame, 'nearest independent read' if independent else 'camera-paired read'


def check_depth(keys, fit, ranges, predictions, origin, axis):
    paired = [i for i in keys if i in ranges]
    result = dict(valid_range_frames=paired, comparisons=[], low_confidence=[], uneven_depth=[], outside_image=[])
    if fit is None:
        result['reason'] = 'No positive pose scale'
        return result
    scale = fit[1]
    for key in paired:
        depth, confidence, k = predictions[key]
        endpoint = origin + ranges[key]*axis
        if endpoint[2] <= 0 or axis[2] <= 0:
            result["outside_image"].append(key)
            continue
        px = k @ endpoint
        x,y = np.rint(px[:2]/px[2]).astype(int)
        if not (2 <= x < depth.shape[1]-2 and 2 <= y < depth.shape[0]-2):
            result["outside_image"].append(key)
            continue
        z = depth[y-2:y+3,x-2:x+3]
        c = confidence[y-2:y+3,x-2:x+3]
        ok = np.isfinite(z) & (z>0) & (c>1.5)
        if ok.sum() < 13:
            result["low_confidence"].append(key)
            continue
        z = z[ok]
        if np.ptp(np.percentile(z,[10,90])) > .15*np.median(z):
            result["uneven_depth"].append(key)
            continue
        value = (scale*np.median(z)-origin[2])/axis[2]
        error = abs(value-ranges[key])
        result['comparisons'].append(dict(frame=key, measured_m=ranges[key], predicted_m=float(value),
                                          abs_error_m=float(error), agrees=bool(error<=max(.10,.15*ranges[key]))))
    rows = result['comparisons']
    result.update(median_error_m=median([r['abs_error_m'] for r in rows]), within_tolerance=sum(r['agrees'] for r in rows))
    return result


def experiment(run):
    model = json.loads((run/'survey.json').read_text())
    index = Path(model['feature_index']).stem
    scale = 1.
    scale_source = 'metric map'
    if model['units'] != 'm':
        # Use the same metric conversion as the clearance replay.
        from clearance import artifact_dir
        root = run/('gaussian-v1-'+index)
        report = json.loads((artifact_dir(run, root)/'report.json').read_bytes())
        scale, scale_source = report['scale'], report['scale_source']
    elif model.get('tof_scale'):
        scale_source = 'ToF-scaled map'
    frame_id = lambda name:int(Path(str(name)).stem.removeprefix('frame'))
    reference = {frame_id(p['frame']):np.column_stack((p['rotation'],scale*np.array(p['xyz'])))
                 for p in model['poses'] if not p.get('method') and 'rotation' in p}
    frames = {frame_id(r['frame']):r for r in load_csv(run/'timestamps.csv') if r.get('pi_encoder_callback_ns')}
    def seconds(a,b):
        return (int(frames[b]['pi_encoder_callback_ns'])-int(frames[a]['pi_encoder_callback_ns']))/1e9
    with np.load(run/('lingbot-v1-'+index)/'predictions.npz') as archive:
        ids = [frame_id(s) for s in archive['frames']]
        predicted = dict(zip(ids,archive['camera_to_world'].astype(float)))
        depths = dict(zip(ids,zip(archive['depth'],archive['confidence'],archive['intrinsics'])))
    ranges, source = read_ranges(run,frames)
    path = run/'tof-extrinsics.json'
    mount = json.loads(path.read_text()) if path.is_file() else dict(direction=[0,0,1],origin_m=[0,0,0])
    origin, axis = np.asarray(mount['origin_m']), np.asarray(mount['direction'],float)
    axis /= np.linalg.norm(axis)
    registered = sorted(reference.keys() & frames.keys())
    intervals = [(a,b) for a,b in zip(registered,registered[1:]) if seconds(a,b)>MIN_GAP_S]
    cuts = [registered[0],*[k for pair in intervals for k in pair],registered[-1]]
    blocks = [[i for i in ids if a<=i<=b and i in reference] for a,b in zip(cuts[::2],cuts[1::2])]
    gaps = []
    for n,(a,b) in enumerate(intervals):
        left, right = blocks[n][-MAX_ANCHORS:], blocks[n+1][:MAX_ANCHORS]
        inside = [i for i in ids if a<i<b]
        left_fit, right_fit = [pose_fit(keys,predicted,reference) for keys in (left,right)]
        forward = pose_errors(left_fit,right,predicted,reference)
        backward = pose_errors(right_fit,left,predicted,reference)
        # A joint fit can mask disagreement across a gap. Report it separately,
        # with alternating anchors held out on each side.
        joint = pose_fit(left[::2]+right[::2],predicted,reference)
        side_tests = [pose_errors(joint,keys[1::2],predicted,reference) for keys in (left,right)]
        alternative = position_only_fit(left[::2]+right[::2],predicted,reference)
        sensitivity = [pose_errors(alternative,keys[1::2],predicted,reference) for keys in (left,right)]
        checks = [forward,backward,*side_tests]
        pose_pass = all(r and r['median_m']<=MAX_POSITION_M and r['median_deg']<=MAX_ANGLE_DEG for r in checks)
        depth = check_depth(inside,joint,ranges,depths,origin,axis)
        enough_depth = len(depth['comparisons'])>=3 and depth.get('within_tolerance',0)>=.8*len(depth['comparisons'])
        gaps.append(dict(boundaries=[a,b],duration_s=seconds(a,b),left_anchors=left,right_anchors=right,
                         sampled_inside=inside,forward=forward,backward=backward,joint_holdouts=side_tests,
                         joint_metres_per_lingbot_unit=joint[1] if joint else None, depth=depth,
                         position_only_holdouts=sensitivity,
                         passes_pose=pose_pass, passes_depth=enough_depth, accepted=bool(pose_pass and enough_depth)))
    controls = []
    for block in blocks:
        # Hold back successive pairs of known poses. Score interior predictions
        # without giving the estimator the reference positions of either pose.
        for j in range(4,len(block)-5,2):
            inside = block[j:j+2]
            train = block[max(0,j-6):j]+block[j+2:j+8]
            score = pose_errors(pose_fit(train,predicted,reference),inside,predicted,reference)
            if score:
                controls.append(score)
    return dict(walk=run.name, map_metres_per_unit=scale, scale_source=scale_source,
                mount='measured' if path.is_file() else 'camera-axis approximation', range_source=source,
                predicted_views=len(ids), exact_overlaps=sum(i in reference for i in ids),
                gaps=gaps, masked_controls=controls)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('walk',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    data=json.dumps(experiment(args.walk),indent=2,allow_nan=False)
    if args.output:
        args.output.write_text(data+'\n')
    else:
        print(data)
