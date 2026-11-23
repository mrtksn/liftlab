"""Recover one connected sparse model from a snapshot of a handheld recording."""
import csv
import json
from pathlib import Path
import tempfile

from fusion import (STREAMS, gravity_alignment, estimate_scale, levelling_rotation, range_ratio,
                    load_observations, sensor_pairs, up_alignment)
from localize import save_index
from recover import align_gap, recover_poses
import cv2
import numpy as np
import pycolmap as sfm


def atomic_json(path, value):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(value, separators=(",", ":"), allow_nan=False))
    tmp.replace(path)


def build(run, names, report, *, recover=False, index=True, streams=STREAMS):
    observations = load_observations(run, streams)
    calibrated = (run / "calibration.json").exists()
    reader = sfm.ImageReaderOptions()
    reader.camera_model = "SIMPLE_RADIAL_FISHEYE"
    # Equidistant initial guess for this 640x480 stream. The uncalibrated
    # solve refines it; COLMAP requires an explicit prior for fisheye pairs.
    reader.camera_params = "260,320,240,0"
    if calibrated:
        calib = json.loads((run / "calibration.json").read_text())
        reader.camera_model = "OPENCV_FISHEYE"
        reader.camera_params = ",".join(map(str, calib["params"]))
    extract = sfm.FeatureExtractionOptions(num_threads=3)
    extract.sift.max_num_features = 2500
    matcher = sfm.FeatureMatchingOptions(num_threads=3)
    options = sfm.IncrementalPipelineOptions(num_threads=3, multiple_models=True,
                                              min_model_size=4, max_num_models=20,
                                              image_names=names)
    options.mapper.init_min_tri_angle = 6
    if calibrated:
        options.ba_refine_focal_length = options.ba_refine_extra_params = False
        options.mapper.abs_pose_refine_focal_length = options.mapper.abs_pose_refine_extra_params = False
    # Isolated snapshot databases avoid reusing features from a different calibration
    # or feeding half-written frames into the matcher.
    with tempfile.TemporaryDirectory(dir=run) as workspace:
        work = Path(workspace)
        database = work / "features.db"
        image_path = run / "frames"
        report(f"Finding features in {len(names)} views")
        sfm.extract_features(database, image_path, names, sfm.CameraMode.SINGLE, reader, extract, sfm.Device.cpu)
        report("Matching overlapping views")
        if len(names) <= 45:
            sfm.match_exhaustive(database, matcher, device=sfm.Device.cpu)
        else:
            # Include return visits to the start of the walk.
            pairs = { (a,b) for i,a in enumerate(names) for b in names[i+1:i+9] }
            pairs.update((a,b) for a in names[:6] for b in names[-10:] if a < b)
            # Retrieve visually similar non-neighbours throughout the walk.
            # COLMAP verifies each candidate using local features and geometry.
            thumbs = []
            for name in names:
                gray = cv2.imread(str(image_path / name), cv2.IMREAD_GRAYSCALE)
                thumb = cv2.resize(gray, (24, 18), interpolation=cv2.INTER_AREA).flatten().astype(float)
                thumb -= thumb.mean()
                thumbs.append(thumb / max(np.linalg.norm(thumb), 1))
            similarity = np.asarray(thumbs) @ np.asarray(thumbs).T
            for i, row in enumerate(similarity):
                distant = [j for j in range(len(names)) if abs(j-i) > 8]
                for j in sorted(distant, key=lambda j: row[j], reverse=True)[:8]:
                    pairs.add(tuple(sorted((names[i], names[j]))))
            pairs.update(sensor_pairs(names, similarity, observations))
            pair_file = work / "pairs.txt"
            pair_file.write_text("".join(f"{a} {b}\n" for a,b in sorted(pairs)))
            sfm.match_image_pairs(database, pairing_options=sfm.ImportedPairingOptions(match_list_path=pair_file),
                                  matching_options=matcher, device=sfm.Device.cpu)
        out = work / "models"
        out.mkdir()
        report("Solving camera positions and triangulating landmarks")
        models = sfm.incremental_mapping(database, image_path, out, options)
        if not models:
            raise ValueError("No connected reconstruction yet. Move sideways past textured objects with overlapping views.")
        model = max(models.values(), key=lambda m: (m.num_reg_images(), m.num_points3D()))
        recovery_frames = sorted(p.name for p in image_path.glob("*.jpg")
                                 if names[0] <= p.name <= names[-1]) if recover else []
        if recover:
            report("Recovering skipped camera poses against mapped landmarks")
        result = export_model(run, model, names, calibrated, observations, recovery_frames, indexed=index,
                              gravity="gyro" in streams and "accel" in streams)
        result["streams"] = list(streams)
        return result


def startup_rotation(run, images):
    """A level airframe and measured camera mount define up during the initial hold."""
    reference = run/'level-start.json'
    if not reference.exists():
        return None, None
    mount = json.loads(reference.read_text())
    with (run/'timestamps.csv').open() as stream:
        samples = list(csv.DictReader(stream))
    if not samples:
        return None, 'No startup frames'
    t0 = float(samples[0]['pi_camera_relative_us'])
    start = {s['frame'] for s in samples if 0 <= float(s['pi_camera_relative_us'])-t0 <= 2e6}
    ticks = [float(s['pi_encoder_callback_ns'])*1e-9 for s in samples if
             s['frame'] in start and s.get('pi_encoder_callback_ns')]
    if ticks and (run/'motion.csv').exists():
        with (run/'motion.csv').open() as stream:
            rates = [np.linalg.norm([float(row[k]) for k in ('gx_rad_s','gy_rad_s','gz_rad_s')])
                     for row in csv.DictReader(stream) if min(ticks)<=float(row['pi_uart_monotonic_s'])<=max(ticks)]
        if len(rates)>=5 and np.percentile(rates,90)>np.deg2rad(5):
            return None, 'IMU detected rotation during level start'
    views = [im for im in images if im.name in start]
    if not views:
        return None, 'No mapped pose during level start'
    pitch,roll = np.deg2rad([mount['pitch'],mount['roll']])
    tilt = cv2.Rodrigues(np.array([-pitch,0.,0.]))[0]
    twist = cv2.Rodrigues(np.array([0.,0.,roll]))[0]
    camera_level = tilt @ np.array([[1,0,0],[0,0,1],[0,-1,0]]) @ twist
    first = views[0].cam_from_world().rotation.matrix()
    for im in views[1:]:
        turn = cv2.Rodrigues(im.cam_from_world().rotation.matrix() @ first.T)[0]
        if np.linalg.norm(turn)>np.deg2rad(4):
            return None, 'Camera moved during level start'
    return camera_level @ first, 'Level start'


def export_model(run, model, names, calibrated, observations, recovery_frames=(), *, indexed=True, gravity=True):
    images = sorted((model.image(i) for i in model.reg_image_ids()), key=lambda im: im.name)
    if len(images) < 3 or not model.num_points3D():
        raise ValueError("Too few registered views for a room survey.")
    # LiftLab is Z-up. Use the first registered camera as the local origin,
    # looking along +Y with image right along +X. Gravity is unobserved.
    first = images[0]
    rotation = np.array([[1,0,0],[0,0,1],[0,-1,0]]) @ first.cam_from_world().rotation.matrix()
    paired = [im for im in images if gravity and "q" in observations.get(im.name,{})]
    alignment = gravity_alignment([im.cam_from_world().rotation.matrix().T for im in paired],
                                  [observations[im.name]["q"] for im in paired],
                                  [int(im.name[:-4]) for im in paired] if paired and all(im.name[:-4].isdigit() for im in paired) else None)
    level_rotation,level_status = startup_rotation(run,images)
    up_fit = None
    if level_rotation is not None:
        rotation = level_rotation
    elif alignment is not None:
        rotation = alignment
    else:
        # Handheld walks rarely turn about two axes. Level the first-camera
        # frame from IMU up vectors instead, keeping its heading along +Y.
        levelled = [im for im in images if "up" in observations.get(im.name,{})]
        up_fit = up_alignment([im.cam_from_world().rotation.matrix().T for im in levelled],
                              [observations[im.name]["up"] for im in levelled])
        if up_fit is not None:
            rotation = levelling_rotation(rotation @ up_fit.pop("up")) @ rotation
    origin = first.projection_center()
    def xyz(p):
        return (rotation @ (p - origin)).round(6).tolist()
    point_ids = list(model.points3D.keys())
    points = [[*xyz(model.point3D(i).xyz), *model.point3D(i).color.tolist()] for i in point_ids]
    labels = {pid: i for i, pid in enumerate(point_ids)}
    views = {im.name: sorted({labels[p.point3D_id] for p in im.points2D if p.has_point3D()})
             for im in images}
    extra_ratios = {}

    def observed(frame, ids, camera, pixels, depths):
        views[frame] = sorted({labels[i] for i in ids})
        ratio = range_ratio(observations.get(frame, {}), camera, pixels, depths)
        if ratio:
            extra_ratios[frame] = ratio

    index = save_index(run, images, first.camera, labels) if indexed else None
    poses = []
    for im in images:
        camera_rotation = rotation @ im.cam_from_world().rotation.matrix().T
        poses.append(dict(frame=im.name, xyz=xyz(im.projection_center()),
                          rotation=camera_rotation.round(7).tolist(),
                          forward=camera_rotation[:, 2].round(7).tolist()))
    if recovery_frames:
        poses.extend(recover_poses(run, model, images, recovery_frames, xyz, rotation, observed))
        poses.sort(key=lambda pose: pose["frame"])
    scale = estimate_scale(model, images, observations, extra_ratios)
    if scale:
        factor = scale["metres_per_unit"]
        for point in points:
            point[:3] = (np.asarray(point[:3])*factor).round(7).tolist()
        for pose in poses:
            pose["xyz"] = (np.asarray(pose["xyz"])*factor).round(7).tolist()
    return dict(landmark_views=views, format="liftlab-room-survey-v1", units="m" if scale else "relative", tof_scale=scale, gravity_aligned=(alignment is not None or up_fit is not None) and level_rotation is None, up_fit=up_fit, level_status=level_status, feature_index=index, poses=poses, registered_views=len(images),
        path_frames=recovery_frames or names,
        points=points, views=len(names), sampled_frames=names,
        mean_error_px=round(model.compute_mean_reprojection_error(), 3),
        lens="checkerboard" if calibrated else "self-estimated fisheye")

def recover_gaps(run, model, report):
    frames = model["path_frames"]
    order = {name: i for i, name in enumerate(frames)}
    poses = [order[p["frame"]] for p in model["poses"]]
    gaps = [(a,b) for a,b in zip(poses, poses[1:]) if 20 <= b-a <= 200]
    # Keep the final solve bounded and require context on both sides.
    for left, right in sorted(gaps, key=lambda gap: gap[1]-gap[0], reverse=True)[:3]:
        first, last = max(0,left-40), min(len(frames),right+26)
        names = sorted(set(frames[first:last:2] +
                           frames[max(first,left-20):left+8] +
                           frames[right-16:min(last,right+18)]))
        report(f"Checking overlap across frames {int(frames[left][:7])}–{int(frames[right][:7])}")
        try:
            local = build(run, names, lambda _: None, recover=True, index=False, streams=model["streams"])
        except ValueError:
            continue
        model["poses"].extend(align_gap(model, local, frames[left], frames[right]))
    model["poses"].sort(key=lambda pose: pose["frame"])
