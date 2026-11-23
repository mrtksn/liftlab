"""Recover nearby camera poses against existing triangulated landmarks."""
from bisect import bisect_left
from functools import lru_cache

import cv2
import numpy as np
import pycolmap as sfm


def recover_poses(run, model, images, names, xyz, world_rotation, on_observation=None, limits=None):
    limits = dict(dict(max_frames=4, min_inliers=20, min_ratio=.65, min_spread=65, max_error=2.5, max_jump=.20), **(limits or {}))
    @lru_cache(maxsize=12)
    def gray(name):
        return cv2.imread(str(run / "frames" / name), cv2.IMREAD_GRAYSCALE)

    @lru_cache(maxsize=12)
    def landmarks(index):
        points = [p for p in images[index].points2D if p.has_point3D()]
        pixels = np.asarray([p.xy for p in points], dtype=np.float32) - .5
        world = np.asarray([model.point3D(p.point3D_id).xyz for p in points])
        return pixels, world, np.array([p.point3D_id for p in points])

    options = sfm.AbsolutePoseEstimationOptions()
    options.ransac.max_error = limits["max_error"]
    anchors = [im.name for im in images]
    recovered = []
    for name in names:
        i = bisect_left(anchors, name)
        if i < len(anchors) and anchors[i] == name:
            continue
        target = gray(name)
        if target is None:
            continue
        for j in sorted((k for k in (i-1,i) if 0 <= k < len(anchors)),
                        key=lambda k: abs(int(anchors[k][:7])-int(name[:7]))):
            if abs(int(anchors[j][:7])-int(name[:7])) > limits["max_frames"]:
                break
            pixels, world, ids = landmarks(j)
            if len(pixels) < limits["min_inliers"]:
                continue
            source = gray(anchors[j])
            flowed, forward, _ = cv2.calcOpticalFlowPyrLK(source, target, pixels, None)
            if flowed is None:
                continue
            back, backward, _ = cv2.calcOpticalFlowPyrLK(target, source, flowed, None)
            if back is None:
                continue
            mask = ((forward.ravel() != 0) & (backward.ravel() != 0)
                    & (np.linalg.norm(back-pixels, axis=1) < .75))
            if mask.sum() < limits["min_inliers"]:
                continue
            fit = sfm.estimate_and_refine_absolute_pose(flowed[mask]+.5, world[mask],
                                                        images[j].camera, options)
            if fit is None:
                continue
            inliers = np.asarray(fit["inlier_mask"], dtype=bool)
            if (inliers.sum() < limits["min_inliers"] or inliers.mean() < limits["min_ratio"]
                    or np.any(np.ptp(flowed[mask][inliers],axis=0) < limits["min_spread"])):
                continue
            pose = fit["cam_from_world"].inverse()
            # Reject pose flips and jumps large relative to the observed depth.
            relative = pose.rotation.matrix().T @ images[j].cam_from_world().rotation.matrix().T
            depth = np.median(np.linalg.norm(world[mask][inliers] - images[j].projection_center(),axis=1))
            if np.trace(relative)<2.6 or np.linalg.norm(pose.translation-images[j].projection_center()) > limits["max_jump"]*depth:
                continue
            if on_observation:
                observed = world[mask][inliers]
                on_observation(name, ids[mask][inliers], images[j].camera, flowed[mask][inliers]+.5,
                               (fit["cam_from_world"] * observed)[:, 2])
            orientation = world_rotation @ pose.rotation.matrix()
            recovered.append(dict(frame=name, xyz=xyz(pose.translation), method="optical_flow",
                                  rotation=orientation.round(7).tolist(),
                                  forward=orientation[:, 2].round(7).tolist()))
            break
    return recovered


def align_gap(base, local, left, right):
    """Place a local path in the main map using poses shared on both sides."""
    primary = {p["frame"]: p for p in base["poses"]}
    window = {p["frame"]: p for p in local["poses"]}
    added = [name for name in sorted(window) if left < name < right and name not in primary]
    if not added:
        return []
    shared = {name for name in primary.keys() & window.keys()
              if primary[name].get("method") != "aligned_window"}
    before = sorted(name for name in shared if name <= left)
    after = sorted(name for name in shared if name >= right)
    if min(len(before), len(after)) < 9:
        return []
    ids = before + after
    source = np.array([window[name]["xyz"] for name in ids])
    target = np.array([primary[name]["xyz"] for name in ids])
    test = np.arange(len(ids)) % 3 == 0
    training = target[~test]
    spread = np.linalg.svd(training-training.mean(axis=0), compute_uv=False)
    span = np.linalg.norm(np.ptp(training,axis=0))
    if span == 0 or spread[1] < .12*spread[0]:
        return []
    transform = sfm.estimate_sim3d(source[~test], training)
    if transform is None or not np.isfinite(transform.scale) or transform.scale <= 0:
        return []
    matrix = transform.matrix()
    errors = np.linalg.norm(source @ matrix[:,:3].T + matrix[:,3] - target, axis=1)
    # Reject a fit if it agrees with only one end or moves the seam too far.
    before_mask = np.arange(len(ids)) < len(before)
    if any(np.percentile(errors[test & side],90) > .065*span for side in (before_mask,~before_mask)):
        return []
    if max(errors[len(before)-1], errors[len(before)]) > .08*span:
        return []
    result = []
    rotation = transform.rotation.matrix()
    for name in added:
        pose = window[name]
        aligned = dict(frame=name, method="aligned_window",
                       xyz=(matrix[:, :3]@pose["xyz"]+matrix[:, 3]).round(6).tolist())
        if "rotation" in pose:
            orientation = rotation @ pose["rotation"]
            aligned.update(rotation=orientation.round(7).tolist(),
                           forward=orientation[:, 2].round(7).tolist())
        result.append(aligned)
    return result
