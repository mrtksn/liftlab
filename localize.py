"""Image-to-map localization in the coordinate system of a saved survey."""
from pathlib import Path
import uuid
import cv2
import numpy as np
import pycolmap as sfm


def features(gray):
    keys, desc = cv2.SIFT_create(nfeatures=1400).detectAndCompute(gray, None)
    return np.asarray([k.pt for k in keys], np.float32).reshape(-1, 2) + .5, desc


def save_index(run, images, camera, ids):
    """Associate OpenCV descriptors with triangulated COLMAP observations."""
    rows, labels = [], []
    matcher = cv2.BFMatcher()
    for im in images:
        observations = [p for p in im.points2D if p.has_point3D()]
        if not observations:
            continue
        xy, desc = features(cv2.imread(str(run/'frames'/im.name), cv2.IMREAD_GRAYSCALE))
        if desc is None:
            continue
        found = matcher.match(xy, np.float32([p.xy for p in observations]))
        for m in found:
            if m.distance < 1.6:
                rows.append(desc[m.queryIdx])
                labels.append(ids[observations[m.trainIdx].point3D_id])
    if len(rows) < 12:
        return None
    name = 'features-' + uuid.uuid4().hex + '.npz'
    np.savez_compressed(run/name, descriptors=np.uint8(rows), labels=labels,
                        camera_model=camera.model.name, params=camera.params)
    return name


class Locator:
    def __init__(self, run, model):
        with np.load(run / Path(model['feature_index']).name) as data:
            self.camera = sfm.Camera(model=str(data['camera_model']), width=640, height=480,
                                     params=data['params'])
            self.labels = data['labels']
            self.matcher = cv2.FlannBasedMatcher(dict(algorithm=1,trees=4),dict(checks=64))
            self.matcher.add([data['descriptors'].astype(np.float32)])
            self.matcher.train()
        self.xyz = np.asarray(model['points'])[:,:3]

    def locate(self, jpeg, minimum):
        gray = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_GRAYSCALE)
        xy, desc = features(gray)
        pairs = {}
        if desc is not None:
            for neighbours in self.matcher.knnMatch(desc,k=min(8,len(self.labels))):
                best = neighbours[0]
                label = self.labels[best.trainIdx]
                other = next((m for m in neighbours[1:] if self.labels[m.trainIdx] != label),None)
                if other and best.distance < .78 * other.distance:
                    if label not in pairs or best.distance < pairs[label].distance:
                        pairs[label] = best
        result = dict(status='Searching', matches=len(pairs), inliers=0)
        if len(pairs) < minimum:
            return result
        pixels = xy[[m.queryIdx for m in pairs.values()]]
        options = sfm.AbsolutePoseEstimationOptions()
        options.ransac.max_error = 3
        fit = sfm.estimate_and_refine_absolute_pose(pixels,self.xyz[list(pairs)],self.camera,options)
        if fit:
            mask = np.asarray(fit['inlier_mask'],bool)
            result['inliers'] = int(mask.sum())
            if mask.sum() >= minimum and mask.mean() >= .3 and np.all(np.ptp(pixels[mask],axis=0)>45):
                pose = fit['cam_from_world'].inverse()
                result.update(status='Located', xyz=pose.translation.tolist(),
                              rotation=pose.rotation.matrix().tolist(), pixels=pixels[mask].tolist())
        return result
