"""End-to-end geometry regression with a textured synthetic room and a known path."""
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from reconstruct import atomic_json, build
from server import Survey


def render_room(folder):
    frames = folder / "frames"
    frames.mkdir()
    rng = np.random.default_rng(154)
    textures = []
    for _ in range(6):
        texture = np.full((700,700,3), 60, np.uint8)
        for _ in range(2300):
            x,y = rng.integers(0,690,2).tolist()
            cv2.rectangle(texture,(x,y),(x+int(rng.integers(2,16)),y+int(rng.integers(3,15))),
                          rng.integers(30,255,3).tolist(),-1)
        textures.append(texture)
    h,w = 480,640
    yy,xx = np.indices((h,w))
    dx,dy = (xx-w/2)/270,(yy-h/2)/270
    theta = np.hypot(dx,dy)
    sinc = np.sinc(theta/np.pi)
    rays = np.stack((dx*sinc,dy*sinc,np.cos(theta)),axis=-1)
    path = []
    for i,x in enumerate(np.linspace(-.7,.7,14)):
        c = np.array([x,.12*np.sin(i/4),.10*np.cos(i/4)])
        path.append(c)
        face = np.zeros((h,w),int)
        depth = np.full((h,w),np.inf)
        for axis,(lo,hi) in enumerate([(-2,2),(-1.3,1.4),(-2,3)]):
            for side,wall in enumerate([lo,hi]):
                with np.errstate(divide='ignore'):
                    t = (wall-c[axis])/rays[:,:,axis]
                ok = (t>0)&(t<depth)
                face[ok],depth[ok]=axis*2+side,t[ok]
        world = c + rays*depth[:,:,None]
        image = np.zeros((h,w,3),np.uint8)
        for f,texture in enumerate(textures):
            a,b=[k for k in range(3) if k!=f//2]
            u,v=[((world[:,:,k]+4)*120).astype(np.float32) for k in (a,b)]
            surface=cv2.remap(texture,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_WRAP)
            image[face==f]=surface[face==f]
        cv2.imwrite(str(frames/f"{i:07}.jpg"),image)
    atomic_json(folder/"calibration.json",dict(size=[w,h],params=[270,270,320,240,0,0,0,0]))
    return np.array(path)


class ReconstructionTest(unittest.TestCase):
    def test_unmeasured_lens(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            render_room(run)
            (run / "calibration.json").unlink()
            names = sorted(p.name for p in (run/"frames").iterdir())
            model = build(run, names, lambda _: None)
            self.assertEqual(model["units"], "relative")
            self.assertGreaterEqual(len(model["poses"]), 10)
            self.assertGreater(len(model["points"]), 100)

    def test_metric_walk(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            survey=Survey(root,"unused")
            run=root/"walk"
            run.mkdir()
            path=render_room(run)
            names=sorted(p.name for p in (run/"frames").iterdir())
            refs={"A":names[0],"B":names[-1],"metres":float(np.linalg.norm(path[-1]-path[0]))}
            survey.map(run,names,refs,recover=True)
            self.assertTrue((run/"survey.json").is_file(),survey.message)
            model=json.loads((run/"survey.json").read_text())
            self.assertEqual(model["units"],"m")
            self.assertEqual(model["surfaces"]["status"],"ready")
            self.assertTrue(model["surfaces"]["patches"])
            first_face = model["surfaces"]["patches"][0]
            for i, vertex in zip(first_face["landmarks"], first_face["vertices"]):
                self.assertEqual(vertex, model["points"][i][:3])
            self.assertGreater(len(model["points"]),100)
            self.assertGreaterEqual(len(model["poses"]),12)
            for pose in model["poses"]:
                truth=path[int(pose["frame"][:7])]-path[0]
                expected=[truth[0],truth[2],-truth[1]]
                self.assertLess(np.linalg.norm(np.array(pose["xyz"])-expected),.06)


if __name__ == "__main__":
    unittest.main()
