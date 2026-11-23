"""Compare coarse feature-tracking time on the Mac and Pi using the same saved images."""
import argparse
import io
import json
from pathlib import Path
import shlex
import subprocess
import sys
import time

import numpy as np

WIDTH, HEIGHT = 160, 120


def track(a, b):
    # One textured patch per tile. Score corners by the smaller eigenvalue
    # of the local gradient covariance, then match intensity patches by SSD.
    a = a.astype(np.float32)
    gy, gx = np.gradient(a)
    margin = 8
    tiled = np.lib.stride_tricks.sliding_window_view
    def window_mean(v):
        return tiled(v,(5,5)).mean(axis=(-2,-1))
    xx, yy, xy = window_mean(gx*gx), window_mean(gy*gy), window_mean(gx*gy)
    score = (xx+yy-np.sqrt((xx-yy)**2 + 4*xy**2))/2
    centres = []
    for y in range(margin,HEIGHT-margin-15,16):
        for x in range(margin,WIDTH-margin-15,16):
            tile = score[y-2:y+14,x-2:x+14]
            j,i = np.unravel_index(tile.argmax(),tile.shape)
            if tile[j,i] > 20:
                centres.append((y+j,x+i))
    if not centres:
        return 0
    patches_a = tiled(a,(7,7))
    patches_b = tiled(b.astype(np.float32),(7,7))
    y,x = np.array(centres).T - 3
    dy,dx = np.mgrid[-4:5,-4:5].reshape(2,-1)
    delta = patches_b[y[:,None]+dy,x[:,None]+dx] - patches_a[y,x][:,None]
    costs = (delta*delta).mean(axis=(-2,-1))
    best = np.partition(costs,1,axis=1)[:,:2]
    return int(np.sum((best[:,0] < .85*best[:,1]) & (best[:,0]<500)))


def measure(stream):
    elapsed, counts, previous = [], [], None
    while buf := stream.read(WIDTH*HEIGHT):
        frame = np.frombuffer(buf,dtype=np.uint8).reshape(HEIGHT,WIDTH)
        if previous is not None:
            start = time.perf_counter()
            counts.append(track(previous,frame))
            elapsed.append((time.perf_counter()-start)*1000)
        previous = frame
    if not counts:
        raise ValueError("At least two frames required")
    return dict(pairs=len(counts), median_ms=round(float(np.median(elapsed)),2),
                p95_ms=round(float(np.percentile(elapsed,95)),2),
                median_matched_patches=float(np.median(counts)))


if __name__ == "__main__":
    if sys.argv[1:2] == ["--worker"]:
        WIDTH, HEIGHT = (int(v) for v in sys.argv[2:4]) if len(sys.argv) > 3 else (WIDTH, HEIGHT)
        print(json.dumps(measure(sys.stdin.buffer)))
    else:
        import cv2
        from node_params import device_settings
        ap = argparse.ArgumentParser(description=__doc__)
        ap.add_argument("walk", type=Path, help="survey directory containing frames/")
        ap.add_argument("--pi", help="SSH destination; defaults to the survey folder's device settings")
        ap.add_argument("--frames", type=int, default=48, help="frames streamed to both computers")
        ap.add_argument("--width", type=int, default=WIDTH, help="downsampled width")
        ap.add_argument("--height", type=int, default=HEIGHT, help="downsampled height")
        args = ap.parse_args()
        WIDTH, HEIGHT = args.width, args.height
        args.pi = args.pi or device_settings(args.walk.parent)["pi_host"]
        data = bytearray()
        for path in sorted((args.walk/"frames").glob("*.jpg"))[:args.frames]:
            gray = cv2.imread(str(path),cv2.IMREAD_GRAYSCALE)
            data.extend(cv2.resize(gray,(WIDTH,HEIGHT),interpolation=cv2.INTER_AREA).tobytes())
        mac = measure(io.BytesIO(data))
        code = Path(__file__).read_text()
        command = "python3 -c " + shlex.quote(code) + f" --worker {WIDTH} {HEIGHT}"
        result = subprocess.run(["ssh","-T","-o","BatchMode=yes","-o","ConnectTimeout=6",args.pi,command],
                                input=data,capture_output=True,timeout=120)
        if result.returncode:
            sys.exit(result.stderr.decode(errors="replace"))
        print(json.dumps({"mac":mac,"pi":json.loads(result.stdout)},indent=2))
