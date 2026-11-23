'use strict';
// Replay path: the operator "replay-path" (operators.py), run in this browser. It makes one camera pose per recorded
// frame from the poses it is given (a camera map's, or a regenerated camera path) and the walk's frame times:
//   mapped        a given pose, as it is;
//   interpolated  (fill_gaps) a frame between two given poses: on the straight line between them, at its time;
//   extrapolated  (extend_ends) a frame before the first or after the last given pose: moved on at the speed measured
//                 over the nearest end_frames consecutive given poses;
//   held          as extrapolated, but no speed could be measured, so it stays at the end pose.
// With blend_directions, a frame whose pose has no saved direction gets one blended between the nearest given poses
// (axes: right, down, forward). Every pose says how it was made (source), so the viewer only draws what it is given.
const {replayPath, cameraAxes} = (() => {
  const unit = v => v.map(x => x / Math.hypot(...v));
  const cross3 = (a, b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];

  // A pose's camera axes [right, down, forward]: its rotation, else its forward direction kept level.
  function cameraAxes(pose) {
    if (pose?.rotation) return pose.rotation[0].map((_, i) => pose.rotation.map(row => row[i]));
    if (pose?.axes) return pose.axes;
    if (!pose?.forward) return null;
    const forward = unit(pose.forward);
    const right = unit(cross3(forward, Math.abs(forward[2]) > .999 ? [0, 1, 0] : [0, 0, 1]));
    return [right, cross3(forward, right), forward];
  }
  function orientationQuaternion([[xx, yx, zx], [xy, yy, zy], [xz, yz, zz]]) {
    const diagonal = [1+xx+yy+zz, 1+xx-yy-zz, 1-xx+yy-zz, 1-xx-yy+zz];
    const major = diagonal.indexOf(Math.max(...diagonal)), scale = 2*Math.sqrt(diagonal[major]);
    const values = [[diagonal[0], zy-yz, xz-zx, yx-xy], [zy-yz, diagonal[1], xy+yx, xz+zx],
                    [xz-zx, xy+yx, diagonal[2], yz+zy], [yx-xy, xz+zx, yz+zy, diagonal[3]]];
    return unit(values[major].map(v => v / scale));
  }
  function quaternionAxes([w, x, y, z]) {
    return [[1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)],
            [2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)],
            [2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)]];
  }
  function blendDirection(a, b, t) {
    if (!a || !b) return (a || b)?.axes;
    const sign = a.quat.reduce((sum, v, i) => sum + v*b.quat[i], 0) < 0 ? -1 : 1;
    return quaternionAxes(unit(a.quat.map((v, i) => v*(1-t) + sign*b.quat[i]*t)));
  }
  // Where a frame falls between two others, by time (by count when they share a timestamp).
  function frameFraction(frames, index, left, right) {
    const elapsed = frames[right].ms - frames[left].ms;
    return elapsed > 0 ? (frames[index].ms - frames[left].ms) / elapsed : (index - left) / (right - left);
  }

  // poses: [{frame, xyz, rotation?, …}]; frames: the walk's recorded frames [{id, ms}] in order; settings: the node's.
  function replayPath(poses, frames, settings) {
    const order = new Map(frames.map((f, i) => [f.id, i]));
    const knots = poses.filter(p => Array.isArray(p?.xyz) && order.has(p.frame))
      .map(pose => ({pose, index: order.get(pose.frame)})).sort((a, b) => a.index - b.index);
    const path = new Map();
    for (const {pose, index} of knots) path.set(index, {...pose, ms: frames[index].ms, source: 'mapped'});
    if (settings.fill_gaps) {
      for (let i = 1; i < knots.length; i++) {
        const left = knots[i-1], right = knots[i];
        for (let j = left.index + 1; j < right.index; j++) {
          const t = frameFraction(frames, j, left.index, right.index);
          path.set(j, {frame: frames[j].id, ms: frames[j].ms, source: 'interpolated',
            xyz: left.pose.xyz.map((v, k) => v + t*(right.pose.xyz[k] - v)), between: [left.pose.frame, right.pose.frame]});
        }
      }
    }
    if (settings.extend_ends && knots.length) {
      for (const step of [1, -1]) {
        const start = step === 1, at = start ? 0 : knots.length - 1, anchor = knots[at];
        const end = start ? 0 : frames.length - 1;
        if (end === anchor.index) continue;
        // The speed over up to end_frames consecutive given poses at this end of the walk.
        let reference = anchor;
        for (let offset = step; Math.abs(offset) < settings.end_frames; offset += step) {
          const sample = knots[at + offset];
          if (!sample || sample.index !== anchor.index + offset) break;
          reference = sample;
        }
        // Fall back to the nearest pose with a different timestamp when the end poses are sparse.
        if (frames[reference.index].ms === frames[anchor.index].ms) {
          const neighbours = start ? knots : knots.slice().reverse();
          reference = neighbours.find(k => frames[k.index].ms !== frames[anchor.index].ms) || anchor;
        }
        const held = reference === anchor;
        for (let j = anchor.index - step; start ? j >= end : j <= end; j -= step) {
          const elapsed = frames[j].ms - frames[anchor.index].ms, interval = frames[reference.index].ms - frames[anchor.index].ms;
          path.set(j, {frame: frames[j].id, ms: frames[j].ms, source: held ? 'held' : 'extrapolated', from: [anchor.pose.frame, reference.pose.frame],
            xyz: anchor.pose.xyz.map((v, k) => held ? v : v + (reference.pose.xyz[k] - v)*elapsed/interval)});
        }
      }
    }
    if (settings.blend_directions) {
      const oriented = knots.flatMap(({pose, index}) => {
        const axes = cameraAxes(pose);
        return axes ? [{index, axes, quat: orientationQuaternion(axes)}] : [];
      });
      let next = 0;
      for (const index of [...path.keys()].sort((a, b) => a - b)) {
        while (next < oriented.length && oriented[next].index < index) next++;
        const pose = path.get(index);
        if (cameraAxes(pose)) continue;
        const left = oriented[next-1], right = oriented[next];
        const axes = blendDirection(left, right, left && right ? frameFraction(frames, index, left.index, right.index) : 0);
        if (axes) path.set(index, {...pose, axes, direction: 'blended'});
      }
    }
    return [...path.entries()].sort((a, b) => a[0] - b[0]).map(([, pose]) => pose);
  }
  return {replayPath, cameraAxes};
})();
