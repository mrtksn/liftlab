'use strict';
// Room Survey (survey/README.md) in LiftLab: its workspace dialog, and survey data brought into the World.
//
// The workspace is the survey's own page, survey/index.html, served with this one, in a frame. It talks to the
// survey server (survey/server.py) on this computer, which drives the camera and runs the processing; ?companion=
// tells the page where that server is. Only this computer's addresses: the page shows and runs what the server sends.
//
// Survey data reaches the World one way, as a model in the World editor's object import (js/object-library.js):
// from the survey viewer's Send to World (a 'liftlab-survey-scene' message from the frame), or from a saved survey
// map file (survey.json, format liftlab-room-survey-v1). surveyModel() turns either into one Z-up OBJ.

// Turn survey data into a World object import. data: {name, units ('m' | 'relative'), levelled, scale (an output's
// ToF estimate, for a map without metres), polygons [{outer, holes}], points [[x, y, z, r, g, b]], paths [[[x, y, z]…]],
// viewpoints [[x, y, z]…]}. Resolves to {ok, text}: what happened, for the survey page or the World panel to show.
async function surveyToWorld(data) {
  try {
    const room = surveyModel(data);
    if (fleet.selected && !fleetSelect(null)) throw Error('Finish what the selected drone is doing, then send it again.');
    worldEditSet(true);
    if (!worldEditOn()) throw Error('The World editor can\'t open now (a live drone or another edit is active).');
    $('#surveyDlg')?.close();
    const file = { name: room.name + '.obj', arrayBuffer: async () => new TextEncoder().encode(room.obj).buffer };
    const parts = [[room.triangles, 'triangle'], [room.points, 'point'], [room.paths, 'path']].filter(([n]) => n)
      .map(([n, what]) => n.toLocaleString() + ' ' + what + (n === 1 ? '' : 's'));
    const said = wedit.say, started = await objectImportStart([file], 'world', {
      up: 'z', units: room.metres ? 'm' : 'custom', scale: room.scale?.k, worldCollision: room.triangles ? 'mesh' : 'boxes',
      say: parts.join(', ').replace(/, ([^,]*)$/, ' and $1') + ' from the survey' +
        (room.triangles ? '' : ' (no surfaces, so nothing to collide with)') + '. ' +
        (room.metres ? 'Measured in metres from the range sensor.' : room.scale ? room.scale.text
          : 'Relative scale: set the room\'s longest side.') +
        (room.levelled ? '' : ' The map isn\'t levelled: level it under Survey leveling, or turn the placed copy.'),
    });
    // It says why it failed in the World editor's message; without a new message, it couldn't start.
    if (!started) throw Error(wedit.say !== said && wedit.say ? wedit.say : 'Another import or edit is open in LiftLab.');
    return { ok: true, text: 'In LiftLab’s World editor: check its size, then Save to object library and Place in world.' };
  } catch (err) { return { ok: false, text: err.message }; }
}

// What the survey showed as one OBJ model, Z up, map coordinates already levelled:
//   surfaces  polygons (outer ring and holes) triangulated, each triangle facing the nearest walk position, since
//             that's the side the camera saw (and LiftLab draws front faces);
//   points    [x, y, z, r, g, b] (0–255) as an OBJ point cloud with vertex colours (none given: the theme's muted ink);
//   paths     polylines (camera paths), in the accent colour.
// A world object takes MESH_MAX_TRIANGLES triangles at most; the import checks the file's size. Throws on anything
// malformed.
function surveyModel(data) {
  const point = p => Array.isArray(p) && p.length >= 3 && p.slice(0, 3).every(Number.isFinite);
  const list = a => a === undefined ? [] : Array.isArray(a) ? a : null;
  const polygons = list(data.polygons), points = list(data.points), paths = list(data.paths);
  if (!polygons || !points || !paths) throw Error('The survey sent malformed data.');
  if (!polygons.length && !points.length && !paths.length) throw Error('The viewer shows nothing to send.');
  const nearest = nearestPoint(Array.isArray(data.viewpoints) ? data.viewpoints.filter(point) : []);
  const out = ['# LiftLab room survey'], colour = c => c.map(v => +v.toFixed(3)).join(' ');
  let next = 1;
  const vertex = (p, c) => { out.push('v ' + p.slice(0, 3).map(v => +v.toFixed(5)).join(' ') + ' ' + colour(c)); return next++; };

  // Surfaces
  const ids = new Map(), faces = [], surfaceVerts = [];
  const vid = p => {
    const key = p.slice(0, 3).map(v => +v.toFixed(5)).join(' ');
    let i = ids.get(key);
    if (i === undefined) { i = next + surfaceVerts.length; ids.set(key, i); surfaceVerts.push(p); }
    return i;
  };
  const open = r => r.length > 3 && r[0].every((v, i) => v === r[r.length - 1][i]) ? r.slice(0, -1) : r;
  const face = (a, b, c) => {
    const n = new THREE.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]).cross(new THREE.Vector3(c[0] - a[0], c[1] - a[1], c[2] - a[2]));
    if (n.lengthSq() < 1e-20) return;
    const m = [0, 1, 2].map(k => (a[k] + b[k] + c[k]) / 3), seen = nearest(m);
    if (seen && n.x * (seen[0] - m[0]) + n.y * (seen[1] - m[1]) + n.z * (seen[2] - m[2]) < 0) [b, c] = [c, b];
    faces.push('f ' + vid(a) + ' ' + vid(b) + ' ' + vid(c));
  };
  const ring = r => Array.isArray(r) && r.length >= 3 && r.every(point);
  for (const poly of polygons) {
    if (!ring(poly?.outer)) throw Error('The survey sent a malformed surface.');
    const outer = open(poly.outer), holes = (Array.isArray(poly.holes) ? poly.holes : []).filter(ring).map(open);
    if (outer.length === 3 && !holes.length) { face(...outer); continue; }
    const n = [0, 0, 0];   // Newell's normal, to lay the polygon flat for triangulation
    outer.forEach((a, i) => { const b = outer[(i + 1) % outer.length];
      n[0] += (a[1] - b[1]) * (a[2] + b[2]); n[1] += (a[2] - b[2]) * (a[0] + b[0]); n[2] += (a[0] - b[0]) * (a[1] + b[1]); });
    const normal = new THREE.Vector3(...n);
    if (normal.lengthSq() < 1e-24) continue;
    normal.normalize();
    const u = new THREE.Vector3().crossVectors(Math.abs(normal.x) < .9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0), normal).normalize();
    const w = new THREE.Vector3().crossVectors(normal, u);
    const flat = r => r.map(p => new THREE.Vector2(u.x * p[0] + u.y * p[1] + u.z * p[2], w.x * p[0] + w.y * p[1] + w.z * p[2]));
    const all = [outer, ...holes].flat();
    for (const [i, j, k] of THREE.ShapeUtils.triangulateShape(flat(outer), holes.map(flat))) face(all[i], all[j], all[k]);
  }
  if (faces.length > MESH_MAX_TRIANGLES)
    throw Error(faces.length.toLocaleString() + ' triangles: a world object takes ' + MESH_MAX_TRIANGLES.toLocaleString() + ' at most. Simplify the surface in the survey first.');
  if (faces.length) { out.push('o surfaces'); for (const p of surfaceVerts) vertex(p, [1, 1, 1]); out.push(...faces); }

  // Points
  const kept = points.filter(point), plain = colorOf('--muted').toArray();
  if (kept.length) {
    out.push('o points');
    const first = next;
    for (const p of kept) vertex(p, p.length >= 6 && p.slice(3, 6).every(Number.isFinite)
      ? p.slice(3, 6).map(v => Math.min(1, Math.max(0, v / 255))) : plain);
    for (let i = first; i < next; i += 1000) out.push('p ' + Array.from({ length: Math.min(1000, next - i) }, (_, k) => i + k).join(' '));
  }

  // Paths, as segments (the OBJ reader draws a line's vertices in pairs)
  const accent = colorOf('--accent').toArray();
  let pathCount = 0;
  for (const line of paths) {
    const pts = Array.isArray(line) ? line.filter(point) : [];
    if (pts.length < 2) continue;
    out.push('o path-' + ++pathCount);
    const first = next;
    for (const p of pts) vertex(p, accent);
    for (let i = first; i + 1 < next; i++) out.push('l ' + i + ' ' + (i + 1));
  }
  if (!faces.length && !kept.length && !pathCount) throw Error('The surfaces have no area.');
  const name = (typeof data.name === 'string' && data.name.trim() ? data.name.trim() : 'Room survey').replace(/[^\w .·()+-]+/g, ' ').slice(0, 60);
  return { name, obj: out.join('\n') + '\n', triangles: faces.length, points: kept.length, paths: pathCount,
    metres: data.units === 'm', levelled: !!data.levelled, scale: data.units === 'm' ? null : surveyScale(data.scale) };
}

// The nearest of a set of points to a query, found in a uniform grid of about one point per cell (exact).
function nearestPoint(points) {
  if (!points.length) return () => null;
  const lo = [0, 1, 2].map(k => points.reduce((m, p) => Math.min(m, p[k]), Infinity));
  const hi = [0, 1, 2].map(k => points.reduce((m, p) => Math.max(m, p[k]), -Infinity));
  const n = Math.max(1, Math.round(Math.cbrt(points.length)));
  const size = Math.max(...hi.map((v, k) => v - lo[k]), 1e-9) / n;
  const cell = p => [0, 1, 2].map(k => Math.min(n - 1, Math.max(0, Math.floor((p[k] - lo[k]) / size))));
  const grid = new Map();
  for (const p of points) { const key = cell(p).join(); if (!grid.has(key)) grid.set(key, []); grid.get(key).push(p); }
  return q => {
    const [ci, cj, ck] = cell(q), outside = Math.hypot(...[0, 1, 2].map(k => Math.max(lo[k] - q[k], 0, q[k] - hi[k])));
    let best = null, bestD = Infinity;
    for (let r = 0; r <= n; r++) {
      // Every point within this ring's reach has been seen once its inner edge is farther than the best so far.
      if (best && (r - 1) * size > Math.sqrt(bestD) + outside) break;
      for (let i = ci - r; i <= ci + r; i++) for (let j = cj - r; j <= cj + r; j++) for (let k = ck - r; k <= ck + r; k++) {
        if (Math.max(Math.abs(i - ci), Math.abs(j - cj), Math.abs(k - ck)) !== r) continue;
        for (const p of grid.get(i + ',' + j + ',' + k) || []) {
          const d = (p[0] - q[0]) ** 2 + (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2;
          if (d < bestD) { bestD = d; best = p; }
        }
      }
    }
    return best;
  };
}

// A size for a surface without metres: an output's ToF estimate ({metres_per_unit, median_error_m, p80_error_m,
// hits, eligible}), with how well it held up, else null.
function surveyScale(s) {
  const k = s?.metres_per_unit, num = v => Number.isFinite(v) && v >= 0 ? v : null;
  if (!Number.isFinite(k) || k <= 0) return null;
  const med = num(s.median_error_m), p80 = num(s.p80_error_m), hits = num(s.hits), of = num(s.eligible);
  const cm = v => Math.round(v * 100) + ' cm';
  return { k, text: 'Sized from the range sensor: ' + (+k.toPrecision(3)) + ' m per survey unit' +
    (med !== null ? ', readings within ' + cm(med) + (p80 !== null ? ' (80% within ' + cm(p80) + ')' : '') : '') +
    (hits !== null && of !== null ? ' on ' + hits + ' of ' + of + ' held-out beams' : '') + '. Change the longest side if it looks wrong.' };
}

// The World panel's Room survey section: the workspace, and a saved survey map file brought in as a World object.
(() => {
  const KEY = 'liftlab.survey.companion', DEFAULT = 'http://127.0.0.1:8779';   // survey/server.py's default --port
  const byId = id => document.getElementById(id);
  const dlg = byId('surveyDlg'), addr = byId('surveyAddr'), frame = byId('surveyFrame'), help = byId('surveyHelp');
  const link = byId('surveyLink'), tab = byId('surveyTab'), why = byId('surveyWhy'), status = byId('surveyStatus');
  let generation = 0;
  const companionOrigin = text => {
    try {
      const u = new URL(text);
      return u.protocol === 'http:' && ['127.0.0.1', 'localhost', '[::1]'].includes(u.hostname) && !u.username && !u.password ? u.origin : '';
    } catch { return ''; }
  };
  const saved = () => { try { return localStorage.getItem(KEY) || DEFAULT; } catch { return DEFAULT; } };
  function problem(text) {
    frame.hidden = true; tab.hidden = true; help.hidden = false;
    why.textContent = text; link.textContent = 'Not connected';
  }
  async function connect() {
    const g = ++generation, base = companionOrigin(addr.value.trim());
    if (!base) return problem('The survey server runs on this computer: http://127.0.0.1:PORT or http://localhost:PORT.');
    addr.value = base;
    try { localStorage.setItem(KEY, base); } catch {}
    if (location.protocol === 'file:')
      return problem('This page is opened as a file, and the survey server only answers pages from a web address. Open LiftLab from its site, or from a local server (python3 -m http.server in its folder, then http://localhost:8000).');
    link.textContent = 'Connecting to ' + base + '…';
    try {
      const r = await fetch(base + '/state', { cache: 'no-store', signal: AbortSignal.timeout(4000) });
      if (!r.ok) throw Error('The survey server at ' + base + ' answered ' + r.status + '.');
      await r.json();
    } catch (e) {
      if (g !== generation) return;
      return problem(e instanceof TypeError || e.name === 'TimeoutError'
        ? 'No survey server answered at ' + base + '. If it is running, the browser may be asking whether this site can reach devices on your local network, or the server may not accept this page (' + location.origin + '): start it with --origin ' + location.origin + '.'
        : e.message);
    }
    if (g !== generation) return;
    const src = new URL('survey/index.html?' + new URLSearchParams({ companion: base }), location.href).href;
    if (frame.src !== src) frame.src = src;
    frame.hidden = false; help.hidden = true; tab.href = src; tab.hidden = false;
    link.textContent = 'Connected to ' + base;
  }
  byId('surveyWorkspace').onclick = () => {
    if (!addr.value) addr.value = saved();
    if (!dlg.open) dlg.showModal();
    connect();
  };
  addr.onchange = connect;
  addr.onkeydown = e => { if (e.key === 'Enter') { e.preventDefault(); connect(); } };
  byId('surveyRetry').onclick = connect;
  byId('surveyClose').onclick = () => dlg.close();

  // Send to World from the survey viewer: only from this page's own frame.
  addEventListener('message', async e => {
    if (e.origin !== location.origin || e.source !== frame.contentWindow || e.data?.type !== 'liftlab-survey-scene') return;
    const { ok, text } = await surveyToWorld(e.data);
    e.source.postMessage({ type: 'liftlab-survey-reply', id: e.data.id, ok, text }, location.origin);
  });

  // A saved survey map (the survey page's map download): its landmarks and camera path.
  const input = byId('surveyFile');
  byId('surveyOpen').onclick = () => input.click();
  input.onchange = async () => {
    const file = input.files[0]; input.value = '';
    if (!file) return;
    try {
      const s = JSON.parse(await file.text());
      if (s.format !== 'liftlab-room-survey-v1' || !Array.isArray(s.points) || !Array.isArray(s.poses))
        throw Error('Choose a survey map: the Room Survey page’s map download (survey.json).');
      const path = s.poses.map(p => p?.xyz).filter(Array.isArray);
      const { text } = await surveyToWorld({ name: ['Camera map', s.run].filter(Boolean).join(' · '), units: s.units,
        levelled: !!s.gravity_aligned, points: s.points, paths: [path], viewpoints: path });
      status.textContent = text;
    } catch (e) { status.textContent = e.message; }
  };
})();
