'use strict';
// Inspection overlay in World view. The reconstruction's first camera is its local origin.
(() => {
  const input = document.getElementById('surveyFile');
  const clear = document.getElementById('surveyClear');
  const status = document.getElementById('surveyStatus');
  let overlay;
  function dispose() {
    if (overlay) {
      scene.remove(overlay);
      for (const item of overlay.children) { item.geometry.dispose(); item.material.dispose(); }
      overlay = null;
    }
    clear.hidden = true;
  }
  document.getElementById('surveyOpen').onclick = () => input.click();
  clear.onclick = () => { dispose(); status.textContent = 'Survey cleared.'; };
  input.onchange = async () => {
    const file = input.files[0]; input.value = '';
    if (!file) return;
    try {
      if (file.size > 32 * 1024 * 1024) throw Error('The survey exceeds 32 MB.');
      const s = JSON.parse(await file.text());
      const vector = (v, n) => Array.isArray(v) && v.length === n && v.every(Number.isFinite);
      if (s.format !== 'liftlab-room-survey-v1' || !['relative','m'].includes(s.units) ||
          !Array.isArray(s.points) || !s.points.length || s.points.length > 500000 ||
          !Array.isArray(s.poses) || !s.poses.length || s.poses.length > 100000 ||
          !s.points.every(p => vector(p, 6)) || !s.poses.every(p => vector(p?.xyz, 3)))
        throw Error('Choose a room-survey JSON exported from the capture dashboard.');
      const positions = new Float32Array(s.points.length * 3), colours = new Float32Array(positions.length);
      s.points.forEach((p,i) => {
        positions.set(p.slice(0,3),i*3); colours.set(p.slice(3).map(v => v/255),i*3);
      });
      const geom = new THREE.BufferGeometry();
      geom.setAttribute('position',new THREE.BufferAttribute(positions,3));
      geom.setAttribute('color',new THREE.BufferAttribute(colours,3));
      const points = new THREE.Points(geom,new THREE.PointsMaterial({vertexColors:true,size:3,sizeAttenuation:false,depthTest:false}));
      const path = new THREE.BufferGeometry().setFromPoints(s.poses.map(p => new THREE.Vector3(...p.xyz)));
      const line = new THREE.Line(path,new THREE.LineBasicMaterial({color:0x60efba,depthTest:false}));
      dispose();
      overlay = new THREE.Group(); overlay.add(points,line); scene.add(overlay);
      points.renderOrder = 10; line.renderOrder = 11; clear.hidden = false;
      path.computeBoundingSphere();
      cam.target.copy(path.boundingSphere.center);
      cam.dist = Math.max(3,Math.min(90, path.boundingSphere.radius*4));
      updateCamera();
      status.textContent = s.poses.length+' camera positions · '+s.points.length+' landmarks · '+
        (s.units==='m'?'measured scale in metres':'relative scale')+' · visual overlay';
    } catch (e) { status.textContent = e.message; }
  };
})();

// Room Survey workspace (survey/README.md): the survey's own page, survey/index.html, served with this one, in a
// frame. It talks to the companion, survey/server.py on this computer, which drives the camera and runs the
// processing; ?companion= tells the page where it is. Only this computer's addresses: the page shows and runs what
// the companion sends.
(() => {
  const KEY = 'liftlab.survey.companion', DEFAULT = 'http://127.0.0.1:8779';
  const $ = id => document.getElementById(id);
  const dlg = $('surveyDlg'), addr = $('surveyAddr'), frame = $('surveyFrame'), help = $('surveyHelp');
  const link = $('surveyLink'), tab = $('surveyTab'), why = $('surveyWhy');
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
    if (!base) return problem('The companion runs on this computer: http://127.0.0.1:PORT or http://localhost:PORT.');
    addr.value = base;
    try { localStorage.setItem(KEY, base); } catch {}
    if (location.protocol === 'file:')
      return problem('This page is opened as a file, and the companion only answers pages from a web address. Open LiftLab from its site, or from a local server (python3 -m http.server in its folder, then http://localhost:8000).');
    link.textContent = 'Connecting to ' + base + '…';
    try {
      const r = await fetch(base + '/state', { cache: 'no-store', signal: AbortSignal.timeout(4000) });
      if (!r.ok) throw Error('The companion at ' + base + ' answered ' + r.status + '.');
      await r.json();
    } catch (e) {
      if (g !== generation) return;
      return problem(e instanceof TypeError || e.name === 'TimeoutError'
        ? 'No companion answered at ' + base + '. If it is running, the browser may be asking whether this site can reach devices on your local network, or the companion may not accept this page (' + location.origin + '): start it with --origin ' + location.origin + '.'
        : e.message);
    }
    if (g !== generation) return;
    const src = new URL('survey/index.html?' + new URLSearchParams({ companion: base }), location.href).href;
    if (frame.src !== src) frame.src = src;
    frame.hidden = false; help.hidden = true; tab.href = src; tab.hidden = false;
    link.textContent = 'Companion at ' + base;
  }
  $('surveyWorkspace').onclick = () => {
    if (!addr.value) addr.value = saved();
    if (!dlg.open) dlg.showModal();
    connect();
  };
  addr.onchange = connect;
  addr.onkeydown = e => { if (e.key === 'Enter') { e.preventDefault(); connect(); } };
  $('surveyRetry').onclick = connect;
  $('surveyClose').onclick = () => dlg.close();

  // Send to World: the survey page posts a surface it built ({type: 'liftlab-survey-surface'}) and this page opens it
  // in the World editor's object import, as a Z-up OBJ, for you to size and save to the object library. Placed copies
  // of it collide with its triangles (Follows model), as a room's walls should. Only from this page's own frame.
  addEventListener('message', async e => {
    if (e.origin !== location.origin || e.source !== frame.contentWindow || e.data?.type !== 'liftlab-survey-surface') return;
    const reply = (ok, text) => e.source.postMessage({ type: 'liftlab-survey-reply', id: e.data.id, ok, text }, location.origin);
    try {
      const room = surveyRoom(e.data);
      if (fleet.selected && !fleetSelect(null)) throw Error('Finish what the selected drone is doing, then send it again.');
      worldEditSet(true);
      if (!worldEditOn()) throw Error('The World editor can\'t open now (a live drone or another edit is active).');
      dlg.close();
      const file = { name: room.name + '.obj', arrayBuffer: async () => new TextEncoder().encode(room.obj).buffer };
      const started = await objectImportStart([file], 'world', {
        up: 'z', units: room.metres ? 'm' : 'custom', scale: room.scale?.k, worldCollision: 'mesh',
        say: room.triangles.toLocaleString() + ' triangles from the survey. ' + (room.metres
          ? 'Measured in metres from the range sensor.' : room.scale ? room.scale.text
          : 'Relative scale: set the room\'s longest side.') +
          (room.levelled ? '' : ' The map isn\'t levelled: level it under Survey leveling, or turn the placed copy.'),
      });
      if (started === false) throw Error('Another import or edit is open in LiftLab.');
      reply(true, 'In LiftLab’s World editor: check its size, then Save to object library and Place in world.');
    } catch (err) { reply(false, err.message); }
  });
})();

// A survey surface as an OBJ model: polygons (outer ring and holes, map coordinates already levelled, Z up)
// triangulated, each triangle facing the nearest walk position, since that's the side the camera saw (and LiftLab
// draws front faces). Throws on anything malformed.
function surveyRoom(data) {
  const point = p => Array.isArray(p) && p.length === 3 && p.every(Number.isFinite);
  const ring = r => Array.isArray(r) && r.length >= 3 && r.length <= 100000 && r.every(point);
  if (!Array.isArray(data.polygons) || !data.polygons.length || data.polygons.length > 200000)
    throw Error('The survey sent no surfaces.');
  const views = Array.isArray(data.viewpoints) ? data.viewpoints.filter(point).slice(0, 20000) : [];
  const verts = [], ids = new Map(), faces = [];
  const vid = p => {
    const key = p.map(v => +v.toFixed(5)).join(' ');
    let i = ids.get(key);
    if (i === undefined) { i = verts.length + 1; ids.set(key, i); verts.push('v ' + key); }
    return i;
  };
  const open = r => r.length > 3 && r[0].every((v, i) => v === r[r.length - 1][i]) ? r.slice(0, -1) : r;
  const face = (a, b, c) => {
    const n = new THREE.Vector3(b[0] - a[0], b[1] - a[1], b[2] - a[2]).cross(new THREE.Vector3(c[0] - a[0], c[1] - a[1], c[2] - a[2]));
    if (n.lengthSq() < 1e-20) return;
    if (views.length) {
      const m = [0, 1, 2].map(k => (a[k] + b[k] + c[k]) / 3);
      let best = views[0], d = Infinity;
      for (const v of views) { const q = (v[0] - m[0]) ** 2 + (v[1] - m[1]) ** 2 + (v[2] - m[2]) ** 2; if (q < d) { d = q; best = v; } }
      if (n.x * (best[0] - m[0]) + n.y * (best[1] - m[1]) + n.z * (best[2] - m[2]) < 0) [b, c] = [c, b];
    }
    faces.push('f ' + vid(a) + ' ' + vid(b) + ' ' + vid(c));
  };
  for (const poly of data.polygons) {
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
  if (!faces.length) throw Error('The surfaces have no area.');
  if (faces.length > 100000) throw Error(faces.length.toLocaleString() + ' triangles: a world object takes 100,000 at most. Simplify the surface in the survey first.');
  const name = (typeof data.name === 'string' && data.name.trim() ? data.name.trim() : 'Room survey').replace(/[^\w .·()-]+/g, ' ').slice(0, 60);
  return { name, obj: '# LiftLab room survey\n' + verts.join('\n') + '\n' + faces.join('\n') + '\n', triangles: faces.length,
    metres: data.units === 'm', levelled: !!data.levelled, scale: data.units === 'm' ? null : surveyScale(data.scale) };
}

// A size for a surface without metres: an output's ToF estimate ({metres_per_unit, median_error_m, p80_error_m,
// hits, eligible}), with how well it held up, else null. Readings out of range are dropped.
function surveyScale(s) {
  const k = s?.metres_per_unit, num = v => Number.isFinite(v) && v >= 0 ? v : null;
  if (!Number.isFinite(k) || k < 1e-4 || k > 1e3) return null;
  const med = num(s.median_error_m), p80 = num(s.p80_error_m), hits = num(s.hits), of = num(s.eligible);
  const cm = v => Math.round(v * 100) + ' cm';
  return { k, text: 'Sized from the range sensor: ' + (+k.toPrecision(3)) + ' m per survey unit' +
    (med !== null ? ', readings within ' + cm(med) + (p80 !== null ? ' (80% within ' + cm(p80) + ')' : '') : '') +
    (hits !== null && of !== null ? ' on ' + hits + ' of ' + of + ' held-out beams' : '') + '. Change the longest side if it looks wrong.' };
}
