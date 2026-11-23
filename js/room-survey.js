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
})();
