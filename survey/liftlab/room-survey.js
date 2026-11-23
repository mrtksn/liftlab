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
