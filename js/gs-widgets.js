'use strict';
// Data displays for received telemetry: small widgets the ground station is built from (and any panel may use).
// Each one is { el, set(…), age(now) }: set gives it a value and the time it was received; age(now) greys it out
// when the value is getting old (over 1.5 s) and blanks it when it's stale (over 5 s), so a link that has gone quiet
// can't pass for a drone that holds still.
const GSW = (() => {
  const mk = (tag, cls, txt) => { const e = document.createElement(tag); if (cls) e.className = cls; if (txt != null) e.textContent = txt; return e; };
  const txt = (n, s) => { if (n.textContent !== s) n.textContent = s; };   // (only what changed: a reading stays selectable)
  const tok = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const staleness = (el, t, now) => { const a = t == null ? Infinity : now - t; el.classList.toggle('w-old', a > 1.5 && a <= 5); el.classList.toggle('w-stale', a > 5); return a; };
  const fit = cv => {   // a canvas at the device's pixel ratio, sized by its CSS box
    const dpr = Math.min(window.devicePixelRatio || 1, 2), W = cv.clientWidth, H = cv.clientHeight;
    if (cv.width !== Math.round(W * dpr) || cv.height !== Math.round(H * dpr)) { cv.width = Math.round(W * dpr); cv.height = Math.round(H * dpr); }
    const c = cv.getContext('2d'); c.setTransform(dpr, 0, 0, dpr, 0, 0); c.clearRect(0, 0, W, H); return { c, W, H };
  };

  // A number with its label and unit. opts: { unit, dp, fmt(v) → text, tone(v) → '' | 'good' | 'warn' | 'bad' }
  function value(label, opts = {}) {
    const el = mk('div', 'w-value' + (opts.text ? ' w-text' : '')), v = mk('span', 'w-num', '—'), u = mk('span', 'w-unit', opts.unit || '');
    el.append(mk('span', 'w-label', label), mk('span', 'w-read', null)); el.lastChild.append(v, u);
    let t = null, last = null;
    return {
      el,
      set(x, at) { last = x; t = at; txt(v, x == null || !isFinite(x) && typeof x === 'number' ? '—' : opts.fmt ? opts.fmt(x) : (+x).toFixed(opts.dp ?? 1)); el.dataset.tone = opts.tone ? opts.tone(x) : ''; },
      age(now) { if (staleness(el, t, now) > 5) txt(v, '—'); else if (last != null && opts.fmt) txt(v, opts.fmt(last)); },
    };
  }
  // A horizontal bar: opts { min, max, unit, dp, tone(v) }
  function bar(label, opts = {}) {
    const el = mk('div', 'w-bar'), fill = mk('i'), num = mk('span', 'w-num', '—'), track = mk('span', 'w-track');
    track.append(fill); el.append(mk('span', 'w-label', label), track, num);
    let t = null; const lo = opts.min ?? 0, hi = opts.max ?? 1;
    return {
      el,
      set(x, at) { t = at; const f = clamp((x - lo) / (hi - lo), 0, 1); fill.style.width = (f * 100).toFixed(1) + '%'; txt(num, opts.fmt ? opts.fmt(x) : `${(+x).toFixed(opts.dp ?? 0)}${opts.unit || ''}`); el.dataset.tone = opts.tone ? opts.tone(x) : ''; },
      age(now) { if (staleness(el, t, now) > 5) { txt(num, '—'); fill.style.width = '0'; } },
    };
  }
  // A word in a pill (the flight mode).
  function badge() {
    const el = mk('span', 'w-badge', '—'); let t = null;
    return { el, set(text, at, tone) { t = at; txt(el, text); el.dataset.tone = tone || ''; }, age(now) { if (staleness(el, t, now) > 5) txt(el, 'NO DATA'); } };
  }
  // An artificial horizon: roll and pitch move the horizon, the heading in the corner.
  function horizon() {
    const el = mk('div', 'w-horizon'), cv = mk('canvas'); el.append(cv);
    let a = null, t = null, stale = false;
    const draw = () => {
      const { c, W, H } = fit(cv); if (!W) return;
      const cx = W / 2, cy = H / 2, R = Math.hypot(W, H);
      const sky = stale ? tok('--panel-2') : tok('--accent-soft'), ground = stale ? tok('--line') : tok('--warn-soft'), ink = tok('--ink'), muted = tok('--muted');
      const roll = a ? a.roll : 0, pitch = a ? a.pitch : 0, ppr = H / 1.2;   // pixels per radian of pitch
      c.save(); c.translate(cx, cy); c.rotate(-roll); c.translate(0, pitch * ppr);
      c.fillStyle = sky; c.fillRect(-R, -R, 2 * R, R); c.fillStyle = ground; c.fillRect(-R, 0, 2 * R, R);
      c.strokeStyle = ink; c.lineWidth = 1.5; c.beginPath(); c.moveTo(-R, 0); c.lineTo(R, 0); c.stroke();
      c.lineWidth = 1; c.fillStyle = muted; c.font = '10px ' + tok('--f-mono'); c.textAlign = 'left'; c.textBaseline = 'middle';
      for (let d = -30; d <= 30; d += 10) if (d) { const y = -d * D2R * ppr, w = d % 20 ? 14 : 24; c.beginPath(); c.moveTo(-w, y); c.lineTo(w, y); c.stroke(); c.fillText(String(d), w + 4, y); }
      c.restore();
      c.strokeStyle = tok('--bad'); c.lineWidth = 2.5; c.beginPath(); c.moveTo(cx - 40, cy); c.lineTo(cx - 12, cy); c.lineTo(cx - 6, cy + 6); c.moveTo(cx + 40, cy); c.lineTo(cx + 12, cy); c.lineTo(cx + 6, cy + 6); c.stroke();
      c.fillStyle = tok('--bad'); c.beginPath(); c.arc(cx, cy, 2.5, 0, 7); c.fill();
      c.fillStyle = ink; c.font = '600 12px ' + tok('--f-mono'); c.textAlign = 'right'; c.textBaseline = 'top';
      c.fillText(a ? `${String((Math.round(-a.yaw * R2D) % 360 + 360) % 360).padStart(3, '0')}°` : '---', W - 6, 6);   // a compass heading (x north; yaw turns from x toward y, west)
      c.textAlign = 'left'; c.fillText(a ? `R ${(roll * R2D).toFixed(0)}° P ${(pitch * R2D).toFixed(0)}°` : '', 6, 6);
      if (stale) { c.fillStyle = muted; c.textAlign = 'center'; c.textBaseline = 'middle'; c.fillText('NO ATTITUDE', cx, cy + 22); }
    };
    return { el, set(att, at) { a = att; t = at; draw(); }, age(now) { const s = staleness(el, t, now) > 5; if (s !== stale) { stale = s; draw(); } }, draw };
  }
  // A map seen from above: the track, home, the target and the drone, scaled to fit.
  function map() {
    const el = mk('div', 'w-map'), cv = mk('canvas'); el.append(cv);
    let s = { track: [], pos: null, target: null, heading: 0 }, t = null;
    const draw = () => {
      const { c, W, H } = fit(cv); if (!W) return;
      const pts = [[0, 0], ...s.track, ...(s.pos ? [s.pos] : []), ...(s.target ? [s.target] : [])];
      let span = 4; for (const p of pts) span = Math.max(span, Math.abs(p[0]) * 2.4, Math.abs(p[1]) * 2.4);
      const k = Math.min(W, H) / span, X = p => W / 2 - p[1] * k, Y = p => H / 2 - p[0] * k;   // x north up, y west left
      c.strokeStyle = tok('--grid'); c.lineWidth = 1; const g = span > 40 ? 10 : span > 12 ? 5 : 1;
      for (let v = -Math.ceil(span / g) * g; v <= span; v += g) { c.beginPath(); c.moveTo(X([0, v]), 0); c.lineTo(X([0, v]), H); c.moveTo(0, Y([v, 0])); c.lineTo(W, Y([v, 0])); c.stroke(); }
      c.strokeStyle = tok('--accent'); c.lineWidth = 1.5; c.beginPath(); s.track.forEach((p, i) => i ? c.lineTo(X(p), Y(p)) : c.moveTo(X(p), Y(p))); c.stroke();
      c.strokeStyle = tok('--good'); c.lineWidth = 2; c.beginPath(); c.arc(X([0, 0]), Y([0, 0]), 6, 0, 7); c.stroke();
      c.fillStyle = tok('--good'); c.font = '600 10px ' + tok('--f-mono'); c.fillText('H', X([0, 0]) - 3, Y([0, 0]) + 3.5);
      if (s.target) { c.strokeStyle = tok('--warn'); c.lineWidth = 1.5; const x = X(s.target), y = Y(s.target); c.beginPath(); c.moveTo(x - 5, y - 5); c.lineTo(x + 5, y + 5); c.moveTo(x + 5, y - 5); c.lineTo(x - 5, y + 5); c.stroke(); }
      if (s.pos) {
        const x = X(s.pos), y = Y(s.pos), h = s.heading;
        c.fillStyle = tok('--ink'); c.beginPath(); c.moveTo(x - Math.sin(h) * 9, y - Math.cos(h) * 9);
        c.lineTo(x + Math.sin(h + 2.5) * 6, y + Math.cos(h + 2.5) * 6); c.lineTo(x + Math.sin(h - 2.5) * 6, y + Math.cos(h - 2.5) * 6); c.closePath(); c.fill();
      }
      c.fillStyle = tok('--muted'); c.font = '10px ' + tok('--f-mono'); c.textAlign = 'right'; c.fillText(`grid ${g} m`, W - 6, H - 6); c.textAlign = 'left';
    };
    return { el, set(st, at) { s = st; t = at; draw(); }, age(now) { staleness(el, t, now); }, draw };
  }
  // Several bars side by side (motor throttles).
  function columns(label) {
    const el = mk('div', 'w-cols'), row = mk('div', 'w-colrow'); el.append(mk('span', 'w-label', label), row);
    let t = null, cells = [];
    return {
      el,
      set(vals, at, names, marks) {
        t = at;
        if (cells.length !== vals.length) { row.textContent = ''; cells = vals.map((v, i) => { const w = mk('span', 'w-colwrap'), c = mk('span', 'w-col'), f = mk('i'), n = mk('em', null, names ? names[i] : String(i + 1)); c.append(f); w.append(c, n); row.append(w); return { f, c }; }); }
        vals.forEach((v, i) => { cells[i].f.style.height = (clamp(v, 0, 1) * 100).toFixed(0) + '%'; cells[i].c.title = `${names ? names[i] : 'motor ' + (i + 1)}: ${Math.round(v * 100)}%`;
          const mk2 = marks && marks[i]; cells[i].c.dataset.tone = mk2 == null ? '' : mk2 < 0 ? 'bad' : mk2 < 0.97 ? 'warn' : ''; });
      },
      age(now) { staleness(el, t, now); },
    };
  }
  // A log of messages, newest first; sev as MAVLink (≤3 bad, 4 warn).
  function log(max = 30) {
    const el = mk('ol', 'w-log'); let shown = '';
    return {
      el,
      set(items) {
        const key = items.length + ':' + (items[0] ? items[0].t + items[0].text : ''); if (key === shown) return; shown = key;
        el.textContent = '';
        for (const m of items.slice(0, max)) { const li = mk('li'); li.dataset.tone = m.sev <= 3 ? 'bad' : m.sev === 4 ? 'warn' : ''; li.append(mk('b', null, m.t.toFixed(1) + ' s'), document.createTextNode(' ' + m.text)); el.append(li); }
        if (!items.length) el.append(mk('li', 'muted', 'Nothing yet.'));
      },
      age() {},
    };
  }
  // A short line chart of the last n values: push one, or set them all (a history kept elsewhere).
  function sparkline(label, opts = {}) {
    const el = mk('div', 'w-spark'), cv = mk('canvas'), num = mk('span', 'w-num', '—'); el.append(mk('span', 'w-label', label), cv, num);
    let vals = [], t = null;
    const draw = () => {
      const v = vals[vals.length - 1]; if (v == null) return; txt(num, opts.fmt ? opts.fmt(v) : v.toFixed(opts.dp ?? 0));
      const { c, W, H } = fit(cv); if (!W) return;
      const val = x => typeof x === 'function' ? x() : x, lo = val(opts.min) ?? Math.min(...vals), hi = val(opts.max) ?? Math.max(...vals, lo + 1e-6);
      c.strokeStyle = tok('--accent'); c.lineWidth = 1.5; c.beginPath();
      vals.forEach((x, i) => { const px = i / Math.max(1, (opts.n || 60) - 1) * W, py = H - 2 - clamp((x - lo) / (hi - lo || 1), 0, 1) * (H - 4); i ? c.lineTo(px, py) : c.moveTo(px, py); }); c.stroke();
    };
    return {
      el,
      push(v, at) { t = at; vals.push(v); if (vals.length > (opts.n || 60)) vals.shift(); draw(); },
      set(list, at) { t = at; vals = list.slice(-(opts.n || 60)); draw(); },
      age(now) { if (staleness(el, t, now) > 5) txt(num, '—'); },
    };
  }
  return { value, bar, badge, horizon, map, columns, log, sparkline };
})();
