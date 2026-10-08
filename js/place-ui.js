'use strict';
// Where a new part goes. Each Add button opens a list of the airframe as it is (the frame, then each servo, rod and
// latch, nested the way the parts list nests them), and you pick where the part goes:
//   on Frame / on a servo, rod or latch: it's attached there, the way dragging it there would;
//   between a holder and a part (servos, rods and latches only): it goes in above that part, which then hangs from it.
//   Nothing moves: the new part takes the place where the joint is, and the part keeps its place.

const INSERTABLE = new Set(['latch', 'joint', 'link']);
const KIND_WORD = { battery: 'battery', motor: 'motor', tilt: 'motor on a servo', joint: 'servo', link: 'rod', mass: 'mass', hang: 'cable mass', wing: 'wing', latch: 'latch',
  imu: 'IMU', mag: 'compass', baro: 'barometer', fix: 'position fix', flow: 'flow sensor' };

// Put a just-made part (c, already in cfg.comps; with its own parts, a motor on a servo) where place says.
function placeNew(c, place) {
  if (!place) return;
  if (place.on) {
    const a = place.on;
    if (a.type === 'latch' && !place.point) {   // hung from the hook, its top at the hook
      const to = add(hookOf(a), [0, 0, -topOf(c)]), d = sub(to, c.pos);
      c.pos = to.map(v => +v.toFixed(4)); shiftSubtree(c, d); c.parent = a.id;
    } else attachTo(c, a === cfg.frame ? null : a, place.point || null);
    return;
  }
  const x = place.above, p = parentOf(x);   // in between p (or the frame) and x
  if (c.type === 'latch') c.pos = sub(add(x.pos, [0, 0, topOf(x)]), LATCH_HOOK).map(v => +v.toFixed(4));   // its hook at x's top
  else if (c.type === 'joint') c.pos = x.pos.slice();                                                       // its pivot at x
  else if (c.type === 'link') {                                                                               // from where x hangs, to x
    const from = !p ? [0, 0, x.pos[2]] : p.type === 'link' ? linkTip(p) : p.type === 'latch' ? hookOf(p) : p.pos;
    const d = sub(x.pos, from);
    if (nrm(d) > 0.01) { c.pos = from.map(v => +v.toFixed(4)); c.length = +nrm(d).toFixed(4); setDirAzEl(c, d, 'az', 'el'); }
    else { c.length = 0.1; c.az = 0; c.el = -90; c.pos = add(x.pos, [0, 0, 0.1]).map(v => +v.toFixed(4)); }   // (x is right on it: a short rod down to x)
  }
  c.parent = p ? p.id : null; x.parent = c.id;
}

// A latch does nothing until a board drives it: the Cargo task. Put that on the flight controller if no board has it.
function latchNeedsBoard() {
  if (hasTask('cargo')) return '';
  const C = JSON.parse(JSON.stringify(computers()));
  const b = C.boards.find(b => b.tasks.includes('core')) || C.boards[0]; b.tasks.push('cargo');
  setComputers(C, 'cargo');
  const msg = `the Cargo task is now on ${b.name}: it drives the latches`;
  if (typeof cargoSay === 'function') cargoSay(msg);
  if (typeof rnEvent === 'function') rnEvent(msg);
  return msg;
}

/* ───────── the picker ───────── */
let placeMenu = null;
function closePlace() { if (placeMenu) { placeMenu.node.remove(); placeMenu.btn.setAttribute('aria-expanded', 'false'); placeMenu = null; } }
function openPlace(btn, type, add = place => addComp(type, place)) {
  const again = placeMenu && placeMenu.btn === btn; closePlace(); if (again) return;
  const word = KIND_WORD[type] || type, ins = INSERTABLE.has(type);
  const box = el('div', { class: 'place-menu', role: 'menu', 'aria-label': 'Where to add the ' + word });
  box.append(el('div', { class: 'place-head', text: `Where does the ${word} go?` }));
  const pick = (label, place, depth, cls) => {
    const b = UI.button( { class: 'place-opt' + (cls ? ' ' + cls : ''), type: 'button', role: 'menuitem', style: `padding-left:${10 + 16 * depth}px` });
    b.innerHTML = label;
    b.addEventListener('click', () => { closePlace(); add(place); });
    box.append(b);
    return b;
  };
  const esc = s => String(s).replace(/[&<>"]/g, ch => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[ch]));
  const KIND = { joint: 'servo', link: 'rod', latch: 'latch' };
  pick('on <b>Frame</b>', null, 0, 'holder');
  for (const p of partPoints(cfg.frame)) pick(`on <b>Frame</b> · ${esc(p.name)}`, { on: cfg.frame, point: p.id }, 1, 'holder');
  const walk = (list, depth, under) => {
    for (const c of list) {
      const kids = childrenOf(c);
      if (ins) pick(`<span class="ins">↳ between</span> ${esc(under)} <span class="ins">and</span> ${esc(c.name)}`, { above: c }, depth, 'between');
      if (isHolder(c)) {
        pick(`on <b>${esc(c.name)}</b> <span class="kind">${KIND[c.type] || 'mount'}</span>`, { on: c }, depth, 'holder');
        for (const p of partPoints(c)) pick(`on <b>${esc(c.name)}</b> · ${esc(p.name)}`, { on: c, point: p.id }, depth + 1, 'holder');
      }
      walk(ins ? kids : kids.filter(isHolder), depth + 1, c.name);
    }
  };
  // Without "between" options only the holders matter (only they carry anything); with them, every part (each can get one above it).
  const roots = cfg.comps.filter(x => !parentOf(x));
  walk(ins ? roots : roots.filter(isHolder), 1, 'Frame');
  if (!ins && !cfg.comps.some(isHolder)) box.append(el('p', { class: 'hint', text: 'Add a servo, rod or latch to attach parts to it.' }));
  else if (ins) box.append(el('p', { class: 'hint', text: 'Between: it goes in above that part, which then hangs from it. Nothing moves.' }));
  const host = btn.closest('.sec') || btn.parentElement;
  host.style.position = 'relative'; host.append(box);
  box.style.top = (btn.offsetTop + btn.offsetHeight + 4) + 'px';
  btn.setAttribute('aria-expanded', 'true');
  placeMenu = { node: box, btn };
  (box.querySelector('.place-opt') || box).focus();
}
$('#addPart').addEventListener('click', () => { closePlace(); $('#addPartDlg').showModal(); });
document.querySelectorAll('[data-add]').forEach(b => {
  b.addEventListener('click', e => {
    e.stopPropagation(); $('#addPartDlg').close(); openPlace($('#addPart'), b.dataset.add);
  });
});
document.addEventListener('click', e => { if (placeMenu && !placeMenu.node.contains(e.target)) closePlace(); });
document.addEventListener('keydown', e => {
  if (!placeMenu) return;
  if (e.key === 'Escape') { const b = placeMenu.btn; closePlace(); b.focus(); e.stopPropagation(); }
  else if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
    const o = [...placeMenu.node.querySelectorAll('.place-opt')], i = o.indexOf(document.activeElement);
    o[(i + (e.key === 'ArrowDown' ? 1 : o.length - 1)) % o.length].focus(); e.preventDefault();
  }
}, true);
