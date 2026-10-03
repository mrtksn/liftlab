'use strict';
// Where each number on the page comes from. A simulator shows things no drone can know (the true position, the true
// temperatures), things the drone's own code worked out (read straight from the boards), what came down the radio,
// and the simulator's own analysis of the design; these tags keep them apart. Mark an element with
// data-src="sim board" and applySrcTags() puts the tags in it; srcTag()/srcDot() make them for built content.

const SRC = {
  sim: ['Simulated', 'The simulated world itself: what really happens (true position, thrust, temperatures). A real drone can\'t know this.'],
  sensor: ['Sensor', 'What the drone\'s simulated sensors report: the truth with each sensor\'s noise, delay and limits. This is all the drone gets.'],
  board: ['On board', 'Worked out by the drone\'s own flight code, read straight from its boards, as if a cable were plugged in. Nothing here went over the radio.'],
  tlm: ['Telemetry', 'Received from the drone over the radio link (decoded by the command module): late, partial or missing when the link is weak.'],
  gnd: ['Command module', 'Worked out by the command module\'s code, the pilot\'s side of the radio (the same C as on an ESP32, a Pi or a Mac).'],
  cmp: ['Vs truth', 'The drone\'s belief compared with the simulated truth. Only a simulator can show this.'],
  calc: ['Calculated', 'The simulator\'s analysis of your design. The drone never works this out.'],
  you: ['You', 'What you set or do: settings, targets, the sticks.'],
};
const SRC_ORDER = ['sim', 'sensor', 'board', 'tlm', 'gnd', 'cmp', 'calc', 'you'];

function srcTag(k) {
  const d = SRC[k]; if (!d) return document.createTextNode('');
  const t = document.createElement('span'); t.className = 'src src-' + k; t.title = `${d[0]}: ${d[1]}`;
  const i = document.createElement('i'); t.append(i, d[0]);
  return t;
}
function srcDot(k) {
  const d = SRC[k]; const t = document.createElement('i'); t.className = 'src-dot src-' + k;
  if (d) { t.title = `${d[0]}: ${d[1]}`; t.setAttribute('aria-label', d[0]); t.setAttribute('role', 'img'); }
  return t;
}
function srcTags(keys) {
  const s = document.createElement('span'); s.className = 'srcs';
  for (const k of keys) s.append(srcTag(k));
  return s;
}
// Tags for every element marked data-src (once each). In a section heading they go after the title.
function applySrcTags(root) {
  for (const e of (root || document).querySelectorAll('[data-src]')) {
    if (e.querySelector(':scope > .srcs')) continue;
    const tags = srcTags(e.dataset.src.split(/\s+/).filter(Boolean));
    const small = e.querySelector(':scope > small');
    if (small) e.insertBefore(tags, small); else e.append(tags);
  }
}
// The key: the tags in a line, and what each means when opened.
function srcLegend(keys) {
  const d = document.createElement('details'); d.className = 'src-legend';
  const s = document.createElement('summary'); s.append(Object.assign(document.createElement('span'), { className: 'lbl', textContent: 'Where it comes from' }));
  for (const k of keys) s.append(srcTag(k));
  const dl = document.createElement('dl');
  for (const k of keys) { const dt = document.createElement('dt'), dd = document.createElement('dd'); dt.append(srcTag(k)); dd.textContent = SRC[k][1]; dl.append(dt, dd); }
  d.append(s, dl);
  return d;
}

// The static panels: their tags, and the key at the top of the simulator's readouts.
applySrcTags();
{ const p = document.getElementById('telemetry'); if (p) p.prepend(srcLegend(['sim', 'sensor', 'board', 'cmp', 'calc', 'you'])); }
