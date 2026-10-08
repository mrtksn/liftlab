'use strict';
// Computers → Live data: every board's data bus (runner/fc/bus.h, docs/topic-bus.md), live. Each topic: this board's
// own or a copy from another board, how old it is, how often it comes, its values (hover a row for what they are).

// A topic's values, named from its layout ("q[4] w[3]" → q[0] … q[3], w[0] … w[2]).
function busLabels(layout) {
  const out = [];
  for (const f of (layout || '').split(' ')) { const m = /^(\w+)(?:\[(\d+)\])?$/.exec(f); if (!m) continue; const k = +(m[2] || 1); for (let i = 0; i < k; i++) out.push(m[2] ? `${m[1]}[${i}]` : m[1]); }
  return out;
}
const busUi = { rows: new Map(), prev: new Map(), sig: '' };
const busFmt = v => !isFinite(v) ? '—' : Math.abs(v) >= 1000 ? v.toExponential(1) : Math.abs(v) >= 100 ? v.toFixed(0) : v.toFixed(2);
const busAge = a => a < 0 ? 'never' : a < 1 ? Math.round(a * 1000) + ' ms' : a.toFixed(1) + ' s';

// Everything on every board now: [{ b, w, topics }].
// (the selected drone's, whichever drone's state happens to be installed when the timer fires)
function busSnapshot() {
  const read = () => computers().boards.map(b => ({ b, w: brt.inst.get(b.id) })).filter(x => x.w && brt.ready).map(x => ({ ...x, topics: busTopics(x.w), now: brt.t, err: brt.err }));
  return typeof fleet !== 'undefined' && fleet.ready && fleet.selected ? withDrone(fleet.selected, read) : read();
}
function renderBusView(box) {
  $('#computerDlgTitle').textContent = 'Live data';
  busUi.rows.clear(); busUi.prev.clear();
  box.append(el('p', { text: 'Each board keeps its programs\' latest values as named topics, and boards copy the topics they ask each other for over their link. ● this board\'s own; ↓ a copy from another board, as old as the link has made it.' }));
  const snap = busSnapshot();
  busUi.sig = snap.map(x => x.b.id + ':' + x.topics.map(t => t.name).join(',')).join('|');
  if (!snap.length) { box.append(el('p', { class: 'hint', text: 'The flight computers aren\'t running.' })); return; }
  const owner = name => snap.find(x => x.topics.some(t => t.name === name && !t.mirror));
  for (const { b, topics } of snap) {
    const body = el('tbody');
    for (const t of topics) {
      const from = t.mirror ? owner(t.name) : null;
      const cells = { age: el('td', { class: 'bus-num' }), rate: el('td', { class: 'bus-num' }), vals: el('td', { class: 'bus-vals' }) };
      body.append(el('tr', { 'data-topic': t.name }, el('td', {}, el('span', { class: 'bus-kind', text: t.mirror ? '↓' : '●', title: t.mirror ? 'a copy' : 'this board\'s' }), ' ', el('code', { text: t.name })),
        el('td', { class: 'hint', text: t.mirror ? (from ? from.b.name : 'another board') : 'here' }), cells.age, cells.rate, cells.vals));
      busUi.rows.set(b.id + '|' + t.name, cells);
    }
    box.append(UI.card({ class: 'hw-device bus-board', 'data-bus-board': b.id }, el('div', { class: 'hw-device-title' }, el('b', { text: b.name }), el('span', { class: 'hw-kind', text: BOARD_KINDS[b.kind].label })),
      topics.length ? el('div', { class: 'bus-scroll' }, el('table', { class: 'bus-table' }, el('thead', {}, el('tr', {}, ...['Topic', 'From', 'Age', 'Rate', 'Values'].map(h => el('th', { text: h })))), body))
        : el('p', { class: 'hint', text: 'No topics: this board runs no flight program.' })));
  }
  box.append(el('p', { class: 'hint', text: 'The flight core and the navigation publish their topics every step; a copy comes at the rate its board asked for, or when its values change (and every half second regardless). See docs/topic-bus.md.' }));
  updateBusView();
}
// Refresh the numbers in place (rebuild if the topics themselves changed).
function updateBusView() {
  const snap = busSnapshot(), sig = snap.map(x => x.b.id + ':' + x.topics.map(t => t.name).join(',')).join('|');
  if (sig !== busUi.sig) { renderComputerDetail(); return; }
  const now = snap.length ? snap[0].now : 0;
  for (const { b, topics } of snap) for (const t of topics) {
    const cells = busUi.rows.get(b.id + '|' + t.name); if (!cells) continue;
    const key = b.id + '|' + t.name, p = busUi.prev.get(key);
    let rate = p ? p.rate : 0;
    if (p && now > p.t + 0.2) { const d = (t.got - p.got + 16777216) % 16777216; rate = p.rate ? 0.5 * p.rate + 0.5 * d / (now - p.t) : d / (now - p.t); busUi.prev.set(key, { got: t.got, t: now, rate }); }
    else if (!p) busUi.prev.set(key, { got: t.got, t: now, rate: 0 });
    setText(cells.age, busAge(t.age)); setText(cells.rate, rate >= 0.5 ? (rate >= 100 ? Math.round(rate) : rate.toFixed(1)) + '/s' : '—');
    const labels = busLabels(t.layout), shown = t.vals.slice(0, 6).map(busFmt).join('  ') + (t.vals.length > 6 ? `  … +${t.vals.length - 6}` : '');
    setText(cells.vals, shown + (t.bad ? `  (${t.bad} refused)` : ''));
    cells.vals.title = t.layout + '\n' + t.vals.map((v, i) => (labels[i] || '#' + i) + ': ' + busFmt(v)).join('\n');
  }
}
setInterval(() => { const d = document.getElementById('computerDlg'); if (d && d.open && COMP.view && COMP.view.kind === 'bus' && (typeof fleet === 'undefined' || fleet.selected)) updateBusView(); }, 250);
