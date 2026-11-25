// Exercise the browser controls with a simulated serial connection, never hardware.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

function panel() {
  const elements = new Map(), windowEvents = {}, documentEvents = {}, serialEvents = {};
  const intervals = new Map(), commands = [];
  let intervalId = 0;
  function element(id) {
    if (!elements.has(id)) elements.set(id, {
      value: id === 'width' ? '1500' : '', checked: false, textContent: '',
      classList: {toggle() {}}, events: {}, attributes: {},
      setAttribute(name, value) { this.attributes[name] = value; },
      addEventListener(type, fn) { this.events[type] = fn; },
      setPointerCapture() {},
    });
    return elements.get(id);
  }
  const preset = element('preset'); preset.dataset = {width: '1000'};
  const document = {
    hidden: false, getElementById: element,
    querySelectorAll: selector => selector === '[data-width]' ? [preset] : [],
    addEventListener: (type, fn) => documentEvents[type] = fn,
  };
  const context = vm.createContext({
    document, window: {addEventListener: (type, fn) => windowEvents[type] = fn},
    navigator: {serial: {addEventListener: (type, fn) => serialEvents[type] = fn}},
    TextEncoder, TextDecoder, setTimeout, clearTimeout,
    setInterval: fn => { intervals.set(++intervalId, fn); return intervalId; },
    clearInterval: id => intervals.delete(id),
    fakeWriter: {write: async bytes => commands.push(new TextDecoder().decode(bytes).trim())},
  });
  const run = source => vm.runInContext(source, context);
  vm.runInContext(fs.readFileSync(`${__dirname}/app.js`, 'utf8'), context);
  const flush = () => new Promise(resolve => setImmediate(resolve));
  return {
    element, preset, commands, intervals, windowEvents, documentEvents, serialEvents, document, run, flush,
    arm() { run('writer = fakeWriter; active = ready = armed = true; draw();'); },
    on() { element('continuous').onclick(); },
    off() { element('continuous').onclick(); },
  };
}

test('continuous run requires arming, updates live, and switches off', async () => {
  const p = panel();
  assert.equal(p.element('continuous').disabled, true);
  p.on(); assert.equal(p.element('continuous').attributes['aria-pressed'], 'false');
  p.arm(); p.on(); await p.flush();
  assert.deepEqual(p.commands, ['RUN 1500']);
  assert.equal(p.intervals.size, 1);
  assert.equal(p.element('width').disabled, false);
  assert.equal(p.element('run').disabled, true);
  p.element('width').value = '1800'; p.element('width').oninput(); await p.flush();
  assert.equal(p.commands.at(-1), 'RUN 1800');
  p.preset.onclick(); await p.flush(); assert.equal(p.commands.at(-1), 'RUN 1000');
  // A delayed release from the hold button must not end continuous mode.
  p.element('run').events.lostpointercapture();
  assert.equal(p.element('continuous').attributes['aria-pressed'], 'true');
  for (const tick of p.intervals.values()) tick();
  await p.flush(); assert.equal(p.commands.at(-1), 'RUN 1000');
  p.off(); await p.flush();
  assert.equal(p.commands.at(-1), 'STOP');
  assert.equal(p.intervals.size, 0);
  assert.equal(p.element('continuous').attributes['aria-pressed'], 'false');
  const count = p.commands.length;
  p.element('width').oninput(); await p.flush(); assert.equal(p.commands.length, count);
});

test('press and hold still stops on release', async () => {
  const p = panel(); p.arm();
  p.element('run').events.pointerdown({preventDefault() {}, pointerId: 1});
  await p.flush(); assert.equal(p.commands.at(-1), 'RUN 1500');
  assert.equal(p.element('width').disabled, true);
  assert.equal(p.element('continuous').disabled, true);
  p.element('run').events.pointerup(); await p.flush();
  assert.equal(p.commands.at(-1), 'STOP');
  assert.equal(p.element('width').disabled, false);
  assert.equal(p.intervals.size, 0);
});

for (const action of ['stop', 'disarm', 'blur', 'hidden', 'timeout', 'device disarm', 'USB loss', 'write failure', 'disconnect']) {
  test(`continuous mode resets after ${action} and never resumes automatically`, async () => {
    const p = panel(); p.arm(); p.on(); await p.flush();
    if (action === 'stop' || action === 'disarm') p.element(action).onclick();
    if (action === 'blur') p.windowEvents.blur();
    if (action === 'hidden') { p.document.hidden = true; p.documentEvents.visibilitychange(); }
    if (action === 'timeout') p.run('onLine("ESC TIMEOUT")');
    if (action === 'device disarm') p.run('onLine("ESC DISARMED")');
    if (action === 'USB loss') p.serialEvents.disconnect();
    if (action === 'write failure') {
      p.run('writer.write = async () => { throw new Error("USB failure"); };');
      for (const tick of p.intervals.values()) tick();
    }
    if (action === 'disconnect') await p.run('disconnect()');
    await p.flush();
    assert.equal(p.element('continuous').attributes['aria-pressed'], 'false');
    assert.equal(p.intervals.size, 0);
    if (['stop', 'blur', 'hidden'].includes(action)) assert.equal(p.commands.at(-1), 'STOP');
    if (action === 'disarm') assert.equal(p.commands.at(-1), 'DISARM');
    p.arm(); assert.equal(p.element('continuous').attributes['aria-pressed'], 'false');
    const count = p.commands.length;
    p.element('width').oninput(); await p.flush(); assert.equal(p.commands.length, count);
  });
}

test('Stop cancels queued run commands before they reach serial', async () => {
  const p = panel(); p.arm(); p.on();
  p.element('width').value = '1900'; p.element('width').oninput();
  p.element('stop').onclick(); await p.flush();
  assert.ok(p.commands.length > 0);
  assert.ok(p.commands.every(command => command === 'STOP'), p.commands.join(', '));
});
