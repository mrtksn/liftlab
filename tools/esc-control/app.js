'use strict';
const $ = id => document.getElementById(id);
let port, writer, reader, active = false, armed = false, ready = false, holding = false;
let heartbeat = null, writeChain = Promise.resolve(), carry = '', readyTimer;
let runGeneration = 0;
const limits = {490:2000, 400:2450, 300:3000};
let selectedFreq = 490;
const enc = new TextEncoder();
function log(message) { const el = $('log'); if (el.textContent === 'Waiting for connection.') el.textContent = ''; el.textContent += message + '\n'; el.scrollTop = el.scrollHeight; }
function draw() {
  $('status').classList.toggle('on', ready);
  $('status').innerHTML = '<span></span>' + (ready ? 'Connected' : active ? 'Connecting' : 'Disconnected');
  $('connect').disabled = active; $('disconnect').disabled = !active;
  $('arm').disabled = !ready || armed || (selectedFreq !== 490 && !$('ack').checked); $('stop').disabled = !ready; $('disarm').disabled = !ready;
  $('frequencies').disabled = active;
  $('run').disabled = !ready || !armed; $('width').disabled = holding;
  document.querySelectorAll('[data-width]').forEach(b => b.disabled = holding);
  $('run').classList.toggle('active', holding); $('run').textContent = holding ? 'Release to stop' : 'Hold to run';
  $('mode').textContent = holding ? 'Running signal' : armed ? 'Armed · stop signal' : 'Disarmed';
  $('modeHelp').textContent = holding ? 'The run pulse is being sent repeatedly.' : armed ? 'Stop-like signal is being held. Hold Run to apply the selected width.' : 'Arm sends a stop-like signal and enables Run.';
}
function write(data) {
  const current = writer;
  if (!current) return Promise.reject(new Error('Not connected'));
  writeChain = writeChain.then(() => current.write(enc.encode(data)));
  return writeChain.catch(err => { log('Serial error: ' + err.message); emergencyLocal(); throw err; });
}
function send(command) { return write(command + '\n'); }
function emergencyLocal() { runGeneration++; clearInterval(heartbeat); heartbeat = null; holding = false; armed = false; ready = false; draw(); }
function stopRun() {
  if (!holding) return;
  runGeneration++; holding = false; clearInterval(heartbeat); heartbeat = null; draw();
  if (ready) send('STOP').catch(() => {});
}
function onLine(line) {
  if (!line.includes('ESC ')) return;
  if (line.includes('ESC READY')) { log('Device ready; configuring frequency.'); send('CONFIG ' + selectedFreq).catch(() => {}); }
  else if (line.includes('ESC CONFIGURED ' + selectedFreq)) { clearTimeout(readyTimer); ready = true; log('Configured at ' + selectedFreq + ' Hz. No PWM sent yet.'); draw(); }
  else if (line.includes('ESC CONFIG ERROR')) { log('Frequency configuration failed. Disconnect and retry.'); emergencyLocal(); }
  else if (line.includes('ESC TIMEOUT')) { runGeneration++; holding = false; armed = false; clearInterval(heartbeat); heartbeat = null; log('Run heartbeat timed out; device requested stop and disarmed.'); draw(); }
  else if (line.includes('ESC ARMED')) { armed = true; log('Stop-like signal applied; controls armed.'); draw(); }
  else if (line.includes('ESC DISARMED')) { armed = false; log('Disarmed; stop-like signal held.'); draw(); }
  else if (line.includes('ESC STOPPED')) log('Stop-like signal applied.');
}
async function readLoop() {
  const dec = new TextDecoder();
  try {
    while (active && port.readable) {
      const result = await reader.read(); if (result.done) break;
      carry += dec.decode(result.value, {stream:true});
      const lines = carry.split(/[\r\n]+/); carry = lines.pop().slice(-4096); lines.forEach(onLine);
    }
  } catch (e) { if (active) log('Serial read ended: ' + e.message); }
  finally { if (active) { emergencyLocal(); active = false; log('Connection lost. Disconnect the battery if the motor has not stopped.'); draw(); } }
}
async function disconnect() {
  stopRun();
  if (ready) { try { await send('DISARM'); await send('QUIT'); } catch (_) {} }
  active = false; emergencyLocal(); clearTimeout(readyTimer);
  try { if (reader) { await reader.cancel(); reader.releaseLock(); } } catch (_) {}
  try { if (writer) writer.releaseLock(); } catch (_) {}
  try { if (port) await port.close(); } catch (_) {}
  reader = writer = port = null; writeChain = Promise.resolve(); carry = ''; log('Disconnected. Output is no longer controlled.'); draw();
}
$('connect').onclick = async () => {
  if (!navigator.serial) { log('Web Serial is unavailable. Open this page in Chrome or Edge on localhost.'); return; }
  try {
    port = await navigator.serial.requestPort(); await port.open({baudRate:115200});
    writer = port.writable.getWriter(); reader = port.readable.getReader(); active = true; draw(); readLoop();
    const response = await fetch('device.py', {cache:'no-store'}); if (!response.ok) throw new Error('Cannot load device program');
    const code = await response.text();
    await write('\x03\x03\r'); await new Promise(resolve => setTimeout(resolve, 250));
    // Raw REPL avoids echoing the program text, which contains protocol markers.
    await write('\x01'); await new Promise(resolve => setTimeout(resolve, 100));
    await write('exec(' + JSON.stringify(code) + ')\x04');
    readyTimer = setTimeout(() => { if (!ready) { log('Device did not respond. Disconnect and retry with the ESC battery disconnected.'); disconnect(); } }, 5000);
  } catch (e) { log('Connection failed: ' + e.message); await disconnect(); }
};
$('disconnect').onclick = disconnect;
$('ack').onchange = draw;
document.querySelectorAll('input[name="freq"]').forEach(radio => radio.onchange = () => {
  selectedFreq = Number(radio.value);
  $('width').max = limits[selectedFreq];
  if (Number($('width').value) > limits[selectedFreq]) $('width').value = limits[selectedFreq];
  $('value').textContent = $('width').value;
  $('maxLabel').textContent = limits[selectedFreq].toLocaleString() + ' µs';
  $('freqNote').textContent = selectedFreq === 490 ? '490 Hz: stop has been observed.' : selectedFreq + ' Hz: stop and ESC response are untested.';
  $('ack').checked = false; $('ackLabel').hidden = selectedFreq === 490; draw();
});
$('arm').onclick = () => { $('arm').disabled = true; send('ARM').catch(() => {}); };
$('stop').onclick = () => { stopRun(); send('STOP').catch(() => {}); };
$('disarm').onclick = () => { stopRun(); armed = false; draw(); send('DISARM').catch(() => {}); };
function startRun(e) {
  e.preventDefault(); if (!ready || !armed || holding) return;
  holding = true; const generation = ++runGeneration; draw();
  const tick = () => {
    const current = writer, value = $('width').value;
    writeChain = writeChain.then(() => {
      if (generation !== runGeneration || !holding || !ready || !armed || current !== writer) return;
      return current.write(enc.encode('RUN ' + value + '\n'));
    }).catch(err => { log('Serial error: ' + err.message); emergencyLocal(); });
  };
  tick(); heartbeat = setInterval(tick, 180);
  if (e.pointerId !== undefined) try { $('run').setPointerCapture(e.pointerId); } catch (_) {}
}
$('run').addEventListener('pointerdown', startRun);
$('run').addEventListener('pointerup', stopRun); $('run').addEventListener('pointercancel', stopRun); $('run').addEventListener('lostpointercapture', stopRun);
$('run').addEventListener('keydown', e => { if ((e.key === ' ' || e.key === 'Enter') && !e.repeat) startRun(e); });
$('run').addEventListener('keyup', e => { if (e.key === ' ' || e.key === 'Enter') { e.preventDefault(); stopRun(); } });
window.addEventListener('blur', stopRun); document.addEventListener('visibilitychange', () => { if (document.hidden) stopRun(); });
$('width').oninput = () => $('value').textContent = $('width').value;
document.querySelectorAll('[data-width]').forEach(b => b.onclick = () => { $('width').value = b.dataset.width; $('width').oninput(); });
if (navigator.serial) navigator.serial.addEventListener('disconnect', () => { if (active) { emergencyLocal(); log('USB disconnected. Disconnect the battery if the motor has not stopped.'); } });
draw();
