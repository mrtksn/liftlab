'use strict';
// Sharing a design with a link, without a server: the whole design (airframe, computers, edited formulas) is compressed
// into the link's fragment (#design=…), which the browser never sends anywhere; the page that opens it unpacks it.
// The link needs the page at an address the other person can open (hosted somewhere); for a copy of the page on their
// own computer there's the code alone, to paste into Share → Open a shared design.
//
// The code: "v1." and the design file's JSON (design files, designs.js) compressed (deflate) in base64url; "j1." and the
// JSON uncompressed where the browser can't compress.

const SHARE_KEY = 'design=';
const b64u = bytes => { let s = ''; for (let i = 0; i < bytes.length; i += 0x8000) s += String.fromCharCode.apply(null, bytes.subarray(i, i + 0x8000)); return btoa(s).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, ''); };
const unb64u = t => { const s = atob(t.replace(/-/g, '+').replace(/_/g, '/') + '==='.slice((t.length + 3) % 4)); const b = new Uint8Array(s.length); for (let i = 0; i < s.length; i++) b[i] = s.charCodeAt(i); return b; };
async function pipeBytes(bytes, stream) { return new Uint8Array(await new Response(new Blob([bytes]).stream().pipeThrough(stream)).arrayBuffer()); }

async function designCode(name) {
  const json = JSON.stringify({ format: FILE_FORMAT, version: 1, name, design: await partDesignWithAssets(JSON.parse(designSnap())) });
  const raw = new TextEncoder().encode(json);
  if (typeof CompressionStream === 'function') return 'v1.' + b64u(await pipeBytes(raw, new CompressionStream('deflate-raw')));
  return 'j1.' + b64u(raw);
}
// A code, or a whole link with one in it: the design file it holds (name, design), or an error.
async function readDesignCode(text) {
  let t = String(text || '').trim(); const i = t.indexOf('#' + SHARE_KEY); if (i >= 0) t = t.slice(i + 1 + SHARE_KEY.length); else if (t.startsWith(SHARE_KEY)) t = t.slice(SHARE_KEY.length);
  t = decodeURIComponent(t).replace(/\s+/g, '');
  const m = /^(v1|j1)\.([A-Za-z0-9_-]+)$/.exec(t); if (!m) throw new Error('that isn\'t a design link or code');
  let bytes = unb64u(m[2]);
  if (m[1] === 'v1') { if (typeof DecompressionStream !== 'function') throw new Error('this browser can\'t unpack it (it\'s too old)'); bytes = await pipeBytes(bytes, new DecompressionStream('deflate-raw')); }
  return readDesignFile(new TextDecoder().decode(bytes));
}
async function openDesignCode(text, from) {
  const got = await readDesignCode(text), name = got.name || 'Shared design';
  // The locked shared-code operation can add its own drone from world view.
  if (typeof fleet !== 'undefined' && fleet.ready && !fleet.selected) {
    if (!fleetCreate('quadx',{design:got.design,name,designName:name},true)) return false;
    designNote(`Added “${name}” from ${from}. Save it to keep it in your designs.`);
    return true;
  }
  if (!await askToSave(name)) return false;
  applyDesign(got.design); designLoaded(null, name); afterLoad();
  designNote(`Opened “${name}” from ${from}. Save it to keep it in your designs.`);
  return true;
}
const shareBase = () => location.href.split('#')[0];

/* ───────── the dialog ───────── */
async function openShare(pasteOnly = false) {
  const dlg = $('#shareDlg'); dlg.classList.toggle('paste-only', !!pasteOnly);
  if (pasteOnly) { $('#shareOpenMsg').textContent = ''; $('#shareIn').value = ''; dlg.showModal(); $('#shareIn').focus(); return; }
  const name = (($('#designName').value || '').trim()) || designs.name || (designs.preset && PRESETS[designs.preset] ? PRESETS[designs.preset].label.replace(/\s*\(.*\)\s*$/, '') + ' (changed)' : 'Untitled design');
  setText($('#shareName'), name); $('#shareMsg').textContent = ''; $('#shareOpenMsg').textContent = ''; $('#shareIn').value = '';
  let code = '';
  try { code = await designCode(name); } catch (e) { $('#shareMsg').textContent = 'Couldn’t pack it: ' + e.message; }
  const link = shareBase() + '#' + SHARE_KEY + code;
  $('#shareLink').value = link; $('#shareLink').dataset.code = code;
  setText($('#shareSize'), `${link.length.toLocaleString()} characters${link.length > 4000 ? ' · long: some chat apps cut links this long, send the design file instead' : ''}`);
  $('#shareLocal').hidden = location.protocol !== 'file:';
  dlg.showModal(); $('#shareLink').select();
}
async function copyText(t, what) {
  let ok = false;
  try { await navigator.clipboard.writeText(t); ok = true; } catch (e) {
    const ta = UI.textarea( { style: 'position:fixed;opacity:0' }); ta.value = t; document.body.append(ta); ta.select(); try { ok = document.execCommand('copy'); } catch (x) {} ta.remove();
  }
  $('#shareMsg').textContent = ok ? `${what} copied.` : `Couldn’t copy: select it and copy it yourself.`; $('#shareMsg').className = 'hint ' + (ok ? 'good' : 'bad');
}
$('#designShare').addEventListener('click', () => openShare(false));
$('#shareCopy').addEventListener('click', () => copyText($('#shareLink').value, 'The link'));
$('#shareCopyCode').addEventListener('click', () => copyText($('#shareLink').dataset.code, 'The code'));
$('#shareClose').addEventListener('click', () => $('#shareDlg').close());
$('#shareOpen').addEventListener('click', async () => {
  const msg = $('#shareOpenMsg');
  try { msg.textContent = ''; $('#shareDlg').close(); await openDesignCode($('#shareIn').value, 'a shared code'); }
  catch (e) { $('#shareDlg').showModal(); msg.textContent = e.message; msg.className = 'hint bad'; }
});

// A link with a design in it: open it (asking first if the airframe on screen has unsaved changes), then tidy the address.
async function openSharedLink() {
  if (!location.hash.startsWith('#' + SHARE_KEY)) return;
  const h = location.hash; history.replaceState(null, '', location.pathname + location.search);
  try { await openDesignCode(h, 'a shared link'); } catch (e) { designNote('The link’s design couldn’t be opened: ' + e.message); }
}
window.addEventListener('hashchange', openSharedLink);
window.addEventListener('load', () => setTimeout(openSharedLink, 0));
