'use strict';
// Local parser + sanitizer; cached per feed item so tool updates do not reparse old replies.
const ChatMarkdown = (() => {
  const cache = new WeakMap();
  const escape = text => String(text).replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const renderer = new marked.Renderer();
  renderer.html = token => escape(token.text); // raw HTML is shown as text
  renderer.image = token => `<a href="${escape(token.href)}">${escape(token.text || 'Image')}</a>`;
  function render(text) {
    const box = document.createElement('div'); box.className = 'ai-markdown';
    try {
      const html = marked.parse(String(text ?? ''), {gfm:true, async:false, renderer});
      const fragment = DOMPurify.sanitize(html, {
        RETURN_DOM_FRAGMENT:true,
        ALLOWED_TAGS:['p','br','hr','strong','em','del','ul','ol','li','h1','h2','h3','h4','h5','h6','blockquote','pre','code','a','table','thead','tbody','tr','th','td','input'],
        ALLOWED_ATTR:['href','title','start','align','class','type','checked','disabled'],
        ALLOW_DATA_ATTR:false, ALLOW_ARIA_ATTR:false,
      });
      for (const a of fragment.querySelectorAll('a')) {
        const href = a.getAttribute('href');
        if (!href) continue;
        let url; try { url = new URL(href, document.baseURI); } catch (_) {}
        if (!url || !['http:','https:','mailto:'].includes(url.protocol)) { a.removeAttribute('href'); continue; }
        if (url.protocol !== 'mailto:') { a.target = '_blank'; a.rel = 'noopener noreferrer'; }
      }
      for (const input of fragment.querySelectorAll('input')) { input.type = 'checkbox'; input.disabled = true; }
      for (const table of fragment.querySelectorAll('table')) {
        const wrap = document.createElement('div'); wrap.className = 'ai-table'; wrap.tabIndex = 0;
        wrap.setAttribute('role','region'); wrap.setAttribute('aria-label','Message table');
        table.replaceWith(wrap); wrap.append(table);
      }
      box.append(fragment);
    } catch (_) { box.textContent = String(text ?? ''); }
    return box;
  }
  function message(item) {
    const text = String(item.text ?? ''); let entry = cache.get(item);
    if (!entry || entry.text !== text) { entry = {text, node:render(text)}; cache.set(item, entry); }
    return entry.node.cloneNode(true);
  }
  return {render, message};
})();
