'use strict';
// LiftLab's look for the survey page: its tokens and components (../css/tokens.css, base.css, components.css; see
// docs/ui-design-system.md) style the page, and this marks the controls the page makes, now and later, with the
// component classes LiftLab's own UI.hydrate gives its controls (buttons as .btn.btn-sm, fields as .ui-control). Inside
// LiftLab's Room Survey dialog the page drops its own header and follows LiftLab's theme (its data-theme), and
// tone(name) reads a theme colour for the canvases, redrawn when the theme changes.
const tones=new Map();
function tone(name){
  if(!tones.has(name))tones.set(name,getComputedStyle(document.documentElement).getPropertyValue(name).trim()||'#888');
  return tones.get(name);
}
(()=>{
  const root=document.documentElement;
  const host=(()=>{try{return parent!==window&&parent.location.origin===location.origin?parent:null;}catch{return null;}})();
  const changed=()=>{tones.clear();if(typeof draw==='function')draw();if(typeof renderShown==='function')renderShown();};
  if(host){
    root.dataset.embedded='';
    const theirs=host.document.documentElement,sync=()=>{
      const theme=theirs.dataset.theme;
      if(theme)root.dataset.theme=theme;else delete root.dataset.theme;
      changed();
    };
    sync();new host.MutationObserver(sync).observe(theirs,{attributes:true,attributeFilter:['data-theme']});
  }
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change',changed);

  // Node cards and their eye toggles are drawn by the graph itself, not as buttons.
  const own=n=>n.closest('.node-card,.node-visible');
  const mark=n=>{
    if(n.tagName==='BUTTON'){if(!own(n))n.classList.add('btn','btn-sm','ui-button');}
    else if(/^(INPUT|SELECT|TEXTAREA)$/.test(n.tagName))n.classList.add('ui-control');
    else if(n.tagName==='DETAILS')n.classList.add('ui-disclosure');
  };
  const hydrate=node=>{
    if(node.nodeType!==1)return;
    mark(node);
    for(const n of node.querySelectorAll('button,input,select,textarea,details'))mark(n);
  };
  const start=()=>{
    hydrate(document.body);
    new MutationObserver(records=>{for(const r of records)for(const n of r.addedNodes)hydrate(n);})
      .observe(document.body,{childList:true,subtree:true});
  };
  if(document.body)start();else addEventListener('DOMContentLoaded',start);
})();
