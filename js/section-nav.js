'use strict';
// One navigation bar outside the panes: it survives tab and device rerenders.
function initSectionNavigation(rootId){
  const root=document.getElementById(rootId),tabs=root.querySelector('.tabs'),panes=[...root.querySelectorAll(':scope > [role="tabpanel"]')];
  const nav=el('nav',{class:'section-nav','aria-label':rootId==='airframe'?'Sections in this tab':'Sections in the readouts tab'}),menu=el('details',{class:'section-jump'}),summary=el('summary'),links=el('div',{class:'section-links'}),current=el('span',{class:'section-current'}),previous=el('a',{class:'section-neighbor section-previous'}),next=el('a',{class:'section-neighbor section-next'});
  summary.append(current);menu.append(summary,links);nav.append(previous,menu,next);tabs.after(nav);
  let sections=[],activePane=null,activeIndex=0,frame=0,nextId=0,refreshNeeded=true;
  const headingSelector='.sec > h2, .hw-group > h3, .ai-view h2, .ai-view h3';
  const title=h=>{const copy=h.cloneNode(true);copy.querySelectorAll('small,button,.srcs,.src-tag').forEach(n=>n.remove());return copy.textContent.trim();};
  const schedule=(refresh=false)=>{refreshNeeded||=refresh;if(!frame)frame=requestAnimationFrame(update);};
  const pageScrolling=()=>getComputedStyle(root).overflowY==='visible';
  const jump=s=>{
    if(!s)return;menu.open=false;
    const page=pageScrolling(),top=(page?0:root.getBoundingClientRect().top+root.clientTop)+tabs.offsetHeight+nav.offsetHeight;
    (page?window:root).scrollTo({top:(page?window.scrollY:root.scrollTop)+s.heading.getBoundingClientRect().top-top-10,behavior:matchMedia('(prefers-reduced-motion: reduce)').matches?'auto':'smooth'});
    s.heading.tabIndex=-1;s.heading.focus({preventScroll:true});schedule();
  };
  previous.addEventListener('click',e=>{e.preventDefault();jump(sections[activeIndex-1]);});
  next.addEventListener('click',e=>{e.preventDefault();jump(sections[activeIndex+1]);});
  function update(){
    frame=0;const pane=panes.find(p=>!p.hidden);if(!pane)return;
    if(refreshNeeded||pane!==activePane){
      refreshNeeded=false;activePane=pane;
      const heads=[...pane.querySelectorAll(headingSelector)].filter(h=>h.getClientRects().length);
      const changed=heads.length!==sections.length||heads.some((h,i)=>h!==sections[i]?.heading||title(h)!==sections[i]?.name);
      if(changed){
        sections=heads.map(h=>{if(!h.id)h.id=rootId+'-section-anchor-'+nextId++;return {heading:h,name:title(h)};});links.textContent='';
        sections.forEach(s=>{s.link=el('a',{href:'#'+s.heading.id,text:s.name,class:s.heading.tagName==='H3'?'section-sub':''});s.link.addEventListener('click',e=>{
          e.preventDefault();jump(s);
        });links.append(s.link);});
      }
    }
    nav.hidden=sections.length<2;if(nav.hidden)return;
    root.style.setProperty('--section-tabs-height',tabs.offsetHeight+'px');
    const top=nav.getBoundingClientRect().bottom+16;let index=0;
    sections.forEach((s,i)=>{if(s.heading.getBoundingClientRect().top<=top)index=i;});
    if(pageScrolling()?root.getBoundingClientRect().top<0&&root.getBoundingClientRect().bottom<=window.innerHeight+3:root.scrollTop>0&&root.scrollHeight-root.scrollTop-root.clientHeight<3)index=sections.length-1;
    activeIndex=index;current.textContent=sections[index].name;
    summary.title=sections[index].name+' · Show all sections';
    summary.setAttribute('aria-label',sections[index].name+' · Show all sections');
    for(const [a,s,dir] of [[previous,sections[index-1],'previous'],[next,sections[index+1],'next']]){
      a.style.visibility=s?'visible':'hidden';a.tabIndex=s?0:-1;
      if(s){a.href='#'+s.heading.id;a.textContent=dir==='previous'?'‹ '+s.name:s.name+' ›';a.title=s.name;a.setAttribute('aria-label',(dir==='previous'?'Previous':'Next')+' section: '+s.name);}
      else {a.removeAttribute('href');a.textContent='';a.removeAttribute('aria-label');a.removeAttribute('title');}
    }
    sections.forEach((s,i)=>{if(i===index)s.link.setAttribute('aria-current','location');else s.link.removeAttribute('aria-current');});
  }
  root.addEventListener('scroll',()=>schedule(),{passive:true});menu.addEventListener('toggle',()=>schedule());
  window.addEventListener('scroll',()=>{if(pageScrolling())schedule();},{passive:true});
  menu.addEventListener('keydown',e=>{if(e.key==='Escape'){menu.open=false;summary.focus();}});
  new ResizeObserver(()=>schedule(true)).observe(root);
  const hasHeading=n=>n.nodeType===1&&(n.matches(headingSelector)||n.querySelector(headingSelector));
  const observer=new MutationObserver(records=>{
    for(const r of records){
      if(r.type==='attributes'&&r.oldValue!==r.target.getAttribute(r.attributeName)){schedule(true);break;}
      if(r.type==='childList'&&(r.target.matches(headingSelector)||[...r.addedNodes,...r.removedNodes].some(hasHeading))){schedule(true);break;}
    }
  });panes.forEach(p=>observer.observe(p,{subtree:true,childList:true,attributes:true,attributeOldValue:true,attributeFilter:['hidden','open']}));
  schedule(true);
}
initSectionNavigation('airframe');
initSectionNavigation('telemetry');
