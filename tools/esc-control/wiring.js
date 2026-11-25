/* Declarative diagram renderer. Device facts, layouts and formulas live in JSON. */
(function(root,factory){const api=factory();if(typeof module==='object')module.exports=api;else root.SignalWiring=api;})(globalThis,()=>{
'use strict';
const C=typeof module==='object'?require('./configuration.js'):globalThis.SignalConfig;
const clone=value=>JSON.parse(JSON.stringify(value));
const esc=s=>String(s??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const number=x=>Number.isFinite(x)?Number(x.toFixed(3)).toLocaleString('en-US',{maximumFractionDigits:3}):String(x);
const TAGS=new Set(['g','rect','circle','text','polyline','path','line']);
const ATTRS=new Set(['x','y','x1','y1','x2','y2','width','height','rx','ry','r','cx','cy','points','d','fill','stroke','stroke-width','text-anchor','class','opacity','transform','stroke-dasharray']);
const OPS=new Set(['add','subtract','multiply','divide','min','max','gt','gte','lt','lte','eq','and','or','not','if']);
const validPath=p=>typeof p==='string'&&/^[A-Za-z0-9_]+(?:\.[A-Za-z0-9_]+)*$/.test(p)&&!p.split('.').some(k=>['__proto__','constructor','prototype'].includes(k));
function read(obj,path){if(!validPath(path))throw Error('Invalid binding: '+path);for(const key of path.split('.')){if(obj==null||!Object.hasOwn(obj,key))throw Error('Missing binding: '+path);obj=obj[key];}return obj;}
function expression(expr,resolve,depth=0){
 if(depth>32)throw Error('Calculation nesting is too deep.');
 if(expr===null||['number','boolean','string'].includes(typeof expr))return expr;
 if(!Array.isArray(expr)&&expr&&Object.keys(expr).length===1&&validPath(expr.ref))return resolve(expr.ref);
 if(!Array.isArray(expr)||!OPS.has(expr[0]))throw Error('Unknown calculation operation.');
 const [op,...args]=expr,at=i=>expression(args[i],resolve,depth+1);const arity={subtract:2,divide:2,gt:2,gte:2,lt:2,lte:2,eq:2,not:1,if:3};if(arity[op]!==undefined&&args.length!==arity[op]||args.length<1)throw Error('Wrong calculation argument count: '+op);
 if(op==='if')return at(0)?at(1):at(2);if(op==='and')return args.every((_,i)=>!!at(i));if(op==='or')return args.some((_,i)=>!!at(i));if(op==='not')return !at(0);
 const values=args.map((_,i)=>at(i));if(op==='eq')return values[0]===values[1];
 if(!values.length||!values.every(v=>typeof v==='number'&&Number.isFinite(v)))throw Error('Calculations need finite numeric values.');
 const [a,b]=values;let value;
 switch(op){case'add':value=values.reduce((x,y)=>x+y,0);break;case'subtract':value=a-b;break;case'multiply':value=values.reduce((x,y)=>x*y,1);break;case'divide':if(b===0)throw Error('Cannot divide by zero.');value=a/b;break;case'min':value=Math.min(...values);break;case'max':value=Math.max(...values);break;case'gt':return a>b;case'gte':return a>=b;case'lt':return a<b;case'lte':return a<=b;}
 if(!Number.isFinite(value))throw Error('Calculation result is not finite.');return value;
}
function validateExpression(expr,depth=0){
 if(depth>32)throw Error('Calculation nesting is too deep.');
 if(expr===null||['boolean','string'].includes(typeof expr)||typeof expr==='number'&&Number.isFinite(expr))return;
 if(!Array.isArray(expr)&&expr&&Object.keys(expr).length===1&&validPath(expr.ref))return;
 if(!Array.isArray(expr)||!OPS.has(expr[0])||expr.length<2||expr.length>20)throw Error('Use supported declarative calculations, not executable code.');
 const arity={subtract:2,divide:2,gt:2,gte:2,lt:2,lte:2,eq:2,not:1,if:3};if(arity[expr[0]]!==undefined&&expr.length-1!==arity[expr[0]])throw Error('Wrong calculation argument count.');expr.slice(1).forEach(e=>validateExpression(e,depth+1));
}
function validateElements(elements,depth=0,budget={n:0}){
 if(!Array.isArray(elements)||depth>12)throw Error('Invalid drawing elements.');
 for(const el of elements){if(++budget.n>1500||!el||!TAGS.has(el.tag)||!el.attrs||typeof el.attrs!=='object'||Array.isArray(el.attrs))throw Error('Drawing uses unsupported SVG elements.');
  for(const [key,value]of Object.entries(el.attrs))if(!ATTRS.has(key)||!['string','number'].includes(typeof value)||typeof value==='string'&&/url\s*\(|javascript:|data:/i.test(value)||typeof value==='number'&&!Number.isFinite(value))throw Error('Unsupported drawing attribute: '+key);
  if(el.text!==undefined&&typeof el.text!=='string')throw Error('Drawing labels must be text.');if(el.children)validateElements(el.children,depth+1,budget);
 }
}
function validate(w){
 if(!w||!w.board||!Array.isArray(w.diagrams)||w.diagrams.length>32)throw Error('Wiring configuration needs a board and at most 32 diagrams.');
 const b=w.board;if(!Array.isArray(b.pins)||b.pins.length>100||!b.drawing||!Array.isArray(b.colors)||!b.colors.length||!b.pinStyle)throw Error('Invalid board profile.');
 if(![b.pinStyle.radius,b.pinStyle.focusRadius].every(v=>Number.isFinite(v)&&v>0&&v<=100)||![...b.colors,...['inactive','reserved','power','background'].map(k=>b.pinStyle[k])].every(v=>typeof v==='string'&&/^#[0-9a-f]{6}$/i.test(v)))throw Error('Invalid board pin presentation.');
 const sources=value=>{if(value!==undefined&&(!Array.isArray(value)||value.some(s=>!s||typeof s.title!=='string'||typeof s.url!=='string'||!/^https?:\/\//.test(s.url))))throw Error('Sources must contain HTTP(S) links and titles.');};sources(b.sources);for(const key of ['legend','notes'])if(b[key]!==undefined&&(!Array.isArray(b[key])||b[key].some(v=>typeof v!=='string')))throw Error('Board notes and legend must contain text.');
 validateElements(b.drawing.elements);if(!Array.isArray(b.drawing.viewBox)||b.drawing.viewBox.length!==4||!b.drawing.viewBox.every(Number.isFinite))throw Error('Invalid board view box.');
 for(const pin of b.pins)if(![pin.x,pin.y,pin.labelX].every(Number.isFinite)||typeof pin.label!=='string'||pin.gpio!==null&&!Number.isInteger(pin.gpio))throw Error('Invalid board pin.');
 if(!b.signal||![b.signal.min,b.signal.max].every(Number.isFinite)||b.signal.min>=b.signal.max)throw Error('Board signal voltage limits are invalid.');
 const ids=new Set();
 for(const d of w.diagrams){if(!d||typeof d.id!=='string'||!d.id||ids.has(d.id)||typeof d.title!=='string'||typeof d.label!=='string')throw Error('Diagrams need unique IDs and labels.');ids.add(d.id);
  sources(d.sources);if(d.channels!==undefined&&(!Array.isArray(d.channels)||d.channels.some(v=>!Number.isInteger(v)||v<0)))throw Error('Diagram channels must be channel indices.');
  if(!Array.isArray(d.directions)||d.directions.some(v=>!['in','out','analog'].includes(v)))throw Error('Choose diagram channel directions.');
  if(!Array.isArray(d.viewBox)||d.viewBox.length!==4||!d.viewBox.every(Number.isFinite)||d.viewBox[2]<=0||d.viewBox[3]<=0)throw Error('Invalid diagram view box.');validateElements(d.elements);
  const paramIds=new Set();if(!Array.isArray(d.parameters)||d.parameters.length>24)throw Error('At most 24 diagram parameters.');
  for(const p of d.parameters){if(!/^[A-Za-z_][A-Za-z0-9_]*$/.test(p.id)||!validPath(p.id)||paramIds.has(p.id)||!['number','text','channel'].includes(p.type))throw Error('Invalid diagram parameter.');paramIds.add(p.id);
   if(p.type==='number'&&(!Number.isFinite(p.value)||p.min!==undefined&&!Number.isFinite(p.min)||p.max!==undefined&&!Number.isFinite(p.max)||p.min!==undefined&&p.value<p.min||p.max!==undefined&&p.value>p.max))throw Error('Parameter value is outside its limits: '+p.id);
   if(p.type==='channel'&&!['in','out','analog'].includes(p.direction))throw Error('Channel parameters need a direction.');
  }
  if(!d.calculations||Object.keys(d.calculations).length>64)throw Error('Invalid diagram calculations.');for(const [key,value]of Object.entries(d.calculations)){if(!validPath(key)||key.includes('.'))throw Error('Invalid calculation name.');validateExpression(value);}
  if(!Array.isArray(d.limits)||!Array.isArray(d.components)||!Array.isArray(d.notes))throw Error('Diagrams need limits, components and notes arrays.');
  for(const limit of d.limits){if(typeof limit.text!=='string'||limit.severity!==undefined&&!['info','warning','danger'].includes(limit.severity))throw Error('Invalid voltage-limit label.');if(limit.when!==undefined)validateExpression(limit.when);}
  if([...d.components,...d.notes].some(t=>typeof t!=='string'))throw Error('Components and notes must be text.');
 }
 for(const id of Object.values(w.defaults||{}))if(id&&!ids.has(id))throw Error('Default diagram does not exist: '+id);
 return w;
}
function available(p,ch){return p.wiring.diagrams.filter(d=>d.directions.includes(ch.direction)&&(!d.channels||d.channels.includes(p.channels.indexOf(ch))));}
function defaultDiagram(p,ch){const list=available(p,ch);return list.find(d=>d.id===p.wiring.defaults?.[ch.direction])||list[0];}
function newWiring(p){
 const diagrams=clone(C.templates.filter(d=>d.includeInNew&&p.channels.some(ch=>d.directions.includes(ch.direction))));
 return {schemaVersion:1,board:clone(C.board),powerTitle:C.board.powerTitle,powerNote:C.board.powerNote,defaults:Object.fromEntries(['in','out','analog'].map(role=>[role,diagrams.find(d=>d.preferred&&d.directions.includes(role))?.id||''])),diagrams};
}
function context(p,index,d,values={}){
 const ch=p.channels[index],b=p.wiring.board,channel={...ch,index,pinLabel:'GPIO'+ch.pin},adcRange=b.adcRanges?.[ch.attenuation],ctx={board:b,channel,adc:{min:adcRange?.[0],max:adcRange?.[1]},params:{},calc:Object.create(null)};
 for(const param of d.parameters){let value=Object.hasOwn(values,param.id)?values[param.id]:param.value;
  if(param.type==='number'){value=Number(value);if(!Number.isFinite(value)||param.min!==undefined&&value<param.min||param.max!==undefined&&value>param.max)throw Error(param.label+': enter '+(param.min??'a finite value')+(param.max!==undefined?'–'+param.max:''));}
  if(param.type==='channel'){const candidates=p.channels.map((c,i)=>({...c,index:i,pinLabel:'GPIO'+c.pin})).filter(c=>c.direction===param.direction);value=candidates.find(c=>c.index===Number(value))||candidates[0];if(!value)throw Error('Configure a '+param.direction+' channel for '+param.label+'.');}
  ctx.params[param.id]=value;
 }
 const computing=new Set(),resolve=path=>{if(path.startsWith('calc.')){const key=path.slice(5);if(!Object.hasOwn(ctx.calc,key)){if(computing.has(key))throw Error('Calculation cycle: '+key);if(!Object.hasOwn(d.calculations,key))throw Error('Unknown calculation: '+key);computing.add(key);ctx.calc[key]=expression(d.calculations[key],resolve);computing.delete(key);}return ctx.calc[key];}return read(ctx,path);};
 for(const key of Object.keys(d.calculations))resolve('calc.'+key);ctx.resolve=resolve;return ctx;
}
function template(value,ctx){return String(value??'').replace(/\{\{\s*([A-Za-z0-9_.]+)\s*\}\}/g,(_,path)=>{const v=ctx.resolve?ctx.resolve(path):read(ctx,path);if(v===undefined||v===null)throw Error('Missing binding: '+path);return typeof v==='number'?number(v):String(v);});}
function elements(nodes,ctx){return nodes.map(el=>`<${el.tag}${Object.entries(el.attrs).map(([k,v])=>` ${k}="${esc(template(v,ctx))}"`).join('')}>${el.text!==undefined?esc(template(el.text,ctx)):''}${el.children?elements(el.children,ctx):''}</${el.tag}>`).join('');}
function svg(d,ctx){return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="${d.viewBox.join(' ')}" role="img" aria-label="${esc(template(d.title,ctx))}"><title>${esc(template(d.title,ctx))}</title>${elements(d.elements,ctx)}</svg>`;}
function board(p,selected){
 const b=p.wiring.board,s=b.pinStyle,ctx={board:b};let body=elements(b.drawing.elements,ctx);
 for(const pin of b.pins){const i=p.channels.findIndex(ch=>ch.pin===pin.gpio),active=i>=0,focus=active&&i===selected,color=active?b.colors[i%b.colors.length]:pin.power?s.power:pin.reserved?s.reserved:s.inactive;
  body+=`<g><title>${esc(pin.gpio===null?pin.id:'GPIO'+pin.gpio)}${active?' · CH'+(i+1)+' '+esc(p.channels[i].name):''}</title>${focus?`<circle cx="${pin.x}" cy="${pin.y}" r="${s.focusRadius}" fill="none" stroke="${esc(color)}" stroke-width="2"/>`:''}<circle cx="${pin.x}" cy="${pin.y}" r="${s.radius}" fill="${esc(active?color:s.background)}" stroke="${esc(color)}" stroke-width="2"/><text x="${pin.labelX}" y="${pin.y+4}" fill="${esc(color)}" class="board-pin${active?' active-pin':''}" text-anchor="${esc(pin.anchor)}">${esc(pin.label)}</text></g>`;
 }
 return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="${b.drawing.viewBox.join(' ')}" role="img" aria-label="${esc(b.title)}"><title>${esc(b.title)}</title>${body}</svg>`;
}
function circuit(p,index,id,values={}){
 const d=p.wiring.diagrams.find(d=>d.id===id);if(!d||!available(p,p.channels[index]).includes(d))throw Error('Choose a configured diagram for this channel.');
 const ctx=context(p,index,d,values),expand=t=>template(t,ctx);
 return {title:expand(d.title),svg:svg({...d,title:d.title},ctx),limits:d.limits.filter(l=>l.when===undefined||expression(l.when,ctx.resolve)).map(l=>({...l,text:expand(l.text)})),parts:d.components.map(expand),notes:d.notes.map(expand),calculations:ctx.calc};
}
function mount({getPreset,dialog,boardHost,contentHost,channelSelect,modeSelect,settingsHost,titleHost}){
 let selected=0,id='',values={};
 function update(renderSettings=true){
  const p=getPreset(),w=p.wiring,d=w.diagrams.find(d=>d.id===id);titleHost.textContent=p.name;boardHost.innerHTML=board(p,selected);
  const q=id=>dialog.querySelector('#'+id);q('wiringBoardCaption').textContent=w.board.description||'';q('wiringPowerTitle').textContent=w.powerTitle||'';q('wiringPowerNote').textContent=w.powerNote||'';
  q('wiringBoardLegend').innerHTML=(w.board.legend||[]).map(t=>`<span>${esc(t)}</span>`).join('');
  q('wiringReferences').innerHTML=(w.board.notes||[]).map(t=>`<p>${esc(t)}</p>`).join('')+`<p>${[...(w.board.sources||[]),...(d?.sources||[])].filter(s=>/^https?:\/\//.test(s.url)).map(s=>`<a href="${esc(s.url)}" target="_blank" rel="noopener">${esc(s.title)}</a>`).join(' · ')}</p>`;
  if(!d){settingsHost.innerHTML='';contentHost.innerHTML='<div class="notice">No diagram configured for this channel. Add one in Edit wiring.</div>';return;}
  if(renderSettings)settingsHost.innerHTML=`<div class="wiring-divider-fields">${d.parameters.map(param=>`<label>${esc(param.label)}${param.type==='channel'?`<select data-diagram-param="${esc(param.id)}">${p.channels.flatMap((c,i)=>c.direction===param.direction?[`<option value="${i}" ${Number(values[param.id]??param.value)===i?'selected':''}>CH${i+1} · GPIO${c.pin} · ${esc(c.name)}</option>`]:[]).join('')}</select>`:`<input data-diagram-param="${esc(param.id)}" type="${param.type==='number'?'number':'text'}" ${param.min!==undefined?'min="'+param.min+'"':''} ${param.max!==undefined?'max="'+param.max+'"':''} step="any" value="${esc(values[param.id]??param.value??'')}">`}</label>`).join('')}</div>`;
  try{const data=circuit(p,selected,id,values);contentHost.innerHTML=`<h3>${esc(data.title)}</h3><div class="wiring-limits">${data.limits.map(l=>`<span class="${l.severity==='danger'?'unsafe':''}">${esc(l.text)}</span>`).join('')}</div><div class="circuit-scroll">${data.svg}</div><div class="wiring-details"><div><h4>Components & power</h4><ul>${data.parts.map(v=>`<li>${esc(v)}</li>`).join('')}</ul></div><div><h4>Connection notes</h4><ul>${data.notes.map(v=>`<li>${esc(v)}</li>`).join('')}</ul></div></div>`;}catch(e){contentHost.innerHTML=`<div class="notice error" role="alert">${esc(e.message)}</div>`;}
  settingsHost.querySelectorAll('[data-diagram-param]').forEach(input=>{const param=d.parameters.find(p=>p.id===input.dataset.diagramParam);const apply=()=>{values[param.id]=param.type==='text'?input.value:Number(input.value);update(false);};input.oninput=input.onchange=apply;});
 }
 function select(){const p=getPreset(),list=available(p,p.channels[selected]);id=defaultDiagram(p,p.channels[selected])?.id||'';values={};modeSelect.innerHTML=list.map(d=>`<option value="${esc(d.id)}">${esc(d.label)}</option>`).join('');modeSelect.value=id;update();}
 channelSelect.onchange=()=>{selected=Number(channelSelect.value);select();};modeSelect.onchange=()=>{id=modeSelect.value;values={};update();};
 return {selection(){return id;},open(){const p=getPreset();validate(p.wiring);selected=0;channelSelect.innerHTML=p.channels.map((c,i)=>`<option value="${i}">CH${i+1} · GPIO${c.pin} · ${esc(c.name)} · ${esc(c.direction)}</option>`).join('');select();dialog.showModal();}};
}
return {validate,validateExpression,available,defaultDiagram,newWiring,expression,context,template,board,circuit,mount};
});
