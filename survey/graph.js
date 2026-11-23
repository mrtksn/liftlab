'use strict';
// Processing pipeline. Operators, their controls and presets are data from the
// server; this file draws and runs whatever graph a preset describes.
//
// Node: {id, op, label?, inputs, settings, controls?, program?, enabled?}
//   inputs   {port: {node, port}} for operators, [{node, port}] for viewers
//   settings {key: value | {var: presetControlKey}}
//   controls overrides of the operator's controls by key (label, default, min,
//            max, options, help, hidden), or extra controls for code and viewer nodes
//   program  JavaScript run in a sandbox; on an operator node it can call
//            runNative(settings) and returns the node's first output
// Code nodes also carry ports [{name, type}] and output (type).
// Preset: {name, description, controls: [...], values: {}, graph: {nodes, selected, viewer}}

let catalog={operators:[],types:{},compatible:{},depth_methods:[]};
const operatorsById=new Map();
const catalogReady=fetch(API+'/operators').then(r=>r.json()).then(c=>{
  catalog=c;
  for(const op of c.operators)operatorsById.set(op.id,op);
  const frames=[['photo','Recorded photo'],...c.depth_methods.map(m=>[m.id,m.label])];
  for(const id of ['frameMethod','rightSource'])
    $(id).replaceChildren(...frames.filter(([v])=>id!=='rightSource'||v!=='photo').map(([v,t])=>new Option(t,v)));
});

let preset=null, graph=null, pipelineRunning=false, runTicket=0;
const undoStack=[];
const results=new Map(), materialized=new WeakMap();
const nodeById=id=>graph?.nodes.find(n=>n.id===id);
const currentNode=()=>nodeById(graph?.selected)||graph?.nodes[0];
const opOf=node=>operatorsById.get(node?.op);
const viewerNode=()=>nodeById(graph?.viewer)||graph?.nodes.find(n=>n.op==='viewer');
function nodeLabel(node){
  if(!node)return 'disconnected';
  const name=node.label||opOf(node)?.label||node.op;
  const peers=graph.nodes.filter(n=>(n.label||opOf(n)?.label||n.op)===name);
  return name+(peers.length>1?' '+(peers.indexOf(node)+1):'');
}
function inputPorts(node){
  if(node.op==='code')return (node.ports||[]).map(p=>({name:p.name,types:[p.type]}));
  if(node.op==='viewer')return [];
  return opOf(node)?.inputs||[];
}
function outputPorts(node){
  if(node.op==='code')return [{name:'value',types:[node.output]}];
  return opOf(node)?.outputs||[];
}
function connections(node){
  if(node.op==='viewer')return (node.inputs||[]).map((link,i)=>({name:String(i),link}));
  return inputPorts(node).map(p=>({name:p.name,link:node.inputs?.[p.name],port:p})).filter(c=>c.link);
}
function outputType(link){return outputPorts(nodeById(link?.node)||{}).find(p=>p.name===link.port)?.types[0];}
function accepts(types,type){return types.includes(type)||types.some(t=>(catalog.compatible[t]||[]).includes(type));}
function dependsOn(node,id,seen=new Set()){
  if(!node||seen.has(node.id))return false;
  if(node.id===id)return true;seen.add(node.id);
  return connections(node).some(c=>dependsOn(nodeById(c.link.node),id,seen));
}

// Controls ---------------------------------------------------------------------------------------
function controlsOf(node){
  const base=opOf(node)?.params||[], overrides=new Map((node.controls||[]).map(c=>[c.key,c]));
  const merged=base.map(p=>({...p,...overrides.get(p.key)}));
  if(['code','viewer'].includes(node.op))for(const c of node.controls||[])if(!base.some(p=>p.key===c.key))merged.push(c);
  return merged;
}
function presetControls(){
  return (preset?.controls||[]).map(c=>{
    if(!c.options_from)return c;
    const source=operatorsById.get(c.options_from.op)?.params.find(p=>p.key===c.options_from.param);
    return {...c,options:source?.options||[],default:c.default??source?.default};
  });
}
function presetValue(key){
  const control=presetControls().find(c=>c.key===key);
  return preset?.values&&Object.hasOwn(preset.values,key)?preset.values[key]:control?.default;
}
const isBinding=v=>!!v&&typeof v==='object'&&!Array.isArray(v)&&typeof v.var==='string';
function resolveSettings(node){
  const values={};
  for(const c of controlsOf(node)){
    const raw=node.settings?.[c.key];
    const v=isBinding(raw)?presetValue(raw.var):raw;
    values[c.key]=v===undefined?c.default:v;
  }
  return values;
}
const visible=(c,values)=>!c.hidden&&(!c.when||c.when.values.includes(values[c.when.param]));
const labelOf=(c,values)=>(c.label_by?.labels?.[values[c.label_by.param]]||c.label)+(c.unit?' ('+c.unit+')':'');

function controlInput(c,value,onChange){
  let input;
  if(c.kind==='choice'){
    input=document.createElement('select');
    input.replaceChildren(...(c.options||[]).map(([v,t])=>new Option(t,v)));
    input.value=value;input.onchange=()=>onChange(input.value);
  }else if(c.kind==='bool'){
    input=document.createElement('input');input.type='checkbox';input.checked=!!value;input.onchange=()=>onChange(input.checked);
  }else if(c.kind==='text'){
    input=document.createElement('input');input.type='text';input.value=value??'';input.required=true;input.style.width='14em';
    if(c.pattern)input.pattern=c.pattern;
    input.onchange=()=>{if(input.reportValidity())onChange(input.value);};
  }else if(c.kind==='button'){
    input=document.createElement('button');input.type='button';input.textContent=c.button||'Run';input.onclick=()=>onChange();
  }else{
    input=document.createElement('input');
    Object.assign(input,{type:'number',step:c.kind==='int'?1:'any',required:true,value});
    if(c.min!=null)input.min=c.min;if(c.max!=null)input.max=c.max;
    input.onchange=()=>{if(input.reportValidity()&&Number.isFinite(input.valueAsNumber))onChange(input.valueAsNumber);};
  }
  return input;
}
function settingRow(c,values,input,note){
  const row=document.createElement('label'),name=document.createElement('span'),small=document.createElement('small');
  row.className='setting';row.title=c.help||'';name.textContent=labelOf(c,values);small.textContent=note||'';
  row.append(name,input,small);return row;
}

function renderPresetControls(){
  const host=$('presetControls'),list=presetControls();
  host.replaceChildren();host.hidden=!list.length;
  if(!list.length)return;
  const heading=document.createElement('strong'),grid=document.createElement('div');
  heading.textContent='Preset controls';grid.className='settings-grid';
  const values=Object.fromEntries(list.map(c=>[c.key,presetValue(c.key)]));
  for(const c of list){
    const users=graph.nodes.filter(n=>Object.values(n.settings||{}).some(v=>isBinding(v)&&v.var===c.key)).map(nodeLabel);
    const input=controlInput(c,values[c.key],value=>{
      if(c.kind==='button'){const target=nodeById(c.run);if(target){selectNode(target);runPipeline();}return;}
      remember();preset.values={...preset.values,[c.key]:value};edited();renderPresetControls();inspectNode(currentNode());
    });
    grid.append(settingRow(c,values,input,c.kind==='button'?'Runs to '+nodeLabel(nodeById(c.run)):users.length?'Sets '+users.join(', '):'Not used by any node'));
  }
  host.append(heading,grid);
}

// Node settings in the inspector: each control can be set here or follow a preset control.
function renderNodeSettings(node){
  const host=$('nodeSettings'),list=controlsOf(node),values=resolveSettings(node);
  host.replaceChildren();
  if(opOf(node)?.walk_params?.length)return renderWalkSettings(node,host);
  const shown=list.filter(c=>visible(c,values));
  host.hidden=!shown.length;
  if(!shown.length)return;
  const groups=new Map();
  for(const c of shown){const g=c.group||'Settings';if(!groups.has(g))groups.set(g,[]);groups.get(g).push(c);}
  for(const [title,controls] of groups){
    const heading=document.createElement('strong'),grid=document.createElement('div');
    heading.textContent=title;grid.className='settings-grid';
    for(const c of controls){
      const raw=node.settings?.[c.key],bound=isBinding(raw);
      const input=controlInput(c,values[c.key],value=>{remember();(node.settings??={})[c.key]=value;edited();inspectNode(node);});
      input.disabled=bound;
      const link=document.createElement('select');link.className='binding';link.title='Where this value comes from';
      link.add(new Option('Set here',''));
      for(const p of presetControls().filter(p=>p.kind===c.kind&&p.kind!=='button'))link.add(new Option('Preset: '+p.label,p.key));
      link.add(new Option('New preset control','+'));
      link.value=bound?raw.var:'';
      link.classList.toggle('bound',bound);
      link.onchange=()=>{
        remember();
        if(link.value==='+'){
          let key=c.key;for(let i=2;presetControls().some(p=>p.key===key);i++)key=c.key+i;
          const {group,when,label_by,...definition}=c;
          preset.controls=[...(preset.controls||[]),{...definition,key,label:labelOf(c,values),default:values[c.key]}];
          node.settings={...node.settings,[c.key]:{var:key}};
        }else if(link.value)node.settings={...node.settings,[c.key]:{var:link.value}};
        else node.settings={...node.settings,[c.key]:values[c.key]};
        edited();renderPresetControls();inspectNode(node);
      };
      const note=c.kind!=='bool'&&values[c.key]!==c.default?'default '+c.default:'';
      const row=settingRow(c,values,input,note);row.append(link);grid.append(row);
    }
    host.append(heading,grid);
  }
}
// Sensor settings belong to the walk, not to the pipeline.
async function renderWalkSettings(node,host){
  const run=state?.run,schema=opOf(node).walk_params;
  if(!run||state.recording){host.hidden=true;return;}
  const response=await fetch(API+'/sensor-settings?'+new URLSearchParams({run}));
  if(!response.ok||currentNode()!==node)return;
  const values=await response.json(),grid=document.createElement('div'),heading=document.createElement('strong'),bar=document.createElement('div');
  heading.textContent='Sensor settings saved with this walk';grid.className='settings-grid';bar.className='controls';
  for(const c of schema)grid.append(settingRow(c,values,controlInput(c,values[c.key],v=>{values[c.key]=v;}),c.help?'':''));
  const save=document.createElement('button'),asDefault=document.createElement('label'),toggle=document.createElement('input'),status=document.createElement('small');
  save.type='button';save.textContent='Save for this walk';toggle.type='checkbox';asDefault.append(toggle,' Also use for new walks');
  save.onclick=async()=>{
    status.textContent='Saving…';
    try{
      const r=await fetch(API+'/sensor-settings',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({run,values,default:toggle.checked})});
      const v=await r.json();if(!r.ok)throw Error(v.error);
      recordedTablesRequest=null;results.clear();
      const t=await fetch(API+'/timeline?'+new URLSearchParams({run}));
      if(t.ok&&timelineRun===run){timeline=await t.json();refreshReplay();}
      renderGraph();status.textContent='Saved. Rerun nodes that read ToF or IMU pairing.';
    }catch(e){status.textContent=e.message;}
  };
  bar.append(save,asDefault,status);host.replaceChildren(heading,grid,bar);host.hidden=false;
}

// Graph editing ----------------------------------------------------------------------------------
function remember(){undoStack.push(structuredClone(preset));if(undoStack.length>60)undoStack.shift();$('undoGraph').disabled=false;}
function edited(){
  $('runTrace').hidden=true;
  $('pipelinePreset').value='';$('presetStatus').textContent='Edited';
  if(state?.run)localStorage.removeItem('room-survey-active-preset-'+state.run);
  renderGraph();
}
$('undoGraph').onclick=()=>{
  if(!undoStack.length)return;
  preset=undoStack.pop();graph=preset.graph;$('undoGraph').disabled=!undoStack.length;
  edited();renderPresetControls();inspectNode(currentNode());draw();
};
document.addEventListener('keydown',e=>{
  if(!(e.metaKey||e.ctrlKey)||e.shiftKey||e.key.toLowerCase()!=='z'||!undoStack.length||e.target.closest('input,textarea,select,[contenteditable=true]'))return;
  e.preventDefault();$('undoGraph').onclick();
});

// Nodes that can follow the selected node: any operator with an input its outputs satisfy.
function candidates(source){
  const offered=outputPorts(source).map(p=>p.types[0]);
  const list=[];
  for(const op of operatorsById.values()){
    if(op.id==='walk')continue;
    if(['code','viewer'].includes(op.id)){if(offered.length||op.id==='viewer')list.push(op);continue;}
    if(op.inputs.some(p=>offered.some(t=>accepts(p.types,t))))list.push(op);
  }
  return list;
}
function nearestOutput(types,from){
  // Prefer the selected node, then its ancestors, then anything else in the graph.
  const order=[from,...graph.nodes.filter(n=>dependsOn(from,n.id)&&n!==from).reverse(),...graph.nodes];
  for(const n of order)for(const p of outputPorts(n))if(accepts(types,p.types[0]))return {node:n.id,port:p.name};
  return null;
}
function addNode(opId){
  const parent=currentNode(),op=operatorsById.get(opId);
  if(!op)return;
  remember();
  const node={id:crypto.randomUUID().slice(0,8),op:op.id,inputs:{},settings:{}};
  if(op.id==='viewer'){node.inputs=outputPorts(parent).map(p=>({node:parent.id,port:p.name}));}
  else if(op.id==='code'){
    const first=outputPorts(parent)[0];
    node.ports=first?[{name:'input',type:first.types[0]}]:[];node.output=first?.types[0]||op.default_type;
    if(first)node.inputs.input={node:parent.id,port:first.name};
    node.program='return input;';
  }else for(const p of op.inputs){const link=nearestOutput(p.types,parent);if(link)node.inputs[p.name]=link;}
  graph.nodes.push(node);
  // Show the new node's first output in the viewer.
  const view=viewerNode(),first=outputPorts(node)[0];
  if(view&&first&&op.id!=='viewer')view.inputs.push({node:node.id,port:first.name});
  if(op.id==='viewer')graph.viewer=node.id;
  edited();selectNode(node);
}
$('addNode').onclick=()=>addNode($('nodeType').value);
$('removeNode').onclick=()=>{
  const node=currentNode();
  if(!node||node.op==='walk')return;
  remember();
  graph.nodes=graph.nodes.filter(n=>n!==node);
  for(const n of graph.nodes){
    if(n.op==='viewer')n.inputs=n.inputs.filter(l=>l.node!==node.id);
    else for(const [k,l] of Object.entries(n.inputs||{}))if(l.node===node.id)delete n.inputs[k];
  }
  if(graph.viewer===node.id)graph.viewer=graph.nodes.find(n=>n.op==='viewer')?.id;
  results.delete(node.id);edited();selectNode(graph.nodes[0]);draw();
};
function selectNode(node){graph.selected=node.id;inspectNode(node);}

// Board -----------------------------------------------------------------------------------------
function renderGraph(){
  if(!graph)return;
  const depth=new Map();
  const level=n=>{
    if(depth.has(n.id))return depth.get(n.id);
    depth.set(n.id,0);
    const d=n.op==='viewer'?0:Math.max(-1,...connections(n).map(c=>nodeById(c.link.node)).filter(Boolean).map(level))+1;
    depth.set(n.id,d);return d;
  };
  for(const n of graph.nodes)level(n);
  const deepest=Math.max(0,...graph.nodes.filter(n=>n.op!=='viewer').map(n=>depth.get(n.id)));
  for(const n of graph.nodes)if(n.op==='viewer')depth.set(n.id,deepest+1);
  const rows=[],positions=new Map(),cards=document.createDocumentFragment(),wires=document.createDocumentFragment();
  for(const n of graph.nodes){
    const col=depth.get(n.id),row=rows[col]=(rows[col]||0)+1;
    const x=16+226*col,y=14+90*(row-1);positions.set(n.id,{x,y});
    const card=document.createElement('button'),title=document.createElement('strong'),sub=document.createElement('small');
    card.type='button';card.className='node-card';card.style.left=x+'px';card.style.top=y+'px';
    card.setAttribute('aria-pressed',n.id===graph.selected);
    const result=results.get(n.id);
    card.dataset.state=result?.error?'error':result?.key===safeKey(n)?'done':'';
    title.textContent=nodeLabel(n);
    sub.textContent=n.op==='viewer'?(n.id===graph.viewer?'Active viewer':'Viewer'):outputPorts(n).map(p=>p.types[0]).join(' · ');
    card.append(title,sub);card.onclick=()=>selectNode(n);
    cards.append(card);
  }
  for(const n of graph.nodes)for(const c of connections(n)){
    const a=positions.get(c.link.node),b=positions.get(n.id);if(!a||!b)continue;
    const wire=document.createElementNS('http://www.w3.org/2000/svg','path');
    const x=a.x+170,y=a.y+35,end=b.y+35,mid=(x+b.x)/2;
    wire.setAttribute('d',`M ${x} ${y} C ${mid} ${y}, ${mid} ${end}, ${b.x} ${end}`);
    if(n.id===graph.selected||c.link.node===graph.selected)wire.classList.add('selected');
    wires.append(wire);
  }
  $('nodeBoard').style.width=(214+226*(deepest+1))+'px';
  $('nodeBoard').style.height=(26+Math.max(1,...rows.filter(Boolean))*90)+'px';
  $('nodeCards').replaceChildren(cards);$('nodeWires').replaceChildren(wires);
}

// Inspector --------------------------------------------------------------------------------------
function inspectNode(node){
  if(!graph||!node)return;
  graph.selected=node.id;
  if(node.op==='viewer'&&graph.viewer!==node.id){graph.viewer=node.id;refreshViewer();}
  renderGraph();
  const op=opOf(node);
  $('nodeTitle').textContent=nodeLabel(node);
  $('nodeOp').textContent=op&&node.label?op.label:'';
  $('nodeDescription').textContent=op?.description||'';
  const options=candidates(node),groups=new Map();
  for(const o of options){if(!groups.has(o.group))groups.set(o.group,document.createElement('optgroup'));const g=groups.get(o.group);g.label=o.group;g.append(new Option(o.label,o.id));}
  $('nodeType').replaceChildren(...groups.values());
  $('appendLabel').hidden=$('addNode').hidden=!options.length;
  $('removeNode').hidden=node.op==='walk';
  renderWiring(node);renderNodeSettings(node);renderCode(node);renderControlsEditor(node);renderOutputs(node);
}
function renderWiring(node){
  const host=$('nodeWiring');host.replaceChildren();
  const heading=document.createElement('strong'),grid=document.createElement('div');
  heading.textContent='Inputs';grid.className='settings-grid';
  if(node.op==='viewer'){
    heading.textContent='Shows';
    for(const src of graph.nodes.filter(n=>n.op!=='viewer'))for(const p of outputPorts(src)){
      const label=document.createElement('label'),box=document.createElement('input');
      box.type='checkbox';box.checked=node.inputs.some(l=>l.node===src.id&&l.port===p.name);
      label.className='setting';label.append(nodeLabel(src)+' · '+p.types[0],box);
      box.onchange=()=>{
        remember();
        node.inputs=box.checked?[...node.inputs,{node:src.id,port:p.name}]:node.inputs.filter(l=>l.node!==src.id||l.port!==p.name);
        edited();refreshViewer();
      };
      grid.append(label);
    }
  }else for(const p of inputPorts(node)){
    const select=document.createElement('select'),label=document.createElement('label'),name=document.createElement('span');
    label.className='setting';name.textContent=p.name+' · '+p.types.join(' or ')+(p.optional?' (optional)':'');
    select.add(new Option(p.optional?'Not connected':'Choose a source',''));
    for(const src of graph.nodes)if(src!==node&&!dependsOn(src,node.id))for(const out of outputPorts(src))
      if(accepts(p.types,out.types[0]))select.add(new Option(nodeLabel(src)+' · '+out.types[0],JSON.stringify({node:src.id,port:out.name})));
    const current=node.inputs?.[p.name];select.value=current?JSON.stringify(current):'';
    select.onchange=()=>{remember();node.inputs??={};if(select.value)node.inputs[p.name]=JSON.parse(select.value);else delete node.inputs[p.name];edited();};
    label.append(name,select);grid.append(label);
  }
  host.hidden=!grid.childElementCount;
  if(node.op==='code'){
    const add=document.createElement('button');add.type='button';add.textContent='Add input port';
    add.onclick=()=>{
      const name=prompt('Input name (letters, digits, underscore)');
      if(!name||!/^[a-zA-Z_]\w*$/.test(name)||node.ports.some(p=>p.name===name))return;
      const type=prompt('Type, one of:\n'+Object.keys(catalog.types).join('\n'),opOf(node).default_type);
      if(!type||!catalog.types[type])return;
      remember();node.ports.push({name,type});edited();inspectNode(node);
    };
    host.append(heading,grid,add);host.hidden=false;return;
  }
  host.append(heading,grid);
}
function renderOutputs(node){
  const host=$('nodeOutputs');host.replaceChildren();
  for(const p of outputPorts(node)){
    const box=document.createElement('details'),title=document.createElement('summary'),body=document.createElement('div');
    const users=graph.nodes.filter(n=>connections(n).some(c=>c.link.node===node.id&&c.link.port===p.name)).map(nodeLabel);
    title.textContent='Output '+p.name+' · '+p.types[0]+(users.length?' → '+users.join(', '):'');
    box.className='port-data';box.append(title,body);
    box.ontoggle=async()=>{
      if(!box.open)return;
      const output=results.get(node.id)?.outputs?.[p.name];
      if(!output){body.textContent=results.get(node.id)?.error||'Run to node to produce this output.';return;}
      body.textContent='Reading…';
      try{showValue(body,await materialize(output,p.types[0]),p.types[0],output);}catch(e){body.textContent=e.message;}
    };
    host.append(box);
  }
  const result=results.get(node.id);
  if(result?.error){const p=document.createElement('p');p.className='muted';p.textContent=result.error;host.append(p);}
}

function renderCode(node){
  const panel=$('codePanel'),code=node.op==='code',viewer=node.op==='viewer';
  panel.hidden=!code&&!viewer&&opOf(node)?.runner==='browser';
  $('codeControls').hidden=!code;$('codeMessage').textContent='';
  if($('codeConsole').dataset.node!==node.id){$('codeConsole').hidden=true;$('codeConsole').dataset.node=node.id;}
  $('nodeProgram').value=node.program??(viewer?viewerProgram(node):code?'return input;':nativeProgram);
  if(code){
    $('codeTitle').value=node.label||'';
    $('codeOut').replaceChildren(...Object.keys(catalog.types).map(t=>new Option(t,t)));$('codeOut').value=node.output;
  }
  $('bindReplay').checked=node.useReplay!==false;
  $('bindReplay').onchange=()=>{remember();node.useReplay=$('bindReplay').checked;edited();};
  const lines=['input = '+(viewer?'connected layer descriptors {node, port, type}':inputPorts(node)[0]?.name+' input'),
    'inputs = {'+(viewer?'node id: {port: value}':inputPorts(node).map(p=>p.name).join(', '))+'}',
    'settings = '+JSON.stringify(resolveSettings(node)),
    'replay = cursor snapshot {frameId, index, timeMs} or null'];
  if(!code&&!viewer)lines.push('await runNative(settings) → the operator\'s '+(outputPorts(node)[0]?.types[0]||'output'));
  if(inputPorts(node).some(p=>p.types.some(isImageType)))lines.push('await readFrame(ref) → {width, height, pixels, format}');
  if(viewer)lines.push('plot.points(rows) · plot.line(vertices) · plot.splats(values) · plot.polygons(faces) · plot.replayFrame(port) · plot.compareFrames(a, b) · plot.layer(port)');
  for(const p of outputPorts(node))lines.push('return → '+p.types[0]+': '+(catalog.types[p.types[0]]?.description||''));
  $('codeSchema').textContent=lines.join('\n');
}
const nativeProgram=`const output = await runNative(settings);
return output; // Edit the value sent to the first output here.`;
$('codeApply').onclick=()=>{
  const node=currentNode();remember();
  if(node.op==='code'){node.label=$('codeTitle').value.trim()||undefined;node.output=$('codeOut').value;}
  const program=$('nodeProgram').value;
  if(node.op==='viewer'&&program===viewerProgram(node))delete node.program;
  else if(!['code','viewer'].includes(node.op)&&program===nativeProgram)delete node.program;
  else node.program=program;
  edited();$('codeMessage').textContent='Applied · Run to node to process.';
};
$('codeRun').onclick=async()=>{
  const editor=$('nodeProgram'),selected=editor.value.slice(editor.selectionStart,editor.selectionEnd);
  $('codeConsole').textContent='';
  const draft=selected?'return (\n'+selected+'\n);':editor.value;
  $('codeRun').disabled=true;
  try{await runPipeline({draft,inspect:!!selected});}finally{$('codeRun').disabled=false;}
};
function renderControlsEditor(node){
  $('controlsJson').value=JSON.stringify(node.controls||[],null,1);
  $('presetControlsJson').value=JSON.stringify(preset.controls||[],null,1);
  $('controlsMessage').textContent='';
}
$('applyControls').onclick=()=>{
  const node=currentNode();
  try{
    const controls=JSON.parse($('controlsJson').value),shared=JSON.parse($('presetControlsJson').value);
    if(!Array.isArray(controls)||!Array.isArray(shared)||[...controls,...shared].some(c=>!c||typeof c.key!=='string'))throw Error('Each control needs a key.');
    remember();node.controls=controls.length?controls:undefined;preset.controls=shared;
    edited();renderPresetControls();inspectNode(node);$('controlsMessage').textContent='Applied';
  }catch(e){$('controlsMessage').textContent=e.message;}
};

// Running ---------------------------------------------------------------------------------------
function safeKey(node){try{return nodeKey(node);}catch{return null;}}
function nodeKey(node,seen=new Set()){
  if(seen.has(node.id))throw Error('The pipeline contains a cycle.');seen.add(node.id);
  const upstream=connections(node).map(c=>{const src=nodeById(c.link.node);return src?[c.name,c.link.port,nodeKey(src,new Set(seen))]:null;});
  const server=!['code','viewer'].includes(node.op)&&opOf(node)?.runner!=='browser';
  return JSON.stringify([state?.run,server?model?.feature_index:null,node.op,resolveSettings(node),node.program,node.ports,node.output,upstream]);
}
function executionOrder(target){
  const ordered=[],done=new Set(),visiting=new Set();
  function visit(node){
    if(done.has(node.id))return;
    if(visiting.has(node.id))throw Error('The pipeline contains a cycle.');
    visiting.add(node.id);
    for(const p of inputPorts(node))if(!p.optional&&!node.inputs?.[p.name])throw Error('Connect '+p.name+' on '+nodeLabel(node)+'.');
    for(const c of connections(node)){
      const src=nodeById(c.link.node);
      if(!src)throw Error(nodeLabel(node)+' lost its '+c.name+' input.');
      if(c.port&&!accepts(c.port.types,outputType(c.link)))throw Error(nodeLabel(node)+' needs '+c.port.types.join(' or ')+' for '+c.name+'.');
      visit(src);
    }
    visiting.delete(node.id);done.add(node.id);ordered.push(node);
  }
  visit(target);return ordered;
}
async function postJson(url,body){
  const response=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
  const value=await response.json();
  if(!response.ok)throw Error(value.error||'Request failed.');
  return value;
}
async function waitIdle(progress){
  for(;;){
    const s=await (await fetch(API+'/state')).json();
    if(!s.busy&&!s.buildingWalks?.length)return s;
    progress(s.message);
    await new Promise(r=>setTimeout(r,700));
  }
}
// Upstream references for server operators: operators by name and settings, browser data inline.
async function refOf(node){
  if(['code','viewer'].includes(node.op)||opOf(node)?.runner==='browser'||node.program){
    const result=results.get(node.id),first=outputPorts(node)[0];
    return {data:await materialize(result.outputs[first.name],first.types[0])};
  }
  const op=opOf(node),settings=resolveSettings(node),inputs={};
  for(const c of connections(node))inputs[c.name]=await refOf(nodeById(c.link.node));
  return {op:node.op,params:Object.fromEntries(op.params.map(p=>[p.key,settings[p.key]])),inputs};
}
async function callOperator(node,params,progress){
  const run=state.run,inputs={};
  for(const c of connections(node))inputs[c.name]=await refOf(nodeById(c.link.node));
  let extra={};
  for(;;){
    const reply=await postJson(API+'/operator',{run,op:node.op,params,inputs,...extra});
    if(reply.outputs)return reply.outputs;
    if(reply.request){extra=await browserRequests[reply.request.kind](reply.request,progress);continue;}
    progress(reply.pending);
    await waitIdle(progress);await update();
  }
}
// Work an operator asks the browser to do before it can finish.
const browserRequests={
  'depth-predictions':(request,progress)=>new Promise((resolve,reject)=>{
    progress('Preparing the local depth model · first use downloads about 38 MB');
    const worker=new Worker('depth-worker.js?'+new URLSearchParams({companion:API||location.origin}),{type:'module'});
    activeWorker=worker;
    worker.onerror=e=>{worker.terminate();reject(Error(e.message||'The local depth runtime could not load.'));};
    worker.onmessage=({data})=>{
      if(data.error){worker.terminate();reject(Error(data.error));return;}
      if(data.status)progress(data.status);
      if(data.predictions){worker.terminate();activeWorker=null;resolve({predictions:data.predictions});}
    };
    worker.postMessage({run:state.run,index:request.index,frames:request.frames});
  })
};
let activeWorker=null;
const browserOperators={
  'splat-fit':async(node,inputs)=>({splats:{data:fitGaussianAttributes(inputs.points)}})
};
async function inputValues(node){
  const values=Object.create(null),refs=[];
  for(const c of connections(node)){
    const src=nodeById(c.link.node),output=results.get(src.id)?.outputs?.[c.link.port];
    if(!output)throw Error('Run '+nodeLabel(src)+' first.');
    const type=outputType(c.link),value=await materialize(output,type);
    values[c.name]=value;
    if(isImageType(type))refs.push(...value);
  }
  return {values,refs};
}
function sandbox(node,program,input,inputs,refs,native,onLog){
  return runProgram(program,input,inputs,pixelReader(refs,state.run,model?.feature_index),native,
    node.useReplay!==false?replaySnapshot():null,onLog,resolveSettings(node));
}
async function execute(node,progress,draft=null,onLog=null){
  const op=opOf(node);
  if(!op)throw Error('Unknown operator '+node.op+'.');
  const settings=resolveSettings(node),program=draft??node.program;
  if(node.op==='viewer')return {outputs:{},scene:await viewerScene(node,program,onLog)};
  if(node.op==='code'){
    const {values,refs}=await inputValues(node),first=inputPorts(node)[0];
    const value=await sandbox(node,program,first?values[first.name]:undefined,values,refs,null,onLog);
    if(draft===null)validateTransform(node.output,value);
    return {outputs:{value:{data:value}},value};
  }
  if(op.runner==='browser'){
    const {values}=await inputValues(node);
    return {outputs:await browserOperators[node.op](node,values,settings)};
  }
  const params=Object.fromEntries(op.params.map(p=>[p.key,settings[p.key]]));
  if(!program)return {outputs:await callOperator(node,params,progress)};
  // Node code wraps the operator and replaces its first output.
  const {values,refs}=await inputValues(node),first=outputPorts(node)[0];
  let outputs=null;
  const native={settings:params,execute:async options=>{
    if(outputs)throw Error('The operator can run once per execution.');
    outputs=await callOperator(node,{...params,...options},progress);
    return materialize(outputs[first.name],first.types[0]);
  }};
  const value=await sandbox(node,program,values[inputPorts(node)[0]?.name],values,refs,native,onLog);
  if(draft===null)validateTransform(first.types[0],value);
  return {outputs:{...outputs,[first.name]:{data:value}},value};
}
async function runPipeline({draft=null,inspect=false}={}){
  if(pipelineRunning||!state?.run||state.busy||state.recording||!graph)return;
  const target=currentNode(),ticket=++runTicket,steps=[],progress=$('pipelineProgress');
  pipelineRunning=true;$('runPipeline').disabled=true;
  const say=message=>{if(ticket===runTicket)progress.textContent=message;};
  try{
    const order=draft!==null?[...executionOrder(target).filter(n=>n!==target),target]:executionOrder(target);
    for(const [i,node] of order.entries()){
      const key=nodeKey(node),drafting=draft!==null&&node===target;
      if(!drafting&&node.op!=='viewer'&&results.get(node.id)?.key===key&&!results.get(node.id).error)continue;
      say((i+1)+' / '+order.length+' · '+nodeLabel(node));
      const started=performance.now();
      try{
        const result=await execute(node,m=>say(nodeLabel(node)+' · '+m),drafting?draft:null,drafting?entry=>consoleLine(entry):null);
        if(drafting){consoleLine(['return',result.value??result.scene]);if(!inspect&&node.op==='viewer')showScene(node,result.scene);say('Code finished');renderGraph();return;}
        results.set(node.id,{key,outputs:result.outputs});
        if(node.op==='viewer')showScene(node,result.scene);
      }catch(e){results.set(node.id,{key,error:e.message});throw e;}
      steps.push({node:nodeLabel(node),op:node.op,settings:resolveSettings(node),seconds:+((performance.now()-started)/1000).toFixed(3)});
      renderGraph();
    }
    if(target.op!=='viewer')await refreshViewer();
    say('Output ready');
    $('runTrace').hidden=false;$('runTraceData').textContent=JSON.stringify(steps,null,2);
    inspectNode(currentNode());
  }catch(e){say(e.message);if(draft!==null)consoleLine(['error',e.message]);inspectNode(currentNode());}
  finally{pipelineRunning=false;$('runPipeline').disabled=false;updatePipelineButton();}
}
$('runPipeline').onclick=()=>runPipeline();
function updatePipelineButton(){$('runPipeline').disabled=pipelineRunning||!state?.run||!!state?.busy||!!state?.recording||!graph;}

// Data ------------------------------------------------------------------------------------------
async function fetchBuffer(url){const r=await fetch(api(url));if(!r.ok)throw Error('Could not read '+url.split('?')[0]+'.');return r.arrayBuffer();}
async function fetchJson(url){const r=await fetch(api(url));if(!r.ok)throw Error(r.status===404?'Output not found. Rerun the node.':'Could not read the output.');return r.json();}
function materialize(output,type){
  if(!output)return Promise.reject(Error('Run the source node first.'));
  if(!materialized.has(output))materialized.set(output,load(output,type).catch(e=>{materialized.delete(output);throw e;}));
  return materialized.get(output);
}
async function load(output,type){
  if('data' in output)return output.data;
  if(displayOf(type)==='map'){
    const m=output.index&&model?.feature_index===output.index?model:await fetchJson(output.url);
    return {poses:m.poses,points:m.points,units:m.units};
  }
  if(output.format==='splat-ply'){const g=new GaussianView();g.load(await fetchBuffer(output.url),false);return g.attributes.subarray(0,g.order.length*16);}
  if(output.format==='mesh-ply'){
    const m=new GaussianMeshView();m.load(await fetchBuffer(output.url));
    return Array.from({length:m.indices.length/3},(_,i)=>({vertices:[0,1,2].map(k=>m.vertices[m.indices[3*i+k]])}));
  }
  return fetchJson(output.url);
}
// Geometry the 3D view draws for one output.
async function layerFor(output,type){
  if(output.format==='splat-ply'){const splats=new GaussianView();splats.load(await fetchBuffer(output.url),false);return {splats};}
  if(output.format==='mesh-ply'){const solid=new GaussianMeshView();solid.load(await fetchBuffer(output.url));return {solid};}
  const value=await materialize(output,type),kind=displayOf(type);
  if(kind==='map')return {points:value.points,poses:value.poses};
  if(kind==='path')return {line:value.map(p=>p.xyz),poses:value};
  if(kind==='points')return {points:value};
  if(kind==='triangles')return {mesh:{patches:value.map(vertices=>({vertices,landmarks:[]})),retired:[]}};
  if(kind==='polygons'){
    const summary=output.summary||{};
    return {mesh:{...summary,patches:value.map(p=>({...p,landmarks:p.landmarks||[]})),retired:summary.retired||[]},
      cut:summary.heightRange?Number($('balloonCut').value):null};
  }
  if(kind==='splats'&&value.length){const splats=new GaussianView();splats.loadAttributes(value);return {splats};}
  return null;
}
function viewerProgram(node){
  return ['const layers = [];',...node.inputs.map((l,i)=>'layers.push(plot.layer(input['+i+'])); // '+nodeLabel(nodeById(l.node))+' · '+outputType(l)),'return layers;'].join('\n');
}
async function viewerScene(node,program,onLog){
  const bindings=node.inputs.filter(l=>nodeById(l.node)&&nodeById(l.node).enabled!==false).map(l=>({...l,type:outputType(l),isFrame:isImageType(outputType(l))}));
  if(!program)return {bindings,geometry:[],replayViews:[]};
  const inputs=Object.create(null),refs=[];
  for(const b of bindings){
    const output=results.get(b.node)?.outputs?.[b.port];
    if(!output)continue;
    const value=await materialize(output,b.type);
    (inputs[b.node]??=Object.create(null))[b.port]=value;
    if(isImageType(b.type))refs.push(...value);
  }
  const value=await sandbox(node,program,bindings,inputs,refs,null,onLog);
  if(!Array.isArray(value))throw Error('Return an array of plot layers.');
  const scene={bindings:[],geometry:[],replayViews:[]};
  for(const p of value){
    if(p?.draw==='replayFrames'){
      scene.replayViews.push({split:50,streams:p.ports.map(port=>{
        const b=bindings.find(b=>b.node===port?.node&&b.port===port.port);
        if(!b||!isImageType(b.type))throw Error('Choose a connected image port.');
        return {binding:b,byFrame:new Map((inputs[b.node]?.[b.port]||[]).map(f=>[f.frame,f]))};
      })});
    }else if(p?.draw==='points'){validateTransform('Colored points',p.rows);scene.geometry.push({points:p.rows});}
    else if(p?.draw==='line')scene.geometry.push({line:p.vertices});
    else if(p?.draw==='splats'){validateTransform('Point-fitted splats',p.values);const splats=new GaussianView();splats.loadAttributes(p.values);scene.geometry.push({splats});}
    else if(p?.draw==='polygons')scene.geometry.push({mesh:{patches:p.faces.map(f=>({...f,landmarks:[]})),retired:[]}});
    else if(bindings.some(b=>b.node===p?.node&&b.port===p.port))scene.bindings.push(p);
    else throw Error('Expected a plot command or a connected layer.');
  }
  return scene;
}
let scene={layers:[],feeds:[],replayViews:[],panels:[]},sceneTicket=0;
async function showScene(node,built){
  const ticket=++sceneTicket,layers=[],feeds=[],panels=[];
  for(const b of built.bindings){
    const type=b.type||outputType(b),output=results.get(b.node)?.outputs?.[b.port];
    if(!output)continue;
    if(isTableType(type)||isImageType(type)){feeds.push({...b,type,output});continue;}
    panels.push({type,panel:catalog.types[type]?.panel,output,node:nodeById(b.node)});
    try{const layer=await layerFor(output,type);if(layer)layers.push({...layer,type,source:b.node});}
    catch(e){feeds.push({...b,type,error:e.message});}
  }
  for(const g of built.geometry)layers.push({...g,type:'Plot',source:node.id});
  if(ticket!==sceneTicket)return;
  scene={layers,feeds,replayViews:built.replayViews,panels};
  $('viewerName').textContent=nodeLabel(node);
  $('viewerOutput').textContent=[...new Set([...layers,...feeds].map(l=>l.type)),...(built.replayViews.length?['Frame comparison']:[])].join(' + ')||'Nothing connected';
  $('viewerFrames').dataset.key='';
  $('surfaceTimeControl').hidden=!layers.some(l=>l.mesh?.patches?.some(p=>p.first!=null));
  showTypePanels(panels);refreshDisplayedPath();fitMapView();draw();
}
async function refreshViewer(){
  const view=viewerNode();
  if(!view){scene={layers:[],feeds:[],replayViews:[],panels:[]};draw();return;}
  try{
    const built=await viewerScene(view,view.program,null);
    built.bindings=view.program?built.bindings:built.bindings.filter(b=>results.get(b.node)?.outputs?.[b.port]);
    await showScene(view,built);
  }catch(e){$('viewerOutput').textContent=e.message;}
}
// The 3D view and replay code read these.
function graphLayers(){return scene.layers;}
function viewBindings(){return [...scene.layers.filter(l=>l.type!=='Plot').map(l=>({type:l.type,node:l.source})),...scene.feeds];}

// Output panels by data type ---------------------------------------------------------------------
let activeGaussian=null;
// Tools shown beside an output, named by its type's `panel` in the catalogue.
const typePanels={
  gaussian:{element:'gaussianPanel',show:({output})=>{activeGaussian=output;loadGaussianReport();}},
  clearance:{element:'clearancePanel',show:({output})=>{activeGaussian=output;refreshClearance();}},
  lingbot:{element:'lingbotPanel',show:({output})=>showLingbot(output)},
  mesh:{element:'balloonCutControl',show:({output})=>{$('balloonCutControl').hidden=!output.summary?.heightRange;}},
};
function showTypePanels(panels){
  activeGaussian=null;
  for(const [name,p] of Object.entries(typePanels))$(p.element).hidden=!panels.some(x=>x.panel===name);
  $('typePanels').hidden=!panels.some(x=>typePanels[x.panel]);
  for(const panel of panels)typePanels[panel.panel]?.show(panel);
  if(!panels.some(p=>p.panel==='lingbot'))showLingbot(null);
  refreshTofEvidence();
}
async function loadGaussianReport(){
  const output=activeGaussian;
  $('gaussianStatus').textContent='Opening Gaussian map…';
  try{
    const report=await fetchJson(gaussianUrl('report.json'));
    if(output!==activeGaussian)return;
    $('gaussianTestFrame').replaceChildren(...report.evaluation.map(e=>new Option(parseInt(e.frame)+' · '+e.psnr_db+' dB',e.frame)));
    gaussianPair();$('gaussianEval').hidden=false;
    $('gaussianStatus').textContent=report.train_views+' training views · '+report.test_views+' withheld · '+report.mean_psnr_db+' dB PSNR'+(report.steps?' · '+report.steps+' steps':'');
    $('gaussianDownload').href=gaussianUrl('gaussians.ply');$('gaussianDownload').download='gaussians.ply';$('gaussianDownload').hidden=false;
  }catch(e){$('gaussianStatus').textContent=e.message;}
}
function showLingbot(output){
  const viewer=$('lingbotView');
  if(!output){viewer.hidden=true;viewer.removeAttribute('src');$('viewNav').hidden=!model;return;}
  fetchJson(output.url).then(report=>{
    $('lingbotStatus').textContent=report.views+' predicted poses · '+report.points.toLocaleString()+' depth samples · relative scale';
    viewer.src=api(output.view);viewer.hidden=false;$('viewNav').hidden=true;
    $('lingbotCloud').href=api(output.cloud);$('lingbotCloud').hidden=false;
  }).catch(e=>{$('lingbotStatus').textContent=e.message;});
}

// Frame and table feeds beside the 3D view.
function viewerFrames(){
  const panel=$('viewerFrames'),key=JSON.stringify([scene.feeds.map(f=>[f.node,f.port,f.type]),timelineRun,scene.replayViews.length,
    scene.feeds.some(f=>isImageType(f.type))?cursor:null]);
  const spatial=scene.layers.length>0;
  $('mapPanel').classList.toggle('frames-only',!spatial&&(scene.feeds.length>0||scene.replayViews.length>0));
  $('mapPanel').classList.toggle('has-path',scene.layers.some(l=>l.poses));
  $('mapPanel').classList.toggle('has-points',scene.layers.some(l=>l.points));
  if(panel.dataset.key===key){for(const u of panel.replayUpdates||[])u();return;}
  panel.dataset.key=key;panel.replaceChildren();panel.hidden=!scene.feeds.length&&!scene.replayViews.length;
  panel.replayUpdates=scene.replayViews.map(v=>{const s=document.createElement('section');panel.append(s);const u=mountReplayFrames(s,v);u();return u;});
  for(const f of scene.feeds){
    const group=document.createElement('section'),heading=document.createElement('strong'),body=document.createElement('div');
    heading.textContent=nodeLabel(nodeById(f.node))+' · '+f.type;group.append(heading,body);panel.append(group);
    if(f.error){body.textContent=f.error;continue;}
    materialize(f.output,f.type).then(v=>showValue(body,v,f.type,f.output)).catch(e=>{body.textContent=e.message;});
  }
}
function mountReplayFrames(host,view){
  const title=document.createElement('strong'),stage=document.createElement('div');
  stage.className='replay-compare';host.append(title,stage);
  const panes=view.streams.map(({binding})=>{const pane=document.createElement('div');pane.className='replay-pane';stage.append(pane);
    return {pane,label:nodeLabel(nodeById(binding.node))+' · '+binding.type};});
  if(panes.length===2){
    const slider=document.createElement('input');slider.type='range';slider.min=0;slider.max=100;slider.value=view.split;
    slider.setAttribute('aria-label','Comparison divider');
    slider.oninput=()=>{view.split=slider.valueAsNumber;stage.style.setProperty('--split',view.split+'%');};slider.oninput();stage.append(slider);
  }
  let previous;
  return ()=>{
    const frameId=replaySnapshot()?.frameId;
    if(frameId===previous&&title.textContent)return;previous=frameId;
    title.textContent=panes.map(p=>p.label).join('  |  ')+' · '+(frameId||'No replay frame');
    const refs=view.streams.map(s=>s.byFrame.get(frameId));
    for(const [i,{pane,label}] of panes.entries()){
      const ref=refs[i];pane.replaceChildren();
      if(!ref){pane.textContent='No matching frame';continue;}
      if(ref.pixels){const c=document.createElement('canvas');paintFramePixels(c,ref);pane.append(c);continue;}
      const image=document.createElement('img');image.crossOrigin='anonymous';image.alt=label+' · '+frameId;
      const endpoint=ref.method?API+'/frame-depth.png':refs.some(f=>f?.method)?API+'/depth-input.jpg':API+'/recorded.jpg';
      image.onerror=()=>image.replaceWith(document.createTextNode('Frame unavailable · '+frameId));
      image.src=endpoint+'?'+new URLSearchParams({run:timelineRun,index:model?.feature_index||'',frame:ref.frame,method:ref.method||'photo'});
      pane.append(image);
    }
  };
}
// Inspect any value by its type.
function showValue(body,value,type,output={}){
  body.replaceChildren();
  const text=s=>{const p=document.createElement('pre');p.textContent=s;body.append(p);return p;};
  const rows=(label,count,get,initial=0)=>{
    if(!count){text('No samples available.');return;}
    const control=document.createElement('label'),input=document.createElement('input');
    input.type='number';input.min=1;input.max=count;input.value=Math.min(count,initial+1);
    control.append(label+' ',input,' of '+count.toLocaleString());body.append(control);
    const out=text('');const refresh=()=>{const i=input.valueAsNumber-1;if(Number.isInteger(i)&&i>=0&&i<count)out.textContent=JSON.stringify(get(i),null,2);};
    input.oninput=refresh;refresh();
  };
  const plot=points=>{
    const box=document.createElement('details'),title=document.createElement('summary'),host=document.createElement('div');
    title.textContent='Plot point coordinates';box.append(title,host);body.append(box);
    box.ontoggle=()=>{if(box.open&&!host.childElementCount)recordDisplay(host,Array.from(points,([x,y,z,red,green,blue])=>({x,y,z,red,green,blue})),{mode:'scatter',x:'x',y:'y'});};
  };
  const kind=displayOf(type);
  if(kind==='pixels'){displayPixels(body,value);return;}
  if(isTableType(type)){recordDisplay(body,value);return;}
  if(isImageType(type)){
    const ref=value.find(r=>r.frame===timeline[cursor]?.id)||value[0];
    if(!ref){text('No frames.');return;}
    const query=new URLSearchParams({run:timelineRun,index:model?.feature_index||'',method:ref.method||'photo',frame:ref.frame});
    const image=document.createElement('img');image.crossOrigin='anonymous';image.alt=type+' · '+ref.frame;image.src=(ref.method?API+'/frame-depth.png?':API+'/recorded.jpg?')+query;
    body.append(image);text(ref.frame+' · '+(ref.ms/1000).toFixed(2)+' s · '+value.length+' frames · scrub the replay to change frame');
    if(ref.method){
      const out=text('Reading depth sample…');
      const sample=async(x,y)=>{
        const q=new URLSearchParams(query);if(x!==undefined){q.set('x',x);q.set('y',y);}
        try{
          const s=await fetchJson(API+'/frame-depth-sample?'+q);
          out.textContent='Pixel ('+s.x+', '+s.y+') · '+(s.value??'missing')+' '+(output.units||'')+' · '+s.width+' × '+s.height;
          image.title='Click a pixel to inspect depth';
          image.onclick=e=>{const b=image.getBoundingClientRect();sample(Math.min(s.width-1,Math.floor((e.clientX-b.left)*s.width/b.width)),Math.min(s.height-1,Math.floor((e.clientY-b.top)*s.height/b.height)));};
        }catch{out.textContent='Depth sample unavailable';}
      };sample();
    }
    return;
  }
  if(kind==='map'){rows('Pose',value.poses.length,i=>value.poses[i]);rows('Landmark',value.points.length,i=>({xyz:value.points[i].slice(0,3),rgb:value.points[i].slice(3)}));plot(value.points);return;}
  if(value instanceof Float32Array){
    rows('Splat',value.length/16,i=>{const f=(o,n)=>Array.from(value.subarray(16*i+o,16*i+o+n));return {center:f(0,3),opacity:f(3,1)[0],rgb:f(4,3),radii:f(8,3),rotation_wxyz:f(12,4)};});
    if(output.url){const a=document.createElement('a');a.href=api(output.url);a.textContent='Download PLY';a.download='';body.append(a);}
    return;
  }
  if(Array.isArray(value)){
    rows({points:'Point',polygons:'Face',triangles:'Triangle',path:'Pose'}[kind]||'Record',value.length,i=>value[i]);
    if(kind==='points')plot(value);
    if(output.summary)text(JSON.stringify(output.summary,null,2));
    return;
  }
  text(JSON.stringify(value,null,2).slice(0,200000));
}
function consoleLine([label,...values]){
  const box=$('codeConsole');box.hidden=false;
  for(const value of values)box.append(inspectConsoleValue(label,value));
}

// Presets ---------------------------------------------------------------------------------------
let presetList=[];
async function loadPresets(){
  presetList=await fetchJson(API+'/presets');
  const select=$('pipelinePreset'),current=select.value;
  select.replaceChildren(new Option('Current graph',''),...presetList.map(p=>new Option(p.name+(p.editable?' · yours':''),p.name)));
  select.value=presetList.some(p=>p.name===current)?current:'';
}
function openPreset(source){
  preset=structuredClone(source);
  preset.values??={};preset.controls??=[];
  graph=preset.graph;graph.nodes??=[];
  undoStack.length=0;$('undoGraph').disabled=true;results.clear();
  $('presetName').value=source.editable?source.name:'';
  $('presetDescription').textContent=source.description||'';
  $('deletePreset').hidden=!source.editable;
  renderPresetControls();
  inspectNode(currentNode());
  scene={layers:[],feeds:[],replayViews:[],panels:[]};showTypePanels([]);draw();
}
async function restorePipeline(run){
  await catalogReady;await loadPresets();
  const name=localStorage.getItem('room-survey-active-preset-'+run);
  const saved=presetList.find(p=>p.name===name)||presetList.find(p=>p.default)||presetList[0];
  if(!saved)return;
  $('pipelinePreset').value=saved.name;openPreset(saved);
  // Show what is already built without starting new work.
  if(state?.hasMap)await runQuiet(viewerNode());
}
// Collect outputs that exist already; operators that would start a job stop here.
async function runQuiet(target){
  if(!target)return;
  try{
    for(const node of executionOrder(target)){
      if(node.op==='viewer')continue;
      const op=opOf(node);
      if(op.runner==='code'||op.runner==='browser'||node.program)break;
      const settings=resolveSettings(node),inputs={};
      for(const c of connections(node))inputs[c.name]=await refOf(nodeById(c.link.node));
      const reply=await postJson(API+'/operator',{run:state.run,op:node.op,params:Object.fromEntries(op.params.map(p=>[p.key,settings[p.key]])),inputs,peek:true});
      if(!reply.outputs)break;
      results.set(node.id,{key:nodeKey(node),outputs:reply.outputs});
    }
  }catch{}
  renderGraph();await refreshViewer();inspectNode(currentNode());
}
$('pipelinePreset').onchange=()=>{
  const p=presetList.find(p=>p.name===$('pipelinePreset').value);
  if(!p)return;
  if(state?.run)localStorage.setItem('room-survey-active-preset-'+state.run,p.name);
  $('presetStatus').textContent='';openPreset(p);if(state?.hasMap)runQuiet(viewerNode());
};
$('savePreset').onclick=async()=>{
  const name=$('presetName').value.trim();
  if(!name){$('presetStatus').textContent='Enter a preset name.';$('presetName').focus();return;}
  try{
    const {name:_,file,editable,...data}=preset;
    presetList=await postJson(API+'/save-preset',{name,preset:data});
    await loadPresets();$('pipelinePreset').value=name;$('deletePreset').hidden=false;
    if(state?.run)localStorage.setItem('room-survey-active-preset-'+state.run,name);
    $('presetStatus').textContent='Saved to presets/user';
  }catch(e){$('presetStatus').textContent=e.message;}
};
$('deletePreset').onclick=async()=>{
  const name=$('pipelinePreset').value||$('presetName').value.trim();
  if(!name||!confirm('Delete the preset “'+name+'”?'))return;
  try{await postJson(API+'/delete-preset',{name});await loadPresets();$('presetStatus').textContent='Deleted';$('deletePreset').hidden=true;}
  catch(e){$('presetStatus').textContent=e.message;}
};

// Device and recording settings: how this Mac drives the camera, sensors and live map.
async function renderDeviceSettings(){
  const host=$('deviceForm');
  try{
    const {params,values}=await fetchJson(API+'/device-settings'),groups=new Map();
    for(const c of params){if(!groups.has(c.group))groups.set(c.group,[]);groups.get(c.group).push(c);}
    const parts=[];
    for(const [title,controls] of groups){
      const heading=document.createElement('strong'),grid=document.createElement('div');
      heading.textContent=title;grid.className='settings-grid';
      for(const c of controls)grid.append(settingRow(c,values,controlInput(c,values[c.key],v=>{values[c.key]=c.kind==='choice'&&typeof c.default==='number'?Number(v):v;}),c.help?'':''));
      parts.push(heading,grid);
    }
    const bar=document.createElement('div'),save=document.createElement('button');
    bar.className='controls';save.type='button';save.textContent='Save device settings';
    save.onclick=async()=>{await act('device-settings',{values});if(!$('error').textContent)renderDeviceSettings();};
    bar.append(save);host.replaceChildren(...parts,bar);host.className='node-settings';
  }catch(e){host.textContent=e.message;}
}
$('deviceSetup').ontoggle=()=>{if($('deviceSetup').open)renderDeviceSettings();};
// The range probe offers the addresses the device settings allow, starting at the configured one.
fetchJson(API+'/device-settings').then(({params,values})=>{
  const spec=params.find(p=>p.key==='tof_address');
  $('address').replaceChildren(...spec.options.map(([v,t])=>new Option(t,v)));$('address').value=values.tof_address;
}).catch(()=>{});

poll();
