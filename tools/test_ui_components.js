'use strict';
// Browser-native regression contracts; open tools/test_ui_components.html from a local preview.
(async()=>{
  const results=[],fixtures=document.getElementById('fixtures');
  const assert=(condition,message)=>{if(!condition)throw new Error(message);};
  const key=(node,value)=>node.dispatchEvent(new KeyboardEvent('keydown',{key:value,bubbles:true,cancelable:true}));
  const event=(node,type)=>node.dispatchEvent(new Event(type,{bubbles:true}));
  const test=async(name,run)=>{try{await run();results.push('PASS '+name);}catch(e){results.push('FAIL '+name+': '+e.message);}};
  await test('Tab click/arrow/Home/End, disabled skipping, panel visibility and roving focus',()=>{
    const root=el('div'),panels=['test-a','test-b','test-c'].map(id=>el('div',{id}));fixtures.append(root,...panels);
    const calls=[];const tabs=UI.tabs({root,label:'Test tabs',items:panels.map((p,i)=>({key:String(i),id:'trigger-'+i,panel:p.id,label:'Page '+i,disabled:i===1})),initial:'0',onChange:k=>calls.push(k)});
    assert(calls.length===0,'bootstrap callback ran too early');
    const first=root.children[0],last=root.children[2];first.focus();key(first,'ArrowRight');
    assert(tabs.active==='2'&&document.activeElement===last,'disabled tab not skipped/focused');
    assert(panels[0].hidden&&!panels[2].hidden,'wrong panel visibility');
    assert(last.tabIndex===0&&first.tabIndex===-1&&last.getAttribute('aria-selected')==='true','selection/focus attributes differ');
    key(last,'Home');assert(tabs.active==='0','Home failed');key(first,'End');assert(tabs.active==='2','End failed');
    first.click();assert(tabs.active==='0'&&calls.length===4,'click callbacks differ');
    tabs.select('missing');assert(tabs.active==='0','unknown key did not normalize');
  });
  await test('Tab restore preserves saved selection until all modules are ready',()=>{
    const storageKey='liftlab-ui-component-test';localStorage.setItem(storageKey,'b');
    const root=el('div'),a=el('div',{id:'restore-a'}),b=el('div',{id:'restore-b'});fixtures.append(root,a,b);
    const tabs=UI.tabs({root,label:'Restore',storageKey,initial:'a',items:[{key:'a',id:'restore-trigger-a',panel:a.id,label:'A'},{key:'b',id:'restore-trigger-b',panel:b.id,label:'B'}]});
    assert(localStorage.getItem(storageKey)==='b','initial rendering erased saved selection');
    tabs.restore();assert(tabs.active==='b'&&!b.hidden,'restore failed');localStorage.removeItem(storageKey);
  });
  await test('Field labels and help associate with direct and grouped inputs',()=>{
    for(const grouped of [false,true]){
      const input=UI.input({type:'text'}),content=grouped?el('div',{},input,UI.button({text:'Action'})):input;
      const field=UI.field({label:'Name',hint:'Help'},content);fixtures.append(field);
      assert(field.querySelector('label').htmlFor===input.id,'label not associated');
      assert(document.getElementById(input.getAttribute('aria-describedby')).textContent==='Help','help not associated');
    }
  });
  await test('Committed selects defer keyboard changes until Enter/blur; pointer changes apply once',()=>{
    const values=[];const select=UI.choice({label:'Choice',options:[[1,'One'],[2,'Two']],value:1,commit:true,onChange:v=>values.push(v)});fixtures.append(select);
    key(select,'ArrowDown');select.value='2';event(select,'change');assert(values.length===0,'keyboard change applied immediately');
    key(select,'Enter');assert(values.join()==='2','Enter failed');
    event(select,'pointerdown');select.value='1';event(select,'change');assert(values.join()==='2,1','pointer applied incorrectly');
    key(select,'ArrowDown');select.value='2';event(select,'change');event(select,'blur');assert(values.join()==='2,1,2','blur failed');
  });
  await test('Disclosure visibility/ARIA, guarded activation and external selection',()=>{
    const head=UI.button({text:'Expand'}),body=el('div'),states=[];fixtures.append(head,body);let blocked=true;
    const control=UI.bindDisclosure(head,body,{open:false,beforeToggle:()=>!blocked,onToggle:v=>states.push(v)});
    head.click();assert(body.hidden&&states.length===0,'guard ignored');blocked=false;head.click();
    assert(!body.hidden&&head.getAttribute('aria-expanded')==='true'&&head.getAttribute('aria-controls')===body.id,'open attributes differ');
    control.setOpen(false);assert(body.hidden&&states.join()==='true,false','external state failed');
  });
  await test('Native disclosure preserves rich summary content and reports open/close',async()=>{
    const states=[],details=UI.details({title:[el('b',{text:'Driver'}),' settings'],onToggle:value=>states.push(value)},el('p',{text:'Content'}));fixtures.append(details);
    assert(details.querySelector('summary').textContent==='Driver settings','summary content changed');
    for(const expected of [true,false]) {
      const toggled=new Promise(resolve=>details.addEventListener('toggle',resolve,{once:true}));
      details.querySelector('summary').click();await toggled;
      assert(details.open===expected&&states.at(-1)===expected,'native open state or callback differs');
    }
  });
  await test('Numeric fields preserve units, editable range, fraction bounds and disable state',()=>{
    let mass=1;const field=numField('test-mass',{label:'Mass',min:0.1,max:2,step:0.1,u:'g',dp:0,k:1000},()=>mass,v=>mass=v);fixtures.append(field.node);
    const number=field.node.querySelector('input[type=number]');assert(number.value==='1000','unit display wrong');
    number.value='5000';event(number,'input');assert(mass===5,'typed range incorrectly limited to slider');
    number.value='-5';event(number,'input');assert(mass===5,'negative mass accepted');event(number,'change');assert(number.value==='5000','tidy failed');
    field.setOff(true,'Disconnected');assert(number.disabled&&field.node.textContent.includes('Disconnected'),'disabled reason lost');
    let amount=0.5;const fraction=numField('test-fraction',{label:'Amount',min:0,max:1,step:0.1,u:'',dp:1},()=>amount,v=>amount=v);fixtures.append(fraction.node);
    const f=fraction.node.querySelector('input[type=number]');f.value='2';event(f,'input');assert(amount===1,'fraction bound failed');
  });
  await test('Menu skips disabled actions and restores focus with Escape',()=>{
    const menu=menuButton({text:'Actions',items:()=>[{value:0,label:'Unavailable',disabled:true},{value:1,label:'Available'}],onPick:()=>{}});fixtures.append(menu.node);
    menu.btn.click();assert(document.activeElement.textContent==='Available','disabled action focused');
    key(document.activeElement,'Escape');assert(menu.btn.getAttribute('aria-expanded')==='false'&&document.activeElement===menu.btn,'menu focus not restored');
  });
  await test('Theme tokens resolve across controls, cards and tabs in light and dark',()=>{
    const swatch=el('span',{style:'color:var(--accent)',text:'Accent token'});
    const card=UI.card({},UI.button({class:'btn primary',text:'Primary'}),swatch);fixtures.append(card);
    for(const theme of ['light','dark']) {
      document.documentElement.dataset.theme=theme;
      const tokens=getComputedStyle(document.documentElement),button=getComputedStyle(card.firstChild);
      assert(tokens.getPropertyValue('--panel').trim()===tokens.getPropertyValue('--palette-panel-'+theme).trim(),'panel palette not applied');
      assert(button.backgroundColor===getComputedStyle(swatch).color,'primary button ignores accent token');
      assert(getComputedStyle(card).borderRadius===tokens.getPropertyValue('--radius-card').trim(),'card radius ignores token');
    }
    delete document.documentElement.dataset.theme;
  });
  fixtures.prepend(UI.field({label:'Preview theme'},UI.choice({label:'Preview theme',options:[['auto','System'],['light','Light'],['dark','Dark']],value:'auto',onChange:value=>{if(value==='auto')delete document.documentElement.dataset.theme;else document.documentElement.dataset.theme=value;}})));
  const failed=results.filter(r=>r.startsWith('FAIL')).length;
  const output=document.getElementById('testResults');output.textContent=results.join('\n')+'\n\n'+(failed?failed+' FAILED':'ALL '+results.length+' PASSED');output.className=failed?'test-fail':'test-pass';
})();
