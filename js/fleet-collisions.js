'use strict';
// Broad-phase craft bounds, then oriented component boxes and sampled spinning
// prop rims. Contacts use equal/opposite impulses and whole-craft inertia.
// Articulated collision response approximates the joints as held during impact.
function fleetBox(center,R,size,part=null) {
  const half=size.map(x=>Math.max(.003,x/2));
  return {center,axes:[[R[0],R[3],R[6]],[R[1],R[4],R[7]],[R[2],R[5],R[8]]],half,radius:Math.hypot(...half),part};
}
function fleetShapes(d) {
  return withDrone(d,()=>{
    const R=qmat(S.q), boxes=[fleetBox(S.p,m3m(R,frameRot()),frameDims())], props=[];
    for(const c of liveComps()) {
      if(c.type==='hang')continue;
      const pose=poseOf(c), center=add(S.p,m3v(R,pose.p)), rotation=m3m(R,pose.R);
      const parent=parentOf(c), joint=parentJoint(c);
      const from=parent?.type==='link' ? add(poseOf(parent).p,m3v(poseOf(parent).R,scl(linkDir(parent),parent.length))) : joint ? poseOf(joint).p : [0,0,0];
      const arm=sub(pose.p,from),length=nrm(arm);
      if(length>.015)boxes.push(fleetBox(add(S.p,m3v(R,scl(add(from,pose.p),.5))),m3m(R,frameFrom(scl(arm,1/length),[1,0,0])),[.014,.014,length],c));
      if(c.type==='motor') {
        boxes.push(fleetBox(center,rotation,[.035,.035,.03],c));
        const ro=rotorNow(c),st=act.get(c.id);
        if(st && st.Omega*propR(c)>12 && !hsOf(c).prop)props.push({c,center:add(S.p,m3v(R,ro.p)),axis:m3v(R,ro.d),radius:propR(c)});
      } else if(c.type==='mass') {
        const size=c.shape==='sphere'?[2*c.radius,2*c.radius,2*c.radius]:c.shape==='cylinder'?[2*c.radius,2*c.radius,c.length]:c.size;
        boxes.push(fleetBox(center,m3m(rotation,massRot(c)),size,c));
      } else if(c.type==='link') {
        const dir=m3v(rotation,linkDir(c)), axes=frameFrom(dir,[1,0,0]);
        boxes.push(fleetBox(add(center,scl(dir,c.length/2)),axes,[.014,.014,c.length],c));
      } else boxes.push(fleetBox(center,rotation,c.type==='joint'?[.04,.02,.036]:[.025,.025,.018],c));
    }
    return {boxes,props};
  });
}
function fleetBoxContact(a,b) {
  const delta=sub(a.center,b.center);
  if(dot(delta,delta)>(a.radius+b.radius)**2)return null;
  const axes=[...a.axes,...b.axes];
  for(const x of a.axes)for(const y of b.axes){const v=crs(x,y),l=nrm(v);if(l>1e-7)axes.push(scl(v,1/l));}
  let depth=Infinity,normal;
  for(const axis of axes){
    const ra=a.axes.reduce((s,v,i)=>s+Math.abs(dot(v,axis))*a.half[i],0),rb=b.axes.reduce((s,v,i)=>s+Math.abs(dot(v,axis))*b.half[i],0);
    const distance=dot(delta,axis),overlap=ra+rb-Math.abs(distance);
    if(overlap<=0)return null;
    if(overlap<depth){depth=overlap;normal=scl(axis,distance>=0?1:-1);}
  }
  const closest=(box,p)=>box.axes.reduce((v,axis,i)=>add(v,scl(axis,clamp(dot(sub(p,box.center),axis),-box.half[i],box.half[i]))),box.center.slice());
  return {depth,normal,point:scl(add(closest(a,b.center),closest(b,a.center)),.5)};
}
function fleetPointInBox(p,b) {const v=sub(p,b.center);return b.axes.every((axis,i)=>Math.abs(dot(v,axis))<=b.half[i]+.003);}
function fleetPropHit(prop,shapes) {
  // Also test the disk's interior: a thin arm can pass between sampled rim points.
  for(const box of shapes.boxes){
    if(fleetPointInBox(prop.center,box))return true;
    const corners=Array.from({length:8},(_,i)=>box.axes.reduce((p,a,k)=>add(p,scl(a,box.half[k]*(i&(1<<k)?1:-1))),box.center.slice()));
    for(let i=0;i<8;i++)for(let k=0;k<3;k++)if(!(i&(1<<k))){
      const a=sub(corners[i],prop.center),b=sub(corners[i|(1<<k)],prop.center),ha=dot(a,prop.axis),hb=dot(b,prop.axis);
      if(Math.min(ha,hb)>.006||Math.max(ha,hb)<-.006)continue;
      const t=Math.abs(hb-ha)>1e-9?clamp(-ha/(hb-ha),0,1):clamp(-dot(a,sub(b,a))/Math.max(1e-12,dot(sub(b,a),sub(b,a))),0,1);
      const p=add(a,scl(sub(b,a),t)),h=dot(p,prop.axis);
      if(Math.abs(h)<=.006 && nrm(sub(p,scl(prop.axis,h)))<=prop.radius)return true;
    }
  }
  const axis=prop.axis,e1=unit(crs(axis,Math.abs(axis[2])<.9?[0,0,1]:[1,0,0])),e2=crs(axis,e1);
  for(let i=0;i<16;i++){
    const a=i*Math.PI/8,p=add(prop.center,scl(add(scl(e1,Math.cos(a)),scl(e2,Math.sin(a))),prop.radius));
    if(shapes.boxes.some(b=>fleetPointInBox(p,b)))return true;
    for(const other of shapes.props){const v=sub(p,other.center),height=dot(v,other.axis);if(Math.abs(height)<.006 && nrm(sub(v,scl(other.axis,height)))<other.radius)return true;}
  }
  return false;
}
function fleetPointVelocity(d,point) {
  const s=d.state,R=qmat(s.S.q);
  return add(s.S.v,m3v(R,crs(s.S.w,m3v(m3T(R),sub(point,s.S.p)))));
}
function fleetInverseMass(d,point,normal) {
  const s=d.state,R=qmat(s.S.q),RT=m3T(R),r=sub(m3v(RT,sub(point,s.S.p)),s.truth.c),n=m3v(RT,normal);
  return 1/s.truth.m+dot(n,crs(m3v(s.truth.Jinv,crs(r,n)),r));
}
function fleetImpulse(d,point,J) {
  const s=d.state,R=qmat(s.S.q),RT=m3T(R),c=s.truth.c,r=sub(m3v(RT,sub(point,s.S.p)),c);
  const vc=add(s.S.v,m3v(R,crs(s.S.w,c)));
  s.S.w=add(s.S.w,m3v(s.truth.Jinv,crs(r,m3v(RT,J))));
  s.S.v=sub(add(vc,scl(J,1/s.truth.m)),m3v(R,crs(s.S.w,c)));
}
function fleetCollisions() {
  if(fleet.drones.length<2)return;
  const geometry=new Map();
  const shape=d=>{if(!geometry.has(d))geometry.set(d,fleetShapes(d));return geometry.get(d);};
  for(let i=0;i<fleet.drones.length;i++)for(let j=i+1;j<fleet.drones.length;j++){
    const a=fleet.drones[i],b=fleet.drones[j],reach=a.state.cReach+b.state.cReach+.06;
    if(nrm(sub(a.state.S.p,b.state.S.p))>reach)continue;
    const A=shape(a),B=shape(b);
    for(const [d,own,other] of [[a,A,B],[b,B,A]])for(const prop of own.props)if(fleetPropHit(prop,other))withDrone(d,()=>breakDevice(prop.c,'prop','hit '+fleetName(d===a?b:a)));
    let contact=null;
    for(const x of A.boxes)for(const y of B.boxes){const c=fleetBoxContact(x,y);if(c && (!contact || c.depth>contact.depth))contact=c;}
    if(!contact)continue;
    const {point,normal,depth}=contact,relative=sub(fleetPointVelocity(a,point),fleetPointVelocity(b,point)),vn=dot(relative,normal);
    const inv=fleetInverseMass(a,point,normal)+fleetInverseMass(b,point,normal),speed=Math.max(0,(-1.1*Math.min(0,vn)+Math.min(.8,Math.max(0,depth-.001)*.15/PDT))/Math.max(1e-6,inv));
    let J=scl(normal,speed);
    const tangent=sub(relative,scl(normal,vn)),length=nrm(tangent);
    if(length>1e-6){const t=scl(tangent,1/length),ti=fleetInverseMass(a,point,t)+fleetInverseMass(b,point,t);J=sub(J,scl(t,Math.min(.3*speed,length/ti)));}
    fleetImpulse(a,point,J);fleetImpulse(b,point,scl(J,-1));
    // Remove deep overlaps with mass-weighted shifts, avoiding an explosive spring.
    const ia=1/a.state.truth.m,ib=1/b.state.truth.m,shift=Math.max(0,depth-.002)*.2;
    a.state.S.p=add(a.state.S.p,scl(normal,shift*ia/(ia+ib)));b.state.S.p=sub(b.state.S.p,scl(normal,shift*ib/(ia+ib)));
    for(const d of [a,b]){
      d.contacts=(d.contacts || 0)+1;
      if(vn < -3)withDrone(d,()=>crash('Hit '+fleetName(d===a?b:a)+' at '+(-vn).toFixed(1)+' m/s.'));
    }
  }
}
