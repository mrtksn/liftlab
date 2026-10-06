'use strict';
// Saved geometry at rest, independent of dropped cargo, failed parts and paused flight state.
let designStatsCache = null;
function droneDesignStats() {
  const b = battCfg(), air = flightAtmosphere(), motors = actuators(), core = boardOf('core');
  const key = [designRevision, b.cells, b.capacity, air.rho, air.sound, core?.id,
    ...motors.map(c => `${c.id}:${wiredTo(c)?.id}:${flightDuty(c)}`)].join('|');
  if (designStatsCache?.key === key) return designStatsCache.value;
  const rigid = massProps('truth', cfg.comps, restAngle);
  const payload = cfg.comps.filter(c => c.type === 'hang').reduce((sum,c) => sum+c.mass,0);
  const mass = rigid.m+payload, voltage = b.cells*4.2;
  let area = 0, thrust = 0, rpm = 0, mach = 0;
  for (const c of motors) {
    const radius = propR(c); area += Math.PI*radius*radius;
    if (!core || wiredTo(c) !== core) continue;
    const mp = flightMotor(c), load = isCollective(c) ? collectiveLoad(c,mp,1) : null;
    const point = FlightPhysics.equilibrium(load ? {...mp,table:null,kT:load.kT,kQ:load.kQ,rhoRef:1.225} : mp, flightDuty(c), voltage, air.rho, air.sound);
    const omega = load ? Math.min(load.Og,point.Omega) : point.Omega;
    const force = load ? load.kT*omega*omega*air.rho/1.225 : point.T;
    thrust += force*Math.max(0,rotorNow(c,restAngle).d[2])*clamp((c.health ?? 100)/100,0,1);
    rpm = Math.max(rpm,omega*60/(2*Math.PI)); mach = Math.max(mach,omega*radius/air.sound);
  }
  const value = {mass,rigid,payload,area,motors:motors.length,thrust,tw:mass>0?thrust/(mass*G):0,
    diskLoading:area>0?mass*G/area:null,energy:b.cells*3.7*b.capacity,rpm,mach};
  designStatsCache = {key,value}; return value;
}
function renderDesignHud() {
  const box = document.getElementById('hudDesign'); if (!box) return;
  box.hidden = !editMode || !view.readouts;
  if (box.hidden) return;
  const s = droneDesignStats();
  const rows = [
    `mass ${s.mass.toFixed(3)} kg · ${s.motors} rotors / ${s.area.toFixed(4)} m²`,
    `disk loading ${s.diskLoading==null?'—':s.diskLoading.toFixed(1)+' N/m²'} · T/W ${s.tw.toFixed(2)}×`,
    `CoG (${s.rigid.c.map(x=>(x*1000).toFixed(0)).join(',')}) mm · battery ${s.energy.toFixed(1)} Wh`,
    `max ${s.motors?Math.round(s.rpm).toLocaleString():'—'} rpm · tip Mach ${s.motors?s.mach.toFixed(2):'—'}`,
  ];
  document.querySelectorAll('#hudDesignStats .hud-design-row').forEach((node,i) => setText(node,rows[i]));
}
