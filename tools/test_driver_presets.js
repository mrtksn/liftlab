'use strict';
const fs=require('node:fs'),os=require('node:os'),path=require('node:path'),cp=require('node:child_process');
const {DRIVER_PRESETS}=require('../js/driver-presets');
const dir=fs.mkdtempSync(path.join(os.tmpdir(),'liftlab-driver-presets-'));
try {
  for(const [name,p] of Object.entries(DRIVER_PRESETS)) {
    const file=path.join(dir,name+'.h');fs.writeFileSync(file,p.code);
    cp.execFileSync(process.env.CC||'cc',['-Wall','-Wextra','-Werror','-fsyntax-only','-Irunner/fc','-Irunner','-DCUSTOM_SENSOR_HEADER="'+file+'"','tools/test_sensor_drivers.c'],{stdio:'inherit'});
  }
  console.log('All four C editor presets compile.');
} finally { fs.rmSync(dir,{recursive:true,force:true}); }
