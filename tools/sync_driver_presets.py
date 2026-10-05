#!/usr/bin/env python3
"""Generate browser C presets from the compiled custom driver header."""
import json,pathlib,sys
root=pathlib.Path(__file__).resolve().parent.parent
header=(root/'runner/fc/esp32/main/custom_sensors.h').read_text()
presets={'10dof':{'label':'10DOF: MPU6050 + BMP180 + HMC5883L','code':header}}
sections=[('IMU',('custom_imu_init(int addr)','custom_imu_read(fc_imu *m)'),'m'),('BAROMETER',('custom_baro_init(int addr)','custom_baro_read(float *alt)'),'alt'),('COMPASS',('custom_mag_init(int addr)','custom_mag_read(float out[3])'),'out')]
for key,section,label in [('mpu6050','IMU','MPU6050'),('bmp180','BAROMETER','BMP180'),('hmc5883l','COMPASS','HMC5883L')]:
    code=header
    for other,names,arg in sections:
        if other==section:continue
        a=code.index('// BEGIN '+other);b=code.index('// END '+other)+len('// END '+other)
        code=code[:a]+f'// BEGIN {other}\nstatic int {names[0]} {{ (void)addr; return -1; }}\nstatic int {names[1]} {{ (void){arg}; return {"-1" if other=="IMU" else "0"}; }}\n// END {other}'+code[b:]
    presets[key]={'label':label,'code':code}
output="'use strict';\n// C driver presets, matching runner/fc/esp32/main/custom_sensors.h.\nconst DRIVER_PRESETS = "+json.dumps(presets,indent=2)+";\nif(typeof module!=='undefined') module.exports={DRIVER_PRESETS};\n"
p=root/'js/driver-presets.js'
if '--check' in sys.argv:
    if p.read_text()!=output:raise SystemExit('C presets out of date: run python3 tools/sync_driver_presets.py')
else:p.write_text(output)
