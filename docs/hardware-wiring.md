# Hardware wiring and custom drivers

Computers → Hardware wiring lists every motor, servo and sensor in the airframe. Assign a board, select a signal GPIO or sensor driver/address, and configure that board’s shared I²C SDA/SCL pins. In-use output pins are disabled; conflicting bus/output assignments, duplicate addresses, chip PWM limits and invalid calibrations are reported before settings are sent. GPIO numbers are chip GPIOs (BCM on Pi), not connector positions.

Wiring and custom C source are stored inside `computers.wiring` in the design. Browser storage, named designs, exported/shared design JSON, undo and redo include them. Older designs retain inferred defaults: actuators/IMU/compass/barometer follow the flight core; GPS/flow follow navigation. Explicit board IDs survive board deletion; a connection to a deleted board stays disconnected rather than moving to another board.

## Connections and profiles

Select **Use 10DOF module: MPU6050 + BMP180 + HMC5883L** to configure the three existing sensor parts (or add missing parts) on the flight board. They are placed together at the frame origin with one shared I²C bus, at addresses 0x68, 0x77 and 0x1e. Adjust mounting in Airframe to match the physical module. This names the actual chips rather than assuming every module sold as “10DOF” has the same silicon.

The built-in firmware now detects MPU6050-family IMUs, BMP280/BME280, BMP180/BMP085, and HMC5883L (when selected). MPU bypass exposes the compass on modules with auxiliary I²C wiring. HMC overflow/no-data/read failures are rejected; compass samples expire after 100 ms. Compass bias is in microtesla, with separate XYZ scale correction; the mounting matrix maps sensor axes into body axes. This is manual calibration, not automatic magnetic calibration. QMC5883L and the MPU9250’s AK8963 are different devices and need other drivers.

Motor output settings describe **standard PWM ESCs**, not direct motor power: frequency, minimum/maximum pulse. Servo rows define centre pulse and microseconds per radian; negative scaling reverses direction. DShot, direct brushed H-bridges and other output protocols need additional drivers. Physical motor/prop/servo response is configured in Airframe; simulator models remain separate from sensor register code.

Real outputs and onboard IMU/barometer/compass drivers currently live on the flight-core ESP. Other board assignments can be planned and are flagged as unsupported for deployment. NMEA GPS uses a Pi serial device path. Pi I²C sensors, distributed motor outputs and optical-flow hardware are not implemented. The simulator only feeds the flight core sensors assigned to its board; motor outputs assigned elsewhere/off are held off.

## Sending a design

Install the new chip-specific firmware, then send hardware settings and reboot, then send the airframe. The installer and manual commands use the saved wiring. Firmware checks each setting independently and refuses reserved pins/conflicts; a failed sequence may leave pending settings changed but does not save them or alter running wiring. `show` reports pending/running wiring, sensor profile numbers and detected chips.

Profiles: `imu=driver,address`, `baro=driver,address`, `mag=driver,address`. Drivers are -1 off, 0 auto, 1 MPU/BMP280/HMC, 2 LIS3DH/BMP180 (not compass), 3 custom C. Address 0 uses the driver’s defaults. `i2c=SDA,SCL`; `mag_matrix=` nine row-major rotation values; `mag_bias=` three offsets; `mag_scale=` three scale factors. Existing version-4 saved firmware wiring migrates to version 5; existing IMU/barometer auto-detection remains, compass starts disabled.

## Custom C driver editor

Each flight board has a folded **Custom low-level sensor drivers (C)** editor, with MPU6050, BMP180, HMC5883L and combined 10DOF presets. Load a preset, edit, and **Save driver code in design**. Choose **Custom C driver** on each corresponding sensor row. Saving source does not execute or compile it.

Export `custom_sensors.h`, copy it to `runner/fc/esp32/main/custom_sensors.h` and rebuild with ESP-IDF 5.3.2:

```sh
cp /path/to/custom_sensors.h runner/fc/esp32/main/custom_sensors.h
sh tools/build_firmware.sh
```

In Install, choose **Use firmware files from this computer**, select the bootloader, partition table and application from the correct `firmware/<chip>-flight/` folder, and flash. Then send the saved settings. If the board already has the matching custom build, the explicit checkbox allows configuration without reflashing. Chip-ID checks apply to custom binaries too. Runtime C source upload, browser C compilation and automatic binary/source matching are not implemented.

The interface uses `custom_read(address, register, buffer, length)` and `custom_write(address, register, value)` (0 success, -1 failure). Register transactions are limited to 7-bit addresses 8–119 and reads of 1–64 bytes. Driver init returns 0 success. IMU read returns 0 success and sets `have_gyro`; barometer/compass read returns 1 only for a new sample, 0 otherwise. Units: gyro rad/s, acceleration m/s², relative height m, compass µT in sensor axes. Mounting/bias/scale are applied afterwards; custom IMU gyro bias is measured at startup. Use `esp_timer_get_time()` for conversion deadlines. Do not delay/block inside read callbacks; init runs before flight tasks start.

These are native C drivers with access to the firmware, not sandboxed programs. They must be compiled and tested for timing and sensor units. The browser simulation uses its existing sensor models; it does not execute these C register drivers. A single compiled header serves the board; separate custom implementations for multiple sensors of the same kind need firmware extensions.

## Validation

`node tools/test_hardware_wiring.js` checks profiles, pin/address conflicts, persistence and board-ID stability. `node tools/test_board_install.js` checks bundles and installation sequencing. Compile/run `tools/test_sensor_drivers.c` to check the Bosch BMP180 published pressure example, nonblocking conversion sequence, MPU units/bypass, HMC axis ordering/overflow, and I²C failures. `python3 tools/sync_driver_presets.py --check` prevents the browser presets drifting from the compiled template.

Firmware compilation and fake-register tests have passed; physical sensors and flight are untested. Datasheet references: [Bosch BMP180](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf), [Honeywell HMC5883L](https://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf).
