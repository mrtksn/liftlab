# Hardware wiring and custom drivers

Computers shows compact cards for boards, duty assignments, motors/servos, sensors, cargo latches and radio links. Click a board to edit its name/model and the wiring of its devices, or click a device to assign a board and edit its connection, GPIOs, driver and address. **Wiring overview…** opens all connections in one view, including the ground command module; its editable command pins are below the overview. **Add a board** opens a grouped picker, and **Install / export…** opens the existing installation guide and downloadable board files. I²C SDA/SCL selectors appear on every I²C sensor card; changing them updates the shared bus for that board. Calibration, board timing and C source are folded into advanced settings. In-use output pins are disabled; conflicting bus/output assignments, duplicate addresses, chip PWM limits and invalid calibrations are reported before settings are sent. GPIO numbers are chip GPIOs (BCM on Pi), not connector positions.

Wiring and custom C source are stored inside `computers.wiring` in the design. Browser storage, named designs, exported/shared design JSON, undo and redo include them. Older designs retain inferred defaults: actuators/IMU/compass/barometer follow the flight core; GPS/flow follow navigation; latches follow cargo. Explicit board IDs survive board deletion; a connection to a deleted board stays disconnected rather than moving to another board.

## Connections and profiles

Select **Use 10DOF module: MPU6050 + BMP180 + HMC5883L** to configure the three existing sensor parts (or add missing parts) on the flight board. They are placed together at the frame origin with one shared I²C bus, at addresses 0x68, 0x77 and 0x1e. Adjust mounting in Airframe to match the physical module. This names the actual chips rather than assuming every module sold as “10DOF” has the same silicon.

The built-in firmware now detects MPU6050-family IMUs, BMP280/BME280, BMP180/BMP085, and HMC5883L (when selected). MPU bypass exposes the compass on modules with auxiliary I²C wiring. HMC overflow/no-data/read failures are rejected; compass samples expire after 100 ms. Compass bias is in microtesla, with separate XYZ scale correction; the mounting matrix maps sensor axes into body axes. This is manual calibration, not automatic magnetic calibration. QMC5883L and the MPU9250’s AK8963 are different devices and need other drivers.

Each motor card selects **PWM ESC** or **Brushed motor · MOSFET**. ESCs use the board’s pulse frequency and minimum/maximum width. MOSFETs use a gate-control GPIO, shared board frequency (1000–30000 Hz, default 20000 Hz) and individual duty ceiling (1–100%). Modes may be mixed on one flight-core ESP. Servos keep their own 50 Hz timer; motor count and servo count still share the chip’s channel limit.

MOSFET mode is active-high and runs in one direction. It switches an external power stage; the motor never connects directly to a GPIO. Use a gate driver or a MOSFET suitable for the GPIO’s 3.3 V drive, external gate pulldown, common ground and flyback protection appropriate to the motor circuit. The external pulldown holds the gate off before firmware takes control. Software starts with a low gate and zero duty, stops unused motors, and shuts outputs down on peripheral failures. ESCs retain their minimum pulse when stopped. The flight core’s controlled-descent failsafe remains active; crash/disarm cutoffs and hardware shutdown yield zero MOSFET duty.

MOSFET output uses 10-bit PWM (0–1023); ESC/servo pulses use 14-bit timers on the APB clock. The frequency is shared across MOSFET motors on the board, while duty ceilings are individual. A duty ceiling clips the requested output; controller allocation, headroom estimates and native learning telemetry currently use the full motor model/requested throttle. Match the Airframe motor model to the actual motor/prop and validate reduced authority separately. H-bridge direction control, DShot and custom motor protocols remain unsupported.

Servo rows define centre pulse and microseconds per radian; negative scaling reverses direction. Physical motor/prop/servo response is configured in Airframe; simulator models remain separate from sensor register code.

Battery voltage uses an ADC1 GPIO and a resistor-divider ratio; ExpressLRS uses a pair of RX/TX GPIOs on the flight-core ESP running telemetry (receiver TX → board RX). These settings are now part of the design and sent by Install; sending hardware settings replaces battery/radio assignments previously entered manually. Command-module buttons, stick ADCs, LED/buzzer and transmitter UART pins are also saved in the design and used by its installation guide.

Pi latch cards offer hardware PWM on GPIO 18/19 (channels 0/1) or digital on/off. The cargo task must run on that Pi. Install derives the `--latch pwmN,gpioN,dry` list in airframe order; disconnected latches stay `dry` and hold their current position in the simulator. PWM uses the existing shared closed/open pulses of 1000/2000 µs. Digital GPIO drives an external latch switch/driver.

Pi board-link, GPS and radio cards use serial device paths. `/dev/serial0` maps to fixed GPIO 14 TX / 15 RX; a USB adapter uses no Pi GPIO. The default flight link reserves GPIO 14/15; moving it to a USB serial path frees them. Duplicate serial ports and GPIO claims are rejected. ESP UART0 pins remain fixed by chip.

Real outputs and onboard IMU/barometer/compass drivers currently live on the flight-core ESP. Other board assignments can be planned and are flagged as unsupported for deployment. NMEA GPS uses a Pi serial device path. Pi I²C sensors, distributed motor outputs and optical-flow hardware are not implemented. The simulator only feeds the flight core sensors assigned to its board; motor outputs assigned elsewhere/off are held off.

## Sending a design

MOSFET mode requires the updated chip-specific flight firmware (wiring version 6). Old saved designs and old flash wiring default to PWM ESCs. Keep MOSFET motor power disconnected while flashing/configuring; send the saved driver settings and restart before applying motor power. Factory/reset ESC pulses are not a zero-duty MOSFET signal. Install the new chip-specific firmware, then send hardware settings and reboot, then send the airframe. The installer and manual commands use the saved wiring. Firmware checks each setting independently and refuses reserved pins/conflicts; a failed sequence may leave pending settings changed but does not save them or alter running wiring. `show` reports pending/running wiring, sensor profile numbers and detected chips.

The airframe button currently bypasses the hardware-settings validation and does not track whether the intended settings/custom firmware were installed and rebooted. Follow the sequence above and verify `show`; uploading an airframe alone does not configure its GPIOs or motor drivers. The install telemetry can also incorrectly show **NO GYRO** for a working custom IMU driver. These are pending bugs, described in the [review](review-2026-10-06.md).

Profiles: `imu=driver,address`, `baro=driver,address`, `mag=driver,address`. Drivers are -1 off, 0 auto, 1 MPU/BMP280/HMC, 2 LIS3DH/BMP180 (not compass), 3 custom C. Address 0 uses the driver’s defaults. `motor_driver=` lists 0 ESC / 1 MOSFET in airframe order; `motor_max=` lists duty ceilings in percent; `brushed_hz=` sets the shared duty-PWM frequency. Empty driver/ceiling lists reset to ESC/100% defaults. `i2c=SDA,SCL`; `mag_matrix=` nine row-major rotation values; `mag_bias=` three offsets; `mag_scale=` three scale factors. Existing version-2 through version-5 saved firmware wiring migrates to version 6; existing IMU/barometer auto-detection remains, compass starts disabled.

## Custom C driver editor

Each flight board has a folded **custom sensor drivers (C)** editor under Board settings & custom code, with MPU6050, BMP180, HMC5883L and combined 10DOF presets. Load a preset, edit, and **Save driver code in design**. Choose **Custom C driver** on each corresponding sensor row. Saving source does not execute or compile it.

Export `custom_sensors.h`, copy it to `runner/fc/esp32/main/custom_sensors.h` and rebuild with ESP-IDF 5.3.2:

```sh
cp /path/to/custom_sensors.h runner/fc/esp32/main/custom_sensors.h
sh tools/build_firmware.sh
```

In Install, choose **Use firmware files from this computer**, select the bootloader, partition table and application from the correct `firmware/<chip>-flight/` folder, and flash. Then send the saved settings. If the board already has the matching custom build, the explicit checkbox allows configuration without reflashing. Chip-ID checks apply to custom binaries too. Runtime C source upload, browser C compilation and automatic binary/source matching are not implemented.

The interface uses `custom_read(address, register, buffer, length)` and `custom_write(address, register, value)` (0 success, -1 failure). Register transactions are limited to 7-bit addresses 8–119 and reads of 1–64 bytes. Driver init returns 0 success. IMU read returns 0 success and sets `have_gyro`; barometer/compass read returns 1 only for a new sample, 0 otherwise. Units: gyro rad/s, acceleration m/s², relative height m, compass µT in sensor axes. Mounting/bias/scale are applied afterwards; custom IMU gyro bias is measured at startup. Use `esp_timer_get_time()` for conversion deadlines. Do not delay/block inside read callbacks; init runs before flight tasks start.

These are native C drivers with access to the firmware, not sandboxed programs. They must be compiled and tested for timing and sensor units. Unsaved editor drafts survive pin changes within the current design; loading another design clears them. Save driver code to include it in exported designs.

The browser simulation uses its existing sensor models; it does not execute these C register drivers. A single compiled header serves the board; separate custom implementations for multiple sensors of the same kind need firmware extensions.

## AI agent

The AI agent can inspect and edit saved hardware wiring without enabling **Run JavaScript**. `get_computers` returns stable board IDs; `set_computers` preserves those IDs when renaming, reordering or changing a board. Include existing IDs in edits. New boards receive new IDs; removing a board leaves its explicit connections disconnected.

Current API caveat: `get_computers` includes read-only `flight_loop_hz` on the flight-core board, but `set_computers` rejects that field. Until the round-trip bug is fixed, pass only `id`, `name`, `kind` and `tasks` for each board. `get_install_settings.ready` checks wiring, not complete physical task support; it currently misses the unsupported navigation-on-MCU assignment. Check the [board capability table](boards.md) as well.

`get_hardware` lists boards, devices and profiles, or returns one board’s pins, assignments and checks. `get_wiring_overview` reads the connection list. `set_hardware` changes device assignments/GPIOs, sensor profiles/addresses, shared buses, motor drivers/duty limits, servo calibration, battery/radio wiring, Pi serial ports and command-module inputs. Related changes can be batched atomically. New validation errors reject the entire edit; `draft=true` deliberately saves incomplete/unsupported wiring and reports blockers. Existing errors can remain during incremental repairs. `get_install_settings` reads the resulting settings; it does not send them.

`apply_hardware_preset` uses the same 10DOF preset as the UI. `get_driver_code` reads saved source or presets in chunks; `set_driver_code` saves full source or a preset, respects the formula-edit confirmation preference, and refuses to overwrite an unsaved editor draft. Changes use the normal design save/reset/undo paths. Custom C still requires export, an ESP-IDF rebuild, custom firmware installation and **Custom C driver** selection on the sensors. These tools do not compile C or flash/control physical hardware.

Examples: “Check my wiring for GPIO conflicts”, “Use a brushed MOSFET driver for M1 with a 60% ceiling at 12 kHz”, or “Configure my 10DOF module and show its wiring.”

## Validation

`node tools/test_agent_hardware.js` checks registered agent tools: board identity, atomic GPIO/port validation, installation settings, ground wiring persistence, 10DOF sensor creation, C source chunking and editor-draft/confirmation protection. The host CI jobs run these checks.

`tools/test_motor_outputs.c` compiles the production configuration/output modules against mocked GPIO/LEDC calls for ESP32/S3/C3. It checks mixed timers, zero-duty startup/shutdown, duty ceilings, invalid commands, failed setup/writes, channel limits and versioned flash migration. These host tests run in CI alongside chip firmware builds; they do not verify physical waveforms.

`node tools/test_hardware_wiring.js` checks profiles, pin/address conflicts, persistence and board-ID stability. `node tools/test_board_install.js` checks bundles and generated installation-command ordering; it does not prove the airframe upload is gated on wiring validation. Compile/run `tools/test_sensor_drivers.c` to check the Bosch BMP180 published pressure example, nonblocking conversion sequence, MPU units/bypass, HMC axis ordering/overflow, and I²C failures. `python3 tools/sync_driver_presets.py --check` prevents the browser presets drifting from the compiled template.

Firmware compilation and fake-register tests have passed; physical sensors and flight are untested. Datasheet references: [Bosch BMP180](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf), [Honeywell HMC5883L](https://cdn.sparkfun.com/datasheets/Sensors/Magneto/HMC5883L-FDS.pdf).

## Why C compilation is not in the browser yet

C compilation in a browser is possible: [Wasmer demonstrates Clang running in WebAssembly](https://wasmer.io/posts/clang-in-browser). This project currently contains the compiled flight controller and a formula compiler, but no C compiler/runtime toolchain for editor source.

There are two distinct outputs: a WebAssembly driver for browser tests, and native ESP firmware built against ESP-IDF for the selected chip. A browser driver additionally needs simulated I²C registers, conversion timing and device responses; the current sensor models produce measurements rather than emulate registers. WebAssembly output cannot be flashed as the ESP application. ESP32/S3 use Xtensa, while C3 uses RISC-V ([Espressif toolchains](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/tools/idf-tools.html)).

Follow-up architecture: load a C compiler on demand in a worker, compile against a small portable driver interface, run against virtual register buses with time limits and diagnostics, then provide a chip-specific firmware build path. Pure GitHub Pages currently provides neither that compiler nor an ESP-IDF build service. Custom SPI, UART and ADC sensor drivers need additional firmware interfaces; the current C callbacks expose I²C.
