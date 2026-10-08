# Real board support

The Computers tab offers an installation path for every listed board, but physical firmware supports fewer task and device combinations than the simulator. Microcontroller binaries are specific to the chip; changing a simulated board never makes another chip’s binary compatible.

| Board | Installation | PWM outputs (motors + servos) | Default control loop |
| --- | --- | --- | --- |
| ESP32 | Browser flashing: flight or ground firmware | 16 | 1000 Hz |
| ESP32-S3 | Browser flashing: flight or ground firmware | 8 | 1000 Hz |
| ESP32-C3 | Browser flashing: flight or ground firmware | 6 | 250 Hz, single core |
| Pi Zero / Zero 2 W / Pi 4 | Generated Raspberry Pi OS build/configuration/service commands | Pi latch GPIO / hardware PWM | Companion tasks |
| Mac or PC | Build/run dfb_ground on macOS or Linux | None | Ground controller |

Flight firmware currently runs the flight core and telemetry/radio, with PWM ESC, active-high brushed MOSFET and servo outputs. Pi `dfb_pi` implements navigation, learning, supervision and optional latch outputs. Arbitrary task assignments in the simulator do not add hardware drivers.

| Capability | Browser simulation | Physical implementation |
| --- | --- | --- |
| Navigation | Pi or flight-core MCU | Pi; MCU navigation is not integrated |
| Cargo | Any board | Pi PWM/digital latch outputs; no load-switch input |
| IMU, barometer, compass | Simulated measurements | Flight-core ESP I²C drivers; built-in profiles or rebuilt custom C |
| GPS / optical flow | Navigation sensor models | Pi NMEA GPS; no optical-flow hardware driver |
| Learning / supervision | Pi task instances in WebAssembly | Pi companion code; no motor-temperature/ESC-current sensor drivers |
| Motor / servo assignments | Flight-core outputs; other assignments are held off | Flight-core ESP outputs; distributed outputs are not implemented |
| Custom C sensor source | Saved and exported; not executed | ESP-IDF rebuild and matching chip firmware required |

Flight formula uploads are temporary and last until reboot. Pi companion formulas and ESP ground formulas use their compiled built-in program; native `dfb_ground` on Pi/macOS/Linux can load an edited `.rnp` with `--program`. These are separate from custom C sensor drivers.

## Connections

The provided firmware uses UART0, not native USB Serial/JTAG, for its live console/design link. Use the board’s USB-to-UART connector (CP210x/CH340/FTDI) or an external 3.3 V USB-to-UART adapter. A native USB connector can flash S3/C3, but after flashing select the UART adapter to send the design. A board with only native USB needs that adapter for the running firmware.

| Chip | UART0 TX → adapter/Pi RX | UART0 RX ← adapter/Pi TX | Default I2C SDA, SCL | Suggested motor pins |
| --- | --- | --- | --- | --- |
| ESP32 | 1 | 3 | 21, 22 | 25, 26, 27, 14 (then 32, 33, 4, 13) |
| S3 | 43 | 44 | 17, 18 | 4, 5, 6, 7 |
| C3 | 21 | 20 | 0, 1 | 4, 5, 6, 7 |

Connect grounds. Do not drive UART pins from a USB adapter and Pi at the same time. The installer maps motors by name and validates chip/output limits and duplicate pins when sending hardware settings. Send wiring first, allow reboot, verify it with `show`, then send the airframe. This order is currently an instruction, not an enforced upload gate: **Send the airframe** does not check the saved wiring or matching custom firmware. Keep MOSFET motor power disconnected until the board runs the intended driver settings. Defaults are conservative DevKit profiles: check the exact module schematic, especially PSRAM and onboard peripherals. On ESP32 WROVER, 16/17 belong to PSRAM; change servo and ground-controller wiring accordingly.

A freshly flashed board (or one whose saved wiring was refused) drives **no** motor or servo pin: the suggested pins above are only offered by the installer and printed at power-on. Nothing is driven until `motors=`/`servos=` are sent, saved and the board restarts. A board flashed with firmware from before this change keeps whatever wiring it saved, including the old default ESC pulses on the suggested pins (1000 µs at 400 Hz: a 40% duty cycle on an H-bridge or MOSFET input).

**Which build, which chips.** `version` makes a board answer `firmware COMMIT CHIP ROLE`; the install dialog asks on Connect and compares it with the page's `firmware/manifest.json` (same build, another build, another chip or role, or firmware from before version reporting). `scan` (flight firmware, disarmed) lists every address answering on the sensors' I²C bus with what usually sits there: 0x1E is an HMC5883L, 0x0D a QMC5883L (no driver yet), 0x68/0x69 the MPU-6050 family, 0x76/0x77 BMP280/BMP180. A GY-87's compass answers only once the IMU driver has turned the MPU's bypass on.

**Watching a board in the 3D view.** In the install dialog's step 3, **Show it in the 3D view** pauses the simulation and makes the drone follow the connected flight board's telemetry (attitude, and height with a barometer) over the same USB cable, on any ESP32 (no Bluetooth needed). It only watches: nothing is sent. The bar over the view has **Console** (back to the dialog, same connection) and **Stop**; Run, Disconnect or unplugging ends it too. The roll/pitch/yaw signs follow the Bluetooth live view's and are not yet confirmed against a physical board.

C3 has few available pins: its default servo pins (3,10) are also the default radio UART choice. Remove unused servos or move the receiver to free pins before enabling CRSF. Its sample ground wiring leaves throttle unassigned because four analog axes plus UART would overlap; for manual throttle, free an ADC pin by leaving another axis unassigned.

### Pi command network

`dfb_pi` opens UDP port 14560 on all IPv4 interfaces by default. It accepts flight and latch text commands without authentication or a sender allowlist. The generated service uses this behavior too. `--port 0` binds an arbitrary port; it does **not** disable this listener. Temporary mitigation: restrict inbound access with the Pi's firewall/network isolation before running physical actuators. Follow-up: a loopback default, explicit LAN opt-in, a disable option and authenticated remote control. Native `dfb_ground` already defaults its separate UDP port 14561 to loopback and can disable it with `--port 0`.

## Building and validation

With ESP-IDF 5.3.2 activated, run `sh tools/build_firmware.sh`. It builds both roles for each chip, writes `firmware/<chip>-<role>/`, and records offsets, sizes and SHA-256 in a version-2 manifest. It leaves sdkconfig/build products in `/tmp/liftlab-firmware-build` by default (`BUILD_DIR` overrides this). Rebuild firmware after changing C hardware profiles; original-ESP32 binaries cannot be reused for S3/C3. The installer also checks image chip IDs for locally selected files.

Run `node tools/test_board_install.js` for manifest/image/wiring checks. `.github/workflows/boards.yml` builds the six ESP bundles (uploaded with their manifest as the `firmware-COMMIT` artifact, and a warning when the checked-in ones are stale) and host programs on Linux/macOS, and runs wiring/agent/driver checks. It does not currently run the native flight/navigation/runner/ground behavioral suites, and it doesn't commit the bundles: source changes need those bundles committed before Pages can serve new firmware; see [deployment](deployment.md). `node tools/test_usb_view.cjs` (Playwright) checks the install dialog's version comparison and the USB 3D view against a mock serial port.

Local compilation and browser tests do not establish flight stability, sensor compatibility or timing on a physical board. Bench-check outputs and loop timing with props removed before flight. The [2026-10-06 review](review-2026-10-06.md) records two repeatable macOS navigation test failures and the current installer/diagnostic gaps.

Configure saved pins, sensor profiles and custom C drivers in Computers → Hardware wiring. See [hardware wiring and drivers](hardware-wiring.md).
