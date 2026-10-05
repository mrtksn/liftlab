# Real board support

The Computers tab offers a deployment path for every listed board. Microcontroller binaries are specific to the chip; changing a simulated board never makes another chip’s binary compatible.

| Board | Installation | PWM outputs (motors + servos) | Default control loop |
| --- | --- | --- | --- |
| ESP32 | Browser flashing: flight or ground firmware | 16 | 1000 Hz |
| ESP32-S3 | Browser flashing: flight or ground firmware | 8 | 1000 Hz |
| ESP32-C3 | Browser flashing: flight or ground firmware | 6 | 250 Hz, single core |
| Pi Zero / Zero 2 W / Pi 4 | Generated Raspberry Pi OS build/configuration/service commands | Pi latch GPIO / hardware PWM | Companion tasks |
| Mac or PC | Build/run dfb_ground on macOS or Linux | None | Ground controller |

Flight firmware currently runs the flight core and telemetry/radio, with standard PWM ESC/servo outputs. Navigation and latch outputs remain on the Pi; arbitrary task assignments in the simulator do not add hardware drivers. Pi and ESP ground firmware use built-in formulas; flight formula uploads last until reboot.

## Connections

The provided firmware uses UART0, not native USB Serial/JTAG, for its live console/design link. Use the board’s USB-to-UART connector (CP210x/CH340/FTDI) or an external 3.3 V USB-to-UART adapter. A native USB connector can flash S3/C3, but after flashing select the UART adapter to send the design. A board with only native USB needs that adapter for the running firmware.

| Chip | UART0 TX → adapter/Pi RX | UART0 RX ← adapter/Pi TX | Default I2C SDA, SCL | Default motor pins |
| --- | --- | --- | --- | --- |
| ESP32 | 1 | 3 | 21, 22 | 25, 26, 27, 14 (then 32, 33, 4, 13) |
| S3 | 43 | 44 | 17, 18 | 4, 5, 6, 7 |
| C3 | 21 | 20 | 0, 1 | 4, 5, 6, 7 |

Connect grounds. Do not drive UART pins from a USB adapter and Pi at the same time. The installer maps motors by name, validates chip/output limits and duplicate pins, and requires wiring first, reboot, then the airframe. Defaults are conservative DevKit profiles: check the exact module schematic, especially PSRAM and onboard peripherals. On ESP32 WROVER, 16/17 belong to PSRAM; change servo and ground-controller wiring accordingly.

C3 has few available pins: its default servo pins (3,10) are also the default radio UART choice. Remove unused servos or move the receiver to free pins before enabling CRSF. Its sample ground wiring leaves throttle unassigned because four analog axes plus UART would overlap; for manual throttle, free an ADC pin by leaving another axis unassigned.

## Building and validation

With ESP-IDF 5.3.2 activated, run `sh tools/build_firmware.sh`. It builds both roles for each chip, writes `firmware/<chip>-<role>/`, and records offsets, sizes and SHA-256 in a version-2 manifest. It leaves sdkconfig/build products in `/tmp/liftlab-firmware-build` by default (`BUILD_DIR` overrides this). Rebuild firmware after changing C hardware profiles; original-ESP32 binaries cannot be reused for S3/C3. The installer also checks image chip IDs for locally selected files.

Run `node tools/test_board_install.js` for manifest/image/wiring checks. `.github/workflows/boards.yml` compiles the six ESP combinations and host programs on Linux/macOS. Local compilation and browser tests do not establish flight stability, sensor compatibility or timing on a physical board. Bench-check outputs and loop timing with props removed before flight.
