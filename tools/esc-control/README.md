# Signal Workbench / ESC bench notes

This directory now serves the dedicated **ESP32 Signal Workbench**: multi-channel digital sniffing and generation, analog ADC logging, editable presets and sequences, configurable toolbar/large buttons, live sliders, editable UART text/numbers/bytes, waveform visualization, protocol decoders, captures and logs. The toy-drone motor UI is an editable preset. The top-right Wiring diagram button renders preset-owned configuration for GPIO assignments, circuit wiring, power, components and voltage limits. Use Edit wiring to change it; shipped presets, board profiles and diagram-library examples live in `config/*.json`.

See [the workbench guide](../../docs/signal-workbench.md) for capabilities, wiring, presets, firmware installation, and tests. Serve this directory locally and open it in Chrome or Edge:

```sh
python3 -m http.server 8765 --bind 127.0.0.1 --directory tools/esc-control
```

Open <http://127.0.0.1:8765/>. The dedicated firmware is `runner/signalbench` (ESP32-WROOM, USB serial at 921600 baud). The [original ESC panel](legacy/index.html) is preserved under `legacy/` and requires MicroPython; its device script is loaded into the REPL at 115200 baud when connected. The new native firmware and old MicroPython panel are separate protocols.

The hardware observations below describe our particular salvaged 1S toy-drone ESC and remain the basis of the motor-test preset.

## Bench notes — 8–9 November 2026

The setup uses a classic ESP32-D0WDQ6 revision 1.0 with 4 MB flash and MicroPython 1.29.0. GPIO18 is connected to the separate original ESC's blue M input; ESP32 ground and the ESC's battery negative are shared. The ESC powers the connected BLDC motor from its original 1S battery. Tests were reported with the propeller removed. The exact ESC model, signal voltage thresholds, and complete wiring details have not been independently verified.

- Input-only capture of the original flight controller showed approximately 550 µs high / 1,492 µs low at stop; about 745 µs / 1,292 µs at one running setting; and roughly 1,470–1,570 µs high / 465–560 µs low at a higher setting. High and low were sampled separately; initial outliers occurred. These are approximate, not a complete protocol specification or preserved raw capture.
- The user reported the motor remained running at reduced throttle and stopped using the original remote's stop button; after that button, captured timing returned to the initial range.
- With the ESP32 driving the ESC at 490 Hz, 750 µs and 1,000 µs short tests produced sound but no observed movement; 1,500 µs spun the motor, and 550 µs stopped it. A GPIO18-to-GPIO19 loopback confirmed approximately the intended timing, not signal voltage at the ESC.
- In the control panel, the user reported 1,070 µs as the lowest tried setting that started the motor, and release-to-stop worked. This was before the frequency selector was added, at the panel's 490 Hz setting.
- At 300 Hz, the user reported the motor spun at 2,960 µs, did not start at 2,970 µs, and stopped on release at 2,960 µs. The cause of the boundary is unknown.
- The user measured current with the propeller off and reported no increase above 2,000 µs at 300 Hz. Individual current readings, measurement setup, RPM, and thrust were not recorded. This observation alone does not establish maximum motor speed or thrust.

**Working proposal:** Treat 2,000 µs as a provisional upper test command and use 490 Hz as the baseline, since it is close to the original controller's measured frequency and operation and release-to-stop were observed there. The current plateau was observed at **300 Hz**, not 490 Hz, so it does not prove that 2,000 µs is the maximum effective command at 490 Hz or establish a safe motor/ESC limit. At 490 Hz, 2,000 µs is approximately 98% duty cycle. Recheck motor response and current under the intended supply and load before treating this as a calibrated limit.

These are limited bench observations, not flight validation. Use a propeller-free setup with the battery connector accessible, and verify stopping at each frequency before further experiments. The panel's conservative 300 Hz warning predates the limited test; the behavior is observed but not fully characterized.

## ESC supply-voltage test — 18 November 2026

The user tested a spare motor and ESC from the same 1S toy drone using this ESP32-WROOM control panel and confirmed the following behavior for this particular ESC. Voltages below refer to the **ESC power input**, not its PWM signal input.

| ESC supply voltage | User-reported behavior |
| --- | --- |
| Up to and including 5.1 V, within the tested range | Motor and ESC worked normally. |
| Above 5.1 V | Abnormal behavior, including failure to arm the motor. |
| Above 5.5 V | ESC began resetting itself. |

**Observed upper working supply voltage: 5.1 V for this specimen in this test.** This is a bench observation, not a manufacturer rating or a verified continuous operating limit. The reset mechanism has not been diagnosed. The exact PWM setting, supply/current limit, motor load, and test duration were not recorded in the voltage-test report; the earlier PWM observations above remain separate results.

Run the legacy MicroPython watchdog regression with `python3 tools/esc-control/test_device.py`.
Run the simulated browser control tests with `node --test tools/esc-control/test_app.cjs`.
